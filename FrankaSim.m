clc; clear; close all;

%% Load Robot
robot = loadrobot('frankaEmikaPanda', 'DataFormat', 'row', 'Gravity', [0 0 -9.81]);
endEffector = 'panda_hand';

%% Define Scan Grid
rows = 10;
cols = 10;
spacing = 0.05;
z = 0.5;

grid_points = [];

for i = 1:rows
    for j = 1:cols
        x = (j-1) * spacing;
        y = (i-1) * spacing;
        if mod(i,2)==0  % Zig-zag
            x = (cols - j) * spacing;
        end
        grid_points = [grid_points; x, y, z];
    end
end

%% Inverse Kinematics Setup
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0.25 0.25 0.25 1 1 1];  % Moderate orientation importance
initialguess = robot.homeConfiguration;
orientation = axang2quat([1 0 0 pi]);  % Flip 180° around X for downward probe

%% Set up Figure
figure;
ax = gca;
show(robot, initialguess, 'PreservePlot', false, 'Frames', 'off', 'Parent', ax);
hold on;

% Draw Grid Plane - optional grid lines
for i = 0:rows-1
    x_vals = 0:spacing:(cols-1)*spacing;
    y_val = i * spacing;
    z_vals = z * ones(size(x_vals));
    line(x_vals, y_val * ones(size(x_vals)), z_vals, 'Color', [0.6 0.6 0.6], 'LineStyle', '--');
end

for j = 0:cols-1
    y_vals = 0:spacing:(rows-1)*spacing;
    x_val = j * spacing;
    z_vals = z * ones(size(y_vals));
    line(x_val * ones(size(y_vals)), y_vals, z_vals, 'Color', [0.6 0.6 0.6], 'LineStyle', '--');
end

% Plot grid points
scatter3(grid_points(:,1), grid_points(:,2), grid_points(:,3), 40, 'r', 'filled');

% Labels and View
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
axis equal;
view(45, 30);

% Title
sgtitle('Franka Robot Ultrasound Simulation', 'FontWeight', 'bold');

%% Create Probe Model (Cylinder)
[probeX, probeY, probeZ] = cylinder(0.01);  % 1cm radius
probeZ = 0.05 * probeZ;  % 5cm height

probeSurf = surf(probeX, probeY, probeZ, 'FaceColor', 'b', 'EdgeColor', 'none');
lighting gouraud;
camlight;

%% Create Soft Tissue Surface (Placed Below Red Dots Plane)
[Xs, Ys] = meshgrid(0:spacing:(cols-1)*spacing, 0:spacing:(rows-1)*spacing);

% Flat soft tissue surface below red dots
Zs = z - 0.03 * ones(size(Xs));  % Place it below the red dots (a bit of depth)

% Darker skin tone color for better visibility
skinTone = [0.8, 0.5, 0.3];  % Darker tone for better contrast

% Draw soft tissue surface
tissueSurf = surf(Xs, Ys, Zs, ...
    'FaceColor', skinTone, ...
    'EdgeColor', 'none', ...
    'FaceAlpha', 0.8);  % Slight transparency

% Enhance appearance
lighting gouraud;
camlight('right');

%% Initialize Robot at First Scan Point
firstPos = grid_points(1,:);  % First scan target
probeLength = 0.11;  % 10 cm probe
offset = trvec2tform([0 0 -probeLength]);  % Move back along Z axis of probe
firstTargetPose = trvec2tform(firstPos) * quat2tform(orientation) * offset;

% Compute initial joint configuration
[initialConfig, ~] = ik(endEffector, firstTargetPose, weights, initialguess);

% Set the initial guess to this configuration
initialguess = initialConfig;

% Show robot at this starting configuration
show(robot, initialConfig, 'PreservePlot', false, 'Frames', 'off', 'Parent', ax);

drawnow;

filename = 'ultrasound_probe_scan.gif';  % Output file name
gifDelay = 0.05;  % Time between frames in seconds

%% Animate Robot over Grid
probePathX = []; probePathY = []; probePathZ = [];  % Store path for probe

for i = 2:size(grid_points,1)
    pos = grid_points(i,:);  % Target position
    offset = trvec2tform([0 0 -probeLength]);  % Offset remains the same
    targetPose = trvec2tform(pos) * quat2tform(orientation) * offset;
    
    % Inverse kinematics to get the robot configuration
    [configSol, ~] = ik(endEffector, targetPose, weights, initialguess);
    initialguess = configSol;

    % Show robot at the current configuration
    show(robot, configSol, 'PreservePlot', false, 'Frames', 'off', 'Parent', ax);
    
    % Update Probe Pose
    tform = getTransform(robot, configSol, endEffector);

    % Extract rotation and translation of end-effector
    R = tform(1:3, 1:3);
    T = tform(1:3, 4);
    
    % Apply offset in local Z-direction (probe points downward => move "down")
    probeTipPosition = T + R * [0; 0; -probeLength];  % Local Z offset

    % Apply rotation and translation to the probe surface
    probePoints = [probeX(:)'; probeY(:)'; probeZ(:)'];
    probeTransformed = R * probePoints + T;

    probeSurf.XData = reshape(probeTransformed(1,:), size(probeX));
    probeSurf.YData = reshape(probeTransformed(2,:), size(probeY));
    probeSurf.ZData = reshape(probeTransformed(3,:), size(probeZ));

    % Update probe path (store the position)
    probePathX = [probePathX, T(1)];
    probePathY = [probePathY, T(2)];
    probePathZ = [probePathZ, z];  % Force Z to red dot plane height
    
    % Draw the probe path (black line)
    plot3(probePathX, probePathY, probePathZ, 'k-', 'LineWidth', 1);
    
    % Slow down the robot movement between points within a line
    if mod(i, cols) == 0
        pause(0.005);  % Shorter pause to smooth out motion without being too abrupt
    end

    drawnow;

    frame = getframe(gcf);                   % Capture current figure
    im = frame2im(frame);                    % Convert to image
    [imind, cm] = rgb2ind(im, 256);          % Convert to indexed image
    
    if i == 2  % First frame of animation
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', gifDelay);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', gifDelay);
    end
end