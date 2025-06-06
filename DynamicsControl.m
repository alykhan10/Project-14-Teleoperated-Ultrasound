% Newton-Euler Dynamics with PD Control
clc; clear; close all;

n = 7; % number of joint
T = 3; 
dt = 0.0002;
N = T / dt;

% Initial and desired joint positions
q = zeros(n,1);
dq = zeros(n,1);
q_desired = [0.1; -pi/4; 0.1; -pi/2; 0.2; pi/3; 0.1];   % some arbitrary target pose
dq_desired = zeros(n,1);
ddq = zeros(n,1);


% Gains
Kp = diag(ones(n,1) * 20);   % proportional gains
Kd = diag(ones(n,1) * 20);    % derivative gains

% Assumptions of link parameters
for i = 1:n
    links(i).mass = 2+0.5*i;                      % kg
    links(i).com = [0; 0; 0.05+0.01*i];           % center of mass
    inertia_val = 0.01 + 0.001*i;
    links(i).inertia = diag([inertia_val, inertia_val, inertia_val]);  % inertia matrix 3x3
    links(i).axis = [0; 0; 1];                   % z-axis for revolute joint
    links(i).r = [0; 0; 0.1];                    % vector from joint i to i+1
end

% Logging
q_log = zeros(n, N);
tau_log = zeros(n, N);
t_log = (0:N-1) * dt;

for i = 1:N
    % PD Control
    e = q_desired - q;
    de = dq_desired - dq;
    tau_control = Kp * e + Kd * de;
    

    % Initialize
    V = zeros(6,n); 
    a = zeros(6,n); 
    f = zeros(6,n); 
    tau = zeros(n,1);

    % Base velocity and acceleration
    V0 = zeros(6,1);              % base spatial velocity
    a0 = [0; 0; 0; 0; 0; -9.81];  % base spatial acceleration

    % Velocity and acceleration
    for j = 1:n
        S = [zeros(3,1); links(j).axis];  % spatial joint screw
        V(:,j) = V0 + S * dq(j);          % linear approx
        dS = zeros(6,1);                  % assume constant screw
        a(:,j) = a0 + S * ddq(j) + dS * dq(j);  % no coriolis term here
        V0 = V(:,j);
        a0 = a(:,j);
    end

    % Force and Torque
    for j = n:-1:1
        I = [links(j).inertia, zeros(3); zeros(3), links(j).mass * eye(3)];
        f_a = I * a(:,j) + cross_spatial(V(:,j)) * (I * V(:,j));
        f(:,j) = f_a;   % considering no external force
        tau(j) = links(j).axis' * f(4:6,j);  % torque around z-axis
    end

    % Total torque
    % Gravity (approximate)
    tau_gravity = zeros(n,1);
    g = [0; 0; -9.81];
    for j = 1:n
        r = links(j).com;
        z = links(j).axis;
        tau_gravity(j) = -links(j).mass * g' * r;
    end

    % Final torque
    tau_total = tau_control + tau_gravity;
    tau_total = max(min(tau_total, 30), -30);  % limit torque


    % Simple forward Euler integration
    M_full = zeros(n, n);  % n = 7

    for k = 1:n
        m = links(k).mass;
        I = links(k).inertia;  % 3x3 inertia matrix at CoM
        M_full(k,k) = 1 + 0.1 * k;  % scaled mass

    end

    ddq = M_full \ tau_total; 

    dq = dq + ddq * dt;
    dq = max(min(dq, 5), -5);  % prevent runaway speeds 
    q = q + dq * dt;
    q = max(min(q, pi), -pi);  % joint range limit 

    % Log
    q_log(:, i) = q;
    tau_log(:, i) = tau_total;
end

% Plotting
figure;
for j = 1:n
    subplot(4,2,j);
    plot(t_log, q_log(j,:), 'LineWidth', 1.2); hold on;
    yline(q_desired(j), '--r');
    title(['Joint ', num2str(j)]);
    xlabel('Time [s]'); ylabel('q [rad]');
    grid on;
end
sgtitle('Joint Positions vs Time (PD + NE)');

function Sx = cross_spatial(v)
    w = v(1:3); v_lin = v(4:6);
    Sx = [skew(w), zeros(3); skew(v_lin), skew(w)];
end

function M = skew(v)
    M = [  0   -v(3)  v(2);
          v(3)   0   -v(1);
         -v(2) v(1)    0];
end
