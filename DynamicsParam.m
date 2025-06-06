%% Dynamics
robot.DataFormat = 'row'; %setup to allow inverseDynamics to function properly
q = randomConfiguration(robot); 
wrench = [0 0 50 0 0 30]; %torques and forces applied to the body
tau = []; %to store tau values for each body

% assumption of dynamic parameters
mass = 20;

%insert dynamic parameters into Panda Franka Emika robot
for i = 1:11
    j = int2str(i);
    if i == 9
        link_name = "panda_hand";
        mass = 3;
    elseif i == 10
        link_name = "panda_leftfinger";
        mass = 1;
    elseif i == 11
        link_name = "panda_rightfinger";
        mass = 1;
    else
        link_name = "panda_link"+j;
    end
    com = randi([-10,10],1,3);
    inertia = randi([0,100],1,3)/100;
    robot.Bodies{1,i}.Mass = mass;
    robot.Bodies{1,i}.CenterOfMass = com;
    robot.Bodies{1,i}.Inertia = [inertia,0,0,0];
end

%begin dynamics
for i = 1:11  
    j = int2str(i);
    
    if i == 9
        link_name = "panda_hand";
    elseif i == 10
        link_name = "panda_leftfinger";
    elseif i == 11
        link_name = "panda_rightfinger";
    else
        link_name = "panda_link"+j;
    end

    fext = externalForce(robot,link_name,wrench,q); %external force on body i
    jointVel = []; %start from stationary position
    jointTorq = velocityProduct(robot,q,jointVel);
    jointAccel = forwardDynamics(robot,q,jointVel,jointTorq,fext);
    
    tau = [tau;inverseDynamics(robot,q,jointVel,jointAccel,fext)];
end