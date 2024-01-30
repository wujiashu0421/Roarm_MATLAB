clear
clc


%% ROS Initialisation

% Create a Matlab Node
controlNode = ros2node("/Matlab");

% Create publisher for joint state commands along with message definition
control_pub = ros2publisher(controlNode, "/matlab_control", "sensor_msgs/JointState");
control_msg = ros2message(control_pub);

% Create Service Client
feedback_client = ros2svcclient(controlNode, "/robot_feedback", "std_srvs/Trigger");

resp = feedback_client_request(feedback_client)

%% Robot Simulation Initialisation
% Inport Robot File
robot = importrobot("roarm.urdf"); 

% 
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0.25 0.25 0.25 1 1 1];



% Obtain a random configuration of the robot as the starting configuration
Start = robot.randomConfiguration;
tform = getTransform(robot,Start,"l4_link","base_link");
show(robot, Start)

% Speed, acceleration in 1/4096 revolution per unit
joint_states_publish(control_pub, Start, 200, 1, control_msg);

pause(5);

% Try solving using the inverse kinematics solver.
[configSoln,solnInfo] = ik('l4_link', tform, weights, homeConfiguration(robot));
show(robot, configSoln)



%% Functions:

%% Joint State Publisher
% Input: Publisher Object. Configuration Variable. Message Variable
% Output: None
function joint_states_publish(publisher, configuration, speed, acc, msg)
    position = zeros(1, size(configuration, 2)+2);
    for i = 1:size(configuration, 2)
        position(i) = configuration(i).JointPosition;
    end
    position(end-1) = speed;
    position(end) = acc;
    
    msg.position = position;

    send(publisher, msg);

end

%% Service Client
% Input: Client
% Output: Robot feedback information
function resp = feedback_client_request(client)
    req = ros2message(client);
    %waitForServer(client,"Timeout",3)
    resp = call(client,req,"Timeout",3);
end

%%

% Floating base in orbit robotic arm
% https://au.mathworks.com/matlabcentral/fileexchange/60911-spacecraft-robotics-toolkit
%run('SPART\SPART2path.m')