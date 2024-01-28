function [robot,Variables] = Load_SerialDHRobot()
%Loads a random serial robot model (using DH parameters) and generates randomly variables that are used for the SPART tests

%Number of joints/links
data.n=randi(9,1)+1;

%Define DH parameters
for i=1:data.n
    %Add a joint
    data.man(i).type=randi(2,1,1);
    data.man(i).DH.d =rand(1,1);
    data.man(i).DH.alpha =-pi/2 + (pi).*rand(1,1);
    data.man(i).DH.a = rand(1,1);
    data.man(i).DH.theta = 0;
    data.man(i).b = [rand(1,1);rand(1,1);rand(1,1)];
    data.man(i).mass=3*rand(1,1);
    data.man(i).I=diag(rand(3,1))/10;
end

%End-Effector 
data.EE.theta=0; %Rotation around z-axis
data.EE.d=0; %Translation along z-axis

%Firts joint location with respect to base
Angles = -pi + (2*pi).*rand(3,1);
data.base.T_L0_J1=[Angles321_DCM(Angles),[1;0;0];zeros(1,3),1];

%Base-spacecraft inertia matrix
data.base.mass=10*rand(1,1);
data.base.I=diag(rand(3,1))/5;

%--- Compute Robot Model ---%
[robot] = DH_Serial2robot(data);

%--- Random Variables ---%
%Base position and orientation
Angles = -pi + (2*pi).*rand(3,1);
Variables.R0=Angles321_DCM(Angles);
Variables.r0=-1 + (2).*rand(3,1);

%Joint variables
Variables.qm=-pi/2 + (pi).*rand(robot.n_q,1);

%Velocities
Variables.u0=[-pi/2 + (pi).*rand(3,1);-1 + (2).*rand(3,1)];
Variables.um=-pi/2 + (pi).*rand(robot.n_q,1);

%Accelerations
Variables.u0dot=[-pi/2 + (pi).*rand(3,1);-1 + (2).*rand(3,1)];
Variables.umdot=-pi/2 + (pi).*rand(robot.n_q,1);

%External forces
Variables.wF0=(-1 + (2).*rand(6,1))/10;
Variables.wFm=(-1 + (2).*rand(6,robot.n_q))/10;

%Joint torques
Variables.tauq0=(-1 + (2).*rand(6,1))/10;
Variables.tauqm=(-1 + (2).*rand(robot.n_q,1))/10;

end