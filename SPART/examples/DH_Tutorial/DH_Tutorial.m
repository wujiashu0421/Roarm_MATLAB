%SPART DH Tutorial

%--- Clean and clear ---%
clc
close all
clear

%--- Dimensions [mm] ---%
L0=50;
L1=125;
L2=144;
L3=47;
L4=142;
L5=80;
L6=70;

%--- Manipulator Definition ----%
%Number of joints/links
data.n=5;

%First joint
data.man(1).type=1;
data.man(1).DH.d = L1;
data.man(1).DH.alpha = pi/2;
data.man(1).DH.a = 0;
data.man(1).DH.theta=0;
data.man(1).b = [0;L1/2;0];
data.man(1).mass=2;
data.man(1).I=diag([2,1,3])/10;

%Second joint
data.man(2).type=1;
data.man(2).DH.d = 0;
data.man(2).DH.alpha = 0;
data.man(2).DH.a = sqrt(L2^2+L3^2);
data.man(2).DH.theta=atan2(L2,L3);
data.man(2).b = [cos(-data.man(2).DH.theta),-sin(-data.man(2).DH.theta),0;sin(-data.man(2).DH.theta),cos(-data.man(2).DH.theta),0;0,0,1]*[L3^2/2;L2^2/2 + L3*L2;0]/(L2 + L3);
data.man(2).mass=2;
data.man(2).I=diag([2,1,3])/10;

%Third joint
data.man(3).type=1;
data.man(3).DH.d = 0;
data.man(3).DH.alpha = 0;
data.man(3).DH.a =L4;
data.man(3).DH.theta=-atan2(L2,L3);
data.man(3).b = [L4/2;0;0];
data.man(3).mass=2;
data.man(3).I=diag([2,1,3])/10;

%Fourth joint
data.man(4).type=1;
data.man(4).DH.d = 0;
data.man(4).DH.alpha = pi/2;
data.man(4).DH.a = 0;
data.man(4).DH.theta=pi/2;
data.man(4).b = [0;0;-L5/2];
data.man(4).mass=2;
data.man(4).I=diag([2,1,3])/10;

%Fifth joint
data.man(5).type=1;
data.man(5).DH.d = L5+L6;
data.man(5).DH.alpha =-pi/2;
data.man(5).DH.a = 0;
data.man(5).DH.theta=-pi/2;
data.man(5).b = [L6/2;0;0];
data.man(5).mass=2;
data.man(5).I=diag([2,1,3])/10;

%First joint location with respect to base
data.base.T_L0_J1=[eye(3),[0;0;L0];zeros(1,3),1];

%Base-spacecraft mass and inertia
data.base.mass=20;
data.base.I=diag([2,1,3]);

%End-Effector
data.EE.theta=-pi/2;
data.EE.d=0;

%Base position
R0=eye(3);  %Rotation from Base-spacecraft to inertial
r0=[0;0;0]; %Position of the base-spacecraft

%Joint variables [rad]
qm=[0;0;0;0;0];

%Velocities
u0=zeros(6,1); %Base-spacecraft velocity
um=[4;-1;5;2;1]*pi/180; %Joint velocities

%--- Create robot structure ---%
[robot,T_Ln_EE] = DH_Serial2robot(data);

%--- Kinematics ---%
[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);
%End-Effector
TEE=[RL(1:3,1:3,end),rL(1:3,end);zeros(1,3),1]*T_Ln_EE;

%--- Differential Kinematics ---%
%Differential kinematics
[Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robot);
%Jacobian of the Link 3
[J03, Jm3]=Jacob(rL(1:3,3),r0,rL,P0,pm,3,robot);
%End-effector Jacobian
[J0EE, JmEE]=Jacob(TEE(1:3,4),r0,rL,P0,pm,robot.n_links_joints,robot);
%Velocities
[t0,tm]=Velocities(Bij,Bi0,P0,pm,u0,um,robot);

%--- Inertia Matrices ---%
%Inertias in inertial frames
[I0,Im]=I_I(R0,RL,robot);
%Mass Composite Body matrix
[M0_tilde,Mm_tilde]=MCB(I0,Im,Bij,Bi0,robot);
%Generalized Inertia matrix
[H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);
%Generalized Convective Inertia matrix
[C0, C0m, Cm0, Cm] = CIM(t0,tm,I0,Im,M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);

%--- Forward Dynamics ---%

%Gravity
g=9.8; %[m s-2]

%External forces (includes gravity and assumes z is the vertical direction)
wF0=[0;0;0;0;0;-robot.base_link.mass*g];
wFm=[zeros(5,robot.n_q);
    -robot.links(1).mass*g,-robot.links(2).mass*g,-robot.links(3).mass*g,-robot.links(4).mass*g,-robot.links(5).mass*g];

%Joint torques
tauq0=zeros(6,1);
tauqm=zeros(robot.n_q,1);

%Forward Dynamics
[u0dot_FD,umdot_FD] = FD(tauq0,tauqm,wF0,wFm,t0,tm,P0,pm,I0,Im,Bij,Bi0,u0,um,robot);

%--- Inverse Dynamics - Flying ---%

%Accelerations
u0dot=zeros(6,1);
umdot=zeros(robot.n_q,1);

%Accelerations
[t0dot,tmdot]=Accelerations(t0,tm,P0,pm,Bi0,Bij,u0,um,u0dot,umdot,robot);

%Inverse Dynamics - Flying base
[tau0,taum] = ID(wF0,wFm,t0,tm,t0dot,tmdot,P0,pm,I0,Im,Bij,Bi0,robot);

%--- Forward Dynamics - Floating ---%

%Accelerations
umdot=zeros(robot.n_q,1);

%Inverse Dynamics - Floating Base
[taum_floating,u0dot_floating] = Floating_ID(wF0,wFm,Mm_tilde,H0,t0,tm,P0,pm,I0,Im,Bij,Bi0,u0,um,umdot,robot);