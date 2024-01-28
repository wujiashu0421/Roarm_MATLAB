%This script tests the Kinematics and Dynamic functions

%--- Clean and clear ---%
clc
clear
close all

%--- Define manipulator data ---%

%Number of joints/links
data.n=2;

%Define symbolic variables
d = sym('d',[1,data.n],'real');
alpha = sym('alpha',[1,data.n],'real');
a = sym('a',[1,data.n],'real');
theta = sym('theta',[1,data.n],'real');
b = sym('b',[3,data.n],'real');
m = sym('m',[1,data.n],'real');
I = sym('I',[3,data.n],'real');
theta_EE = sym('theta_EE','real');
d_EE = sym('d_EE','real');
m_base = sym('m_base','real');
I_base = sym('I_base',[3,1],'real');

%First joint
data.man(1).type=1;
data.man(1).DH.d = d(1);
data.man(1).DH.alpha = alpha(1);
data.man(1).DH.a = a(1);
data.man(1).DH.theta = theta(1);
data.man(1).b = b(1:3,1);
data.man(1).mass=m(1);
data.man(1).I=diag(I(1:3,1));

%Second joint
data.man(2).type=1;
data.man(2).DH.d = d(2);
data.man(2).DH.alpha = alpha(2);
data.man(2).DH.a = a(2);
data.man(2).DH.theta = theta(2);
data.man(2).b = b(1:3,2);
data.man(2).mass=m(2);
data.man(2).I=diag(I(1:3,2));

%End-Effector 
data.EE.theta=theta_EE; %Rotation around z-axis
data.EE.d=d_EE; %Translation along z-axis

%First joint location with respect to base
data.base.T_L0_J1=[eye(3),[0;0;0];zeros(1,3),1];

%Base-spacecraft inertia matrix
data.base.mass=m_base;
data.base.I=diag(I_base(1:3,1));

%--- Create robot structure ---%
[robot,T_Ln_EE] = DH_Serial2robot(data);

%--- Parameters ---%

%Base position
R0=sym(eye(3));
r0=sym([0;0;0]);

%Joint variables
qm=sym('qm',[data.n,1],'real');

%Velocities
u0=sym('u0',[6,1],'real');
um=sym('um',[data.n,1],'real');

%Accelerations
u0dot=sym('u0dot',[6,1],'real');
umdot=sym('umdot',[data.n,1],'real');

%External forces
wF0=sym(zeros(6,1));
wFm=sym(zeros(6,data.n));

%Joint torques
tauq0=sym(zeros(6,1));
tauqm=sym([0;0]);

%--- Compute Kinematics, Dynamics, ID, and FD ---%

%Start profiling
profile on

%Kinematics
[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);
%End-Effector
TEE=[RL(1:3,1:3,end),rL(1:3,end);zeros(1,3),1]*T_Ln_EE;
%Differential Kinematics
[Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robot);
%End-effector Jacobian
[J0EE, JmEE]=Jacob(TEE(1:3,4),r0,rL,P0,pm,robot.n_links_joints,robot);
%Velocities
[t0,tm]=Velocities(Bij,Bi0,P0,pm,u0,um,robot);
%Accelerations
[t0dot,tmdot]=Accelerations(t0,tm,P0,pm,Bi0,Bij,u0,um,u0dot,umdot,robot);
%Inertias
[I0,Im]=I_I(R0,RL,robot);
%Inverse Dynamics
[tau0,taum] = ID(wF0,wFm,t0,tm,t0dot,tmdot,P0,pm,I0,Im,Bij,Bi0,robot);
%Mass Composite Body matrix
[M0_tilde,Mm_tilde]=MCB(I0,Im,Bij,Bi0,robot);
%Generalized Inertia matrix
[H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);
%Generalized Convective Inertia matrix
[C0, C0m, Cm0, Cm] = CIM(t0,tm,I0,Im,M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);

%Stop profiling
profile viewer

return


%--- Steps that take a very long time to compute ---%

%Inverse Dynamics Floating Base (has an inverse)
[taum_floating,u0dot_floating] = Floating_ID(wF0,wFm,Mm_tilde,H0,t0,tm,P0,pm,I0,Im,Bij,Bi0,u0,um,umdot,robot);
%Forward Dynamics
[u0dot_FD,umdot_FD] = FD(tauq0,tauqm,wF0,wFm,t0,tm,P0,pm,I0,Im,Bij,Bi0,u0,um,robot);

