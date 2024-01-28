% 2DOF Kinematic Manipulability Ellipse
%
%
% This example plots the kinematic manipullability ellipsoid for a 2-link
% planar manipulator.


%=== CODE ===%

%Clean and clear
clc
close all
clear

%--- Style Plot ---%
h=figure(1);
set(gca,'FontSize',16)
axis equal
hold all
grid on
box on
xlabel('x [m]')
ylabel('y [m]')
leg={};

%--- Joint States ---% 

%Joint variables
qm=deg2rad([45;-90]);

%Base position
R0=eye(3);
r0=[0;0;0];

%--- 2DOF Data with base 100 kg and manipulator 10 kg ---%
m0=100;
mi=10;
[robot,TEE_Ln,base_contour,man_contour,man_contour_end]=DOF2_Data(m0,mi);

%--- Kinematic Manipulability ---%
[elps_fixed,km_fixed,elps_floating,km_floating]=Kinematic_Manipulability(R0,r0,m0,mi,qm);

%--- Plot---%
%kinematics
[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);
%End-Effector
TEE=[RL(1:3,1:3,end),rL(1:3,end);zeros(1,3),1]*TEE_Ln;
%Plots
plot(elps_fixed(1,:)+TEE(1,4),elps_fixed(2,:)+TEE(2,4),'k','linewidth',2);
leg(end+1)={'Fixed'};
plot(elps_floating(1,:)+TEE(1,4),elps_floating(2,:)+TEE(2,4),'k:','linewidth',2);
leg(end+1)={sprintf('Floating m_{0}/m_{l}=%d',m0/mi)};

%--- 2DOF Data with base 500 kg and manipulator 10 kg ---%
m0=500;
mi=10;
[robot,TEE_Ln,base_contour,man_contour,man_contour_end]=DOF2_Data(m0,mi);

%--- Kinematic Manipulability ---%
[elps_fixed,km_fixed,elps_floating,km_floating]=Kinematic_Manipulability(R0,r0,m0,mi,qm);

%--- Plot---%
%kinematics
[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);
%End-Effector
TEE=[RL(1:3,1:3,end),rL(1:3,end);zeros(1,3),1]*TEE_Ln;
%Plots
plot(elps_floating(1,:)+TEE(1,4),elps_floating(2,:)+TEE(2,4),'k--','linewidth',2);
leg(end+1)={sprintf('Floating m_{0}/m_{l}=%d',m0/mi)};

%--- Plot Manipulator ---%
Man_Plot(R0,r0,base_contour,man_contour,man_contour_end,RL,rL,robot.n_links_joints);
legend(leg,'Location','best');





