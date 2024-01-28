function [elps_fixed,km_fixed,elps_floating,km_floating]=Kinematic_Manipulability(R0,r0,m0,mi,qm)
%Function that computes the kinematic manipulability


%--- 2DOF Data ---%
[robot,TEE_Ln,base_contour,man_contour,man_contour_end]=DOF2_Data(m0,mi);

%--- Kinematics ---%
%Kinematics
[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);
TEE=[RL(1:3,1:3,end),rL(1:3,end);zeros(1,3),1]*TEE_Ln;
%Differential Kinematics
[Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robot);

%--- End-Effector Jacobian ---%
%End-effector Jacobian
[J0EE, JmEE]=Jacob(TEE(1:3,4),r0,rL,P0,pm,robot.n_links_joints,robot);

%--- Generalized Inertia Matrices ---%

%Inertias
[I0,Im]=I_I(R0,RL,robot);
%Mass Composite Body matrix
[M0_tilde,Mm_tilde]=MCB(I0,Im,Bij,Bi0,robot);
%Generalized Inertia matrix
[H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);

%--- Floating Jacobian ---%
JEE_star = JmEE-J0EE*(H0\H0m);

%--- Manipulability ---%

%Fixed base kinematic manipullabillity
[V,D] = eig(JmEE(4:5,:)*JmEE(4:5,:)');
[ex,ey]=ellipse(D(1,1),D(2,2),0,0,100);
elps_fixed=V*[ex;ey];
%Measure
km_fixed=sqrt(det(JmEE(4:5,:)*JmEE(4:5,:)'));

try
%Floating base kinematic manipullabillity
[V,D] = eig(JEE_star(4:5,:)*JEE_star(4:5,:)');
[ex,ey]=ellipse(D(1,1),D(2,2),0,0,100);
elps_floating=V*[ex;ey];
%Measure
km_floating=sqrt(det(JEE_star(4:5,:)*JEE_star(4:5,:)'));
catch
    elps_floating=[0,0];
    km_floating=0;
end
    