function IM_Test(robot,Variables)
%This function test the Generalized (H) and Convective (C) inertia matrices functions comparing the recursive algorithms to H=N'M*N and C=N'*(M*Ndot+Mdot*N);.

%--- Assign variables ---%
%Base position
R0=Variables.R0;
r0=Variables.r0;

%Joint variables
qm=Variables.qm;

%Velocities
u0=Variables.u0;
um=Variables.um;

%--- Compute H and C using recursive algorithms ---%
%Kinematics
[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);
%Differential Kinematics
[Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robot);
%Velocities
[t0,tm]=Velocities(Bij,Bi0,P0,pm,u0,um,robot);
%Inertias
[I0,Im]=I_I(R0,RL,robot);
%Mass Composite Body matrix
[M0_tilde,Mm_tilde]=MCB(I0,Im,Bij,Bi0,robot);
%Generalized Inertia matrix
[H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);
H=[H0, H0m; H0m', Hm];
%Generalized Convective Inertia matrix
[C0, C0m, Cm0, Cm] = CIM(t0,tm,I0,Im,M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);
C=[C0, C0m; Cm0, Cm];

%--- Compute H and C using NOC matrix ---%
%Natural Orthogonal Complement matrix
N=NOC(r0,rL,P0,pm,robot);
%Generalized inertia matrix
[H_NOC]=GIM_NOC(N,I0,Im,robot);
%NOC time derivative
Ndot=NOCdot(r0,t0,rL,tm,P0,pm,robot);
%Convective inertia matrix
[C_NOC]=CIM_NOC(N,Ndot,t0,tm,I0,Im,robot);


%--- Test ---%
%GIM
test=abs(H-H_NOC)<1e-6;
assert(all(test(:)));
%CIM
test=abs(C-C_NOC)<1e-6;
assert(all(test(:)));




end

