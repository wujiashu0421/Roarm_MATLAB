function FD_Test(robot,Variables)
%This function test the Forward Dynamics functions.

%--- Assign variables ---%
%Base position
R0=Variables.R0;
r0=Variables.r0;

%Joint variables
qm=Variables.qm;

%Velocities
u0=Variables.u0;
um=Variables.um;

%External forces
wF0=Variables.wF0;
wFm=Variables.wFm;

%Joint torques
tauq0=Variables.tauq0;
tauqm=Variables.tauqm;

%--- Compute Kinematic and Dynamic magnitudes ---%
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
%Generalized Convective Inertia matrix
[C0, C0m, Cm0, Cm] = CIM(t0,tm,I0,Im,M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);
%Natural Orthogonal Complement Matrix
N = NOC(r0,rL,P0,pm,robot);

%--- Test Forward Dynamics ---%
H=[H0, H0m; H0m', Hm];
C=[C0, C0m; Cm0, Cm];
qdot=[u0;um];
tauq=[tauq0;tauqm];

%Compute qddot using H*qddot+C*qdot=tau+J'w
qddot_HC=H\(tauq+N'*[wF0;wFm(:)]-C*qdot);

%Forward Dynamics
[u0dot_FD,umdot_FD] = FD(tauq0,tauqm,wF0,wFm,t0,tm,P0,pm,I0,Im,Bij,Bi0,u0,um,robot);
qddot_FD=[u0dot_FD;umdot_FD];

%Test
test=abs(qddot_HC-qddot_FD)<1e-6;
assert(all(test(:)));

end



