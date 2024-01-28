function ID_Test(robot,Variables)
%This function test the Inverse Dynamics recursive functions by comparing them to the solutions obtained by H*qddot+C*qdot=tau+J'w.

%--- Assign variables ---%
%Base position
R0=eye(3);%Variables.R0;
r0=zeros(3,1);%Variables.r0;

%Joint variables
qm=Variables.qm;

%Velocities
u0=Variables.u0;
um=Variables.um;

%Accelerations
u0dot=Variables.u0dot;
umdot=Variables.um;

%External forces
wF0=Variables.wF0;
wFm=Variables.wFm;

%--- Compute Kinematic and Dynamic magnitudes ---%
%Kinematics
[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);
%Differential Kinematics
[Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robot);
%Velocities
[t0,tm]=Velocities(Bij,Bi0,P0,pm,u0,um,robot);
%Accelerations
[t0dot,tmdot]=Accelerations(t0,tm,P0,pm,Bi0,Bij,u0,um,u0dot,umdot,robot);
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

%--- Test Inverse Dynamics - Flying base ---%
H=[H0, H0m; H0m', Hm];
C=[C0, C0m; Cm0, Cm];
qddot=[u0dot;umdot];
qdot=[u0;um];

%Generalized forces using H*qddot+C*qdot=tau+J'w
tau_HC = H*qddot+C*qdot-N'*[wF0;wFm(:)];

%Generalized forces using the dedicated Inverse Dynamics
[tau0_ID,taum_ID] = ID(wF0,wFm,t0,tm,t0dot,tmdot,P0,pm,I0,Im,Bij,Bi0,robot);
tau_ID=[tau0_ID;taum_ID];

%Test
test=abs(tau_HC-tau_ID)<1e-6;
assert(all(test(:)));

%--- Test Inverse Dynamics - Floating base ---%

%Compute u0dot using H*qddot+C*qdot=tau+J'w
u0dot_floating_HC=H0\(N(:,1:6)'*[wF0;wFm(:)]-H0m*umdot-C0*u0-C0m*um);
%Generalized forces using H*qddot+C*qdot=tau+J'w
tau_floating_HC = H*[u0dot_floating_HC;umdot]+C*qdot-N'*[wF0;wFm(:)];
taum_floating_HC=tau_floating_HC(7:end);

%Inverse Dynamics Floating Base
[taum_floating_ID,u0dot_floating_ID] = Floating_ID(wF0,wFm,Mm_tilde,H0,t0,tm,P0,pm,I0,Im,Bij,Bi0,u0,um,umdot,robot);

%Test
test=abs(taum_floating_HC-taum_floating_ID)<1e-6;
assert(all(test(:)));
test=abs(u0dot_floating_HC-u0dot_floating_ID)<1e-6;
assert(all(test(:)));

end


