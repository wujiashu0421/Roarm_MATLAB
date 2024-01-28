%Testing that the results of a double arm robot are the same when we 
%include two fixed joints (zero size and zero mass/inertia).

%--- Load robot models ---%
filename='Test_URDF/ManiSat_2Arm_2_2.urdf';
[robot1,~] = Load_SerialURDFRobot(filename);

filename='Test_URDF/ManiSat_2Arm_2_2_Fixed.urdf';
[robot2,Variables] = Load_SerialURDFRobot(filename);

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
wFm2=Variables.wFm;
wFm2(:,[1,4])=0;
wFm1=wFm2;
wFm1(:,[1,4])=[];

%Joint torques
tauq0=Variables.tauq0;
tauqm=Variables.tauqm;

%Accelerations
u0dot=Variables.u0dot;
umdot=Variables.um;

%--- Kinematics ---%
%No fixed joints
[RJ1,RL1,rJ1,rL1,e1,g1]=Kinematics(R0,r0,qm,robot1);
%Fixed joints
[RJ2,RL2,rJ2,rL2,e2,g2]=Kinematics(R0,r0,qm,robot2);

%--- Differential Kinematics ---%
%No fixed joints
[Bij1,Bi01,P01,pm1]=DiffKinematics(R0,r0,rL1,e1,g1,robot1);
[t01,tm1]=Velocities(Bij1,Bi01,P01,pm1,u0,um,robot1);
%Fixed joints
[Bij2,Bi02,P02,pm2]=DiffKinematics(R0,r0,rL2,e2,g2,robot2);
[t02,tm2]=Velocities(Bij2,Bi02,P02,pm2,u0,um,robot2);

%--- Inertias ---%
%No fixed joints
[I01,Im1]=I_I(R0,RL1,robot1);
%Fixed joints
[I02,Im2]=I_I(R0,RL2,robot2);

%--- Generalized Inertia matrix ---%
%No fixed joints
[M0_tilde1,Mm_tilde1]=MCB(I01,Im1,Bij1,Bi01,robot1);
[H01, H0m1, Hm1] = GIM(M0_tilde1,Mm_tilde1,Bij1,Bi01,P01,pm1,robot1);
%Fixed joints
[M0_tilde2,Mm_tilde2]=MCB(I02,Im2,Bij2,Bi02,robot2);
[H02, H0m2, Hm2] = GIM(M0_tilde2,Mm_tilde2,Bij2,Bi02,P02,pm2,robot2);

%--- Generalized Convective Inertia matrix ---%
%No fixed joints
[C01, C0m1, Cm01, Cm1] = CIM(t01,tm1,I01,Im1,M0_tilde1,Mm_tilde1,Bij1,Bi01,P01,pm1,robot1);
%Fixed joints
[C02, C0m2, Cm02, Cm2] = CIM(t02,tm2,I02,Im2,M0_tilde2,Mm_tilde2,Bij2,Bi02,P02,pm2,robot2);

%--- Natural Orthogonal Complement Matrix ---%
%No fixed joints
N1 = NOC(r0,rL1,P01,pm1,robot1);
%Fixed joints
N2 = NOC(r0,rL2,P02,pm2,robot2);
I=[1:6*7];
I_F1=(2-1)*6+[1:6];
I_F2=(5-1)*6+[1:6];
I([I_F1,I_F2])=[];
N2=N2(I,:);

%--- Accelerations ---%
%No fixed joints
[t0dot1,tmdot1]=Accelerations(t01,tm1,P01,pm1,Bi01,Bij1,u0,um,u0dot,umdot,robot1);
%Fixed joints
[t0dot2,tmdot2]=Accelerations(t02,tm2,P02,pm2,Bi02,Bij2,u0,um,u0dot,umdot,robot2);

%--- Inverse dynamics ---%
%No fixed joints
[tau01,tauqm1] = ID(wF0,wFm1,t01,tm1,t0dot1,tmdot1,P01,pm1,I01,Im1,Bij1,Bi01,robot1);
%Fixed joints
[tau02,tauqm2] = ID(wF0,wFm2,t02,tm2,t0dot2,tmdot2,P02,pm2,I02,Im2,Bij2,Bi02,robot2);

%--- Forward Dynamics ---%
%No fixed joints
[u0dot_FD1,umdot_FD1] = FD(tauq0,tauqm,wF0,wFm1,t01,tm1,P01,pm1,I01,Im1,Bij1,Bi01,u0,um,robot1);
%Fixed joints
[u0dot_FD2,umdot_FD2] = FD(tauq0,tauqm,wF0,wFm2,t02,tm2,P02,pm2,I02,Im2,Bij2,Bi02,u0,um,robot2);


%% === Tests ===%

%% Kinematics tests
assert(isequal(RJ1,RJ2(:,:,[2,3,5,6])));
assert(isequal(RL1,RL2(:,:,[2,3,5,6])));
assert(isequal(rJ1,rJ2(:,[2,3,5,6])));
assert(isequal(rJ1,rJ2(:,[2,3,5,6])));
assert(isequal(e1,e2(:,[2,3,5,6])));
assert(isequal(g1,g2(:,[2,3,5,6])));
%% Differential kinematics tests
assert(isequal(t01,t02));
assert(isequal(tm1,tm2(:,[2,3,5,6])));
%% Inertia tests
assert(isequal(I01,I02));
assert(isequal(Im1,Im2(:,:,[2,3,5,6])));
%% Generalized Inertia tests
assert(isequal(H01,H02));
assert(isequal(H0m1,H0m2));
assert(isequal(Hm1,Hm2));
%% Generalized Convective Inertia tests
assert(isequal(C01,C02));
assert(isequal(C0m1,C0m2));
assert(isequal(Cm1,Cm2));
assert(isequal(Cm01,Cm02));
%% NOC tests
assert(isequal(N1,N2));
%% Accelerations tests
assert(isequal(t0dot1,t0dot2));
assert(isequal(tmdot1,tmdot2(:,[2,3,5,6])));
%% Inverse dynamics tests
assert(isequal(tau01,tau02));
assert(isequal(tauqm1,tauqm2));
%% Forward dynamics tests
assert(isequal(u0dot_FD1,u0dot_FD2));
assert(isequal(umdot_FD1,umdot_FD2));


