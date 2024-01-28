%Testing DH, URDF (one and multiple arms) robots model and 
%kinematic/dynamic functions.
%
%This tests compares the results of different kinematic and dynamic 
%magnitudes when generated with different methods
%
% Velocities -> Dedicated function vs Jacobian
% Inertia Matrices -> Recursive formulation vs NOC-based formulation
% Inverse Dynamics -> Recursive formulation vs Inerta Matrices based
% Forward Dynamics -> Recursive formulation vs Inerta Matrices based


%=== DH Model ===%
%Load robot model
[robot,Variables] = Load_SerialDHRobot();

%% DH Velocity test
Vel_Test(robot,Variables)
%% DH Inertia matrices test
IM_Test(robot,Variables);
%% DH Inverse Dynamics test
ID_Test(robot,Variables);
%% DH Forward Dynamics test
FD_Test(robot,Variables);

%=== URDF Serial Model ===%
%Load robot model
filename='kuka_iiwa.urdf';
[robot,Variables] = Load_SerialURDFRobot(filename);

%% URDF Serial Velocity test
Vel_Test(robot,Variables)
%% URDF Serial Inertia matrices test
IM_Test(robot,Variables);
%% URDF Serial Inverse Dynamics test
ID_Test(robot,Variables);
%% URDF Serial Forward Dynamics test
FD_Test(robot,Variables);

%=== URDF Dual Arm Model with no fixed joints ===%
%Load robot model
filename='Test_URDF/ManiSat_2Arm_2_2.urdf';
[robot,Variables] = Load_SerialURDFRobot(filename);

%% URDF Dual Arm Velocity test
Vel_Test(robot,Variables)
%% URDF Dual Arm Inertia matrices test
IM_Test(robot,Variables);
%% URDF Dual Arm Inverse Dynamics test
ID_Test(robot,Variables);
%% URDF Dual Arm Forward Dynamics test
FD_Test(robot,Variables);

%=== URDF Dual Arm Model with fixed  ===%
%Load robot model
filename='Test_URDF/2_iiwa_base.urdf';
[robot,Variables] = Load_SerialURDFRobot(filename);

%% URDF Dual Arm (Fixed Joints) Velocity test
Vel_Test(robot,Variables)
%% URDF Dual Arm (Fixed Joints) Inertia matrices test
IM_Test(robot,Variables);
%% URDF Dual Arm (Fixed Joints) Inverse Dynamics test
ID_Test(robot,Variables);
%% URDF Dual Arm (Fixed Joints) Forward Dynamics test
FD_Test(robot,Variables);
