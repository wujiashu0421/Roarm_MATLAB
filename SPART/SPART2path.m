%Adds SPART to the path and save it

%Find path of repository
pathstr = fileparts(mfilename('fullpath'));

%Add all the required folders into the path
addpath(sprintf('%s/src/kinematics_dynamics',pathstr));
addpath(sprintf('%s/src/Simulink_library',pathstr));
addpath(sprintf('%s/src/utils',pathstr));
addpath(sprintf('%s/src/attitude_transformations',pathstr));
addpath(sprintf('%s/src/robot_model',pathstr));
addpath(sprintf('%s/URDF_models',pathstr));

%Save path
savepath;

