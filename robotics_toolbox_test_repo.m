% Gesamttest für die Matlab-Robotik-Toolbox
% 
% Führt alle verfügbaren Modultests aus um die Funktionalität sicherzustellen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-03
% (C) Institut für mechatronische Systeme, Universität Hannover

this_repo_path = fullfile(fileparts(which('robotics_toolbox_path_init.m')));
addpath(fullfile(this_repo_path, 'examples_tests'));

angvec_test
quaternion_test
rpy_test
rotvec_test

rigid_body_dynamics_test

clc
close all
fprintf('Alle Testfunktionen dieses Repos ausgeführt\n');