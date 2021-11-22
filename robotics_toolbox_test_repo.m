% Gesamttest für die Matlab-Robotik-Toolbox
% 
% Führt alle verfügbaren Modultests aus um die Funktionalität sicherzustellen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-03
% (C) Institut für Mechatronische Systeme, Universität Hannover

this_repo_path = fileparts(which('robotics_toolbox_path_init.m'));
% Füge die Testfunktionen temporär zum Pfad hinzu. Normalerweise werden sie
% nicht gebraucht und sind nicht im Pfad (robotics_toolbox_path_init)
addpath(fullfile(this_repo_path, 'examples_tests'));
addpath(fullfile(this_repo_path, 'examples_tests', 'impedance_controller'));
addpath(fullfile(this_repo_path, 'examples_tests', 'rotation_controllers'));
addpath(fullfile(this_repo_path, 'examples_tests', 'rotation_integration'));
addpath(fullfile(this_repo_path, 'examples_tests', 'contact_models'));
addpath(fullfile(this_repo_path, 'examples_tests', 'SerRob'));

%% Transformationen
angvec_test; close all;
quaternion_test; close all;
euler_test; close all;
rotvec_test; close all;
tr_test; close all;
%% Impedanzregler
lbr4p_joint_impctrl_test_start; close all;

%% Integration von Rotationsdarstellungen
rotation_integration_comparison; close all;

%% Dynamik
rigid_body_dynamics_test; close all;

%% Orientierungsregler
rotation_controllers_energy_test_start; close all;
rotation_controllers_test_start; close all;

%% Kontaktmodelle
HuntCrossley_LuGre_1DOF_example; close all;
sim('example_hunt_crossley_bouncing_ball', 'StopTime', '10', 'SimulationMode', 'normal');

%% Matlab-Klasse
robot_class_test; close all;

%% Alle Funktionen kompilieren
robotics_toolbox_compile_test; close all;

%% Ende
clc
close all
fprintf('Alle Testfunktionen dieses Repos ausgeführt\n');