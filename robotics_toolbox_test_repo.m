% Gesamttest für die Matlab-Robotik-Toolbox
% 
% Führt alle verfügbaren Modultests aus um die Funktionalität sicherzustellen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-03
% (C) Institut für mechatronische Systeme, Universität Hannover

this_repo_path = fileparts(which('robotics_toolbox_path_init.m'));
% Füge die Testfunktionen temporär zum Pfad hinzu. Normalerweise werden sie
% nicht gebraucht und sind nicht im Pfad (robotics_toolbox_path_init)
addpath(fullfile(this_repo_path, 'examples_tests'));
addpath(fullfile(this_repo_path, 'examples_tests', 'rotation_controllers'));
addpath(fullfile(this_repo_path, 'examples_tests', 'rotation_integration'));
addpath(fullfile(this_repo_path, 'examples_tests', 'contact_models'));

%% Transformationen
angvec_test
quaternion_test
euler_test
rotvec_test

%% Integration von Rotationsdarstellungen
rotation_integration_comparison

%% Dynamik
rigid_body_dynamics_test

%% Orientierungsregler
rotation_controllers_energy_test_start
rotation_controllers_test_start

%% Kontaktmodelle
HuntCrossley_LuGre_1DOF_example
sim('example_hunt_crossley_bouncing_ball', 'StopTime', '10', 'SimulationMode', 'normal');

%% Alle Funktionen kompilieren
robotics_toolbox_compile_test;

%% Ende
clc
close all
fprintf('Alle Testfunktionen dieses Repos ausgeführt\n');