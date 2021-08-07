% Teste Roboterklasse

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-11
% (C) Institut für mechatronische Systeme, Universität Hannover

clc
clear
close all

this_repo_path = fullfile(fileparts(which('robotics_toolbox_path_init.m')));
addpath(fullfile(this_repo_path, 'examples_tests', 'SerRob'));
addpath(fullfile(this_repo_path, 'examples_tests', 'ParRob'));

%% Kompiliere Abhängigkeiten
mex_script_dependencies('ParRob', true);
matlabfcn2mex({'check_collisionset_simplegeom'});

%% Allgemeine Funktionen
ParRob_transformations_test; close all;

%% Serielle Roboter
SerRob_constraints_test; close all;
SerRob_IK_test; close all;
SerRob_class_example_SCARA; close all;
SerRob_class_example_Palettierer; close all;
SerRob_class_example_LBR; close all;
SerRob_class_example_Industrieroboter; close all;
SerRob_joint_stiffness_fdyn_test; close all;
SerRob_benchmark_3T3R_task_redundancy; close all;
SerRob_nullspace_convergence_test; close all;
SerRob_nullspace_collision_avoidance; close all;
%% Parallele Roboter
ParRob_class_example_3RPR; close all;
ParRob_class_example_3RRR; close all;
ParRob_class_example_6UPS; close all;
ParRob_class_example_6RUS_Hexa; close all;
% ParRob_class_example_6UPS_3T2R; close all; % TODO: Nach Aktualisierung des Skripts wieder aktivieren
ParRob_class_example_Delta; close all;
ParRob_class_example_3PRRRR_3T0R; close all;
ParRob_class_example_6PUS; close all;
ParRob_class_example_Gogu_3T1R; close all;
ParRob_class_example_PRRRR_4PRRRRR; close all;
ParRob_class_example_Gogu_3T2R; close all;
ParRob_benchmark_3T3R_task_redundancy; close all;
ParRob_class_example_3T2R_asym; close all;
ParRob_class_example_3T2R_passive_constrleg; close all;
ParRob_class_example_3T2R_sym; close all;
ParRob_cutforce_example; close all;
ParRob_springtorque_example; close all;
ParRob_constraints_test; close all;
ParRob_constraintsD_test; close all;
ParRob_nullspace_convergence_test; close all;
ParRob_nullspace_collision_avoidance; close all;
ParRob_mdltest_invdyn_energy_consistency; close all;
ParRob_mdltest_invdyn_compare_sym_vs_num; close all;
%% Parallele Roboter mit seriell-hybriden Beinketten
ParRob_class_example_hybBKplanar; close all;
ParRob_class_example_hybBKspatial; close all;