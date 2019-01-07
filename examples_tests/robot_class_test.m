% Teste Roboterklasse

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-11
% (C) Institut für mechatronische Systeme, Universität Hannover

clc
clear
close all

this_repo_path = fullfile(fileparts(which('robotics_toolbox_path_init.m')));
addpath(fullfile(this_repo_path, 'examples_tests', 'SerRob'));
addpath(fullfile(this_repo_path, 'examples_tests', 'ParRob'));

%% Serielle Roboter
SerRob_class_example_SCARA
SerRob_class_example_Palettierer
SerRob_class_example_LBR

%% Parallele Roboter
ParRob_class_example_3RPR
ParRob_class_example_3RRR
ParRob_class_example_6UPS
ParRob_constraints_test