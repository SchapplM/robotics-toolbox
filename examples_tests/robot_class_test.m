% Teste Roboterklasse

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-11
% (C) Institut für mechatronische Systeme, Universität Hannover

clc
clear
close all

this_repo_path = fullfile(fileparts(which('robotics_toolbox_path_init.m')));
addpath(fullfile(this_repo_path, 'examples_tests', 'SerRob'));

%% Serielle Roboter
SerRob_class_example_SCARA
SerRob_class_example_Palettierer