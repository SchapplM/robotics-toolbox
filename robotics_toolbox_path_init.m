% Pfad-Initialisierung für die Matlab-Geometrie-Toolbox
% 
% Diese Pfad-Initialisierung muss zeitlich nach der Pfad-Initialisierung
% der Robotics-Toolbox von Peter Corke ausgeführt werden, damit
% gleichnamige Funktionen aus dieser Toolbox geholt werden. Die Funktionen
% von Peter Corke sind teilweise nicht kompatibel mit Simulink.

% Moritz Schappler, schappler@imes.uni-hannover.de, 2018-03
% (C) Institut für mechatronische Systeme, Universität Hannover

this_tb_path = fileparts( mfilename('fullpath') );
addpath(this_tb_path);

addpath(fullfile(this_tb_path, 'contact_model'));
addpath(fullfile(this_tb_path, 'controllers'));
addpath(fullfile(this_tb_path, 'dynamics'));
addpath(fullfile(this_tb_path, 'kinematics'));
addpath(fullfile(this_tb_path, 'mechanics'));
addpath(fullfile(this_tb_path, 'regressor'));
addpath(fullfile(this_tb_path, 'simulink'));
addpath(fullfile(this_tb_path, 'transformation'));