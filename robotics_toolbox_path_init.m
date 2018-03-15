% Pfad-Initialisierung für die Matlab-Geometrie-Toolbox

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