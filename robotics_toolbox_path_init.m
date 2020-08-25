% Pfad-Initialisierung für die Matlab-Robotik-Toolbox
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
addpath(fullfile(this_tb_path, 'transformation'));
addpath(fullfile(this_tb_path, 'matlab_class'));

% Ergebnis-Ordner für Ausgabe der Test-Skripte erstellen
mkdirs(fullfile(this_tb_path, 'examples_tests', 'results'));

% Abhängigkeiten nicht kompilieren; verzögert den Start von Matlab zu sehr
% mex_script_dependencies('ParRob', true);