% Alle Matlab-Funktionen dieser Toolbox kompilieren

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-06
% (C) Institut für mechatronische Systeme, Universität Hannover

clear
clc

tb_path = fileparts(which('robotics_toolbox_path_init.m'));
pp = split(path(),':');
for p = pp'
  % Kompiliere nur Unterordner dieser Toolbox, die auch im Pfad sind.
  if contains(p, tb_path)
    mex_all_matlabfcn_in_dir(p{1}, 1);
  end
  % Jetzt kommt nur noch das Matlab-Stammverzeichnis
  if contains(p, matlabroot)
    break;
  end
end
fprintf('Alle m-Funktionen erfolgreich kompiliert\n');