% Alle Matlab-Funktionen dieser Toolbox kompilieren

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-06
% (C) Institut für mechatronische Systeme, Universität Hannover

clear
clc

tb_path = fileparts(which('robotics_toolbox_path_init.m'));

pp = split(path(),':');
for p = pp'
  if ispc()
    % Unter Windows wird das Laufwerk hintenangestellt (mit ";C"),
    % in der Absoluten Pfadangabe wird das Laufwerk am Anfang geschrieben ("C:"}
    p_abs = sprintf('%s:%s', p{1}(end), p{1}(1:end-2)); % korrigiere Pfadvariable
  else
    % Normales Betriebssystem, kein Windows. Keine Korrektur notwendig.
    p_abs = p{1};
  end
  % Kompiliere nur Unterordner dieser Toolbox, die auch im Pfad sind.
  if contains(p_abs, tb_path)
    mex_all_matlabfcn_in_dir(p_abs, 1);
  end
  % Jetzt kommt nur noch das Matlab-Stammverzeichnis (steht immer am Ende
  % des Pfades
  if contains(p_abs, matlabroot)
    break;
  end
end
fprintf('Alle m-Funktionen erfolgreich kompiliert\n');