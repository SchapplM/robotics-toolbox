% Eintragen der Funktions-Handles in die Roboterstruktur durch Verweis auf
% automatisch generierte Matlab-Funktionen, die als Dateien vorliegen
% müssen.
% 
% Eingabe:
% mex
%   Schalter zur Wahl von vorkompilierten Funktionen (schnellere Berechnung)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für mechatronische Systeme, Universität Hannover

function R = fill_fcn_handles(R, mex)
mdlname = R.mdlname;
for i = 1:length(R.all_fcn_hdl)
  % das erste Feld von ca ist der Name des Funktionshandles in der Klasse,
  % die folgenden Felder sind die Matlab-Funktionsnamen, die von der
  % Dynamik-Toolbox generiert werden.
  ca = R.all_fcn_hdl{i};
  hdlname = ca{1};
  for j = 2:length(ca) % Gehe alle möglichen Funktionsdateien durch
    fcnname_tmp = ca{j};
    if nargin == 1 || mex == 0
      robfcnname = sprintf('%s_%s', mdlname, fcnname_tmp);
    else
      robfcnname = sprintf('%s_%s_mex', mdlname, fcnname_tmp);
    end
    if ~isempty(which(robfcnname)) || j == length(ca)
      % Speichere das Funktions-Handle in der Roboterklasse
      % Falls keine Datei gefunden wurde, nehme das Handle für die letzte
      % Dateioption
      eval(sprintf('R.%s = @%s;', hdlname, robfcnname));
      break;
    end
  end
end