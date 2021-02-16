% Eintragen der Funktions-Handles in die Roboterstruktur durch Verweis auf
% automatisch generierte Matlab-Funktionen, die als Dateien vorliegen
% müssen.
% 
% Eingabe:
% mex
%   Schalter zur Wahl von vorkompilierten Funktionen (schnellere Berechnung)
% compile_missing
%   Schalter zur Starten eines Kompilierversuches für fehlende Funktionen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function fill_fcn_handles(R, mex, compile_missing)

if nargin < 3
  compile_missing = false;
end

mdlname = R.mdlname;
for i = 1:length(R.all_fcn_hdl)
  % das erste Feld von ca ist der Name des Funktionshandles in der Klasse,
  % die folgenden Felder sind die Matlab-Funktionsnamen, die von der
  % Dynamik-Toolbox generiert werden.
  ca = R.all_fcn_hdl{i};
  hdlname = ca{1};
  for j = 2:length(ca) % Gehe alle möglichen Funktionsdateien durch
    fcnname_tmp = ca{j};
    % Bei einigen Funktionen soll nicht die mex-Funktion gewählt werden.
    fcn_j_nomex = false;
    if strcmp(ca{j}, 'convert_par2_MPV_fixb')
      fcn_j_nomex = true;
    end
    if nargin == 1 || mex == 0 || fcn_j_nomex
      robfcnname = sprintf('%s_%s', mdlname, fcnname_tmp);
    else
      robfcnname = sprintf('%s_%s_mex', mdlname, fcnname_tmp);
    end
    % Prüfe ob mex-Datei existiert
    if compile_missing && mex && isempty(which(robfcnname)) && ~fcn_j_nomex
      robfcnbasename = robfcnname(1:end-4); % Endung "_mex" wieder entfernen
      if ~isempty(which(robfcnbasename))
        % Prüfe, ob passende m-Datei verfügbar ist.
        matlabfcn2mex({robfcnbasename});
      end
    end
    
    % Funktion für die Bestimmung der Gelenkvariablen von hybriden Robotern
    if strcmp(hdlname, 'jointvarfcnhdl') && R.Type == 0
      % hat bei seriellen Robotern keine Bedeutung und wird nicht benutzt
      continue
    end
    
    if ~isempty(which(robfcnname))
      % Speichere das Funktions-Handle in der Roboterklasse
      % Falls keine Datei gefunden wurde, lasse das Handle leer. Es wird so
      % oder so zum Fehler kommen. Bei einem leeren Handle kann die
      % nicht-Existenz der m-Datei besser geprüft werden.
      eval(sprintf('R.%s = @%s;', hdlname, robfcnname));
      break;
    end
  end
end
