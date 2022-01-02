% Eintragen der Funktions-Handles in die Roboterstruktur durch Verweis auf
% automatisch generierte Matlab-Funktionen, die als Dateien vorliegen
% müssen.
% 
% Eingabe:
% mex
%   Schalter zur Wahl von vorkompilierten Funktionen (schnellere Berechnung)
% compile_missing
%   Schalter zur Starten eines Kompilierversuches für fehlende Funktionen
% 
% Siehe auch: SerRob/fill_fcn_handles.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-12
% (C) Institut für Mechatronische Systeme, Universität Hannover

function files_missing = fill_fcn_handles(R, mex, compile_missing)
if nargin < 2
  mex = false;
end
if nargin < 3
  compile_missing = false;
end
files_missing = {};

mdlname = R.mdlname;
% Prüfe, ob Modellname dem Schema aus der PKM-Bibliothek entspricht
% Wenn ja, dann passe Funktionsnamen an Eigenschaften dort an
expression = 'P(\d)([RP]+)(\d+)[V]?(\d*)[G]?(\d*)[P]?(\d*)A(\d+)'; % "P3RRR1G1P1A1"/"P3RRR1V1G1P1A1"
[tokens, ~] = regexp(mdlname,expression,'tokens','match');
if isempty(tokens)
  parrob = false;
else
  parrob = true;
  res = tokens{1};
  if isempty(res{4}) % serielle Kette ist keine abgeleitete Variante
    PName_Legs = ['P', res{1}, res{2}, res{3}];
  else % serielle Kette ist eine Variante abgeleitet aus Hauptmodell
    PName_Legs = ['P', res{1}, res{2}, res{3}, 'V', res{4}];
  end
  % Modellvariante ohne Aktuierung, unter dem die Dynamik-Funktionen
  % gespeichert sind. Die Plattform-Nummer wird hier nicht beachtet.
  % Es können verschiedene allgemeine Modelle passen, da die
  % Gestell-Konfigurationen relativ ähnlich sind. Suche passendes.
  % Dadurch wird doppelter Code in der Datenbank vermieden.
  switch str2double(res{5}) % siehe align_base_coupling.m
    case 1
      basecoupling_equiv = [5,4,8];
    case 2
      basecoupling_equiv = [6,4,8];
    case 3
      basecoupling_equiv = [7,4,8];
    case 4
      basecoupling_equiv = 8;
    case 5
      basecoupling_equiv = [1 4 8];
    case 6
      basecoupling_equiv = [2 4 8];
    case 7
      basecoupling_equiv = [3 4 8];
    case 8
      basecoupling_equiv = 4;
    case 9
      basecoupling_equiv = [4,8];
    case 10
      % Beide Methoden haben senkrechte Gestellgelenke
      basecoupling_equiv = 1; % TODO: Prüfen, ob das stimmt
    otherwise
      error('Fall %s nicht definiert', res{5});
  end
  % Code für die eigentlich gesuchte Darstellung vorne und hinten anstellen
  % (zuerst suchen und einstellen, wenn nichts gefunden wurde).
  basecoupling_equiv = [str2double(res{5}), basecoupling_equiv, str2double(res{5})];
  for k = basecoupling_equiv
    % Suche in den Ordnern nach Funktionen
    mdlname_A0 = [PName_Legs, sprintf('G%d',k), 'A0'];
    if~isempty(which([mdlname_A0,'_gravload_para_pf_slag_vp1']))
      % Dieses Modell existiert (da eine beispielhafte Funktion gefunden
      % wurde).
      break;
    end
  end
end

% Liste von Funktionen, die nicht für den allgemeinen Fall "A0" benutzt
% werden, sondern abhängig von der spezifischen Aktuierung "A1","A2",...
% sind
Ai_list = {'Jinv'};

for i = 1:length(R.all_fcn_hdl)
  % das erste Feld von ca ist der Name des Funktionshandles in der Klasse,
  % die folgenden Felder sind die Matlab-Funktionsnamen, die von der
  % Dynamik-Toolbox generiert werden.
  ca = R.all_fcn_hdl{i};
  hdlname = ca{1};
  missing_i = true;
  for j = 2:length(ca) % Gehe alle möglichen Funktionsdateien durch
    fcnname_tmp = ca{j};
    % Bestimme den Roboternamen-Präfix für jede Funktion separat
    if ~parrob || any(strcmp(Ai_list, ca{j}))
      mdlname_j = mdlname;
    else
      mdlname_j = mdlname_A0;
    end
    if strcmp(cell2str(ca(2)),'fkineEE_traj') || ...
       strcmp(cell2str(ca(2)),'fkine_coll') || ...
       contains(cell2str(ca(2)), 'invkin') ||...
       strcmp(cell2str(ca(2)),'constr4D_traj')
      % Kinematik-Funktionen: Hier ist die G-/P-Nummer egal (wird durch die
      % Parameter abgedeckt). Daher auch nicht im Namen enthalten.
      if parrob % PKM ist aus ParRobLib
        mdlname_j = PName_Legs;
      else % nicht aus ParRobLib
        continue; % es gibt keine IK-Funktionen aus parroblib_create_template_functions
      end
    end
    if mex == 0
      robfcnname = sprintf('%s_%s', mdlname_j, fcnname_tmp);
    else
      robfcnname = sprintf('%s_%s_mex', mdlname_j, fcnname_tmp);
    end
    % Prüfe ob mex-Datei existiert
    if compile_missing && mex && isempty(which(robfcnname))
      robfcnbasename = robfcnname(1:end-4); % Endung "_mex" wieder entfernen
      if ~isempty(which(robfcnbasename))
        % Prüfe, ob passende m-Datei verfügbar ist.
        matlabfcn2mex({robfcnbasename});
      end
    end
    if isempty(which(robfcnname))
      files_missing = {files_missing{:}, robfcnname}; %#ok<CCAT>
    else
      missing_i = false;
    end
    % Speichere das Funktions-Handle in der Roboterklasse
    eval(sprintf('R.%s = @%s;', hdlname, robfcnname));
  end
  R.extfcn_available(i) = ~missing_i;
end

% Stelle auch mex-Funktionen für die Beinketten ein, falls das für die PKM
% explizit gefordert wurde. Der Aufruf ohne Argument soll die Beinketten
% nicht ändern.
if nargin > 1
  for i = 1:R.NLEG
    R.Leg(i).fill_fcn_handles(mex, compile_missing);
  end
end