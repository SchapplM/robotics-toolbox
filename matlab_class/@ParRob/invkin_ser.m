% Inverse Kinematik für allgemeinen parallelen Roboter
% Aufruf der Funktion für serielle Roboter. Dadurch Berechnung der IK
% für alle Beine unabhangig und nicht gleichzeitig.
%
% Vorteile: Schnellere Berechnung durch Nutzung kompilierter Funktionen
% Nachteil: Keine Nullraumoptimierung bei 3T2R
%
% Eingabe:
% xE_soll [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS (Soll)
% q0 [Nx1]
%   Startkonfiguration: Alle Gelenkwinkel aller serieller Beinketten der PKM
% s
%   Struktur mit Eingabedaten. Felder, siehe Quelltext. Identisch mit
%   Feldern aus SerRob/invkin.m
%
% Ausgabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM als Lösung der IK
% Phi
%   Erfüllung der kinematischen Zwangsbedingungen in der
%   Gelenkkonfiguration q. Ist bei Erfolg ein Null-Vektor
% Tc_stack_PKM
%   Gestapelte Transformationsmatrizen der PKM (ohne 0001-Zeile). Im
%   Basis-KS. Entspricht mit Abwandlung der Anordnung wie in fkine: 
%   * PKM-Basis
%   * Für jede Beinkette: Basis und alle bewegten Körper-KS. Ohne
%     virtuelles EE-KS
%   * Kein Plattform-KS
% Stats
%   Struktur mit Detail-Ergebnissen für den Verlauf der Berechnung
%
% Siehe auch: SerRob/invkin.m
%
% TODO: Wenn die IK für die Beinkette funktioniert, aber dies keiner
% gültigen IK für die PKM entspricht, wird ein Fehler aufgeworfen.
% Das müsste noch systematischer abgefangen werden.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [q, Phi, Tc_stack_PKM, Stats] = invkin_ser(Rob, xE_soll, q0, s_ser_in, s_par_in)

%% Initialisierung
assert(isreal(xE_soll) && all(size(xE_soll) == [6 1]), ...
  'ParRob/invkin_ser: xE_soll muss 6x1 sein');
assert(isreal(q0) && all(size(q0) == [Rob.NJ 1]), ...
  'ParRob/invkin_ser: q0 muss %dx1 sein', Rob.NJ);
s_ser_std = struct( ...
  'n_min', 0, ... % Minimale Anzahl Iterationen
  'n_max', 1000, ... % Maximale Anzahl Iterationen
  'Phit_tol', 1e-8, ... % Toleranz für translatorischen Fehler
  'Phir_tol', 1e-8, ... % Toleranz für rotatorischen Fehler
  'reci', false, ... % Keine reziproken Winkel für ZB-Def.
  'retry_limit', 100); % Anzahl der Neuversuche
s_par = struct( ...
  'Leg_I_EE_Task', cat(1,Rob.Leg.I_EE_Task), ...
  'I_EE_Task', Rob.I_EE_Task, ...
  'platform_frame', false, ... % Eingabe x ist nicht auf EE-KS, sondern auf Plattform-KS bezogen
  'abort_firstlegerror', false); % Abbruch, wenn IK der ersten Beinkette falsch
% Prüfe Felder der Einstellungs-Struktur und setze Standard-Werte, falls
% Eingabe nicht gesetzt. Nehme nur Felder, die vorgesehen sind, damit keine
% Fehler aufgeworfen werden wg zu vieler Felder (Standard-Werte in SerRob/invkin2)
if nargin < 4 || isempty(s_ser_in)
  % Keine Einstellungen übergeben. Standard-Einstellungen
  s = s_ser_std;
else
  s = s_ser_in;
  for f = fields(s_ser_std)'
    if ~isfield(s, f{1}) % Feld Fehlt in Eingabe. Nehme aus Standard-Einstellungen
      s.(f{1}) = s_ser_std.(f{1});
    end
  end
end
if nargin == 5
  for f = fields(s_par_in)'
    if isfield(s_par, f{1}) % Feld Fehlt in Eingabe. Nehme aus Standard-Einstellungen
      s_par.(f{1}) = s_par_in.(f{1});
    else
      error('Feld %s aus s_par_in kann nicht übergeben werden', f{1});
    end
  end
end
if Rob.I_EE(6) && ~s_par.I_EE_Task(6) % Aufgabenredundanz mit z-Rotation
  s.reci = true; % bei z.B. 3T2R geht es nicht ohne reziproke Winkel (aber auch 3T1R/2T1R mit freier Rotation)
end

if isfield(s, 'K')
  K = s.K;
end
if isfield(s, 'Kn')
  Kn = s.Kn;
end

% Zählung in Rob.NL: Starrkörper der Beinketten, Gestell und Plattform. 
% Hier werden nur die Basis-KS der Beinketten und alle bewegten Körper-KS
% der Beine angegeben.
Tc_stack_PKM = NaN((Rob.NL-1+Rob.NLEG)*3,4); % siehe fkine_legs; dort aber leicht anders
% Basis-KS. Trägt keine Information. Dient nur zum einfacheren Zugriff auf
% die Variable und zur Angleichung an Darstellung im Welt-KS.
Tc_stack_PKM(1:3,1:4) = eye(3,4); % Basis-KS im Basis-KS.
out3_ind1 = 3; % Zeilenzähler für obige Variable (drei Zeilen stehen schon)

if nargout == 4
  Stats = struct('Q', NaN(1+s.n_max, Rob.NJ), 'PHI', NaN(1+s.n_max, Rob.I2constr_red(end)), ...
    'iter', repmat(s.n_max,1,Rob.NLEG), 'retry_number', zeros(1,Rob.NLEG), ...
    'condJ', NaN(1+s.n_max,Rob.NLEG), 'lambda', NaN(s.n_max,2*Rob.NLEG));
end
%% Berechnung der Beinketten-IK
% Ansatz: IK der Beinkette zum Endeffektor der PKM
Phi_ser = NaN(Rob.I2constr_red(end),1);
Phi = Phi_ser;
q = q0;
r_P_P_B_ges = Rob.r_P_B_all;
if ~s_par.platform_frame
  T_P_E = Rob.T_P_E;
else
  T_P_E = eye(4);
end
T_0_E = Rob.x2t(xE_soll);
% IK für alle Beine einzeln berechnen
for i = 1:Rob.NLEG
  % Überschreibe evtl übergebene I_EE für PKM. Das I_EE hier gilt für die Beinkette
  % Damit wird der Standardwert gesetzt, der sowieso in SerRob/invkin2
  % genutzt wird
  s.I_EE = s_par.Leg_I_EE_Task(i,:);

  % Initialisierung: Startwerte der IK
  if i == 1 || all(~isnan(q0(Rob.I1J_LEG(i):Rob.I2J_LEG(i))))
    q0_i = q0(Rob.I1J_LEG(i):Rob.I2J_LEG(i));
  else
    % Nehme die Startwerte für die IK der weiteren Beinkette aus den Er-
    % gebnissen der ersten. Dadurch hoffentlich symmetrisches Ergebnis.
    q0_i = q(Rob.I1J_LEG(1):Rob.I2J_LEG(1));
  end
  
  % Transformation vom letzten Beinketten-KS zum EE-KS der PKM bestimmen
  % Dafür wird die IK berechnet und dieser Wert wird übergeben
  r_P_P_Bi = r_P_P_B_ges(:,i); % Vektor von Koppelpunkt zum Plattform-KS
  T_P_Bi = [eulxyz2r(Rob.phi_P_B_all(:,i)), r_P_P_Bi; [0 0 0 1]];% (P)^T_(Bi)
  T_0i_0 = Rob.Leg(i).T_0_W;
  T_0i_E = T_0i_0 * T_0_E; % Transformation vom Basis-KS der Beinkette zum EE
  s.T_N_E = Rob.Leg(i).T_N_E * invtr(T_P_Bi) * T_P_E; % Anpassung der EE-Transformation der Beinkette für IK
  
  if isfield(s, 'K') && length(s.K) ~= Rob.Leg(i).NJ % passt für Eingabe der K mit Dimension [RP.NJ,1]
    s.K = K(Rob.I1J_LEG(i):Rob.I2J_LEG(i));
  end
  if isfield(s, 'Kn') && length(s.Kn) ~= Rob.Leg(i).NJ % passt für Eingabe der Kn mit Dimension [RP.NJ,1]
    s.Kn = Kn(Rob.I1J_LEG(i):Rob.I2J_LEG(i));
  end
  % Inverse Kinematik für die serielle Beinkette berechnen
  if nargout < 4
    [q_i, Phi_i, Tc_stack_0i] = Rob.Leg(i).invkin2(T_0i_E(1:3,:), q0_i, s); % Aufruf der kompilierten IK-Funktion als Einzeldatei
  else % Aufruf und Verarbeitung der Statistik
    [q_i, Phi_i, Tc_stack_0i, Stats_i] = Rob.Leg(i).invkin2(T_0i_E(1:3,:), q0_i, s);
    Stats.Q(:,Rob.I1J_LEG(i):Rob.I2J_LEG(i)) = Stats_i.Q;
    Stats.PHI(:,Rob.I1constr(i):Rob.I2constr(i)) = Stats_i.PHI;
    Stats.iter(i)= Stats_i.iter;
    Stats.retry_number(i) = Stats_i.retry_number;
    Stats.condJ(:,i) = Stats_i.condJ;
    Stats.lambda(:,(i-1)*2+1:2*i) = Stats_i.lambda;
  end
  if i == 1 && (Rob.I_EE(6) && ~s_par.I_EE_Task(6) || ... % Letzter FG für Aufgabe nicht gesetzt
      all(Rob.I_EE == [1 1 1 1 1 0])) % Roboter hat strukturell 3T2R FG und constr3-Methode.
    if any(isnan(Phi_i))
      % Führungsbeinkette konvergiert nicht. Keine weitere Berechnung möglich
      return
    end
    % Aufgabenredundanz des Rotations-FG (3T2R, 2T0*R/3T0*R und Führungskette. 
    % Die erste Beinkette gibt die EE-Ori für die anderen Beinketten vor.
    [~,T_0_Bi] = Rob.Leg(i).fkineEE(q_i);
    % Aktualisiere die EE-Transformation auf die resultierend aus Bein 1
    T_0_E = T_0_Bi * invtr(T_P_Bi) * T_P_E;
  end
  % Ergebnisse für dieses Bein abspeichern
  q(Rob.I1J_LEG(i):Rob.I2J_LEG(i)) = q_i;
  Phi_ser(Rob.I1constr_red(i):Rob.I2constr_red(i)) = Phi_i;
  if nargout >= 3
    T_0_0i = Rob.Leg(i).T_W_0;
    % Umrechnung auf PKM-Basis-KS. Nehme nur die KS, die auch einem Körper
    % zugeordnet sind. In Tc_stack_0i bei hybriden Systemen teilw. mehr. 
    Tc_stack_0 = NaN(3*Rob.Leg(i).NL,4);
    for kk = 1:Rob.Leg(i).NL
      Tr_0i_kk = Tc_stack_0i((kk-1)*3+1:kk*3,1:4);
      T_0_kk = T_0_0i * [Tr_0i_kk;[0 0 0 1]];
      Tc_stack_0((kk-1)*3+1:kk*3,1:4) = T_0_kk(1:3,:);
    end
    % Eintragen in Ergebnis-Variable
    Tc_stack_PKM(out3_ind1+(1:3*Rob.Leg(i).NL),:) = Tc_stack_0;
    out3_ind1 = out3_ind1 + 3*Rob.Leg(i).NL;
  end
  if s_par.abort_firstlegerror && (any(isnan(Phi_i)) || ...
      any(Phi_i > max([s.Phit_tol;s.Phir_tol])))
    break; % Erste Beinkette funktioniert nicht. Restliche sind egal. Abbruch
  end
end
Phi = Phi_ser;
return
%% Kinematische Zwangsbedingungen nochmal neu für die PKM bestimmen
% Diese Rechnung ist zu zeitaufwändig und muss im Bedarfsfall manuell
% durchgeführt werden.
% Falls aktiviert: Anpassung der IK-Toleranzen eventuell erforderlich.
if all(s_par.I_EE_Task == logical([1 1 1 1 1 0]))
    if(length(q) == 25) % sym
        Phi = Rob.constr2(q, xE_soll, s_par.platform_frame);
    else  % asym mit genau einer Führungskette
        Phi = Rob.constr3(q, xE_soll, s_par.platform_frame);
    end
else
  Phi = Rob.constr1(q, xE_soll, s_par.platform_frame);
end
% Probe: Stimmen die Zwangsbedingungen?
if all(abs(Phi_ser) < 1e-7) && any(abs(Phi)>1e-6)
  warning('Fehler: ZB stimmen nicht überein. Wahrscheinlichste Ursache: EE-KS der Beinkette ist falsch gedreht.');
end