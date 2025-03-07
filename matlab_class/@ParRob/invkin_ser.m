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
% Q0 [N x N_init]
%   Anfangs-Gelenkwinkel für Algorithmus (werden nacheinander ausprobiert)
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% s_ser_in
%   Struktur mit Eingabedaten. Felder, siehe Quelltext. Identisch mit
%   Feldern aus SerRob/invkin2.m
% s_par_in
%   Eingabedaten spezifisch für parallele Anordnung. Felder s.u.
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
%   * Plattform-KS
%   * EE-KS (damit Kollisionskörper zugeordnet werden können)
% Stats
%   Struktur mit Detail-Ergebnissen für den Verlauf der Berechnung. Felder:
%   .condJ [N x NLEG*2]
%     Kondititionszahl im Verlauf der IK-Berechnung für alle Beinketten.
%     Zuerst IK-Jacobi (erste Spalten), dann geom. Jacobi (letzte Spalten)
%
% Siehe auch: SerRob/invkin2.m
% Identische Implementierung mit Funktionsaufruf: ParRob/invkin2.m
%
% TODO: Wenn die IK für die Beinkette funktioniert, aber dies keiner
% gültigen IK für die PKM entspricht, wird ein Fehler aufgeworfen.
% Das müsste noch systematischer abgefangen werden.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [q, Phi, Tc_stack_PKM, Stats] = invkin_ser(Rob, xE_soll, Q0, s_ser_in, s_par_in)

%% Initialisierung
assert(isreal(xE_soll) && all(size(xE_soll) == [6 1]), ...
  'ParRob/invkin_ser: xE_soll muss 6x1 sein');
assert(isreal(Q0) && all(size(Q0,1) == Rob.NJ), ...
  'ParRob/invkin_ser: Q0 muss %dxn sein', Rob.NJ);
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
  'I1constr_red', Rob.I1constr_red, ... % Durch Überschreiben temporär ...
  'I2constr_red', Rob.I2constr_red, ... % ... auch anderer IK-Modus möglich
  'debug', false, ...
  'EE_reference', true, ... % Bezug der IK auf den PKM-Endeffektor und nicht den Koppelpunkt
  'platform_frame', false, ... % Eingabe x ist nicht auf EE-KS, sondern auf Plattform-KS bezogen
  'prefer_q0_over_first_legchain', false, ... % Reihenfolge der Anfangswerte
  'abort_firstlegerror', false); % Abbruch, wenn IK der ersten Beinkette falsch
% Prüfe Felder der Einstellungs-Struktur und setze Standard-Werte, falls
% Eingabe nicht gesetzt. Nehme nur Felder, die vorgesehen sind, damit keine
% Fehler aufgeworfen werden wg zu vieler Felder (Standard-Werte in SerRob/invkin2)
if nargin < 4 || isempty(s_ser_in)
  % Keine Einstellungen übergeben. Standard-Einstellungen
  s = s_ser_std;
else
  s = s_ser_in;
  if isfield(s, 'wn') % Keine Nullraumbewegung sinnvoll (beträfe eh nur ...
    s = rmfield(s, 'wn'); % ... die Führungs-Beinkette)
  end
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
Tc_stack_PKM = NaN((Rob.NL+Rob.NLEG+1)*3,4); % siehe fkine_legs; dort aber leicht anders
% Basis-KS. Trägt keine Information. Dient nur zum einfacheren Zugriff auf
% die Variable und zur Angleichung an Darstellung im Welt-KS.
Tc_stack_PKM(1:3,1:4) = eye(3,4); % Basis-KS im Basis-KS.
out3_ind1 = 3; % Zeilenzähler für obige Variable (drei Zeilen stehen schon)

if nargout == 4
  Stats = struct('Q', NaN(1+s.n_max, Rob.NJ), 'PHI', NaN(1+s.n_max, Rob.I2constr(end)), ...
    'iter', zeros(1,Rob.NLEG), 'retry_number', zeros(1,Rob.NLEG), ...
    'condJ', NaN(1+s.n_max,2*Rob.NLEG), 'lambda', NaN(s.n_max,2*Rob.NLEG));
end
%% Berechnung der Beinketten-IK
% Ansatz: IK der Beinkette zum Endeffektor der PKM
Phi = NaN(s_par.I2constr_red(end),1);
q = Q0(:,1);
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
  I_nan = any( isnan(Q0(Rob.I1J_LEG(i):Rob.I2J_LEG(i),:)) );
  if i == 1 || all(~I_nan)
    Q0_i = Q0(Rob.I1J_LEG(i):Rob.I2J_LEG(i),:);
  else
    % Nehme die Startwerte für die IK der weiteren Beinkette aus den Er-
    % gebnissen der ersten. Dadurch hoffentlich symmetrisches Ergebnis.
    % Zusätzlich danach alle gegebenen Anfangswerte prüfen (sofern nicht NaN).
    if s_par.prefer_q0_over_first_legchain % Bevorzuge die Q0-Werte
      Q0_i = [Q0(Rob.I1J_LEG(i):Rob.I2J_LEG(i),~I_nan), q(Rob.I1J_LEG(1):Rob.I2J_LEG(1))];
    else % Bevorzuge die Ergebnisse der ersten Beinkette
      Q0_i = [q(Rob.I1J_LEG(1):Rob.I2J_LEG(1)),Q0(Rob.I1J_LEG(i):Rob.I2J_LEG(i),~I_nan)];
    end
  end
  
  % Transformation vom letzten Beinketten-KS zum EE-KS der PKM bestimmen
  % Dafür wird die IK berechnet und dieser Wert wird übergeben
  r_P_P_Bi = r_P_P_B_ges(:,i); % Vektor von Koppelpunkt zum Plattform-KS
  T_P_Bi = [eulxyz2r(Rob.phi_P_B_all(:,i)), r_P_P_Bi; [0 0 0 1]];% (P)^T_(Bi)
  T_0i_0 = Rob.Leg(i).T_0_W;
  if s_par.EE_reference
    % Bezug der IK auf den PKM-Endeffektor. Erlaubt bei 3T2R-Koordinaten
    % die Aufgabenredundanz bezüglich der ersten Beinkette
    T_0i_E = T_0i_0 * T_0_E; % Transformation vom Basis-KS der Beinkette zum EE
    s.T_N_E = Rob.Leg(i).T_N_E * invtr(T_P_Bi) * T_P_E; % Anpassung der EE-Transformation der Beinkette für IK
  else
    % Bezug auf Beinketten-Koppelpunkt. Erlaubt bei 3T0R-Koordinaten die
    % reine Erreichung der Koppelpunkt-Position bei willkürlicher Rotation.
    T_0i_E = T_0i_0 * T_0_E * invtr(T_P_E) * T_P_Bi;
    s.T_N_E = Rob.Leg(i).T_N_E;
  end
  if isfield(s, 'K') && length(s.K) ~= Rob.Leg(i).NJ % passt für Eingabe der K mit Dimension [RP.NJ,1]
    s.K = K(Rob.I1J_LEG(i):Rob.I2J_LEG(i));
  end
  if isfield(s, 'Kn') && length(s.Kn) ~= Rob.Leg(i).NJ % passt für Eingabe der Kn mit Dimension [RP.NJ,1]
    s.Kn = Kn(Rob.I1J_LEG(i):Rob.I2J_LEG(i));
  end
  % Inverse Kinematik für die serielle Beinkette berechnen
  if nargout < 4
    [q_i, Phi_i, Tc_stack_0i] = Rob.Leg(i).invkin2(T_0i_E(1:3,:), Q0_i, s); % Aufruf der kompilierten IK-Funktion als Einzeldatei
  else % Aufruf und Verarbeitung der Statistik
    [q_i, Phi_i, Tc_stack_0i, Stats_i] = Rob.Leg(i).invkin2(T_0i_E(1:3,:), Q0_i, s);
    Stats.Q(:,Rob.I1J_LEG(i):Rob.I2J_LEG(i)) = Stats_i.Q;
    Stats.PHI(:,Rob.I1constr(i):Rob.I2constr(i)) = Stats_i.PHI;
    Stats.iter(i)= Stats_i.iter;
    Stats.retry_number(i) = Stats_i.retry_number;
    Stats.condJ(:,[i,i+Rob.NLEG]) = Stats_i.condJ(:,1:2);
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
  Phi(s_par.I1constr_red(i):s_par.I2constr_red(i)) = Phi_i;
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
      any(abs(Phi_i) > max([s.Phit_tol;s.Phir_tol])))
    break; % Erste Beinkette funktioniert nicht. Restliche sind egal. Abbruch
  end
end
% Plattform-KS eintragen
T_0_P = T_0_E * invtr(T_P_E);
Tc_stack_PKM(end-5:end-3,:) = T_0_P(1:3,:);
% EE-KS eintragen (Soll)
Tc_stack_PKM(end-2:end,:) = T_0_E(1:3,:);
if ~s_par.debug
  return
end
%% Kinematische Zwangsbedingungen nochmal neu für die PKM bestimmen
% Diese Rechnung ist zu zeitaufwändig und muss im Bedarfsfall manuell
% durchgeführt werden.
% Falls aktiviert: Anpassung der IK-Toleranzen eventuell erforderlich.
if all(s_par.I_EE_Task == logical([1 1 1 1 1 0]))
  if(length(q) == 25) % sym
    Phi_test = Rob.constr2(q, xE_soll, s_par.platform_frame);
  else  % asym mit genau einer Führungskette
    Phi_test = Rob.constr3(q, xE_soll, s_par.platform_frame);
  end
else
  if Rob.I_EE_Task(6) == Rob.I_EE(6) % normaler Fall mit vollen FG
    [~,Phi_test] = Rob.constr1(q, xE_soll, s_par.platform_frame);
  else % Aufgabenredundanz. Nehme um redundanten FG reduzierte ZB
    Phi_test = Rob.constr3(q, xE_soll, s_par.platform_frame);
  end
end
% Probe: Stimmen die Zwangsbedingungen?
if all(abs(Phi) < 1e-7) && any(abs(Phi_test)>1e-6)
  warning(['Fehler: ZB stimmen nicht überein. invkin_ser: %1.2e. ', ...
    'constr: %1.2e. Wahrscheinlichste Ursache: EE-KS der Beinkette ', ...
    'ist falsch gedreht.'], max(abs(Phi)), max(abs(Phi_test)));
end
