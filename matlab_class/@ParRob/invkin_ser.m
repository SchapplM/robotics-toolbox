% Inverse Kinematik für allgemeinen parallelen Roboter
% Aufruf der Funktion für serielle Roboter. Dadurch Berechnung der IK
% für alle Beine unabhängig und nicht gleichzeitig.
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
%   Feldern aus SerRob/invkin.md
% 
% Ausgabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM als Lösung der IK
% Phi
%   Erfüllung der kinematischen Zwangsbedingungen in der
%   Gelenkkonfiguration q. Ist bei Erfolg ein Null-Vektor
% 
% Siehe auch: SerRob/invkin.m
% 
% TODO: Wenn die IK für die Beinkette funktioniert, aber dies keiner
% gültigen IK für die PKM entspricht, wird ein Fehler aufgeworfen.
% Das müsste noch systematischer abgefangen werden.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [q, Phi] = invkin_ser(Rob, xE_soll, q0, s_in)

%% Initialisierung
assert(isreal(xE_soll) && all(size(xE_soll) == [6 1]), ...
  'ParRob/invkin1: xE_soll muss 6x1 sein');
assert(isreal(q0) && all(size(q0) == [Rob.NJ 1]), ...
  'ParRob/invkin1: q0 muss %dx1 sein', Rob.NJ);
s_std = struct( ...
             'n_min', 0, ... % Minimale Anzahl Iterationen
             'n_max', 1000, ... % Maximale Anzahl Iterationen
             'Phit_tol', 1e-8, ... % Toleranz für translatorischen Fehler
             'Phir_tol', 1e-8, ... % Toleranz für rotatorischen Fehler
             'reci', false, ... % Keine reziproken Winkel für ZB-Def.
             'retry_limit', 100); % Anzahl der Neuversuche

s = s_std;
% Prüfe Felder der Einstellungs-Struktur und setze Standard-Werte, falls
% Eingabe nicht gesetzt. Nehme nur Felder, die vorgesehen sind, damit keine
% Fehler aufgeworfen werden wg zu vieler Felder
if nargin >= 4
  for f = fields(s_in)'
    if isfield(s_std, f{1})
      s.(f{1}) = s_in.(f{1});
    end
  end
end
if all(Rob.I_EE_Task == logical([1 1 1 1 1 0]))
  s.reci = true;
end

% Halbiere die Toleranz, weil durch die Umrechnung auf die EE-Plattform
% noch ein zusätzlicher Hebelarm den Fehler verstärkt
s.Phit_tol = s.Phit_tol/2;
s.Phir_tol = s.Phir_tol/2;
%% Start
Phi_ser = NaN(Rob.I2constr_red(end),1);
q = q0;
% Tc_Pges = Rob.fkine_platform(xE_soll);
r_P_P_B_ges = Rob.r_P_B_all;
r_P_P_E = Rob.T_P_E(1:3,4);
T_0_E = Rob.x2t(xE_soll);
% IK für alle Beine einzeln berechnen
for i = 1:Rob.NLEG
  % Überschreibe evtl übergebene I_EE für PKM. Das I_EE hier gilt für die Beinkette
  % Damit wird der Standardwert gesetzt, der sowieso in SerRob/invkin2
  % genutzt wird
  s.I_EE = Rob.Leg(i).I_EE_Task;

  % Initialisierung
  q0_i = q0(Rob.I1J_LEG(i):Rob.I2J_LEG(i));
  T_0_0i = Rob.Leg(i).T_W_0;

  % Transformation vom letzten Beinketten-KS zum EE-KS der PKM bestimmen
  % Dafür wird die IK berechnet und dieser Wert wird übergeben
  r_P_Bi_E = -r_P_P_B_ges(:,i) + r_P_P_E; % Vektor von Koppelpunkt zum Endeffektor
  T_0i_E = invtr(T_0_0i) * T_0_E; % Transformation vom Basis-KS der Beinkette zum EE
  xE_soll_i = Rob.Leg(i).t2x(T_0i_E); % als Vektor
  s.T_N_E = Rob.Leg(i).T_N_E * transl(r_P_Bi_E); % Anpassung der EE-Transformation für IK

  % Inverse Kinematik für die serielle Beinkette berechnen
  [q_i, Phi_i] = Rob.Leg(i).invkin2(xE_soll_i, q0_i, s); % Aufruf der kompilierten IK-Funktion als Einzeldatei
  
  if all(Rob.I_EE_Task == logical([1 1 1 1 1 0])) && i == 1
    % 3T2R und Führungskette. Die erste Beinkette gibt die EE-Ori für die
    % anderen Beinketten vor.
    [~,T_0_Bi] = Rob.Leg(i).fkineEE(q_i);
    % Aktualisiere die EE-Transformation auf die resultierend aus Bein 1
    T_0_E = T_0_Bi * transl(r_P_Bi_E);
  end
  % Ergebnisse für dieses Bein abspeichern
  q(Rob.I1J_LEG(i):Rob.I2J_LEG(i)) = q_i;
  Phi_ser(Rob.I1constr_red(i):Rob.I2constr_red(i)) = Phi_i;
end
% Kinematische Zwangsbedingungen nochmal neu für die PKM bestimmen
if all(Rob.I_EE_Task == logical([1 1 1 1 1 0]))
  Phi_voll = Rob.constr3(q, xE_soll);
  Phi = Phi_voll(Rob.I_constr_red);
else
  Phi = Rob.constr1(q, xE_soll);
end
% Probe: Stimmen die Zwangsbedingungen?
if all(abs(Phi_ser) < 1e-7) && any(abs(Phi)>1e-6)
  error('Fehler: ZB stimmen nicht überein. Wahrscheinlichste Ursache: EE-KS der Beinkette ist falsch gedreht.');
end
