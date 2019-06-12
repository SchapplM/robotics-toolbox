% Inverse Kinematik für allgemeinen parallelen Roboter
% Aufruf der Funktion für serielle Roboter. Dadurch Berechnung der IK
% für alle Beine unabhängig und nicht gleichzeitig.
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

function [q, Phi] = invkin_ser(Rob, xE_soll, q0, s)

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
if nargin < 4
  % Keine Einstellungen übergeben. Standard-Einstellungen
  s = s_std;
end
% Prüfe Felder der Einstellungs-Struktur und setze Standard-Werte, falls
% Eingabe nicht gesetzt
for f = fields(s_std)'
  if ~isfield(s, f{1})
    s.(f{1}) = s_std.(f{1});
  end
end

%% Start
Phi_ser = NaN(Rob.I2constr_red(end),1);%f�r red dim(Phi_ser)<dim(q)
q = q0;
Tc_Pges = Rob.fkine_platform(xE_soll);% LJN: nur Translation unterschiedlich
% IK für alle Beine einzeln berechnen
for i = 1:Rob.NLEG 
  q0_i = q0(Rob.I1J_LEG(i):Rob.I2J_LEG(i));
  % Soll-Lage des KS des Plattform-Koppelpunktes bestimmen.
  % Entspricht dem virtuellen Endeffektor der Beinkette
  T_0_0i = Rob.Leg(i).T_W_0;
  T_0_Bi = Tc_Pges(:,:,i);
  T_0i_Bi = invtr(T_0_0i) * T_0_Bi;
  % "End-Effektor-Pose" der Beinkette erzeugen. Das ist die Lage des
  % Plattform-Koppel-Koordinatensystems auf Bein-Seite
  xE_soll_i = [T_0i_Bi(1:3,4); r2eul(t2r(T_0i_Bi), Rob.Leg(i).phiconv_W_E)];
  % Inverse Kinematik für die serielle Beinkette berechnen
  [q_i, Phi_i] = Rob.Leg(i).invkin2(xE_soll_i, q0_i, s); % Aufruf der kompilierten IK-Funktion als Einzeldatei.
  % LJN: invkin_eulangresidual.m kann NAN werte ausgeben
  q(Rob.I1J_LEG(i):Rob.I2J_LEG(i)) = q_i;
  Phi_ser(Rob.I1constr_red(i):Rob.I2constr_red(i)) = Phi_i; % origin
%   if size(Phi_i,1)> sum(Rob.I_EE) % LJN: red_Phi nehmen, andere Phi l�schen
%     Phi_ser(Rob.I1constr_red(i):Rob.I2constr_red(i)) = Phi_i(Rob.I_EE);% LJN
%   else
%     Phi_ser(Rob.I1constr_red(i):Rob.I2constr_red(i)) = Phi_i;
%   end
  
end
Phi = Rob.constr1(q, xE_soll);
if all(abs(Phi_ser) < 1e-7) && any(abs(Phi)>1e-6)
  error('Fehler: ZB stimmen nicht überein. Wahrscheinlichste Ursache: EE-KS der Beinkette ist falsch gedreht.');
end
