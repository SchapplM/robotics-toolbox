% Translationskomponente der kinematischen ZB zwischen Ist- und
% Soll-Konfiguration (Zeitableitung)
% Variante 1:
% * Vektor vom Basis- zum Koppelpunkt-KS (unterschiedlich zur Variante 1
%   bei seriellen Robotern; dort Basis bis EE)
% * Entspricht Standard-Modellierung für PKM. Siehe [2_SchapplerTapOrt2019a],
%   Kap. 2; "Parallel Robots" (Merlet); Robotik 2 Skript
% 
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% qD [Nx1]
%   Geschwindigkeit aller Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% xDE [6x1]
%   Zeitableitung der Endeffektorpose des Roboters bezüglich des Basis-KS
% platform_frame [1x1 logical]
%   Benutze das Plattform-KS anstatt das EE-KS als Bezugsgröße für x
% 
% Ausgabe:
% PhitD_red
%   Reduzierte kinematische Zwangsbedingungen (Zeitableitung) (siehe folgendes)
%   Die Reduktion folgt aus der Klassenvariablen I_EE
% PhitD [6Mx1]
%   Kinematische Zwangsbedingungen des Roboters für alle M Beine (Zeitableitung): 
%   Maß für den Positionsfehler zwischen Ist-Pose aus
%   gegebenen Gelenkwinkeln q und Soll-Pose aus gegebenen EE-Koordinaten x

% Quelle:
% [A] Aufzeichnungen Schappler vom 15.06.2020
% [B] Aufzeichnungen Schappler vom 13.02.2020

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [PhitD_red, PhitD] = constr1D_trans(Rob, q, qD, xE, xDE, platform_frame)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr1D_trans: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/constr1D_trans: qD muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1D_trans: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr1D_trans: xDE muss 6x1 sein');
if nargin == 5, platform_frame = false; end

%% Allgemeine Definitionen
NLEG = Rob.NLEG;
PhitD_red = NaN(length(Rob.I_constr_t_red),1);
PhitD = NaN(3*NLEG,1);
I1 = 1;
phi = xE(4:6); % Euler-Winkel
Jw = euljac(phi, Rob.phiconv_W_E); % Euler-Jacobi-Matrix für EE-Orientierung
R_0_E = eul2r(phi, Rob.phiconv_W_E);
r_P_B_all = Rob.r_P_B_all;
if platform_frame
  T_P_E = eye(4);
else
  T_P_E = Rob.T_P_E;
end

r_P_P_E = T_P_E(1:3,4);

%% Berechnung
for i = 1:NLEG
  % Definitionen für Beinkette i
  IJ_i = Rob.I1J_LEG(i):Rob.I2J_LEG(i);
  qs = q(IJ_i); % Gelenkwinkel dieser Kette 
  qsD = qD(IJ_i);
  phi_0_Ai = Rob.Leg(i).phi_W_0;
  R_0_0i = eul2r(phi_0_Ai, Rob.Leg(i).phiconv_W_0);
  r_P_P_Bi = r_P_B_all(:,i);
  r_E_E_Bi = T_P_E(1:3,1:3)' * (-r_P_P_E + r_P_P_Bi);
  
  % Geschwindigkeit des Koppelpunktes auf Seite der Beinkette
  % [B], Gl. 2
  J0i_i_trans = Rob.Leg(i).jacobit(qs);
  J0_i_trans = R_0_0i*J0i_i_trans; % Bezug auf das Basis-KS der PKM
  J_Ai_Bi = J0_i_trans; % Nur xyz-Koordinate in ZB.
  V_bein = J_Ai_Bi*qsD;
  % Geschwindigkeit des Koppelpunktes auf Plattform-Seite
  % [B], Gl. 4
  V_plat = xDE(1:3)-skew(R_0_E*r_E_E_Bi)*Jw*xDE(4:6);

  % Geschwindigkeits-Zwangsbedingung entspricht Differenz am Koppelpunkt
  % [B], Gl. 1
  V_diff = V_bein - V_plat;
  PhitD(I1:I1+2) = V_diff;
  I1 = I1+3;
  % Ausgabe mit reduzierten Einträgen
  J1 = sum(Rob.I_EE(1:3))*(i-1)+1;
  PhitD_red(J1:J1+sum(Rob.Leg(i).I_EE(1:3))-1) = V_diff(Rob.Leg(i).I_EE(1:3));
end

