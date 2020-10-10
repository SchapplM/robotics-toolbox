% Rotationskomponente der kinematischen ZB zwischen Ist- und Soll-
% Konfiguration (Zeitableitung)
% 
% Variante 4:
% * Bezogen auf Winkelgeschwindigkeit des Koppelpunktes Bi
%   (effektiv werden die Geschw.-ZB nach den Gelenk-Geschw. abgeleitet)
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
% 
% Ausgabe:
% PhirD_red
%   Reduzierte kinematische Zwangsbedingungen (siehe folgendes)
%   Die Reduktion folgt aus der Klassenvariablen I_EE
% PhirD [6Mx1]
%   Kinematische Zwangsbedingungen des Roboters für alle M Beine: 
%   Maß für den Orientierungsfehler zwischen Ist-Pose aus
%   gegebenen Gelenkwinkeln q und Soll-Pose aus gegebenen EE-Koordinaten x

% Quelle:
% [A] Aufzeichnungen Schappler vom 13.02.2020

% Junnan Li, WiHi bei Moritz Schappler, 2020-05
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [PhirD_red, PhirD] = constr4D_rot(Rob, q,qD, xE ,xDE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr1D_rot: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/constr1D_rot: qD muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1D_rot: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr1D_rot: xDE muss 6x1 sein');

NLEG = Rob.NLEG;
phi = xE(4:6); % Euler-Winkel
Jw = euljac(phi, Rob.phiconv_W_E); % Euler-Jacobi-Matrix für EE-Orientierung
PhirD = NaN(length(Rob.I_constr_r),1);
PhirD_red = NaN(length(Rob.I_constr_r_red),1);

I1 = 1;
for i = 1:NLEG
  % Definitionen für Beinkette i
  IJ_i = Rob.I1J_LEG(i):Rob.I2J_LEG(i);
  qs = q(IJ_i); % Gelenkwinkel dieser Kette 
  qsD = qD(IJ_i);
  phi_0_Ai = Rob.Leg(i).phi_W_0;
  R_0_0i = eul2r(phi_0_Ai, Rob.Leg(i).phiconv_W_0);
  % Geschwindigkeit des Koppelpunktes auf Plattform-Seite
  % [A], Gl. 7
  J0i_i_rotg = Rob.Leg(i).jacobiw(qs);
  J_Ai_Bi = R_0_0i*J0i_i_rotg;  % Bezug auf das Basis-KS der PKM
  V_bein = J_Ai_Bi*qsD; % ToDo: translatische Bewegung beruecksichtigen?
  
  % Geschwindigkeit des Koppelpunktes auf Seite der Beinkette
  % [A], Gl. 8
  V_plat = Jw * xDE(4:6);
  
  % Geschwindigkeits-Zwangsbedingung entspricht Differenz am Koppelpunkt
  % (kein Punktbezug, da Plattform als Starrkörper für
  % Winkelgeschwindigkeit ausreicht). [B], Gl. 1
  V_diff = V_bein - V_plat;
  PhirD(I1:I1+2) = V_diff;
  I1 = I1+3;
  % Ausgabe mit reduzierten Einträgen
  J1 = sum(Rob.I_EE(4:6))*(i-1)+1;
  PhirD_red(J1:J1+sum(Rob.Leg(i).I_EE_Task(4:6))-1) = V_diff(Rob.Leg(i).I_EE_Task(4:6));
end