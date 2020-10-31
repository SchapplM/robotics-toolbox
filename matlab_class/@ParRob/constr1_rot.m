% Rotationskomponente der kinematischen ZB zwischen Ist- und Soll-Konfiguration
% Vollständige Rotations- und Translationskomponenten
% Variante 1:
% * Absolute Rotation ausgedrückt bspw. in XYZ-Euler-Winkeln
%   (statt XYZ wird die Konvention aus `phiconv_W_E` genommen)
% * Rotationsfehler ausgedrückt genauso bspw. in XYZ-Euler-Winkeln
%   Im Gegensatz zu [2_SchapplerTapOrt2019a] hier Rotation 0(q)-0(x)
% 
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% platform_frame [1x1 logical]
%   Benutze das Plattform-KS anstatt das EE-KS als Bezugsgröße für x
% 
% Ausgabe:
% Phi_red
%   Reduzierte kinematische Zwangsbedingungen (siehe folgendes)
%   Die Reduktion folgt aus der Klassenvariablen I_EE
% Phi [6Mx1]
%   Kinematische Zwangsbedingungen des Roboters für alle M Beine: 
%   Maß für den Orientierungsfehler zwischen Ist-Pose aus
%   gegebenen Gelenkwinkeln q und Soll-Pose aus gegebenen EE-Koordinaten x

% Quellen:
% [2_SchapplerTapOrt2019a] Schappler, M. et al.: Modeling Parallel Robot
% Kinematics for 3T2R and 3T3R Tasks using Reciprocal Sets of Euler Angles
% (Arbeitstitel), Submitted to MDPI Robotics KaRD2, Version of 27.06.2019
% [A] Aufzeichnungen Schappler vom 21.06.2018
% [B] Aufzeichnungen Schappler vom 22.06.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phi_red, Phi] = constr1_rot(Rob, q, xE, platform_frame)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr1_rot: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1_rot: xE muss 6x1 sein');
if nargin == 3, platform_frame = false; end

NLEG = Rob.NLEG;
Phi = NaN(length(Rob.I_constr_r),1);
Phi_red = NaN(length(Rob.I_constr_r_red),1);

if platform_frame
  R_P_E = eye(3);
else
  R_P_E = Rob.T_P_E(1:3,1:3);
end

%% Berechnung
R_0_E_x = eul2r(xE(4:6), Rob.phiconv_W_E);

for iLeg = 1:NLEG
  % Anteil der ZB-Gleichung der Gelenkkette
  % (Aus direkter Kinematik)
  IJ_i = Rob.I1J_LEG(iLeg):Rob.I2J_LEG(iLeg);
  qs = q(IJ_i); % Gelenkwinkel dieser Kette
  T_0i_Bi = Rob.Leg(iLeg).fkineEE(qs);
  
  % Fußpunkt-Orientierung
  phi_0_Ai = Rob.Leg(iLeg).phi_W_0;
  R_0_0i = eul2r(phi_0_Ai, Rob.Leg(iLeg).phiconv_W_0);
  R_P_Bi = eulxyz2r(Rob.phi_P_B_all(:,iLeg));
  R_Bi_P = R_P_Bi.';
  % Differenz-Rotation (z.B. mit XYZ-Euler-Winkel)
  % Kette: 0 -> 0i -> Bi -> E
  % (Bi und P sind nicht gleich orientiert).
  R_0_E_q = R_0_0i * T_0i_Bi(1:3,1:3) * R_Bi_P * R_P_E;
  R_0q_0x = R_0_E_q * R_0_E_x'; % Unterschied zu [2_SchapplerTapOrt2019a]

  % Gl. (A.1, B.25); ähnlich wie [2_SchapplerTapOrt2019a]/(19)
  phi = r2eul(R_0q_0x, Rob.phiconv_W_E);
  J1 = 1+3*(iLeg-1);
  J2 = J1+2;
  Phi(J1:J2,:) = phi;
end

% Reduzierte Zwangsbedingungsgleichungen, für reduzierte EE-FG
K1 = 1; % Zähler für Zwangsbedingungen
for iLeg = 1:NLEG
  % Indizes für volle ZB
  J1 = 1+3*(iLeg-1);
  J2 = J1+2;
  % Indizes für reduzierte ZB
  K2 = K1+sum(Rob.Leg(iLeg).I_EE_Task(4:6))-1; % Anzahl der rot. ZB. dieser Beinkette

  % Auswahl der wirklich benötigten Einträge
  Phi_i = Phi(J1:J2,:);
  Phi_red(K1:K2,:) = Phi_i(Rob.Leg(iLeg).I_EE_Task(4:6));
  K1 = K2 + 1;
end
