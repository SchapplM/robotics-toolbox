% Translationskomponente der kinematischen ZB zwischen Ist- und Soll-Konfiguration
% Variante 2:
% * Implementierung mit Vektor 0-E statt A-B
% 
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% 
% Ausgabe:
% Phi_red
%   Reduzierte kinematische Zwangsbedingungen (siehe folgendes)
%   Die Reduktion folgt aus der Klassenvariablen I_EE
% Phi [6Mx1]
%   Kinematische Zwangsbedingungen des Roboters für alle M Beine: 
%   Maß für den Positionsfehler zwischen Ist-Pose aus
%   gegebenen Gelenkwinkeln q und Soll-Pose aus gegebenen EE-Koordinaten x

% Quellen:
% [2_SchapplerTapOrt2019a] Schappler, M. et al.: Modeling Parallel Robot
% Kinematics for 3T2R and 3T3R Tasks using Reciprocal Sets of Euler Angles
% (Arbeitstitel), Submitted to MDPI Robotics KaRD2, Version of 27.06.2019
% [A] Aufzeichnungen Schappler vom 15.06.2018
% [B] Aufzeichnungen Schappler vom 22.06.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phix_red, Phix] = constr2_trans(Rob, q, xE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr_trans: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr2_trans: xE muss 6x1 sein');

NLEG = Rob.NLEG;

Phix = NaN(3*NLEG,1);
Phix_red = NaN(sum(Rob.I_EE(1:3))*NLEG,1);

%% Berechnung
r_0_0_E_x = xE(1:3); % [2_SchapplerTapOrt2019a]/(14)

for iLeg = 1:NLEG
  % Anteil der ZB-Gleichung der Gelenkkette
  % Direkte Kinematik der Beinkette
  IJ_i = Rob.I1J_LEG(iLeg):Rob.I2J_LEG(iLeg);
  qs = q(IJ_i); % Gelenkwinkel dieser Kette
  
  % Fußpunktkoordinaten
  r_0_0_Ai = Rob.Leg(iLeg).r_W_0;
  phi_0_Ai = Rob.Leg(iLeg).phi_W_0;
  R_0_0i = eul2r(phi_0_Ai, Rob.Leg(iLeg).phiconv_W_0);
  
  T_0i_Bi = Rob.Leg(iLeg).fkineEE(qs);
  r_0i_Ai_Bi_q = T_0i_Bi(1:3,4);
  r_0_Ai_Bi_q = R_0_0i * r_0i_Ai_Bi_q; % Zweiter Term in [2_SchapplerTapOrt2019a]/(17)
  
  r_P_P_Bi = Rob.r_P_B_all(:,iLeg);
  R_0i_Bi = T_0i_Bi(1:3,1:3);
  R_0_Bi = R_0_0i * R_0i_Bi;
  r_0_Bi_P = R_0_Bi * (-r_P_P_Bi);
  r_0_0_P_q = r_0_0_Ai + r_0_Ai_Bi_q + r_0_Bi_P;
  r_0_0_E_q = r_0_0_P_q + R_0_Bi * Rob.r_P_E;  % [2_SchapplerTapOrt2019a]/(17)
  
  J1 = 1+3*(iLeg-1);
  J2 = J1+2;
  K1 = 1+sum(Rob.I_EE(1:3))*(iLeg-1);
  K2 = K1+sum(Rob.I_EE(1:3))-1;

  % [2_SchapplerTapOrt2019a]/(16); Gl. (A.23, B.22)
  Phix(J1:J2,:) = r_0_0_E_q([1 2 3]) - r_0_0_E_x;
  Phix_red(K1:K2,:) = r_0_0_E_q(Rob.I_EE(1:3)) - r_0_0_E_x(Rob.I_EE(1:3));
end

