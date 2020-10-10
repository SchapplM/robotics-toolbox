% Rotationskomponente der kinematischen ZB zwischen Ist- und Soll-Konfiguration
% Vollständige Rotations- und Translationskomponenten
% Variante 2:
% * Absolute Rotation ausgedrückt bspw. in XYZ-Euler-Winkeln
%   (statt XYZ wird die Konvention aus `phiconv_W_E` genommen)
% * Rotationsfehler ausgedrückt in ZYX-Euler-Winkeln
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
%   Maß für den Orientierungsfehler zwischen Ist-Pose aus
%   gegebenen Gelenkwinkeln q und Soll-Pose aus gegebenen EE-Koordinaten x

% Quellen:
% [2_SchapplerTapOrt2019a] Schappler, M. et al.: Modeling Parallel Robot
% Kinematics for 3T2R and 3T3R Tasks using Reciprocal Sets of Euler Angles
% (Arbeitstitel), Submitted to MDPI Robotics KaRD2, Version of 27.06.2019
% [A] Aufzeichnungen Schappler vom 27.07.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phi_red, Phi] = constr2_rot(Rob, q, xE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr2_rot: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr2_rot: xE muss 6x1 sein');
NLEG = Rob.NLEG;
Phi = NaN(3*NLEG,1);
Phi_red = NaN(sum(Rob.I_EE(4:6))*NLEG,1);

R_P_E = Rob.T_P_E(1:3,1:3);

%% Berechnung
R_0_E_x = eul2r(xE(4:6), Rob.phiconv_W_E);
[~,phiconv_W_E_reci] = euler_angle_properties(Rob.phiconv_W_E);

for iLeg = 1:NLEG
  % Anteil der ZB-Gleichung der Gelenkkette
  % (Aus direkter Kinematik)
  IJ_i = Rob.I1J_LEG(iLeg):Rob.I2J_LEG(iLeg);
  qs = q(IJ_i); % Gelenkwinkel dieser Kette
  T_0i_Bi = Rob.Leg(iLeg).fkineEE(qs);
  
  % Fußpunkt-Orientierung
  phi_0_Ai = Rob.Leg(iLeg).phi_W_0;
  R_0_0i = eul2r(phi_0_Ai, Rob.phiconv_W_0); 
  R_P_Bi = eulxyz2r(Rob.phi_P_B_all(:,iLeg));
  R_Bi_P = R_P_Bi.';
  
  % Differenz-Rotation (z.B. mit XYZ-Euler-Winkel)
  % Kette: 0 -> 0i -> Bi -> E
  % (Bi und P sind gleich orientiert)
  R_0_E_q = R_0_0i * T_0i_Bi(1:3,1:3) * R_Bi_P * R_P_E;
  R_Ex_Eq = R_0_E_x' * R_0_E_q;

  % [2_SchapplerTapOrt2019a]/(19); Gl. (A.30)
  phiR = r2eul(R_Ex_Eq, phiconv_W_E_reci); 
  J1 = 1+3*(iLeg-1);
  J2 = J1+2;
  Phi(J1:J2,:) = phiR;
end

% Reduzierte Zwangsbedingungsgleichungen, für reduzierte EE-FG
for iLeg = 1:NLEG
  % Indizes für volle ZB
  J1 = 1+3*(iLeg-1);
  J2 = J1+2;
  % Indizes für reduzierte ZB
  K1 = 1+sum(Rob.I_EE(4:6))*(iLeg-1);
  K2 = K1+sum(Rob.I_EE(4:6))-1;
  % Auswahl der wirklich benötigten Einträge
  Phi_i = Phi(J1:J2,:); % hier noch keine Elimination
  if all(Rob.Leg(iLeg).I_EE == logical([1 1 1 1 1 0]))
    Phi_red(K1:K2,:) = Phi_i([2 3]); % anstatt logical Rob.I_EE (1 1 0)
  end
end