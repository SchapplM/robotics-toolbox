% Translationskomponente der kinematischen ZB zwischen Ist- und Soll-Konfiguration
% Variante 1:
% * Vektor vom Basis- zum Koppelpunkt-KS (unterschiedlich zur Variante 1
%   bei seriellen Robotern; dort Basis bis EE)
% * Entspricht Standard-Modellierung für PKM. Siehe [2_SchapplerTapOrt2019a],
%   Kap. 2; "Parallel Robots" (Merlet); Robotik 2 Skript
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

function [Phi_red, Phi] = constr1_trans(Rob, q, xE, platform_frame)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr_trans: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1_trans: xE muss 6x1 sein');
if nargin == 3, platform_frame = false; end

NLEG = Rob.NLEG;

Phi = NaN(length(Rob.I_constr_t),1);
Phi_red = NaN(length(Rob.I_constr_t_red),1);

%% Berechnung
r_0_0_E = xE(1:3);
R_0_E = eul2r(xE(4:6), Rob.phiconv_W_E);
T_0_E = transl(r_0_0_E)*r2t(R_0_E);
if platform_frame
  T_P_E = eye(4);
else
  T_P_E = Rob.T_P_E;
end
T_0_P = T_0_E * invtr(T_P_E);
R_0_P = T_0_P(1:3,1:3);
r_0_0_P = T_0_P(1:3,4);

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
  r_0_Ai_Bi_q = R_0_0i*r_0i_Ai_Bi_q;
  J1 = 1+3*(iLeg-1);
  J2 = J1+2;
  K1 = 1+sum(Rob.Leg(iLeg).I_EE(1:3))*(iLeg-1);
  K2 = K1+sum(Rob.Leg(iLeg).I_EE(1:3))-1;

  % Anteil der Plattform
  % Aus den Koordinaten der Plattform
  r_P_P_Bi = Rob.r_P_B_all(:,iLeg);
  r_0_P_Bi = R_0_P * r_P_P_Bi;
  r_0_0_Bi = r_0_0_P + r_0_P_Bi;
  r_0_Ai_Bi_x = -r_0_0_Ai + r_0_0_Bi;
  
  % Positions-Differenz Koppelpunkte aus Beinketten- und
  % Plattform-Berechnung
  % [2_SchapplerTapOrt2019a]/(6) bzw. Gl. (A.23, B.22)
  Phi_i = r_0_Ai_Bi_q - r_0_Ai_Bi_x;
  Phi(J1:J2,:) = Phi_i;
  if ~isempty(Phi_red)
    Phi_red(K1:K2,:) = Phi_i(Rob.Leg(iLeg).I_EE(1:3));
  end
end

