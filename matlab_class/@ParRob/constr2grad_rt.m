% Ableitung der Rotationskomponente der kinematischen ZB nach der EE-Position
% Rotation ausgedrückt in XYZ-Euler-Winkeln
% 
% Ausgabe:
% Phipx_red
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% Phipx [3xN]
%   Ableitung der Rotations-ZB nach der EE-Position. Ausgabe Null, da keine
%   Einfluss besteht

% Quellen:
% [2_SchapplerTapOrt2019a] Schappler, M. et al.: Modeling Parallel Robot
% Kinematics for 3T2R and 3T3R Tasks using Reciprocal Sets of Euler Angles
% (Arbeitstitel), Submitted to MDPI Robotics KaRD2, Version of 27.06.2019
% [A] Aufzeichnungen Schappler vom 21.06.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phipx_red, Phipx] = constr2grad_rt(Rob)

% Die Translation hat keinen Einfluss auf die Rotation
% Gl. (A.3); [2_SchapplerTapOrt2019a]/(35) (unten links)
Phipx = zeros(3*Rob.NLEG,3);

% Indizes für Reduktion der Zwangsbedingungen bei 3T2R: Nur für
% symmetrische 3T2R-PKM
n_constr_r_red = 3*Rob.NLEG;
if Rob.NJ == 25 % Behelf zur Erkennung symmetrischer 3T2R-PKM
  n_constr_r_red = 2*Rob.NLEG;
end
Phipx_red = zeros(n_constr_r_red, sum(Rob.I_EE(1:3)) );
