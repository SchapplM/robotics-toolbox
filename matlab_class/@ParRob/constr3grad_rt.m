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

function [Phipx_red, Phipx] = constr3grad_rt(Rob)

% Die Translation hat keinen Einfluss auf die Rotation
% Gl. (A.3); [2_SchapplerTapOrt2019a]/(35) (unten links)
Phipx = zeros(3*Rob.NLEG,3);
Phipx_red = zeros( sum(Rob.I_EE(4:6))*Rob.NLEG, sum(Rob.I_EE(1:3)) );
% TODO: Die reduzierten ZB sind aktuell nicht konsistent für Roboter mit
% Beinketten mit fünf Gelenken. Funktionert bspw. nur für 6UPS-3T2R