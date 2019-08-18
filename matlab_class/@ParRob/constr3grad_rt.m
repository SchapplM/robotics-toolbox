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
% [A] Aufzeichnungen Schappler vom 21.06.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phipx_red, Phipx] = constr3grad_rt(Rob)

% Die Translation hat keinen Einfluss auf die Rotation
% Gl. (A.3)
Phipx = zeros(3*Rob.NLEG,3);
Phipx_red = zeros( sum(Rob.I_EE(4:6))*Rob.NLEG, sum(Rob.I_EE(1:3)) );
% TODO: Die reduzierten ZB sind aktuell nicht konsistent für Roboter mit
% Beinketten mit fünf Gelenken. Funktionert bspw. nur für 6UPS-3T2R