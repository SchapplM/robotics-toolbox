% Homogene Transformationsmatrix mit Elementarrotation um die z-Achse
% 
% Eingabe:
% gamma [1x1]
%   Drehwinkel
% 
% Ausgabe:
% T [4x4] / SE(3)
%   Homogene Transformationsmatrix

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

function T = trotz(gamma)
% Quelle: Skript Robotik I (WS 2015/16), Ortmaier, Uni Hannover, Gl. 2.25
T = [rotz(gamma) [0; 0; 0]; 0 0 0 1];
