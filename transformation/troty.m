% Homogene Transformationsmatrix mit Elementarrotation um die y-Achse
% 
% Eingabe:
% beta [1x1]
%   Drehwinkel
% 
% Ausgabe:
% T [4x4] / SE(3)
%   Homogene Transformationsmatrix

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

function T = troty(beta)
% Quelle: Skript Robotik I (WS 2015/16), Ortmaier, Uni Hannover, Gl. 2.25
T = [roty(beta) [0; 0; 0]; 0 0 0 1];
