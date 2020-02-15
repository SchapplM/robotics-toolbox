% Elementarrotation um die y-Achse
% 
% Eingabe:
% beta [1x1]
%   Drehwinkel
% 
% Ausgabe:
% R [3x3] / SO(3)
%   Rotationsmatrix

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

function R = roty(beta)
%$cgargs {zeros(1,1)}
assert(isreal(beta) && all(size(beta) == [1 1]), ...
  'Rotation angles beta has to be [1x1] (double)');

% Quelle: Skript Robotik I (WS 2015/16), Ortmaier, Uni Hannover, S. 16
cbeta = cos(beta);
sbeta = sin(beta);
R =    [cbeta   0	sbeta
        0       1	0
        -sbeta	0	cbeta];
