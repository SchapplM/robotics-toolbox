% Elementarrotation um die x-Achse
% 
% Eingabe:
% alpha [1x1]
%   Drehwinkel
% 
% Ausgabe:
% R [3x3] / SO(3)
%   Rotationsmatrix

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

function R = rotx(alpha)
%#codegen
%$cgargs {zeros(1,1)}
assert(isreal(alpha) && all(size(alpha) == [1 1]), ...
  'Rotation angles alpha has to be [1x1] (double)');

% Quelle: Skript Robotik I (WS 2015/16), Ortmaier, Uni Hannover, S. 16
calpha = cos(alpha);
salpha = sin(alpha);
R = [1	0	      0; ...
     0	calpha	-salpha; ...
     0	salpha	calpha];
