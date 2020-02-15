% Elementarrotation um die z-Achse
% 
% Eingabe:
% gamma [1x1]
%   Drehwinkel
% 
% Ausgabe:
% R [3x3] / SO(3)
%   Rotationsmatrix

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

function R = rotz(gamma)
%#codegen
%$cgargs {zeros(1,1)}
assert(isreal(gamma) && all(size(gamma) == [1 1]), ...
  'Rotation angles gamma has to be [1x1] (double)');

% Quelle: Skript Robotik I (WS 2015/16), Ortmaier, Uni Hannover, S. 16
cgamma = cos(gamma);
sgamma = sin(gamma);
R =    [cgamma	-sgamma	0
        sgamma	cgamma	0
        0       0       1];
