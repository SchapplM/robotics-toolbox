% Invertierung eines Quaternions
% 
% Eingabe:
% q [4x1]
%   Quaternion (1x1 Realteil, 3x1 Imaginärteil)
% 
% Ausgabe:
% qi [4x1]
%   Inverses Quaternion (1x1 Realteil, 3x1 Imaginärteil)
% 
% Quelle:
% [1] Ortmaier,T.: Robotik I Vorlesungsskript

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-04
% (C) Institut für mechatronische Systeme, Universität Hannover

function qi = invquat(q)
%#codegen
%$cgargs {zeros(4,1)}
assert(isreal(q) && all(size(q) == [4 1]), ...
  'invquat: Einagbe-Quaternion muss [4x1] (double) sein');

s = q(1);
v = q(2:4);

% [1], Gl. (2.48)
q_ad = [s; -v];

% [1], Gl. (2.49)
qi = q_ad / norm(q);