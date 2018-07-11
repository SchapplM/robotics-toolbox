% Multiplikation von Quaternionen
% 
% Eingabe:
% q1, q2 [4x1]
%   Quaternionen (1x1 Realteil, 3x1 Imaginärteil)
% 
% Ausgabe:
% q3 [4x1]
%   Produkt der Quaternionen (1x1 Realteil, 3x1 Imaginärteil)
%   q3=q1*q2
% 
% Quelle:
% [1] Ortmaier,T.: Robotik I Vorlesungsskript

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-04
% (C) Institut für mechatronische Systeme, Universität Hannover

function q3 = quatprod(q1, q2)
%#codegen
%$cgargs {zeros(4,1),zeros(4,1)}
assert(isreal(q1) && all(size(q1) == [4 1]), ...
  'quatprod: Einagbe-Quaternion q1 muss [4x1] double sein');
assert(isreal(q2) && all(size(q2) == [4 1]), ...
  'quatprod: Einagbe-Quaternion q2 muss [4x1] double sein');

% [1], Gl. (2.47)
s1 = q1(1);
v1 = q1(2:4);
s2 = q2(1);
v2 = q2(2:4);

% [1], Gl. (2.52)
q3 = [s1*s2 - v1'*v2; s1*v2+s2*v1+cross(v1,v2)];
