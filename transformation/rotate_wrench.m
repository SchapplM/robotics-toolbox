% Rotiere 6x1-Vektor mit Kraftwinde (wrench) oder allgemeine
% Geschwindingkeit (twist) in ein anderes Koordinatensystem
% 
% Input:
% w_A
%   Vektor in KS A
% R_BA
%   Rotationsmatrix von A nach B
% 
% Output:
% w_B
%   Vektor in KS b

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-09
% (c) Institut für Regelungstechnik, Universität Hannover

function w_B = rotate_wrench(w_A, R_BA)
%#codegen
%$cgargs {zeros(6,1),zeros(3,3)}
assert(isreal(w_A) && all(size(w_A) == [6 1]), ...
  'w_B has to be [6x1] (double)');
assert(isreal(R_BA) && all(size(R_BA) == [3 3]), ...
  'R_BA has to be [3x3] (double)');

w_B = [R_BA*w_A(1:3); R_BA*w_A(4:6)];