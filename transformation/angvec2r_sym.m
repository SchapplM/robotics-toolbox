% Berechnung der Rotationsmatrix aus einer Drehung nach
% Achse-Winkel-Notation
% 
% Eingabe:
% theta [1x1]
%   Drehwinkel
% u [3x1]
%   Drehachse
% 
% Ausgabe:
% R [3x3]
%   Rotationsmatrix
% 
% Quelle:
% [Robotik I], Gl. (2.37)
% 
% Siehe: angvec2r

% Moritz Schappler, schappler@irt.uni-hannover.de, 2017-03
% (c) Institut für Regelungstechnik, Universität Hannover


function R = angvec2r_sym(theta, u)

ux = u(1);
uy = u(2);
uz = u(3);
%% Symbolisch generierter Code
% Siehe rotation_rpy_omega.mw
t6 = cos(theta);
t4 = 0.1e1 - t6;
t10 = t4 * uz;
t5 = sin(theta);
t9 = ux * t5;
t8 = uy * t5;
t7 = uz * t5;
t3 = ux * uy * t4;
t2 = ux * t10;
t1 = uy * t10;
t11 = [t4 * ux ^ 2 + t6 t3 - t7 t2 + t8; t3 + t7 t4 * uy ^ 2 + t6 t1 - t9; t2 - t8 t1 + t9 t4 * uz ^ 2 + t6;];

% Ausgabe
R = t11;