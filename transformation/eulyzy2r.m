% yzy-Euler-Winkel in eine Rotationsmatrix konvertieren

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function R = eulyzy2r(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulyzy2r: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% R = roty(phi1) * rotz(phi2) * roty(phi3):
% aus codeexport/eulyzy2r_matlab.m
t70 = sin(phi1);
t72 = cos(phi2);
t76 = t70 * t72;
t68 = sin(phi3);
t73 = cos(phi1);
t75 = t73 * t68;
t71 = cos(phi3);
t74 = t73 * t71;
t69 = sin(phi2);
t1 = [-t70 * t68 + t72 * t74 -t73 * t69 t70 * t71 + t72 * t75; t69 * t71 t72 t69 * t68; -t71 * t76 - t75 t70 * t69 -t68 * t76 + t74;];
R = t1;
