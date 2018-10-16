% yzx-Euler-Winkel in eine Rotationsmatrix konvertieren

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function R = eulyzx2r(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulyzx2r: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% R = roty(phi1) * rotz(phi2) * rotx(phi3):
% aus codeexport/eulyzx2r_matlab.m
t58 = sin(phi3);
t60 = sin(phi1);
t67 = t60 * t58;
t61 = cos(phi3);
t66 = t60 * t61;
t63 = cos(phi1);
t65 = t63 * t58;
t64 = t63 * t61;
t62 = cos(phi2);
t59 = sin(phi2);
t1 = [t63 * t62 -t59 * t64 + t67 t59 * t65 + t66; t59 t62 * t61 -t62 * t58; -t60 * t62 t59 * t66 + t65 -t59 * t67 + t64;];
R = t1;
