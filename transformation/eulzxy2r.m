% zxy-Euler-Winkel in eine Rotationsmatrix konvertieren
% Konvention: R = rotz(phi1) * rotx(phi2) * roty(phi3). (mitgedrehte Euler-Winkel; intrinsisch)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function R = eulzxy2r(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulzxy2r: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% R = rotz(phi1) * rotx(phi2) * roty(phi3):
% aus codeexport/eulzxy2r_matlab.m (euler_angle_calculations.mw)
t77 = sin(phi3);
t79 = sin(phi1);
t86 = t79 * t77;
t80 = cos(phi3);
t85 = t79 * t80;
t82 = cos(phi1);
t84 = t82 * t77;
t83 = t82 * t80;
t81 = cos(phi2);
t78 = sin(phi2);
t1 = [-t78 * t86 + t83 -t79 * t81 t78 * t85 + t84; t78 * t84 + t85 t82 * t81 -t78 * t83 + t86; -t81 * t77 t78 t81 * t80;];
R = t1;
