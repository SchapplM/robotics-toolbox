% zyx-Euler-Winkel in eine Rotationsmatrix konvertieren

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function R = eulzyx2r(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulzyx2r: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% R = rotz(phi1) * roty(phi2) * rotx(phi3):
% aus codeexport/eulzyx2r_matlab.m
t96 = sin(phi3);
t98 = sin(phi1);
t105 = t98 * t96;
t99 = cos(phi3);
t104 = t98 * t99;
t101 = cos(phi1);
t103 = t101 * t96;
t102 = t101 * t99;
t100 = cos(phi2);
t97 = sin(phi2);
t1 = [t101 * t100 t97 * t103 - t104 t97 * t102 + t105; t98 * t100 t97 * t105 + t102 t97 * t104 - t103; -t97 t100 * t96 t100 * t99;];
R = t1;
