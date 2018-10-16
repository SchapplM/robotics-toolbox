% zxz-Euler-Winkel in eine Rotationsmatrix konvertieren

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function R = eulzxz2r(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulzxz2r: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% R = rotz(phi1) * rotx(phi2) * rotz(phi3):
% aus codeexport/eulzxz2r_matlab.m
t89 = sin(phi1);
t91 = cos(phi2);
t95 = t89 * t91;
t87 = sin(phi3);
t92 = cos(phi1);
t94 = t92 * t87;
t90 = cos(phi3);
t93 = t92 * t90;
t88 = sin(phi2);
t1 = [-t87 * t95 + t93 -t90 * t95 - t94 t89 * t88; t89 * t90 + t91 * t94 -t89 * t87 + t91 * t93 -t92 * t88; t88 * t87 t88 * t90 t91;];
R = t1;
