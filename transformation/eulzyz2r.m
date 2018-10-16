% zyz-Euler-Winkel in eine Rotationsmatrix konvertieren
% Konvention: R = rotz(phi1) * roty(phi2) * rotz(phi3). (mitgedrehte Euler-Winkel; intrinsisch)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function R = eulzyz2r(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulzyz2r: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% R = rotz(phi1) * roty(phi2) * rotz(phi3):
% aus codeexport/eulzyz2r_matlab.m (euler_angle_calculations.mw)
t108 = sin(phi1);
t110 = cos(phi2);
t114 = t108 * t110;
t106 = sin(phi3);
t111 = cos(phi1);
t113 = t111 * t106;
t109 = cos(phi3);
t112 = t111 * t109;
t107 = sin(phi2);
t1 = [-t108 * t106 + t110 * t112 -t108 * t109 - t110 * t113 t111 * t107; t109 * t114 + t113 -t106 * t114 + t112 t108 * t107; -t107 * t109 t107 * t106 t110;];
R = t1;
