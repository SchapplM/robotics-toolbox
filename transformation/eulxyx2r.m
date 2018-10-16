% xyx-Euler-Winkel in eine Rotationsmatrix konvertieren
% Konvention: R = rotx(phi1) * roty(phi2) * rotx(phi3). (mitgedrehte Euler-Winkel; intrinsisch)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function R = eulxyx2r(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulxyx2r: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% R = rotx(phi1) * roty(phi2) * rotx(phi3):
% aus codeexport/eulxyx2r_matlab.m (euler_angle_calculations.mw)
t3 = sin(phi1);
t5 = cos(phi2);
t9 = t3 * t5;
t1 = sin(phi3);
t6 = cos(phi1);
t8 = t6 * t1;
t4 = cos(phi3);
t7 = t6 * t4;
t2 = sin(phi2);
t10 = [t5 t2 * t1 t2 * t4; t3 * t2 -t1 * t9 + t7 -t4 * t9 - t8; -t6 * t2 t3 * t4 + t5 * t8 -t3 * t1 + t5 * t7;];
R = t10;
