% xzx-Euler-Winkel in eine Rotationsmatrix konvertieren
% Konvention: R = rotx(phi1) * rotz(phi2) * rotx(phi3). (mitgedrehte Euler-Winkel; intrinsisch)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function R = eulxzx2r(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulxzx2r: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% R = rotx(phi1) * rotz(phi2) * rotx(phi3):
% aus codeexport/eulxzx2r_matlab.m (euler_angle_calculations.mw)
t22 = sin(phi1);
t24 = cos(phi2);
t28 = t22 * t24;
t20 = sin(phi3);
t25 = cos(phi1);
t27 = t25 * t20;
t23 = cos(phi3);
t26 = t25 * t23;
t21 = sin(phi2);
t1 = [t24 -t21 * t23 t21 * t20; t25 * t21 -t22 * t20 + t24 * t26 -t22 * t23 - t24 * t27; t22 * t21 t23 * t28 + t27 -t20 * t28 + t26;];
R = t1;
