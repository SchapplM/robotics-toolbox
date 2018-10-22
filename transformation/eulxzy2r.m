% xzy-Euler-Winkel in eine Rotationsmatrix konvertieren
% Konvention: R = rotx(phi1) * rotz(phi2) * roty(phi3). (mitgedrehte Euler-Winkel; intrinsisch)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function R = eulxzy2r(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulxzy2r: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% R = rotx(phi1) * rotz(phi2) * roty(phi3):
% aus codeexport/eulxzy2r_matlab.m (euler_angle_calculations.mw)
t29 = sin(phi3);
t31 = sin(phi1);
t38 = t31 * t29;
t32 = cos(phi3);
t37 = t31 * t32;
t34 = cos(phi1);
t36 = t34 * t29;
t35 = t34 * t32;
t33 = cos(phi2);
t30 = sin(phi2);
t1 = [t33 * t32 -t30 t33 * t29; t30 * t35 + t38 t34 * t33 t30 * t36 - t37; t30 * t37 - t36 t31 * t33 t30 * t38 + t35;];
R = t1;
