% xyz-Euler-Winkel in eine Rotationsmatrix konvertieren

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function R = eulxyz2r(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulxyz2r: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% R = rotx(phi1) * roty(phi2) * rotz(phi3):
% aus codeexport/eulxyz2r_matlab.m
t10 = sin(phi3);
t12 = sin(phi1);
t19 = t12 * t10;
t13 = cos(phi3);
t18 = t12 * t13;
t15 = cos(phi1);
t17 = t15 * t10;
t16 = t15 * t13;
t14 = cos(phi2);
t11 = sin(phi2);
t1 = [t14 * t13 -t14 * t10 t11; t11 * t18 + t17 -t11 * t19 + t16 -t12 * t14; -t11 * t16 + t19 t11 * t17 + t18 t15 * t14;];
R = t1;
