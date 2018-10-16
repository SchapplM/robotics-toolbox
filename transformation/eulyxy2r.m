% yxy-Euler-Winkel in eine Rotationsmatrix konvertieren

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function R = eulyxy2r(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulyxy2r: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% R = roty(phi1) * rotx(phi2) * roty(phi3):
% aus codeexport/eulyxy2r_matlab.m
t41 = sin(phi1);
t43 = cos(phi2);
t47 = t41 * t43;
t39 = sin(phi3);
t44 = cos(phi1);
t46 = t44 * t39;
t42 = cos(phi3);
t45 = t44 * t42;
t40 = sin(phi2);
t1 = [-t39 * t47 + t45 t41 * t40 t42 * t47 + t46; t40 * t39 t43 -t40 * t42; -t41 * t42 - t43 * t46 t44 * t40 -t41 * t39 + t43 * t45;];
R = t1;
