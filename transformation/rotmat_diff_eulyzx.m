% Ableitung der Rotationsmatrix nach den sie erzeugenden yzx-Euler-Winkel
% Konvention: R = roty(phi1) * rotz(phi2) * rotx(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   yzx-Euler-Winkel
%
% Ausgabe:
% GradMat [9x3]:
%   Gradientenmatrix: Ableitung der (spaltenweise gestapelten) Rotationsmatrix nach den Euler-Winkeln
%
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function GradMat = rotmat_diff_eulyzx(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'rotmat_diff_eulyzx: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% aus codeexport/rotmat_diff_eulyzx_matlab.m (euler_angle_calculations.mw)
t336 = sin(phi2);
t337 = sin(phi1);
t347 = t337 * t336;
t338 = cos(phi3);
t346 = t337 * t338;
t335 = sin(phi3);
t339 = cos(phi2);
t345 = t339 * t335;
t344 = t339 * t338;
t340 = cos(phi1);
t343 = t340 * t336;
t342 = t340 * t338;
t341 = t340 * t339;
t334 = -t337 * t335 + t336 * t342;
t333 = t335 * t343 + t346;
t332 = t340 * t335 + t336 * t346;
t331 = -t335 * t347 + t342;
t1 = [-t337 * t339 -t343 0; 0 t339 0; -t341 t347 0; t332 -t338 * t341 t333; 0 -t336 * t338 -t345; t334 t337 * t344 t331; t331 t335 * t341 t334; 0 t336 * t335 -t344; -t333 -t337 * t345 -t332;];
GradMat = t1;
