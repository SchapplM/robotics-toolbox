% Ableitung der Rotationsmatrix nach den sie erzeugenden zyx-Euler-Winkel
% Konvention: R = rotz(phi1) * roty(phi2) * rotx(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   zyx-Euler-Winkel
%
% Ausgabe:
% GradMat [9x3]:
%   Gradientenmatrix: Ableitung der (spaltenweise gestapelten) Rotationsmatrix nach den Euler-Winkeln
%
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function GradMat = rotmat_diff_eulzyx(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'rotmat_diff_eulzyx: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% aus codeexport/rotmat_diff_eulzyx_matlab.m (euler_angle_calculations.mw)
t404 = sin(phi2);
t405 = sin(phi1);
t415 = t405 * t404;
t406 = cos(phi3);
t414 = t405 * t406;
t403 = sin(phi3);
t407 = cos(phi2);
t413 = t407 * t403;
t412 = t407 * t406;
t408 = cos(phi1);
t411 = t408 * t404;
t410 = t408 * t406;
t409 = t408 * t407;
t402 = t405 * t403 + t404 * t410;
t401 = t403 * t411 - t414;
t400 = -t408 * t403 + t404 * t414;
t399 = -t403 * t415 - t410;
t1 = [-t405 * t407 -t411 0; t409 -t415 0; 0 -t407 0; t399 t403 * t409 t402; t401 t405 * t413 t400; 0 -t404 * t403 t412; -t400 t406 * t409 -t401; t402 t405 * t412 t399; 0 -t404 * t406 -t413;];
GradMat = t1;
