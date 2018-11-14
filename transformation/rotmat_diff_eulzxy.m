% Ableitung der Rotationsmatrix nach den sie erzeugenden zxy-Euler-Winkel
% Konvention: R = rotz(phi1) * rotx(phi2) * roty(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   zxy-Euler-Winkel
%
% Ausgabe:
% GradMat [9x3]:
%   Gradientenmatrix: Ableitung der (spaltenweise gestapelten) Rotationsmatrix nach den Euler-Winkeln
%
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function GradMat = rotmat_diff_eulzxy(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'rotmat_diff_eulzxy: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% aus codeexport/rotmat_diff_eulzxy_matlab.m (euler_angle_calculations.mw)
t370 = sin(phi2);
t371 = sin(phi1);
t381 = t371 * t370;
t372 = cos(phi3);
t380 = t371 * t372;
t369 = sin(phi3);
t373 = cos(phi2);
t379 = t373 * t369;
t378 = t373 * t372;
t374 = cos(phi1);
t377 = t374 * t370;
t376 = t374 * t372;
t375 = t374 * t373;
t368 = -t371 * t369 + t370 * t376;
t367 = t369 * t377 + t380;
t366 = t374 * t369 + t370 * t380;
t365 = -t369 * t381 + t376;
t1 = [-t367 -t371 * t379 -t366; t365 t369 * t375 t368; 0 t370 * t369 -t378; -t375 t381 0; -t371 * t373 -t377 0; 0 t373 0; t368 t371 * t378 t365; t366 -t372 * t375 t367; 0 -t370 * t372 -t379;];
GradMat = t1;
