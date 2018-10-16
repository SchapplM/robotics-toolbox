% Ableitung der Rotationsmatrix nach den sie erzeugenden yzy-Euler-Winkel
% Konvention: R = roty(phi1) * rotz(phi2) * roty(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   yzy-Euler-Winkel
%
% Ausgabe:
% GradMat [9x3]:
%   Gradientenmatrix: Ableitung der (spaltenweise gestapelten) Rotationsmatrix nach den Euler-Winkeln
%
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function GradMat = rotmat_diff_eulyzy(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'rotmat_diff_eulyzy: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% aus codeexport/rotmat_diff_eulyzy_matlab.m (euler_angle_calculations.mw)
t353 = sin(phi2);
t354 = sin(phi1);
t364 = t354 * t353;
t355 = cos(phi3);
t363 = t354 * t355;
t352 = sin(phi3);
t356 = cos(phi2);
t362 = t356 * t352;
t361 = t356 * t355;
t357 = cos(phi1);
t360 = t357 * t353;
t359 = t357 * t355;
t358 = t357 * t356;
t351 = -t354 * t362 + t359;
t350 = -t357 * t352 - t354 * t361;
t349 = -t352 * t358 - t363;
t348 = t354 * t352 - t355 * t358;
t1 = [t350 -t353 * t359 t349; 0 t361 -t353 * t352; t348 t353 * t363 -t351; t364 -t358 0; 0 -t353 0; t360 t354 * t356 0; t351 -t352 * t360 -t348; 0 t362 t353 * t355; t349 t352 * t364 t350;];
GradMat = t1;
