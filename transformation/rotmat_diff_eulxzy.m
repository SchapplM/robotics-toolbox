% Ableitung der Rotationsmatrix nach den sie erzeugenden xzy-Euler-Winkel
% Konvention: R = rotx(phi1) * rotz(phi2) * roty(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   xzy-Euler-Winkel
%
% Ausgabe:
% GradMat [9x3]:
%   Gradientenmatrix: Ableitung der (spaltenweise gestapelten) Rotationsmatrix nach den Euler-Winkeln
%
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function GradMat = rotmat_diff_eulxzy(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'rotmat_diff_eulxzy: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% aus codeexport/rotmat_diff_eulxzy_matlab.m (euler_angle_calculations.mw)
t285 = sin(phi2);
t286 = sin(phi1);
t296 = t286 * t285;
t287 = cos(phi3);
t295 = t286 * t287;
t284 = sin(phi3);
t288 = cos(phi2);
t294 = t288 * t284;
t293 = t288 * t287;
t289 = cos(phi1);
t292 = t289 * t285;
t291 = t289 * t287;
t290 = t289 * t288;
t283 = t286 * t284 + t285 * t291;
t282 = t284 * t292 - t295;
t281 = -t289 * t284 + t285 * t295;
t280 = -t284 * t296 - t291;
t1 = [0 -t285 * t287 -t294; -t281 t287 * t290 -t282; t283 t286 * t293 t280; 0 -t288 0; -t286 * t288 -t292 0; t290 -t296 0; 0 -t285 * t284 t293; t280 t284 * t290 t283; t282 t286 * t294 t281;];
GradMat = t1;
