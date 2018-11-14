% Ableitung der Rotationsmatrix nach den sie erzeugenden xzx-Euler-Winkel
% Konvention: R = rotx(phi1) * rotz(phi2) * rotx(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   xzx-Euler-Winkel
%
% Ausgabe:
% GradMat [9x3]:
%   Gradientenmatrix: Ableitung der (spaltenweise gestapelten) Rotationsmatrix nach den Euler-Winkeln
%
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function GradMat = rotmat_diff_eulxzx(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'rotmat_diff_eulxzx: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% aus codeexport/rotmat_diff_eulxzx_matlab.m (euler_angle_calculations.mw)
t268 = sin(phi2);
t269 = sin(phi1);
t279 = t269 * t268;
t270 = cos(phi3);
t278 = t269 * t270;
t267 = sin(phi3);
t271 = cos(phi2);
t277 = t271 * t267;
t276 = t271 * t270;
t272 = cos(phi1);
t275 = t272 * t268;
t274 = t272 * t270;
t273 = t272 * t271;
t266 = -t269 * t277 + t274;
t265 = -t272 * t267 - t269 * t276;
t264 = -t267 * t273 - t278;
t263 = t269 * t267 - t270 * t273;
t1 = [0 -t268 0; -t279 t273 0; t275 t269 * t271 0; 0 -t276 t268 * t267; t265 -t268 * t274 t264; -t263 -t268 * t278 t266; 0 t277 t268 * t270; -t266 t267 * t275 t263; t264 t267 * t279 t265;];
GradMat = t1;
