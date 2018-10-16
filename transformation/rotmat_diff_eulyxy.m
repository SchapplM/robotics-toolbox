% Ableitung der Rotationsmatrix nach den sie erzeugenden yxy-Euler-Winkel
% Konvention: R = roty(phi1) * rotx(phi2) * roty(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   yxy-Euler-Winkel
%
% Ausgabe:
% GradMat [9x3]:
%   Gradientenmatrix: Ableitung der (spaltenweise gestapelten) Rotationsmatrix nach den Euler-Winkeln
%
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function GradMat = rotmat_diff_eulyxy(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'rotmat_diff_eulyxy: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% aus codeexport/rotmat_diff_eulyxy_matlab.m (euler_angle_calculations.mw)
t302 = sin(phi2);
t303 = sin(phi1);
t313 = t303 * t302;
t304 = cos(phi3);
t312 = t303 * t304;
t301 = sin(phi3);
t305 = cos(phi2);
t311 = t305 * t301;
t310 = t305 * t304;
t306 = cos(phi1);
t309 = t306 * t302;
t308 = t306 * t304;
t307 = t306 * t305;
t300 = -t303 * t311 + t308;
t299 = -t306 * t301 - t303 * t310;
t298 = -t301 * t307 - t312;
t297 = t303 * t301 - t304 * t307;
t1 = [t298 t301 * t313 t299; 0 t311 t302 * t304; -t300 t301 * t309 t297; t309 t303 * t305 0; 0 -t302 0; -t313 t307 0; -t297 -t302 * t312 t300; 0 -t310 t302 * t301; t299 -t302 * t308 t298;];
GradMat = t1;
