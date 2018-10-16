% Ableitung der Rotationsmatrix nach den sie erzeugenden xyz-Euler-Winkel
% Konvention: R = rotx(phi1) * roty(phi2) * rotz(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   xyz-Euler-Winkel
%
% Ausgabe:
% GradMat [9x3]:
%   Gradientenmatrix: Ableitung der (spaltenweise gestapelten) Rotationsmatrix nach den Euler-Winkeln
%
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function GradMat = rotmat_diff_eulxyz(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'rotmat_diff_eulxyz: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% aus codeexport/rotmat_diff_eulxyz_matlab.m (euler_angle_calculations.mw)
t251 = sin(phi2);
t252 = sin(phi1);
t262 = t252 * t251;
t253 = cos(phi3);
t261 = t252 * t253;
t250 = sin(phi3);
t254 = cos(phi2);
t260 = t254 * t250;
t259 = t254 * t253;
t255 = cos(phi1);
t258 = t255 * t251;
t257 = t255 * t253;
t256 = t255 * t254;
t249 = -t252 * t250 + t251 * t257;
t248 = t250 * t258 + t261;
t247 = t255 * t250 + t251 * t261;
t246 = -t250 * t262 + t257;
t1 = [0 -t251 * t253 -t260; t249 t252 * t259 t246; t247 -t253 * t256 t248; 0 t251 * t250 -t259; -t248 -t252 * t260 -t247; t246 t250 * t256 t249; 0 t254 0; -t256 t262 0; -t252 * t254 -t258 0;];
GradMat = t1;
