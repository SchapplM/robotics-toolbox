% Ableitung der Rotationsmatrix nach den sie erzeugenden xyx-Euler-Winkel
% Konvention: R = rotx(phi1) * roty(phi2) * rotx(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   xyx-Euler-Winkel
%
% Ausgabe:
% GradMat [9x3]:
%   Gradientenmatrix: Ableitung der (spaltenweise gestapelten) Rotationsmatrix nach den Euler-Winkeln
%
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function GradMat = rotmat_diff_eulxyx(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'rotmat_diff_eulxyx: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% aus codeexport/rotmat_diff_eulxyx_matlab.m (euler_angle_calculations.mw)
t234 = sin(phi2);
t235 = sin(phi1);
t245 = t235 * t234;
t236 = cos(phi3);
t244 = t235 * t236;
t233 = sin(phi3);
t237 = cos(phi2);
t243 = t237 * t233;
t242 = t237 * t236;
t238 = cos(phi1);
t241 = t238 * t234;
t240 = t238 * t236;
t239 = t238 * t237;
t232 = -t235 * t243 + t240;
t231 = -t238 * t233 - t235 * t242;
t230 = -t233 * t239 - t244;
t229 = t235 * t233 - t236 * t239;
t1 = [0 -t234 0; t241 t235 * t237 0; t245 -t239 0; 0 t243 t234 * t236; t230 t233 * t245 t231; t232 -t233 * t241 -t229; 0 t242 -t234 * t233; t229 t234 * t244 -t232; t231 -t234 * t240 t230;];
GradMat = t1;
