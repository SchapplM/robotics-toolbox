% Ableitung der Rotationsmatrix nach den sie erzeugenden yxz-Euler-Winkel
% Konvention: R = roty(phi1) * rotx(phi2) * rotz(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   yxz-Euler-Winkel
%
% Ausgabe:
% GradMat [9x3]:
%   Gradientenmatrix: Ableitung der (spaltenweise gestapelten) Rotationsmatrix nach den Euler-Winkeln
%
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function GradMat = rotmat_diff_eulyxz(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'rotmat_diff_eulyxz: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% aus codeexport/rotmat_diff_eulyxz_matlab.m (euler_angle_calculations.mw)
t319 = sin(phi2);
t320 = sin(phi1);
t330 = t320 * t319;
t321 = cos(phi3);
t329 = t320 * t321;
t318 = sin(phi3);
t322 = cos(phi2);
t328 = t322 * t318;
t327 = t322 * t321;
t323 = cos(phi1);
t326 = t323 * t319;
t325 = t323 * t321;
t324 = t323 * t322;
t317 = t320 * t318 + t319 * t325;
t316 = t318 * t326 - t329;
t315 = -t323 * t318 + t319 * t329;
t314 = -t318 * t330 - t325;
t1 = [t316 t320 * t328 t315; 0 -t319 * t318 t327; t314 t318 * t324 t317; t317 t320 * t327 t314; 0 -t319 * t321 -t328; -t315 t321 * t324 -t316; t324 -t330 0; 0 -t322 0; -t320 * t322 -t326 0;];
GradMat = t1;
