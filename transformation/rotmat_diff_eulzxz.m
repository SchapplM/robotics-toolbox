% Ableitung der Rotationsmatrix nach den sie erzeugenden zxz-Euler-Winkel
% Konvention: R = rotz(phi1) * rotx(phi2) * rotz(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   zxz-Euler-Winkel
%
% Ausgabe:
% GradMat [9x3]:
%   Gradientenmatrix: Ableitung der (spaltenweise gestapelten) Rotationsmatrix nach den Euler-Winkeln
%
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function GradMat = rotmat_diff_eulzxz(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'rotmat_diff_eulzxz: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% aus codeexport/rotmat_diff_eulzxz_matlab.m (euler_angle_calculations.mw)
t387 = sin(phi2);
t388 = sin(phi1);
t398 = t388 * t387;
t389 = cos(phi3);
t397 = t388 * t389;
t386 = sin(phi3);
t390 = cos(phi2);
t396 = t390 * t386;
t395 = t390 * t389;
t391 = cos(phi1);
t394 = t391 * t387;
t393 = t391 * t389;
t392 = t391 * t390;
t385 = -t388 * t396 + t393;
t384 = -t391 * t386 - t388 * t395;
t383 = -t386 * t392 - t397;
t382 = t388 * t386 - t389 * t392;
t1 = [t383 t386 * t398 t384; t385 -t386 * t394 -t382; 0 t396 t387 * t389; t382 t387 * t397 -t385; t384 -t387 * t393 t383; 0 t395 -t387 * t386; t394 t388 * t390 0; t398 -t392 0; 0 -t387 0;];
GradMat = t1;
