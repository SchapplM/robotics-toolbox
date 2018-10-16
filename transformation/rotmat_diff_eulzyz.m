% Ableitung der Rotationsmatrix nach den sie erzeugenden zyz-Euler-Winkel
% Konvention: R = rotz(phi1) * roty(phi2) * rotz(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   zyz-Euler-Winkel
%
% Ausgabe:
% GradMat [9x3]:
%   Gradientenmatrix: Ableitung der (spaltenweise gestapelten) Rotationsmatrix nach den Euler-Winkeln
%
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function GradMat = rotmat_diff_eulzyz(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'rotmat_diff_eulzyz: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% aus codeexport/rotmat_diff_eulzyz_matlab.m (euler_angle_calculations.mw)
t421 = sin(phi2);
t422 = sin(phi1);
t432 = t422 * t421;
t423 = cos(phi3);
t431 = t422 * t423;
t420 = sin(phi3);
t424 = cos(phi2);
t430 = t424 * t420;
t429 = t424 * t423;
t425 = cos(phi1);
t428 = t425 * t421;
t427 = t425 * t423;
t426 = t425 * t424;
t419 = -t422 * t430 + t427;
t418 = -t425 * t420 - t422 * t429;
t417 = -t420 * t426 - t431;
t416 = t422 * t420 - t423 * t426;
t1 = [t418 -t421 * t427 t417; -t416 -t421 * t431 t419; 0 -t429 t421 * t420; -t419 t420 * t428 t416; t417 t420 * t432 t418; 0 t430 t421 * t423; -t432 t426 0; t428 t422 * t424 0; 0 -t421 0;];
GradMat = t1;
