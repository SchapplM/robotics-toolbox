% Ableitung der yxz-Euler-Winkel nach der daraus berechneten Rotationsmatrix
% Konvention: R = roty(phi1) * rotx(phi2) * rotz(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% R [3x3]:
%   Rotationsmatrix
%
% Ausgabe:
% GradMat [3x9]:
%   Gradientenmatrix: Ableitung der Euler-Winkel nach der (spaltenweise gestapelten) Rotationsmatrix

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function GradMat = eulyxz_diff_rotmat(R)
%% Init
%#codegen
%$cgargs {zeros(3,3)}
assert(isreal(R) && all(size(R) == [3 3]), 'eulyxz_diff_rotmat: R has to be [3x3] (double)');
r11=R(1,1);r12=R(1,2);r13=R(1,3);
r21=R(2,1);r22=R(2,2);r23=R(2,3); %#ok<NASGU>
r31=R(3,1);r32=R(3,2);r33=R(3,3); %#ok<NASGU>
%% Berechnung
% aus codeexport/eulyxz_diff_rotmat_matlab.m (euler_angle_calculations.mw)
t154 = r21 ^ 2 + r22 ^ 2;
t155 = sqrt(t154);
t156 = r23 / t155;
t153 = 0.1e1 / (r13 ^ 2 + r33 ^ 2);
t152 = 0.1e1 / t154;
t1 = [0 0 0 0 0 0 r33 * t153 0 -r13 * t153; 0 r21 * t156 0 0 r22 * t156 0 0 -t155 0; 0 r22 * t152 0 0 -r21 * t152 0 0 0 0;];
GradMat = t1;
