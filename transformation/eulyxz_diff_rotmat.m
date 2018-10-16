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
t170 = r21 ^ 2 + r22 ^ 2;
t166 = 0.1e1 / (r23 ^ 2 + t170);
t173 = sqrt(t170);
t174 = r23 * t166 / t173;
t169 = 0.1e1 / (r13 ^ 2 + r33 ^ 2);
t168 = 0.1e1 / t170;
t1 = [0 0 0 0 0 0 r33 * t169 0 -r13 * t169; 0 r21 * t174 0 0 r22 * t174 0 0 -t173 * t166 0; 0 r22 * t168 0 0 -r21 * t168 0 0 0 0;];
GradMat = t1;
