% Ableitung der xzx-Euler-Winkel nach der daraus berechneten Rotationsmatrix
% Konvention: R = rotx(phi1) * rotz(phi2) * rotx(phi3).
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

function GradMat = eulxzx_diff_rotmat(R)
%% Init
%#codegen
%$cgargs {zeros(3,3)}
assert(isreal(R) && all(size(R) == [3 3]), 'eulxzx_diff_rotmat: R has to be [3x3] (double)');
r11=R(1,1);r12=R(1,2);r13=R(1,3);
r21=R(2,1);r22=R(2,2);r23=R(2,3); %#ok<NASGU>
r31=R(3,1);r32=R(3,2);r33=R(3,3); %#ok<NASGU>
%% Berechnung
% aus codeexport/eulxzx_diff_rotmat_matlab.m (euler_angle_calculations.mw)
t143 = r21 ^ 2 + r31 ^ 2;
t139 = 0.1e1 / (r11 ^ 2 + t143);
t146 = sqrt(t143);
t147 = r11 * t139 / t146;
t142 = 0.1e1 / (r12 ^ 2 + r13 ^ 2);
t141 = 0.1e1 / t143;
t1 = [0 -r31 * t141 r21 * t141 0 0 0 0 0 0; -t146 * t139 r21 * t147 r31 * t147 0 0 0 0 0 0; 0 0 0 r13 * t142 0 0 -r12 * t142 0 0;];
GradMat = t1;
