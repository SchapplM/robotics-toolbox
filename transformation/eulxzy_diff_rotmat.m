% Ableitung der xzy-Euler-Winkel nach der daraus berechneten Rotationsmatrix
% Konvention: R = rotx(phi1) * rotz(phi2) * roty(phi3).
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

function GradMat = eulxzy_diff_rotmat(R)
%% Init
%#codegen
%$cgargs {zeros(3,3)}
assert(isreal(R) && all(size(R) == [3 3]), 'eulxzy_diff_rotmat: R has to be [3x3] (double)');
r11=R(1,1);r12=R(1,2);r13=R(1,3);
r21=R(2,1);r22=R(2,2);r23=R(2,3); %#ok<NASGU>
r31=R(3,1);r32=R(3,2);r33=R(3,3); %#ok<NASGU>
%% Berechnung
% aus codeexport/eulxzy_diff_rotmat_matlab.m (euler_angle_calculations.mw)
t142 = r11 ^ 2 + r13 ^ 2;
t143 = sqrt(t142);
t144 = r12 / t143;
t141 = 0.1e1 / t142;
t140 = 0.1e1 / (r22 ^ 2 + r32 ^ 2);
t1 = [0 0 0 0 -r32 * t140 r22 * t140 0 0 0; r11 * t144 0 0 -t143 0 0 r13 * t144 0 0; -r13 * t141 0 0 0 0 0 r11 * t141 0 0;];
GradMat = t1;
