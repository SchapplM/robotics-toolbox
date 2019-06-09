% Ableitung der zxz-Euler-Winkel nach der daraus berechneten Rotationsmatrix
% Konvention: R = rotz(phi1) * rotx(phi2) * rotz(phi3).
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

function GradMat = eulzxz_diff_rotmat(R)
%% Init
%#codegen
%$cgargs {zeros(3,3)}
assert(isreal(R) && all(size(R) == [3 3]), 'eulzxz_diff_rotmat: R has to be [3x3] (double)');
r11=R(1,1);r12=R(1,2);r13=R(1,3);
r21=R(2,1);r22=R(2,2);r23=R(2,3); %#ok<NASGU>
r31=R(3,1);r32=R(3,2);r33=R(3,3); %#ok<NASGU>
%% Berechnung
% aus codeexport/eulzxz_diff_rotmat_matlab.m (euler_angle_calculations.mw)
t178 = r31 ^ 2 + r32 ^ 2;
t179 = sqrt(t178);
t180 = r33 / t179;
t177 = 0.1e1 / (r13 ^ 2 + r23 ^ 2);
t176 = 0.1e1 / t178;
t1 = [0 0 0 0 0 0 -r23 * t177 r13 * t177 0; 0 0 r31 * t180 0 0 r32 * t180 0 0 -t179; 0 0 r32 * t176 0 0 -r31 * t176 0 0 0;];
GradMat = t1;
