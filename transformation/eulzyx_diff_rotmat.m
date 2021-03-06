% Ableitung der zyx-Euler-Winkel nach der daraus berechneten Rotationsmatrix
% Konvention: R = rotz(phi1) * roty(phi2) * rotx(phi3).
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

function GradMat = eulzyx_diff_rotmat(R)
%% Init
%#codegen
%$cgargs {zeros(3,3)}
assert(isreal(R) && all(size(R) == [3 3]), 'eulzyx_diff_rotmat: R has to be [3x3] (double)');
r11=R(1,1);r12=R(1,2);r13=R(1,3);
r21=R(2,1);r22=R(2,2);r23=R(2,3); %#ok<NASGU>
r31=R(3,1);r32=R(3,2);r33=R(3,3); %#ok<NASGU>
%% Berechnung
% aus codeexport/eulzyx_diff_rotmat_matlab.m (euler_angle_calculations.mw)
t184 = r32 ^ 2 + r33 ^ 2;
t185 = sqrt(t184);
t186 = r31 / t185;
t183 = 0.1e1 / (r11 ^ 2 + r21 ^ 2);
t182 = 0.1e1 / t184;
t1 = [-r21 * t183 r11 * t183 0 0 0 0 0 0 0; 0 0 -t185 0 0 r32 * t186 0 0 r33 * t186; 0 0 0 0 0 r33 * t182 0 0 -r32 * t182;];
GradMat = t1;
