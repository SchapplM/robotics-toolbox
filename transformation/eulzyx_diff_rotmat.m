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
t215 = r32 ^ 2 + r33 ^ 2;
t211 = 0.1e1 / (r31 ^ 2 + t215);
t218 = sqrt(t215);
t219 = r31 * t211 / t218;
t214 = 0.1e1 / (r11 ^ 2 + r21 ^ 2);
t213 = 0.1e1 / t215;
t1 = [-r21 * t214 r11 * t214 0 0 0 0 0 0 0; 0 0 -t211 * t218 0 0 r32 * t219 0 0 r33 * t219; 0 0 0 0 0 r33 * t213 0 0 -r32 * t213;];
GradMat = t1;
