% Ableitung der xyx-Euler-Winkel nach der daraus berechneten Rotationsmatrix
% Konvention: R = rotx(phi1) * roty(phi2) * rotx(phi3).
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

function GradMat = eulxyx_diff_rotmat(R)
%% Init
%#codegen
%$cgargs {zeros(3,3)}
assert(isreal(R) && all(size(R) == [3 3]), 'eulxyx_diff_rotmat: R has to be [3x3] (double)');
r11=R(1,1);r12=R(1,2);r13=R(1,3);
r21=R(2,1);r22=R(2,2);r23=R(2,3); %#ok<NASGU>
r31=R(3,1);r32=R(3,2);r33=R(3,3); %#ok<NASGU>
%% Berechnung
% aus codeexport/eulxyx_diff_rotmat_matlab.m (euler_angle_calculations.mw)
t124 = r21 ^ 2 + r31 ^ 2;
t125 = sqrt(t124);
t126 = r11 / t125;
t123 = 0.1e1 / (r12 ^ 2 + r13 ^ 2);
t122 = 0.1e1 / t124;
t1 = [0 -r31 * t122 r21 * t122 0 0 0 0 0 0; -t125 r21 * t126 r31 * t126 0 0 0 0 0 0; 0 0 0 r13 * t123 0 0 -r12 * t123 0 0;];
GradMat = t1;
