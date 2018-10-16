% Ableitung der zyz-Euler-Winkel nach der daraus berechneten Rotationsmatrix
% Konvention: R = rotz(phi1) * roty(phi2) * rotz(phi3).
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

function GradMat = eulzyz_diff_rotmat(R)
%% Init
%#codegen
%$cgargs {zeros(3,3)}
assert(isreal(R) && all(size(R) == [3 3]), 'eulzyz_diff_rotmat: R has to be [3x3] (double)');
r11=R(1,1);r12=R(1,2);r13=R(1,3);
r21=R(2,1);r22=R(2,2);r23=R(2,3); %#ok<NASGU>
r31=R(3,1);r32=R(3,2);r33=R(3,3); %#ok<NASGU>
%% Berechnung
% aus codeexport/eulzyz_diff_rotmat_matlab.m (euler_angle_calculations.mw)
t224 = r31 ^ 2 + r32 ^ 2;
t220 = 0.1e1 / (r33 ^ 2 + t224);
t227 = sqrt(t224);
t228 = r33 * t220 / t227;
t223 = 0.1e1 / (r13 ^ 2 + r23 ^ 2);
t222 = 0.1e1 / t224;
t1 = [0 0 0 0 0 0 -r23 * t223 r13 * t223 0; 0 0 r31 * t228 0 0 r32 * t228 0 0 -t227 * t220; 0 0 r32 * t222 0 0 -r31 * t222 0 0 0;];
GradMat = t1;
