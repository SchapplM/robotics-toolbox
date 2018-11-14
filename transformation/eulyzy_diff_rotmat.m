% Ableitung der yzy-Euler-Winkel nach der daraus berechneten Rotationsmatrix
% Konvention: R = roty(phi1) * rotz(phi2) * roty(phi3).
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

function GradMat = eulyzy_diff_rotmat(R)
%% Init
%#codegen
%$cgargs {zeros(3,3)}
assert(isreal(R) && all(size(R) == [3 3]), 'eulyzy_diff_rotmat: R has to be [3x3] (double)');
r11=R(1,1);r12=R(1,2);r13=R(1,3);
r21=R(2,1);r22=R(2,2);r23=R(2,3); %#ok<NASGU>
r31=R(3,1);r32=R(3,2);r33=R(3,3); %#ok<NASGU>
%% Berechnung
% aus codeexport/eulyzy_diff_rotmat_matlab.m (euler_angle_calculations.mw)
t188 = r21 ^ 2 + r23 ^ 2;
t184 = 0.1e1 / (r22 ^ 2 + t188);
t191 = sqrt(t188);
t192 = r22 * t184 / t191;
t187 = 0.1e1 / (r12 ^ 2 + r32 ^ 2);
t186 = 0.1e1 / t188;
t1 = [0 0 0 r32 * t187 0 -r12 * t187 0 0 0; 0 r21 * t192 0 0 -t191 * t184 0 0 r23 * t192 0; 0 -r23 * t186 0 0 0 0 0 r21 * t186 0;];
GradMat = t1;
