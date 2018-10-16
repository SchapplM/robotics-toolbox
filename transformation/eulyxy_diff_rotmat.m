% Ableitung der yxy-Euler-Winkel nach der daraus berechneten Rotationsmatrix
% Konvention: R = roty(phi1) * rotx(phi2) * roty(phi3).
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

function GradMat = eulyxy_diff_rotmat(R)
%% Init
%#codegen
%$cgargs {zeros(3,3)}
assert(isreal(R) && all(size(R) == [3 3]), 'eulyxy_diff_rotmat: R has to be [3x3] (double)');
r11=R(1,1);r12=R(1,2);r13=R(1,3);
r21=R(2,1);r22=R(2,2);r23=R(2,3); %#ok<NASGU>
r31=R(3,1);r32=R(3,2);r33=R(3,3); %#ok<NASGU>
%% Berechnung
% aus codeexport/eulyxy_diff_rotmat_matlab.m (euler_angle_calculations.mw)
t161 = r21 ^ 2 + r23 ^ 2;
t157 = 0.1e1 / (r22 ^ 2 + t161);
t164 = sqrt(t161);
t165 = r22 * t157 / t164;
t160 = 0.1e1 / (r12 ^ 2 + r32 ^ 2);
t159 = 0.1e1 / t161;
t1 = [0 0 0 r32 * t160 0 -r12 * t160 0 0 0; 0 r21 * t165 0 0 -t164 * t157 0 0 r23 * t165 0; 0 -r23 * t159 0 0 0 0 0 r21 * t159 0;];
GradMat = t1;
