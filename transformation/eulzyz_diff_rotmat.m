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
t190 = r31 ^ 2 + r32 ^ 2;
t191 = sqrt(t190);
t192 = r33 / t191;
t189 = 0.1e1 / (r13 ^ 2 + r23 ^ 2);
t188 = 0.1e1 / t190;
t1 = [0 0 0 0 0 0 -r23 * t189 r13 * t189 0; 0 0 r31 * t192 0 0 r32 * t192 0 0 -t191; 0 0 r32 * t188 0 0 -r31 * t188 0 0 0;];
GradMat = t1;
