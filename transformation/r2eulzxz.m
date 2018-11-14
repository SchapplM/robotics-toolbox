% Rotationsmatrix nach zxz-Euler-Winkel konvertieren
% Konvention: R = rotz(phi1) * rotx(phi2) * rotz(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover
%
function phi = r2eulzxz(R)
%% Init
%#codegen
%$cgargs {zeros(3,3)}
assert(isreal(R) && all(size(R) == [3 3]), 'r2eulzxz: R has to be [3x3] (double)');
r11=R(1,1);r12=R(1,2);r13=R(1,3);
r21=R(2,1);r22=R(2,2);r23=R(2,3); %#ok<NASGU>
r31=R(3,1);r32=R(3,2);r33=R(3,3); %#ok<NASGU>
%% Berechnung
% aus codeexport/r2eulzxz_matlab.m (euler_angle_calculations.mw)
t1 = [atan2(r13, -r23); atan2(sqrt(r31 ^ 2 + r32 ^ 2), r33); atan2(r31, r32);];
phi = t1;
