% Ableitung der Rotationsmatrix nach den sie erzeugenden xyx-Euler-Winkel
% 
% Eingabe:
% phi [3x1]
%   Satz Euler-Winkel
% conv [1x1]
%   Nummer der Euler-Winkel-Konvention
%    1 XYX
%    2 XYZ
%    3 XZX
%    4 XZY
%    5 YXY
%    6 YXZ
%    7 YZX
%    8 YZY
%    9 ZXY
%   10 ZXZ
%   11 ZYX
%   12 ZYZ
% 
% Ausgabe:
% GradMat [9x3]:
%   Gradientenmatrix: Ableitung der Rotationsmatrix nach den Euler-Winkeln

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function GradMat = rotmat_diff_eul(phi, conv)

assert(isreal(phi) && all(size(phi) == [3 1]), 'rotmat_diff_eul: phi has to be [3x1] (double)');

switch conv
  case 1 % xyx
    GradMat = rotmat_diff_eulxyx(phi);
  case 2 % xyz
    GradMat = rotmat_diff_eulxyz(phi);
  case 3 % xzx
    GradMat = rotmat_diff_eulxzx(phi);
  case 4 % xzy
    GradMat = rotmat_diff_eulxzy(phi);
  case 5 % yxy
    GradMat = rotmat_diff_eulyxy(phi);
  case 6 % yxz
    GradMat = rotmat_diff_eulyxz(phi);
  case 7 % yzx
    GradMat = rotmat_diff_eulyzx(phi);
  case 8 % yzy
    GradMat = rotmat_diff_eulyzy(phi);
  case 9 % zxy
    GradMat = rotmat_diff_eulzxy(phi);
  case 10 % zxz
    GradMat = rotmat_diff_eulzxz(phi);
  case 11 % zyx
    GradMat = rotmat_diff_eulzyx(phi);
  case 12 % zyz
    GradMat = rotmat_diff_eulzyz(phi);
end