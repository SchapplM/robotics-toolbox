% Euler-Winkel in eine Rotationsmatrix konvertieren
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
% R [3x3]
%   Rotationsmatrix aus Elementardrehungen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function R = eul_diff_rotmat(R, conv)

assert(isreal(R) && all(size(R) == [3 3]), 'eul_diff_rotmat: R has to be [3x3] (double)');

switch conv
  case 1 % xyx
    R = eulxyx_diff_rotmat(R);
  case 2 % xyz
    R = eulxyz_diff_rotmat(R);
  case 3 % xzx
    R = eulxzx_diff_rotmat(R);
  case 4 % xzy
    R = eulxzy_diff_rotmat(R);
  case 5 % yxy
    R = eulyxy_diff_rotmat(R);
  case 6 % yxz
    R = eulyxz_diff_rotmat(R);
  case 7 % yzx
    R = eulyzx_diff_rotmat(R);
  case 8 % yzy
    R = eulyzy_diff_rotmat(R);
  case 9 % zxy
    R = eulzxy_diff_rotmat(R);
  case 10 % zxz
    R = eulzxz_diff_rotmat(R);
  case 11 % zyx
    R = eulzyx_diff_rotmat(R);
  case 12 % zyz
    R = eulzyz_diff_rotmat(R);
  otherwise
    error('eul_diff_rotmat: conv has to be 1 to 12');
end