% Ableitung der Euler-Winkel nach der daraus berechneten Rotationsmatrix
% 
% Eingabe:
% R [3x3]:
%   Rotationsmatrix
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
% GradMat [3x9]
%   Gradientenmatrix: Ableitung der Euler-Winkel nach der (spaltenweise
%   gestapelten) Rotationsmatrix

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function GradMat = eul_diff_rotmat(R, conv)
%% Init
%#codegen
%$cgargs {zeros(3,3), uint8(0)}
assert(isreal(R) && all(size(R) == [3 3]), ...
  'eul_diff_rotmat: R has to be [3x3] (double)');
assert(isa(conv,'uint8') && isscalar(conv), ...
  'eul_diff_rotmat: Number of Euler convention has to be [1x1] uint8');

%% Berechnung der Ausgabe, Fallunterscheidung der Euler-Konvention

switch conv
  case 1 % xyx
    GradMat = eulxyx_diff_rotmat(R);
  case 2 % xyz
    GradMat = eulxyz_diff_rotmat(R);
  case 3 % xzx
    GradMat = eulxzx_diff_rotmat(R);
  case 4 % xzy
    GradMat = eulxzy_diff_rotmat(R);
  case 5 % yxy
    GradMat = eulyxy_diff_rotmat(R);
  case 6 % yxz
    GradMat = eulyxz_diff_rotmat(R);
  case 7 % yzx
    GradMat = eulyzx_diff_rotmat(R);
  case 8 % yzy
    GradMat = eulyzy_diff_rotmat(R);
  case 9 % zxy
    GradMat = eulzxy_diff_rotmat(R);
  case 10 % zxz
    GradMat = eulzxz_diff_rotmat(R);
  case 11 % zyx
    GradMat = eulzyx_diff_rotmat(R);
  case 12 % zyz
    GradMat = eulzyz_diff_rotmat(R);
  otherwise
    error('eul_diff_rotmat: conv has to be 1 to 12');
end