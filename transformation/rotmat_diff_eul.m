% Ableitung der Rotationsmatrix nach den sie erzeugenden Euler-Winkeln
% Erklärung: [2_SchapplerTapOrt2019a] Kap. 5.1, (A21)
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

% Quelle:
% [2_SchapplerTapOrt2019a] Schappler, M. et al.: Modeling Parallel Robot
% Kinematics for 3T2R and 3T3R Tasks using Reciprocal Sets of Euler Angles
% (Arbeitstitel), Submitted to MDPI Robotics KaRD2, Version of 27.06.2019

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function GradMat = rotmat_diff_eul(phi, conv)
%% Init
%#codegen
%$cgargs {zeros(3,1), uint8(0)}
assert(isreal(phi) && all(size(phi) == [3 1]), ...
  'rotmat_diff_eul: phi has to be [3x1] (double)');
assert(isa(conv,'uint8') && isscalar(conv), ...
  'rotmat_diff_eul: Number of Euler convention has to be [1x1] uint8');

%% Berechnung der Ausgabe, Fallunterscheidung der Euler-Konvention

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
  otherwise
    error('rotmat_diff_eul: conv has to be 1 to 12');
end