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

function R = eul2r(phi, conv)

switch conv
  case 1 % xyx
    R = eulxyx2r(phi);
  case 2 % xyz
    R = eulxyz2r(phi);
  case 3 % xzx
    R = eulxzx2r(phi);
  case 4 % xzy
    R = eulxzy2r(phi);
  case 5 % yxy
    R = eulyxy2r(phi);
  case 6 % yxz
    R = eulyxz2r(phi);
  case 7 % yzx
    R = eulyzx2r(phi);
  case 8 % yzy
    R = eulyzy2r(phi);
  case 9 % zxy
    R = eulzxy2r(phi);
  case 10 % zxz
    R = eulzxz2r(phi);
  case 11 % zyx
    R = eulzyx2r(phi);
  case 12 % zyz
    R = eulzyz2r(phi);
end