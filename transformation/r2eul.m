% Euler-Winkel aus einer Rotationsmatrix berechnen
% 
% Eingabe:
% R [3x3]
%   Rotationsmatrix aus Elementardrehungen
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
% phi [3x1]
%   Satz Euler-Winkel

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function phi = r2eul(R, conv)

switch conv
  case 1 % xyx
    phi = r2eulxyx(R);
  case 2 % xyz
    phi = r2eulxyz(R);
  case 3 % xzx
    phi = r2eulxzx(R);
  case 4 % xzy
    phi = r2eulxzy(R);
  case 5 % yxy
    phi = r2eulyxy(R);
  case 6 % yxz
    phi = r2eulyxz(R);
  case 7 % yzx
    phi = r2eulyzx(R);
  case 8 % yzy
    phi = r2eulyzy(R);
  case 9 % zxy
    phi = r2eulzxy(R);
  case 10 % zxz
    phi = r2eulzxz(R);
  case 11 % zyx
    phi = r2eulzyx(R);
  case 12 % zyz
    phi = r2eulzyz(R);
  otherwise
    error('r2eul: conv has to be 1 to 12');
end