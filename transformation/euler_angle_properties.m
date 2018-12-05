% Eigenschaften der Euler-Winkel-Konventionen zurückgeben
% 
% Eingabe:
% conv [1x1 uint8]
%   Nummer der Winkelkonvention
% 
% Ausgabe:
% str
%   Zeichenkette der mitgedrehten Drehachsen "X", "Y" und "Z"
% recconv [1x1 uint8]
%   Nummer der dazu reziproken Winkelkonvention (also mit umgedrehter
%   Reihenfolge der Drehachsen)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function [str, recconv] = euler_angle_properties(conv)

assert(isa(conv,'uint8') && isscalar(conv), ...
  'euler_angle_properties: Number of Euler convention has to be [1x1] uint8');

switch conv
  case 1 % xyx
    str = 'xyx';
    recconv = NaN;
  case 2 % xyz
    str = 'xyz';
    % ZYX ist reciprok zu XYZ
    recconv = uint8(11);
  case 3 % xzx
    str = 'xzx';
    recconv = NaN;
  case 4 % xzy
    str = 'xzy';
    recconv = uint8(7);
  case 5 % yxy
    str = 'yxy';
    recconv = NaN;
  case 6 % yxz
    str = 'yxz';
    recconv = uint8(9);
  case 7 % yzx
    str = 'yzx';
    recconv = uint8(4);
  case 8 % yzy
    str = 'yzy';
    recconv = NaN;
  case 9 % zxy
    str = 'zxy';
    recconv = uint8(6);
  case 10 % zxz
    str = 'zxz';
    recconv = NaN;
  case 11 % zyx
    str = 'zyx';
    recconv = uint8(2);
  case 12 % zyz
    str = 'zyz';
    recconv = NaN;
end