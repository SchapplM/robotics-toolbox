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
%   Reihenfolge der Drehachsen). Null, falls nicht zutreffen.
%   Siehe [SchapplerTapOrt2019], [2_SchapplerTapOrt2019a]

% Quellen:
% [2_SchapplerTapOrt2019a] Schappler, M. et al.: Modeling Parallel Robot
% Kinematics for 3T2R and 3T3R Tasks using Reciprocal Sets of Euler Angles
% (Arbeitstitel), Submitted to MDPI Robotics KaRD2, Version of 27.06.2019
% [SchapplerTapOrt2019] Schappler, M. et al.: Resolution of Functional
% Redundancy for 3T2R Robot Tasks using Two Sets of Reciprocal Euler
% Angles, Proc. of the 15th IFToMM World Congress, 2019

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [str, recconv] = euler_angle_properties(conv)

assert(isa(conv,'uint8') && isscalar(conv), ...
  'euler_angle_properties: Number of Euler convention has to be [1x1] uint8');

switch conv
  case 1 % xyx
    str = 'xyx';
    recconv = uint8(0);
  case 2 % xyz
    str = 'xyz';
    % ZYX ist reziprok zu XYZ
    recconv = uint8(11);
  case 3 % xzx
    str = 'xzx';
    recconv = uint8(0);
  case 4 % xzy
    str = 'xzy';
    recconv = uint8(7);
  case 5 % yxy
    str = 'yxy';
    recconv = uint8(0);
  case 6 % yxz
    str = 'yxz';
    recconv = uint8(9);
  case 7 % yzx
    str = 'yzx';
    recconv = uint8(4);
  case 8 % yzy
    str = 'yzy';
    recconv = uint8(0);
  case 9 % zxy
    str = 'zxy';
    recconv = uint8(6);
  case 10 % zxz
    str = 'zxz';
    recconv = uint8(0);
  case 11 % zyx
    str = 'zyx';
    recconv = uint8(2);
  case 12 % zyz
    str = 'zyz';
    recconv = uint8(0);
  otherwise
    str = 'err';
    recconv = uint8(0);
end