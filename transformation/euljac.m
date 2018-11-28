% Jacobi-Matrix für Euler-Winkel
% Zur Umrechnung zwischen Euler-Winkel-Ableitung und Winkelgeschwindigkeit
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
% J [3x3]:
%   Euler-Jacobimatrix: Linearer Zusammenhang zwischen
%   Winkelgeschwindigkeit und Euler-Winkel-Zeitableitung
%   w = J * phiD

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function J = euljac(phi, conv)
%% Init
%#codegen
%$cgargs {zeros(3,1), uint8(0)}
assert(isreal(phi) && all(size(phi) == [3 1]), ...
  'euljac: phi has to be [3x1] (double)');
assert(isa(conv,'uint8') && isscalar(conv), ...
  'euljac: Number of Euler convention has to be [1x1] uint8');

%% Berechnung der Ausgabe, Fallunterscheidung der Euler-Konvention
switch conv
  case 1 % xyx
    J = eulxyxjac(phi);
  case 2 % xyz
    J = eulxyzjac(phi);
  case 3 % xzx
    J = eulxzxjac(phi);
  case 4 % xzy
    J = eulxzyjac(phi);
  case 5 % yxy
    J = eulyxyjac(phi);
  case 6 % yxz
    J = eulyxzjac(phi);
  case 7 % yzx
    J = eulyzxjac(phi);
  case 8 % yzy
    J = eulyzyjac(phi);
  case 9 % zxy
    J = eulzxyjac(phi);
  case 10 % zxz
    J = eulzxzjac(phi);
  case 11 % zyx
    J = eulzyxjac(phi);
  case 12 % zyz
    J = eulzyzjac(phi);
  otherwise
    error('euljac: conv has to be 1 to 12');
end