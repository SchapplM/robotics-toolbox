% Zeitableitung der Jacobi-Matrix für Euler-Winkel
% Zur Umrechnung zwischen zweiter Euler-Winkel-Ableitung und
% Winkelbeschleunigung
% 
% Eingabe:
% phi [3x1]
%   Satz Euler-Winkel
% phiD [3x1]
%   Zeitableitung der Euler-Winkel
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
% JD [3x3]:
%   Zeitableitung der Euler-Jacobimatrix: Zusammenhang zwischen
%   Winkelgeschwindigkeit und Euler-Winkel-Zeitableitung
%   wD = J * phiDD + JD * phiD

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function J = euljacD(phi, phiD, conv)
%% Init
%#codegen
%$cgargs {zeros(3,1), zeros(3,1), uint8(0)}
assert(isreal(phi) && all(size(phi) == [3 1]), ...
  'euljacD: phi has to be [3x1] (double)');
assert(isreal(phiD) && all(size(phiD) == [3 1]), ...
  'euljacD: phiD has to be [3x1] (double)');
assert(isa(conv,'uint8') && isscalar(conv), ...
  'euljacD: Number of Euler convention has to be [1x1] uint8');

%% Berechnung der Ausgabe, Fallunterscheidung der Euler-Konvention
switch conv
  case 1 % xyx
    J = eulxyxjacD(phi, phiD);
  case 2 % xyz
    J = eulxyzjacD(phi, phiD);
  case 3 % xzx
    J = eulxzxjacD(phi, phiD);
  case 4 % xzy
    J = eulxzyjacD(phi, phiD);
  case 5 % yxy
    J = eulyxyjacD(phi, phiD);
  case 6 % yxz
    J = eulyxzjacD(phi, phiD);
  case 7 % yzx
    J = eulyzxjacD(phi, phiD);
  case 8 % yzy
    J = eulyzyjacD(phi, phiD);
  case 9 % zxy
    J = eulzxyjacD(phi, phiD);
  case 10 % zxz
    J = eulzxzjacD(phi, phiD);
  case 11 % zyx
    J = eulzyxjacD(phi, phiD);
  case 12 % zyz
    J = eulzyzjacD(phi, phiD);
  otherwise
    error('euljacD: conv has to be 1 to 12');
end