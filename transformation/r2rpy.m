% Rotationsmatrix in RPY-Winkelkonvention umwandeln
% 
% Input:
% R:
%   [3x3] Rotationsmatrix
% 
% Output:
% rpy:
%   [1x3] rpy-Winkel
%   Definition: R = rotx(rpy(1))*roty(rpy(2))*rotz(rpy(3));

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-08
% (c) Institut für Regelungstechnik, Universität Hannover

function rpy = r2rpy(R)
%% Init
%#codegen
assert(isa(R,'double') && isreal(R) && all(size(R) == [3 3]), ...
  'r2rpy: R has to be [3x3] double');  

%% Calculation
rpy = tr2rpy(R);