% Homogene Transformationsmatrix Rotations aus Achse-Winkel-Darstellung
% 
% Eingabe:
% theta [1x1]
%   Drehwinkel
% u [3x1]
%   Drehachse
% 
% Ausgabe:
% T [3x3]
%   Homogene Transformationsmatrix

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

function T = angvec2tr(theta, u)

%% Init
%#codegen
%$cgargs {zeros(1,1),zeros(3,1)}
assert(isreal(theta) && all(size(theta) == [1 1]));   
assert(isreal(u) && all(size(u) == [3 1]));  

%% Funktionsaufruf
T = [angvec2r(theta, u), [0;0;0]; [0 0 0 1]];
