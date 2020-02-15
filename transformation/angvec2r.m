% Rotationsmatrix aus Achse-Winkel-Darstellung berechnen
% 
% Eingabe:
% theta [1x1]
%   Drehwinkel
% u [3x1]
%   Drehachse
% 
% Ausgabe:
% R [3x3]
%   Rotationsmatrix

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

function R = angvec2r(theta, u)

%% Init
%#codegen
%$cgargs {zeros(1,1),zeros(3,1)}
assert(isreal(theta) && all(size(theta) == [1 1]), ...
      'angvec2r: theta = [1x1] (double)');   
assert(isreal(u) && all(size(u) == [3 1]), ...
      'angvec2r: u = [3x1] (double)');  

%% Calculation
% Hilfsvariablen
cth = cos(theta);
sth = sin(theta);
vth = (1 - cth);
ux = u(1); uy = u(2); uz = u(3);

% Quelle: Skript Robotik I (WS 2015/16), Ortmaier, Uni Hannover, Gl. 2.38
R = [ ux^2*vth+cth       ux*uy*vth-uz*sth   ux*uz*vth+uy*sth
      ux*uy*vth+uz*sth   uy^2*vth+cth       uy*uz*vth-ux*sth
      ux*uz*vth-uy*sth   uy*uz*vth+ux*sth   uz^2*vth+cth];
