% Konvertiere eine Rotationsdarstellung von Rotationsmatrix zu Rotationsvektor 
% (aus [ZupanSaj2011])
% 
% Eingabe:
% R [3x3]
%   Rotationsmatrix
% Ausgabe:
% rotvec [3x1]
%   Rotationsvektor (Achse x Winkel aus Achse-Winkel-Darstellung
% 
% Quelle:
% [ZupanSaj2011] Integrating rotation from angular velocity , Kap. 3.1

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-02
% (C) Institut für Regelungstechnik, Universität Hannover

function rotvec = r2rotvec(R)

%#codegen
assert(isa(R,'double') && isreal(R) && all(size(R) == [3 3]), ...
	'r2rotvec: R = [3x3] double');   

[theta, n] = r2angvec(R); % Drehwinkel und Drehachse (Einheitsvektor)
if theta == 0
  rotvec = [0;0;0];
else
  rotvec = n * theta;
end