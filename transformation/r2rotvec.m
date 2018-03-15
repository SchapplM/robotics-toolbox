% Konvertiere eine Rotationsdarstellung von Rotationsmatrix zu Rotationsvektor (aus
% [ZupanSaj2011])
% 
% Quelle:
% [ZupanSaj2011] Integrating rotation from angular velocity , Kap. 3.1

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-02
% (c) Institut für Regelungstechnik, Universität Hannover

function rotvec = r2rotvec(R)

[theta, n] = r2angvec(R); % Drehwinkel und Drehachse (Einheitsvektor)
if theta == 0
  rotvec = [0;0;0];
else
  rotvec = n' * theta;
end