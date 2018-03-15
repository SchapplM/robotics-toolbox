% Konvertiere eine Rotationsdarstellung von Rotationsvektor (aus
% [ZupanSaj2011]) in eine Rotationsmatrix
% 
% Quelle:
% [ZupanSaj2011] Integrating rotation from angular velocity , Kap. 3.1

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-02
% (c) Institut für Regelungstechnik, Universität Hannover

function R = rotvec2r(rotvec)

assert(isa(rotvec,'double') && isreal(rotvec) && all(size(rotvec) == [3 1]), ...
      'rotvec2r: rotvec = [3x1] double');  
    
theta = norm(rotvec);
if theta == 0
  R = eye(3);
else
  k = rotvec / theta; % Einheitsvektor, um den rotiert wird
  R = angvec2r(theta, k);
end