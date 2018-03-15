% Umwandlung von Rotationsmatrix nach Achse-Winkel-Konvention
% 
% Eingabe:
% R [3x3]
%   Rotationsmatrix
% [theta, n]
%   Achse-Winkel-Darstellung der Rotation
% 
% Quelle:
% Skript Robotik I
 
% Moritz Schappler, schappler@irt.uni-hannover.de, 2017-11
% (c) Institut für Regelungstechnik, Universität Hannover

function [theta, n] = r2angvec(R)
  %#codegen
  assert(isa(R,'double') && isreal(R) && all(size(R) == [3 3]), ...
    'r2angvec: R = [3x3] double');   
  
  % [Robotik I, Gl. (2.39)]
  cos_theta = 0.5*(R(1,1)+R(2,2)+R(3,3)-1);
  if cos_theta == 1
    theta = 0;
    n = [0;0;1];
  else
    % [Robotik I, Gl. (2.42)]
    sin_theta = 0.5*sqrt((R(3,2)-R(2,3))^2+(R(1,3)-R(3,1))^2+(R(2,1)-R(1,2))^2);
    % [Robotik I, Gl. (2.43)]
    theta = atan2(sin_theta, cos_theta);
    % [Robotik I, Gl. (2.44)]
    n = 1/(2*sin_theta) * [(R(3,2)-R(2,3)); (R(1,3)-R(3,1)); (R(2,1)-R(1,2))];
  end


