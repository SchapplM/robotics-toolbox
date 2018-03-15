% Convert Quaternion Rotation Representation to Rotation Matrix
% 
% Input:
% quat [1x4]
%   Quaternion Orientation:
%   First w (angle), then xyz (rotation axis)
% 
% Output:
% R [3x3]
%   rotation matrix 
% 
% Sources:
% [1] Tobias Ortmeier: Robotik I Vorlesungsskript
% [2] http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Conversion_to_and_from_the_matrix_representation

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-07
% (c) Institut für Regelungstechnik, Universität Hannover

function R = quat2r(quat)
%% Init
assert(isa(quat,'double') && isreal(quat) && all(size(quat) == [4 1]), ...
  'quat2r: Input Quaternion has to be [4x1] double');

%% Calculation
a = quat(1); % Angle (Real Part)
b = quat(2); % Axis (i)
c = quat(3); % Axis (j)
d = quat(4); % Axis (k)

R = [a^2+b^2-c^2-d^2,   2*b*c-2*a*d,        2*b*d+2*a*c; ...
     2*b*c+2*a*d,       a^2-b^2+c^2-d^2,    2*c*d-2*a*b; ...
     2*b*d-2*a*c,       2*c*d+2*a*b,        a^2-b^2-c^2+d^2];