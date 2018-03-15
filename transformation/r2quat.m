% convert rotation matrix to quaternion
% This is robust against singularities but can cause switching between
% different equivalent quaternions with the same rotation. Not suitable for
% controllers!
% 
% Input:
% R [3x3]
%   rotation matrix
% 
% Output:
% q [4x1]
%   quaternion in mathematical convention: First w (angle), then xyz
%   (rotation axis)
%
% Source 
% [1] http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
%   basiert auf Shepperd 1978
% [2] Grassmann: BA (2015).

% Elias Knoechelmann, schappler@irt.uni-hannover.de, 2015-01
% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-08
% (c) Institut für Regelungstechnik, Universität Hannover

function q = r2quat(R)

%% Init
%#codegen
assert(isa(R,'double') && isreal(R) && all(size(R) == [3 3]), ...
  'r2quat: R = [3x3] double');

%% Calculation
tr = R(1,1) + R(2,2) + R(3,3);

if (tr > 0)
  % analog zu [2], equ. (3.14)
  S = sqrt(tr+1.0) * 2; % S=4*qw
  qw = 0.25 * S;
  qx = (R(3,2) - R(2,3)) / S;
  qy = (R(1,3) - R(3,1)) / S;
  qz = (R(2,1) - R(1,2)) / S;
elseif ((R(1,1) > R(2,2))&&(R(1,1) > R(3,3)))
  % analog zu [2], equ. (3.15)
  S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2; % S=4*qx
  qw = (R(3,2) - R(2,3)) / S;
  qx = 0.25 * S;
  qy = (R(1,2) + R(2,1)) / S;
  qz = (R(1,3) + R(3,1)) / S;
elseif (R(2,2) > R(3,3))
  % analog zu [2], equ. (3.16)
  S = sqrt(1.0 - R(1,1) + R(2,2) - R(3,3)) * 2; % S=4*qy
  qw = (R(1,3) - R(3,1)) / S;
  qx = (R(1,2) + R(2,1)) / S;
  qy = 0.25 * S;
  qz = (R(2,3) + R(3,2)) / S;
else
  % analog zu [2], equ. (3.17)
  S = sqrt(1.0 -R(1,1) - R(2,2) + R(3,3)) * 2; % S=4*qz
  qw = (R(2,1) - R(1,2)) / S;
  qx = (R(1,3) + R(3,1)) / S;
  qy = (R(2,3) + R(3,2)) / S;
  qz = 0.25 * S;
end
q = [qw,qx,qy,qz]';

end
