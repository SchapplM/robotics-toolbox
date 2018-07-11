% convert rotation matrix to quaternion
% Algorithm based on [1]
% 
% Input:
% R [3x3]
%   rotation matrix
% 
% Output:
% quat [4x1]
%   quaternion in mathematical convention: First w (angle), then xyz
%   (rotation axis)
%
% Source 
% [1] Klumpp 1976
% [2] Grassmann: BA (2015)

% Elias Knoechelmann, schappler@irt.uni-hannover.de, 2015-01
% (c) Institut für Regelungstechnik, Universität Hannover

function q = r2quat_klumpp(R)

%% Init
%#codegen
%$cgargs {zeros(3,3)}
assert(isreal(R) && all(size(R) == [3 3]));  

%% Calculation
% [2], p.13

qw = 0.5*sqrt(R(1,1)+R(2,2)+R(3,3)+1);

qx = 0.5*sign(R(3,2)-R(2,3))*sqrt(R(1,1)-R(2,2)-R(3,3)+1);

qy = 0.5*sign(R(1,3)-R(3,1))*sqrt(-R(1,1)+R(2,2)-R(3,3)+1);

qz = 0.5*sign(R(2,1)-R(1,2))*sqrt(-R(1,1)-R(2,2)+R(3,3)+1);

q = [qw,qx,qy,qz]';

end
