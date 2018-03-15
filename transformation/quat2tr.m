% convert pose vector with quaternions to transformation matrix
% 
% Input:
% x_quat [7x1]
%   pose
%   first cartesian position [1x3]
%   then quaternion orientation [1x4]. First w (angle), then xyz (rotation axis)
% 
% Output:
% T [4x4]
%   homogenous transformation matrix 

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-08
% (c) Institut für Regelungstechnik, Universität Hannover

function T = quat2tr(x_quat)

%% Init
%#codegen
assert(isa(x_quat,'double') && isreal(x_quat) && all(size(x_quat) == [7 1]), ...
      'quat2tr: x = [7x1] double');   

%% Calculation
% get displacement
xyz = x_quat(1:3);

% assemble quaternion (regard order: w, vx,vy,vz).
% This is NOT the ROS Message order
s = x_quat(4);
v = x_quat(5:7);

% calculate transformation matrix
T = [quat2r([s,v]), xyz'; ...
        0 0 0          1];