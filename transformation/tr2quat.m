% convert transformation matrix to pose vector with quaternions
% 
% Input:
% T [4x4]
%   homogenous transformation matrix 
% 
% Output:
% x_quat [1x7]
%   pose
%   first cartesian position [1x3]
%   then quaternion orientation [1x4]. First w (angle) 
%   then xyz (rotation axis) (mathematical Order)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-08
% (c) Institut für Regelungstechnik, Universität Hannover


function x_quat = tr2quat(T)

%% Init
%#codegen
assert(isa(T,'double') && isreal(T) && all(size(T) == [4 4]), ...
  'tr2quat: T = [4x4] double');   

%% Calculation
xyz = T(1:3,4)';

quat = r2quat(T(1:3,1:3));

x_quat = [xyz, quat];