% convert angle/axis rotation to quaternion representation
% 
% Input:
% theta_ [1x1]
%   angle [rad]
% n_ [1x3]
%   axis
% 
% Output:
% quat [1x4]
%   quaternion orientation [1x4]. First s (angle), then v (rotation axis)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-07
% (c) Institut für Regelungstechnik, Universität Hannover

function quat = angvec2quat(theta_, n_)

%% Init
%#codegen
assert(isa(theta_,'double') && isreal(theta_) && all(size(theta_) == [1 1]), ...
      'angvec2quat: theta_ = [1x1] double');   
assert(isa(n_,'double') && isreal(n_) && all(size(n_) == [3 1]), ...
      'angvec2quat: n_ = [3x1] double');  

%% Calculation
s = cos(theta_ / 2);
v = n_ * sin(theta_/2);
quat = [s, v];

% q = Quaternion(s, v); % Matlab-Robotics-Toolbox-Class
