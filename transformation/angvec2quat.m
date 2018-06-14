% convert angle/axis rotation to quaternion representation
% 
% Input:
% theta_ [1x1]
%   angle [rad]
% n_ [3x1]
%   axis
% 
% Output:
% quat [4x1]
%   quaternion orientation [4x1]. First s (angle), then v (rotation axis)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-07
% (C) Institut für Regelungstechnik, Universität Hannover

function quat = angvec2quat(theta_, n_)

%% Init
%#codegen
%#cgargs {zeros(1,1),zeros(3,1)}
assert(isreal(theta_) && all(size(theta_) == [1 1]), ...
      'angvec2quat: theta_ = [1x1] (double)');   
assert(isreal(n_) && all(size(n_) == [3 1]), ...
      'angvec2quat: n_ = [3x1] (double)');  

%% Calculation
s = cos(theta_ / 2);
v = n_ * sin(theta_/2);
quat = [s; v];

% q = Quaternion(s, v); % Matlab-Robotics-Toolbox-Class
