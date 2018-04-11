% Quaternion derivative
%
% QD is the rate of change of a frame with attitude Q and
% angular velocity OMEGA expressed as a quaternion.
% 
% Input:
% q [4x1]
%   Quaternion (1x1 Real, 3x1 Imaginary)
% omega [3x1]
%   Angular Velocity
% 
% Output:
% qd [4x1]
%   Quaternion time derivative
% 
% Source:
% Peter Corke Toolbox, Quaternion.m, 2015
% [ZupanSaj2011] Integrating rotation from angular velocity (2011)

function qd = quatD_angvel(q, omega)

assert(isa(q,'double') && isreal(q) && all(size(q) == [4 1]), ...
	'quatD: q = [4x1] double');   
assert(isa(omega,'double') && isreal(omega) && all(size(omega) == [3 1]), ...
	'quatD: omega = [3x1] double');

s = q(1);   % Realteil
v = q(2:4); % Imaginärteil

% [ZupanSaj2011] Gl. (19)
E = s*eye(3,3) - skew(v);
qd = [-0.5*v'*omega; 0.5*E*omega];