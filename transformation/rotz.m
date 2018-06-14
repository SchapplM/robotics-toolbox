%ROTZ Rotation about Z axis
%
%	R = ROTZ(theta)
%
% Returns a 3x3 rotation matrix representing a rotation of theta 
% about the Z axis.
%
% See also: ROTX, ROTY, ROTVEC.

% Copyright (C) 1993-2008, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.

function r = rotz(t)
%#codegen
%#cgargs {zeros(1,1)}
assert(isreal(t) && all(size(t) == [1 1]), ...
  'Rotation angles t has to be [1x1] (double)');

	ct = cos(t);
	st = sin(t);
	r =    [ct	-st	0
		st	ct	0
		0	0	1];
