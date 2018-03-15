% Convert a symmetric Inertia Tensor to a Vector to save space
% 
% Input:
% I_vec [1x6]
%   Inertia Vector
%   Order: Ixx, Iyy, Izz, Ixy, Ixz, Iyz
% 
% Output:
% I_mat [3x3]
%   Inertia Tensor

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-09
% (c) Institut für Regelungstechnik, Universität Hannover

function I_mat = inertiavector2matrix(I_vec)

assert(all(size(I_vec) == [1 6]), ...
  'The stored Inertia Vector has to be [1x6]');

I_mat = [I_vec(1), I_vec(4), I_vec(5); ...
         I_vec(4), I_vec(2), I_vec(6); ...
         I_vec(5), I_vec(6), I_vec(3)];