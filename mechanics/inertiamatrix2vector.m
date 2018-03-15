% Convert an Inertia Tensor stored in a vector back to a matrix
% 
% Input:
% I_mat [3x3]
%   Inertia Tensor
% 
% Output:
% I_vec [1x6]
%   Inertia Vector
%   Order: Ixx, Iyy, Izz, Ixy, Ixz, Iyz

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-09
% (c) Institut für Regelungstechnik, Universität Hannover

function I_vec = inertiamatrix2vector(I_mat)

assert(all(size(I_mat) == [3 3]), ...
  'The Inertia Tensor has to be [3x3]');

I_vec = [I_mat(1,1), I_mat(2,2), I_mat(3,3), ...
  I_mat(1,2), I_mat(1,3), I_mat(2,3)];
