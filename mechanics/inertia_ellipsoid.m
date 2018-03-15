% Plot the inertia ellipsoid of a rigid body to express the inertia
% graphically
% 
% Input:
% I [3x3]
%   inertia tensor in body frame
% V [1x1]
%   Volume of the body. Set V = m/rho to accurately plot the ellipsoid
% r [1x3]
%   Center of Gravity in body frame
% T [4x4]
%   Transformation matrix from world frame to body frame
% 
% Output:
% hdl
%   handle to the created surface object
% 
% Source:
% [1] http://de.wikipedia.org/wiki/Tr%C3%A4gheitsellipsoid
% [2] https://groups.google.com/a/rethinkrobotics.com/forum/#!topic/brr-users/5X1-6w-Ja1I
% 

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-07
% (c) Institut für Regelungstechnik, Universität Hannover

function hdl = inertia_ellipsoid(I, V, r, T)
%% Init
assert(isa(I,'double') && isreal(I) && all(size(I) == [3 3]), ...
  'Inertia Tensor has to be 3x3 double');
assert(isa(V,'double') && isreal(V) && all(size(V) == [1 1]), ...
  'Scaling Factor has to be 1x1 double');      
assert(isa(r,'double') && isreal(r) && all(size(r) == [1 3]), ...
  'Center of Mass has to be 1x3 double');   
assert(isa(T,'double') && isreal(T) && all(size(T) == [4 4]), ...
  'Transformation Matrix has to be 4x4 double');  
%% Calculation
% Calculate Eigenvalues (=principal inertia moments) and Eigenvectors
% (=principal axis)
[EigVec, EigVal] = eig(I);

% Calculate lengths of semi-axis
a = ( 3/(4*pi) * V * sqrt(EigVal(2,2)*EigVal(3,3)) / EigVal(1,1) )^(1/3);
b = a*sqrt(EigVal(1,1)/EigVal(2,2));
c = a*sqrt(EigVal(1,1)/EigVal(3,3));

% Test
% V = 4*pi/3*a*b*c;
% m_test = V*rho;

% Create Ellipsoid (in principal axis frame)
n_el = 30;
[x_el_p, y_el_p, z_el_p] = ellipsoid(0,0,0, ... % Center Point
    a, b, c, ... % lengths of the semi-axis
    n_el); % number of surface grid points

% loop through all surface points
x_el_w = NaN(n_el+1, n_el+1);
y_el_w = x_el_w; z_el_w = x_el_w;

for j = 1:n_el+1
for k = 1:n_el+1
    % current surface point of the ellipse
    p = [x_el_p(j,k);   y_el_p(j,k);   z_el_p(j,k)];

    % Rotate the Elipsoids into their respective body frames from the
    % priciple axis frames
    p = EigVec*p;
    
    % Move the elipsoid to the center of gravity
    p = p + r'; % in body frame 
    
    % Transform the ellipsoid into the world frame
    p = T(1:3,:) * [p; 1];
    
    % save the ellipsoid surface point (in world frame)
    x_el_w(j,k) = p(1);
    y_el_w(j,k) = p(2);
    z_el_w(j,k) = p(3);
end
end

% plot the ellipsoid
hdl = surf(x_el_w, y_el_w, z_el_w);

