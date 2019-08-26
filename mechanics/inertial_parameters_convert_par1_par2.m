% Convert dynamic parameters for from parameter set 1 to set 2
% 
% set 1:
%   Mass, Center of Mass, Inertia around CoM ("barycentric parameters")
% set 2:
%   Mass, First Moment, Inertia round Frame Origin ("inertial parameters")
% 
% Input:
% mges [Nx1 double]
%   masses of N bodies
% rSges [Nx3 double]
%   center of mass of N bodies
% ISges [Nx6 double]
%   inertia of all N bodies around their center of mass
%   order: xx, yy, zz, xy, xz, yz
% 
% Output:
% mrSges [Nx3 double]
%   first moment of all N bodies
% Ifges [Nx6 double]
%   inertia of all N bodies around the frame origin
%   order: xx, yy, zz, xy, xz, yz

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-02
% (c) Institut für Regelungstechnik, Universität Hannover


function [mrSges, Ifges] = ...
  inertial_parameters_convert_par1_par2(rSges, ISges, mges)
assert(size(rSges,2) == 3, 'Schwerpunkte müssen zeilenweise vorliegen: Nx3');
assert(size(ISges,2) == 6, 'Trägheitstensoren müssen zeilenweise vorliegen: Nx6');

N = size(rSges,1);
%% Erstes Moment
mrSges = NaN(N,3);
for i = 1:N
  mrSges(i,:) = mges(i) * rSges(i,:);
end

%% Trägheitstensor um das Körperkoordinatensystem
Ifges = NaN(N,6);
for i = 1:N
  % Trägheitstensor um den Schwerpunkt
  I_iSi = inertiavector2matrix(ISges(i,:));
  
  % Steinerscher-Verschiebungssatz: Trägheitstensor um Koordinatensystem
  I_ii = inertia_steiner(I_iSi, rSges(i,:)', mges(i));
  
  % Ausgabe zusammenstellen
  Ifges(i,:) = inertiamatrix2vector(I_ii);
end
