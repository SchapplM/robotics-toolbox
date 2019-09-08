% Convert dynamic parameters for from parameter set 2 to set 1
% 
% set 2:
%   Mass, First Moment, Inertia around Frame Origin ("inertial parameters")
% set 1:
%   Mass, Center of Mass, Inertia around CoM ("barycentric parameters")
% 
% Input:
% mges [Nx1 double]
%   masses of N bodies
% mrSges [Nx3 double]
%   first moment of all N bodies
% Ifges [Nx6 double]
%   inertia of all N bodies around the frame origin
%   order: xx, yy, zz, xy, xz, yz
% 
% Output:
% rSges [Nx3 double]
%   center of mass of N bodies
% ISges [Nx6 double]
%   inertia of all N bodies around their center of mass
%   order: xx, yy, zz, xy, xz, yz
% 
% See also:
% inertial_parameters_convert_par1_par2.m, inertia_steiner.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover


function [rSges, ISges] = ...
  inertial_parameters_convert_par2_par1(mrSges, Ifges, mges)
assert(size(mrSges,2) == 3, 'Erstes Moment muss zeilenweise vorliegen: Nx3');
assert(size(Ifges,2) == 6, 'Zweites Moment muss zeilenweise vorliegen: Nx6');

N = size(mrSges,1);
%% Schwerpunkte rekonstruieren
rSges = NaN(N,3);
for i = 1:N
  rSges(i,:) = mrSges(i,:)/mges(i);
end
rSges(mges==0, :) = 0;

%% Trägheitstensor um den Schwerpunkt
ISges = NaN(N,6);
for i = 1:N
  % Trägheitstensor um den Ursprung
  I_ii = inertiavector2matrix(Ifges(i,:));
  
  % Steiner'schen-Verschiebungssatz rückgängig machen
  I_iSi = I_ii - mges(i)*skew(rSges(i,:))'*skew(rSges(i,:));
  
  % Ausgabe zusammenstellen
  ISges(i,:) = inertiamatrix2vector(I_iSi);
end
