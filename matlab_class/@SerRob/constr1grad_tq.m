% Ableitung der Translationskomponente der kinematischen Zwangsbedingungen nach den Gelenkwinkeln
% Variante 1:
% * Vektor vom Basis- zum EE-KS (unterschiedlich zur Variante 1 bei PKM)
% 
% Eingabe:
% q
%   Gelenkkoordinaten des Roboters
% 
% Ausgabe:
% dPhidqJi [3xN]
%   Matrix mit Ableitungen der 3 translatorischen Zwangsbedingungskomponenten
%   (in den Zeilen) nach den N Gelenkwinkeln (in den Spalten)

% Quellen:
% [SchapplerTapOrt2019a] Schappler, M. et al.: Modeling Parallel Robot
% Kinematics for 3T2R and 3T3R Tasks using Reciprocal Sets of Euler Angles,
% MDPI Robotics KaRD2, 2019
% [A] Aufzeichnungen Schappler vom 15.06.2018
% [B] Aufzeichnungen Schappler vom 22.06.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function dPhidqJi = constr1grad_tq(Rob, q)

assert(isreal(q) && all(size(q) == [Rob.NQJ 1]), ...
  'SerRob/constr1grad_tq: q muss %dx1 sein', Rob.NQJ);

% Bein-Jacobi; siehe [SchapplerTapOrt2019a]/(A15)
J0_i_trans = Rob.jacobit(q);
dPhidqJi = J0_i_trans; % Nur xyz-Koordinate in ZB.
