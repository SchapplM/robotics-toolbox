% Ableitung der kinematischen Zwangsbedingungen nach den Gelenkwinkeln
% Variante 1:
% * Absolute Rotation ausgedrückt in XYZ-Euler-Winkeln
% * Rotationsfehler ausgedrückt in XYZ-Euler-Winkeln
%   Rotationsfehler wird als R_0_E * R_0_D angenommen (also 0(q)->0(x))
%   (anders herum als in [SchapplerTapOrt2019])
% 
% Eingabe:
% q
%   Gelenkkoordinaten des Roboters
% xE
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% 
% Ausgabe:
% Phi_Grad
%   Matrix mit Ableitungen der Zwangsbedingungskomponenten (auf den Zeilen)
%   nach den Gelenkwinkeln (in den Spalten)

% Quelle:
% [SchapplerTapOrt2019] Schappler, M. et al.: Resolution of Functional
% Redundancy for 3T2R Robot Tasks using Two Sets of Reciprocal Euler
% Angles, Proc. of the 15th IFToMM World Congress, 2019

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Phi_Grad = constr1grad(Rob, q, xE)

assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'SerRob/constr_trans: q muss %dx1 sein', Rob.NJ);

% Entspricht Gradient in [SchapplerTapOrt2019]/(23)
Phi_Grad = [Rob.constr1grad_tq(q); Rob.constr1grad_rq(q, xE)];
