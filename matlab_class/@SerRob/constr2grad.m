% Ableitung der kinematischen Zwangsbedingungen nach den Gelenkwinkeln
% Variante 2:
% * Absolute Rotation ausgedrückt in XYZ-Euler-Winkeln (entspricht PKM
%   Variante 2)
% * Rotationsfehler ausgedrückt in ZYX-Euler-Winkeln (um raumfeste Achsen)
%   (entspricht PKMVariante 2)
% 
% Eingabe:
% q
%   Gelenkkoordinaten des Roboters
% xE
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% 
% Ausgabe:
% Phi_Grad [6xN]
%   Matrix mit Ableitungen der 6 Zwangsbedingungskomponenten (auf den Zeilen)
%   nach den N Gelenkwinkeln (in den Spalten)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für mechatronische Systeme, Universität Hannover

function Phi_Grad = constr2grad(Rob, q, xE)

assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'SerRob/constr_trans: q muss %dx1 sein', Rob.NJ);

Phi_Grad = [Rob.constr1grad_tq(q); Rob.constr2grad_rq(q, xE)];