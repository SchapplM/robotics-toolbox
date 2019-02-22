% Ableitung der kinematischen Zwangsbedingungen nach den Gelenkwinkeln
% Variante 2:
% * Vektor vom Basis- zum EE-KS (kein Unterschied zu SKM-Variante 1)
% * Absolute Rotation ausgedrückt in XYZ-Euler-Winkeln (entspricht PKM
%   Variante 2)
% * Rotationsfehler ausgedrückt in Euler-Winkeln (um raumfeste Achsen), je
%   nach Eingabeargument `reci` (entspricht teilweise PKM-Variante 2)
% 
% Eingabe:
% q
%   Gelenkkoordinaten des Roboters
% xE
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% reci (Optional)
%   true: Nehme reziproke Euler-Winkel für Orientierungsfehler (z.B.
%   ZYX-Orientierungsfehler für XYZ-Absolutorientierung)
%   false: Gleiche Euler-Winkel für Fehler und Absolut [Standard]
% 
% Ausgabe:
% Phi_Grad [6xN]
%   Matrix mit Ableitungen der 6 Zwangsbedingungskomponenten (auf den Zeilen)
%   nach den N Gelenkwinkeln (in den Spalten)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Phi_Grad = constr2grad(Rob, q, xE, reci)

assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'SerRob/constr_trans: q muss %dx1 sein', Rob.NJ);
if nargin < 4
  reci = false;
end

Phi_Grad = [Rob.constr1grad_tq(q); Rob.constr2grad_rq(q, xE, reci)];