% Ableitung der kinematischen Zwangsbedingungen nach den Gelenkwinkeln
% Variante 1:
% * Absolute Rotation ausgedrückt in XYZ-Euler-Winkeln
% * Rotationsfehler ausgedrückt in XYZ-Euler-Winkeln
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

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für mechatronische Systeme, Universität Hannover

function Phi_Grad = constr1grad(Rob, q, xE)

assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'SerRob/constr_trans: q muss %dx1 sein', Rob.NJ);


Phi_Grad = [Rob.constr1grad_tq(q); Rob.constr1grad_rq(q, xE)];
