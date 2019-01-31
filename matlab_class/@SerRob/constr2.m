% Kinematische Zwangsbedingungen zwischen Ist- und Soll-Konfiguration
% Vollständige Rotations- und Translationskomponenten
% Variante 2:
% * Vektor vom Basis- zum EE-KS (kein Unterschied zu SKM-Variante 1)
% * Absolute Rotation ausgedrückt in XYZ-Euler-Winkeln (entspricht PKM
%   Variante 2)
% * Rotationsfehler ausgedrückt in Euler-Winkeln (um raumfeste Achsen), je
%   nach Eingabeargument `reci` (entspricht teilweise PKM-Variante 2)
% 
% Eingabe:
% q
%   Gelenkwinkel des Roboters
% xE
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% reci (Optional)
%   true: Nehme reziproke Euler-Winkel für Orientierungsfehler (z.B.
%   ZYX-Orientierungsfehler für XYZ-Absolutorientierung)
%   false: Gleiche Euler-Winkel für Fehler und Absolut [Standard]
% 
% Ausgabe:
% Phi
%   Kinematische Zwangsbedingungen des Roboters: Maß für den Positions- und
%   Orientierungsfehler zwischen Ist-Pose aus gegebenen Gelenkwinkeln q und
%   Soll-Pose aus gegebenen EE-Koordinaten x

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Phi = constr2(R, q, xE, reci)
assert(isreal(q) && all(size(q) == [R.NQJ 1]), ...
  'SerRob/constr2: q muss %dx1 sein', R.NQJ);
if nargin < 4
  reci = false;
end
Phir = R.constr1_trans(q, xE); % Kein Unterschied zu Variante 1
Phip = R.constr2_rot(q, xE, reci);
Phi = [Phir; Phip];