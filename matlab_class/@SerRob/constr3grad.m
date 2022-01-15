% Ableitung der kinematischen Zwangsbedingungen nach den Gelenkwinkeln
% Variante 3:
% * 2T2R-Aufgaben
% * Projektion der Z-Achse des aktuellen Endeffektor-KS und des Ziel-KS 
%   ergibt durch zweidimensionalen Abstand das translatorische Residuum
% 
% Eingabe:
% q
%   Gelenkkoordinaten des Roboters
% Tr0E_soll [3x4]
%   EE-Lage (Sollwert); homogene Transformationsmatrix ohne letzte Zeile
% reci (Optional)
%   true: Nehme reziproke Euler-Winkel für Orientierungsfehler (z.B.
%   ZYX-Orientierungsfehler für XYZ-Absolutorientierung)
%   false: Gleiche Euler-Winkel für Fehler und Absolut [Standard]
% XZ_Modus [1x1 logical] (optional)
%   true: Benutze die Projektion auf die XZ-Ebene statt die XY-Ebene
% 
% Ausgabe:
% Phi_Grad [6xN]
%   Matrix mit Ableitungen der 6 Zwangsbedingungskomponenten (auf den Zeilen)
%   nach den N Gelenkwinkeln (in den Spalten)

% Quellen:
% [SchapplerBluJob2022] Schappler, M. et al.: Geometric Model for Serial-
% Chain Robot Inverse Kinematics in the Case of Two Translational DoF with
% Spatial Rotation and Task Redundancy, Submitted to ARK 2022 
% [Blum2021] Blum, T.: Inverse Kinematik aufgabenredundanter Roboter für
% Aufgaben mit zwei translatorischen und zwei rotatorischen Freiheits-
% graden, Masterarbeit M-03/2021-1013
% [SchapplerTapOrt2019] Schappler, M. et al.: Resolution of Functional
% Redundancy for 3T2R Robot Tasks using Two Sets of Reciprocal Euler
% Angles, Proc. of the 15th IFToMM World Congress, 2019

% Tobias Blum, Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function Phi_Grad = constr3grad(Rob, q, Tr0E_soll, reci, XZ_Modus)

assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'SerRob/constr3grad: q muss %dx1 sein', Rob.NJ);
assert(isreal(Tr0E_soll) && all(size(Tr0E_soll) == [3 4]), ...
  'SerRob/constr3grad: Tr0E_soll muss 3x4 sein');
if nargin < 4
  reci = false;
end
if nargin < 5
  XZ_Modus = false;
end
Phi_Grad = [ Rob.constr3grad_tq(q, XZ_Modus);
             Rob.constr2grad_rq(q, Tr0E_soll, reci)];
