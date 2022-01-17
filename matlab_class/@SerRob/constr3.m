% Kinematische Zwangsbedingungen zwischen Ist- und Soll-Konfiguration
% Vollständige Rotations- und Translationskomponenten
% Variante 3:
% * 2T2R-Aufgaben
% * Projektion der Z-Achse des aktuellen Endeffektor-KS und des Ziel-KS 
%   ergibt durch zweidimensionalen Abstand das translatorische Residuum
%
% Eingabe:
% q
%   Gelenkwinkel des Roboters
% Tr0E_soll [3x4]
%   EE-Lage (Sollwert); homogene Transformationsmatrix ohne letzte Zeile
% reci (Optional)
%   true: Nehme reziproke Euler-Winkel für Orientierungsfehler (z.B.
%   ZYX-Orientierungsfehler für XYZ-Absolutorientierung)
%   false: Gleiche Euler-Winkel für Fehler und Absolut [Standard]
% XZ_Modus [1x1 logicl] (optional)
%   true: Benutze die Projektion auf die XZ-Ebene statt die XY-Ebene
% 
% Ausgabe:
% Phi
%   Kinematische Zwangsbedingungen des Roboters: Maß für den Positions- und
%   Orientierungsfehler zwischen Ist-Pose aus gegebenen Gelenkwinkeln q und
%   Soll-Pose aus gegebenen EE-Koordinaten x

% Quelle:
% [SchapplerBluJob2022] Schappler, M. et al.: Geometric Model for Serial-
% Chain Robot Inverse Kinematics in the Case of Two Translational DoF with
% Spatial Rotation and Task Redundancy, Submitted to ARK 2022 
% [Blum2021] Blum, T.: Inverse Kinematik aufgabenredundanter Roboter für
% Aufgaben mit zwei translatorischen und zwei rotatorischen Freiheits-
% graden, Masterarbeit M-03/2021-1013

% Tobias Blum, Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function Phi = constr3(R, q, Tr0E_soll, reci, XZ_Modus)
assert(isreal(q) && all(size(q) == [R.NQJ 1]), ...
  'SerRob/constr3: q muss %dx1 sein', R.NQJ);
if nargin < 4
  reci = false;
end
if nargin < 5
  XZ_Modus = false;
end
% [SchapplerBluJob2022], Gl. 2
Phi_t = R.constr3_trans(q, Tr0E_soll, XZ_Modus);
% [SchapplerBluJob2022], Gl. 9
Phi_r = R.constr2_rot(q, Tr0E_soll, reci);
% [SchapplerBluJob2022], Text unter Gl. 9
Phi = [Phi_t; Phi_r];