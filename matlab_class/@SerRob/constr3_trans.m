% Translatorische Komponente der Zwangsbedingungen
% Variante 3:
% * 2T2R-Aufgaben
% * Projektion der Z-Achse des aktuellen Endeffektor-KS und des Ziel-KS 
%   ergibt durch zweidimensionalen Abstand das translatorische Residuum
% 
% Eingabe:
% qJ
%   Gelenkkoordinaten des Roboters
% Tr0E_soll [3x4]
%   EE-Lage (Sollwert); homogene Transformationsmatrix ohne letzte Zeile
% XZ_Modus [1x1 logical] (optional)
%   true: Benutze die Projektion auf die XZ-Ebene statt die XY-Ebene
% 
% Ausgabe:
% Phit [3x1]
%   Kinematische Zwangsbedingungen des Roboters: Maß für den Positions- und
%   Orientierungsfehler zwischen Ist-Pose aus gegebenen Gelenkwinkeln q und
%   Soll-Pose aus gegebenen EE-Koordinaten x. Translatorischer Teil.

% Quellen:
% [SchapplerBluJob2022] Schappler, M. et al.: Geometric Model for Serial-
% Chain Robot Inverse Kinematics in the Case of Two Translational DoF with
% Spatial Rotation and Task Redundancy, Submitted to ARK 2022 
% [Blum2021] Blum, T.: Inverse Kinematik aufgabenredundanter Roboter für
% Aufgaben mit zwei translatorischen und zwei rotatorischen Freiheits-
% graden, Masterarbeit M-03/2021-1013

% Tobias Blum, Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function Phit = constr3_trans(Rob, qJ, Tr0E_soll, XZ_Modus)

assert(isreal(qJ) && all(size(qJ) == [Rob.NQJ 1]), ...
  'SerRob/constr3_trans: q muss %dx1 sein', Rob.NQJ);
assert(isreal(Tr0E_soll) && all(size(Tr0E_soll) == [3 4]), ...
  'SerRob/constr3_trans: Tr0E_soll muss 3x4 sein');
if nargin < 4
  XZ_Modus = false;
end
T_0_E = Rob.fkineEE(qJ);

%% Schnittpunkt Z-Achse/XY-Ebene des KS0
% Zielpunkt D
% Rotationsmatrix aus Transformationsmatrix zusammenstellen
R_0_D = Tr0E_soll(1:3,1:3);
% Translationsvektor aus Transformationsmatrix zusammenstellen
t_0_D = Tr0E_soll(1:3,4);
% Einheitsvektor der z-Achse aus Rotationsmatrix bestimmen
e_z_OD = R_0_D(:,3);
% Schnittpunkt EE-Z-Achse mit KS0-XY-Ebene durch Lösen eines GLS
% Allgemeines Vorgehen: SchapplerBluJob2022, Gl. 3
% g_0E_PoseE = t_0E_PoseE + lambda_PoseE*e_z_OE   [Blum2021]
g_0D = zeros(3,1);
if XZ_Modus == false
  lambdaD = (-t_0_D(3))/e_z_OD(3); % SchapplerBluJob2022, Gl. 4
  g_0D(1) = t_0_D(1) + lambdaD*e_z_OD(1);  
  g_0D(2) = t_0_D(2) + lambdaD*e_z_OD(2);
  g_0D(3) = 0;
else % XZ_Modus == true
  lambdaD = (-t_0_D(2))/e_z_OD(2);   
  g_0D(1) = t_0_D(1) + lambdaD*e_z_OD(1);  
  g_0D(2) = 0;
  g_0D(3) = t_0_D(3) + lambdaD*e_z_OD(3);
end
p_intersectD = g_0D;

% Aktueller Punkt E
R_0E = T_0_E(1:3,1:3);
t_0E = T_0_E(1:3,4);
e_z_OE = R_0E(:,3);
g_0E = zeros(3,1);
if XZ_Modus == false
  lambdaE = (-t_0E(3))/e_z_OE(3);
  g_0E(1) = t_0E(1) + lambdaE*e_z_OE(1);  
  g_0E(2) = t_0E(2) + lambdaE*e_z_OE(2);
  g_0E(3) = 0;
else % XZ_Modus == true
  lambdaE = (-t_0E(2))/e_z_OE(2);   
  g_0E(1) = t_0E(1) + lambdaE*e_z_OE(1);  
  g_0E(2) = 0;
  g_0E(3) = t_0E(3) + lambdaE*e_z_OE(3);
end
p_intersectE = g_0E;

% Residuum berechnen
Phit = - p_intersectD + p_intersectE;
