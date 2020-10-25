% Kinematische Zwangsbedingungen zwischen Ist- und Soll-Konfiguration
% Vollständige Rotations- und Translationskomponenten (Zeitableitung)
% Variante 1:
% * Vektor vom Basis- zum Koppelpunkt-KS (unterschiedlich zur Variante 1
%   bei seriellen Robotern; dort Basis bis EE)
% * Absolute Rotation ausgedrückt bspw. in XYZ-Euler-Winkeln
%   (statt XYZ wird die Konvention aus `phiconv_W_E` genommen)
% * Rotationsfehler definiert als 0x-0q ausgedrückt in XYZ-Euler-Winkeln
% 
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% qD [Nx1]
%   Geschwindigkeit aller Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% xDE [6x1]
%   Zeitableitung der Endeffektorpose des Roboters bezüglich des Basis-KS
% platform_frame [1x1 logical]
%   Benutze das Plattform-KS anstatt das EE-KS als Bezugsgröße für x
% 
% Ausgabe:
% PhiD_red
%   Reduzierte kinematische Zwangsbedingungen (Zeitableitung) (siehe folgendes)
%   Die Reduktion folgt aus der Klassenvariablen I_EE
% PhiD [6x1]
%   Kinematische Zwangsbedingungen des Roboters (Zeitableitung): Maß für
%   die Änderungsrate des Positions- und Orientierungsfehlers zwischen
%   Ist-Pose aus gegebenen Gelenkwinkeln q und Soll-Pose aus gegebenen
%   EE-Koordinaten x.
% 
% Siehe auch: constr1.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [PhiD_red, PhiD] = constr1D(R, q, qD, xE, xDE, platform_frame)

%% Initialisierung
assert(isreal(q) && all(size(q) == [R.NJ 1]), ...
  'ParRob/constr1D: q muss %dx1 sein', R.NJ);
assert(isreal(qD) && all(size(qD) == [R.NJ 1]), ...
  'ParRob/constr1D: qD muss %dx1 sein', R.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1D: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr1D: xDE muss 6x1 sein');
if nargin == 5, platform_frame = false; end

% rotatorischer und translatorischer Teil der ZB
[Phit_red_D, PhitD] = R.constr1D_trans(q, qD, xE, xDE, platform_frame);
[Phir_red_D, PhirD] = R.constr1D_rot  (q, qD, xE, xDE, platform_frame);

% Sortierung der ZB-Zeilen in den Matrizen nach Beingruppen, nicht nach ZB-Art
% Indizierung auch mit Klassenvariablen I_constr_t, I_constr_r, ...
PhiD_red = NaN(size(Phit_red_D,1)+size(Phir_red_D,1), 1);
PhiD =     NaN(size(PhitD,1)    +size(PhirD ,1),    1);
PhiD_red(R.I_constr_t_red) = Phit_red_D;
PhiD_red(R.I_constr_r_red) = Phir_red_D;
PhiD(R.I_constr_t) = PhitD;
PhiD(R.I_constr_r) = PhirD;

