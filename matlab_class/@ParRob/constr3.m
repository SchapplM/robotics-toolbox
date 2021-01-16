% Kinematische Zwangsbedingungen zwischen Ist- und Soll-Konfiguration
% Vollständige Rotations- und Translationskomponenten
% Variante 3: Andere Implementierung als Variante 2:
% * Implementierung mit Führungs-Beinkette und Folge-Beinketten
% * Translation mit Vektor 0-E statt A-B (im Gegensatz zu Var. 1)
% * Absolute Rotation ausgedrückt bspw. in XYZ-Euler-Winkeln
%   (statt XYZ wird die Konvention aus `phiconv_W_E` genommen)
% * Rotationsfehler mit Orientierungsfehler ZYX-Rotation um festes KS
%   (Linksmultiplikation)
% 
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% platform_frame [1x1 logical]
%   Benutze das Plattform-KS anstatt das EE-KS als Bezugsgröße für x
% 
% Ausgabe:
% Phi_red
%   Reduzierte kinematische Zwangsbedingungen (siehe folgendes)
%   Die Reduktion folgt aus der Klassenvariablen I_EE
% Phi [6x1]
%   Kinematische Zwangsbedingungen des Roboters: Maß für den Positions- und
%   Orientierungsfehler zwischen Ist-Pose aus gegebenen Gelenkwinkeln q und
%   Soll-Pose aus gegebenen EE-Koordinaten x

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phi_red, Phi] = constr3(R, q, xE, platform_frame)

%% Initialisierung
assert(isreal(q) && all(size(q) == [R.NJ 1]), ...
  'ParRob/constr3: q muss %dx1 sein', R.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr3: xE muss 6x1 sein');
if nargin == 3, platform_frame = false; end

% rotatorischer und translatorischer Teil der ZB
[Phit_red, Phit] = R.constr2_trans(q, xE, platform_frame); % Translation identisch mit Var. 2
[Phir_red, Phir] = R.constr3_rot(q, xE, platform_frame);

% Sortierung der ZB-Zeilen in den Matrizen nach Beingruppen, nicht nach ZB-Art
Phi_red = NaN(size(Phit_red,1)+size(Phir_red,1), 1);
Phi =     NaN(size(Phit,1)    +size(Phir ,1),    1);
Phi_red(R.I_constr_t_red') = Phit_red;
Phi_red(R.I_constr_r_red') = Phir_red;

for i = 1:R.NLEG
  Phi((i-1)*6+1:(i)*6, :) = ...
    [Phit((i-1)*3+1:(i)*3, :); ...
     Phir((i-1)*3+1:(i)*3, :)];
end
