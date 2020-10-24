% Kinematische Zwangsbedingungen zwischen Ist- und Soll-Konfiguration
% Vollständige Rotations- und Translationskomponenten
% Variante 2: Andere Formel und Implementierung als Variante 1:
% * Translation mit Vektor 0-E statt A-B
% * Absolute Rotation ausgedrückt bspw. in XYZ-Euler-Winkeln
%   (statt XYZ wird die Konvention aus `phiconv_W_E` genommen)
% * Rotationsfehler mit Orientierungsfehler ZYX-Rotation (bzw. reziproke
% Konvention zu absoluter Orientierung); Fehler ausgedrückt als E(q)-E(x)
% 
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
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

function [Phi_red, Phi] = constr2(R, q, xE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [R.NJ 1]), ...
  'ParRob/constr2: q muss %dx1 sein', R.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr2: xE muss 6x1 sein');

% Indizes für Reduktion der Zwangsbedingungen bei 3T2R: Nur für
% symmetrische 3T2R-PKM
I_constr_red = 1:6*R.NLEG;
if R.NJ == 25 % Behelf zur Erkennung symmetrischer 3T2R-PKM
  I_constr_red(4:6:end) = []; % entspricht z-Euler-Winkel
end

% rotatorischer und translatorischer Teil der ZB
[~, Phit] = R.constr2_trans(q, xE);
[~, Phir] = R.constr2_rot(q, xE);

% Sortierung der ZB-Zeilen in den Matrizen nach Beingruppen, nicht nach ZB-Art
Phi = NaN(6*R.NLEG, 1);
for i = 1:R.NLEG
  Phi((i-1)*6+1:(i)*6, :) = ...
    [Phit((i-1)*3+1:(i)*3, :); ...
     Phir((i-1)*3+1:(i)*3, :)];
end
Phi_red = Phi(I_constr_red);