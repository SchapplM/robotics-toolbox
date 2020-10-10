% Kinematische Zwangsbedingungen zwischen Ist- und Soll-Konfiguration
% Vollständige Rotations- und Translationskomponenten (Zeitableitung)
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
% qD [Nx1]
%   Geschwindigkeit aller Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% xDE [6x1]
%   Zeitableitung der Endeffektorpose des Roboters bezüglich des Basis-KS
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

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [PhiD_red, PhiD] = constr2D(R, q, qD, xE, xDE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [R.NJ 1]), ...
  'ParRob/constr2D: q muss %dx1 sein', R.NJ);
assert(isreal(qD) && all(size(qD) == [R.NJ 1]), ...
  'ParRob/constr2D: qD muss %dx1 sein', R.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr2D: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr2D: xDE muss 6x1 sein');

[~, Phi_q] = R.constr2grad_q(q, xE);
[~, Phi_x] = R.constr2grad_x(q, xE);
PhiD = Phi_q*qD + Phi_x*xDE;
% TODO: Hier richtig machen:
PhiD_red = PhiD; % Phi_q_red*qD + Phi_x_red*xDE;
