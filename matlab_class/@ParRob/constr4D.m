% Kinematische Zwangsbedingungen zwischen Ist- und Soll-Konfiguration
% Vollständige Rotations- und Translationskomponenten (Zeitableitung)
% 
% Variante 4:
% * Bezogen auf Winkelgeschwindigkeit des Koppelpunktes Bi
%   (effektiv werden die Geschw.-ZB nach den Gelenk-Geschw. abgeleitet)
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

% Quelle:
% [A] Aufzeichnungen Schappler vom 13.02.2020

% Junnan Li, WiHi bei Moritz Schappler, 2020-05
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [PhiD_red, PhiD] = constr4D(R, q, qD, xE, xDE, platform_frame)

%% Initialisierung
assert(isreal(q) && all(size(q) == [R.NJ 1]), ...
  'ParRob/constr4D: q muss %dx1 sein', R.NJ);
assert(isreal(qD) && all(size(qD) == [R.NJ 1]), ...
  'ParRob/constr4D: qD muss %dx1 sein', R.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr4D: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr4D: xDE muss 6x1 sein');
if nargin == 5, platform_frame = false; end

% rotatorischer und translatorischer Teil der ZB
[PhiDt_red, PhiDt] = R.constr1D_trans(q, qD, xE, xDE, platform_frame); % identische Modellierung 1/4
[PhiDr_red, PhiDr] = R.constr4D_rot(q, qD, xE, xDE);

PhiD_red = NaN(size(PhiDt_red,1)+size(PhiDr_red,1), 1);
PhiD =     NaN(size(PhiDt,1)    +size(PhiDr ,1),    1);

PhiD_red(R.I_constr_t_red) = PhiDt_red;
PhiD_red(R.I_constr_r_red) = PhiDr_red;
PhiD(R.I_constr_t) = PhiDt;
PhiD(R.I_constr_r) = PhiDr;

