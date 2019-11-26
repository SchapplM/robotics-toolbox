% Rotationskomponente der kinematischen ZB zwischen Ist- und Soll-Konfiguration
% Vollständige Rotations- und Translationskomponenten (Zeitableitung)
% Variante 1:
% * Absolute Rotation ausgedrückt bspw. in XYZ-Euler-Winkeln
%   (statt XYZ wird die Konvention aus `phiconv_W_E` genommen)
% * Rotationsfehler ausgedrückt genauso bspw. in XYZ-Euler-Winkeln
%   Im Gegensatz zu [2_SchapplerTapOrt2019a] hier Rotation 0(q)-0(x)
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
% Phi_red
%   Reduzierte kinematische Zwangsbedingungen (siehe folgendes)
%   Die Reduktion folgt aus der Klassenvariablen I_EE
% Phi [6Mx1]
%   Kinematische Zwangsbedingungen des Roboters für alle M Beine: 
%   Maß für den Orientierungsfehler zwischen Ist-Pose aus
%   gegebenen Gelenkwinkeln q und Soll-Pose aus gegebenen EE-Koordinaten x

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [PhiDr_red, PhiDr] = constr1D_rot(Rob, q,qD, xE ,xDE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr1D_rot: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/constr1D_rot: qD muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1D_rot: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr1D_rot: xDE muss 6x1 sein');

% Differentieller Zusammenhang kann über bereits vorhandene Gradienten
% berechnet werden (ist von der Herleitung und vom Ergebnis identisch)
[Phi_rq_red, Phi_rq] = Rob.constr1grad_rq(q, xE);
[Phi_rt_red,Phi_rt] = Rob.constr1grad_rt();
[Phi_rr_red,Phi_rr] = Rob.constr1grad_rr(q, xE);

Phi_tx_red = [Phi_rt_red,Phi_rr_red];
Phi_tx = [Phi_rt, Phi_rr];

PhiDr = Phi_rq * qD + Phi_tx * xDE;
PhiDr_red = Phi_rq_red * qD + Phi_tx_red * xDE(Rob.I_EE);