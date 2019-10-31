% Translationskomponente der kinematischen ZB zwischen Ist- und
% Soll-Konfiguration (Zeitableitung)
% Variante 1:
% * Vektor vom Basis- zum Koppelpunkt-KS (unterschiedlich zur Variante 1
%   bei seriellen Robotern; dort Basis bis EE)
% * Entspricht Standard-Modellierung für PKM. Siehe [2_SchapplerTapOrt2019a],
%   Kap. 2; "Parallel Robots" (Merlet); Robotik 2 Skript
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
% PhiD [6Mx1]
%   Kinematische Zwangsbedingungen des Roboters für alle M Beine (Zeitableitung): 
%   Maß für den Positionsfehler zwischen Ist-Pose aus
%   gegebenen Gelenkwinkeln q und Soll-Pose aus gegebenen EE-Koordinaten x

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [PhiDt_red, PhiDt] = constr1D_trans(Rob, q, qD, xE ,xDE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr1D_trans: q muss %dx1 sein', Rob.NJ);
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr1D_trans: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1D_trans: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr1D_trans: xDE muss 6x1 sein');

% Differentieller Zusammenhang kann über bereits vorhandene Gradienten
% berechnet werden (ist von der Herleitung und vom Ergebnis identisch)
[Phi_tq_red, Phi_tq] = Rob.constr1grad_tq(q);
[Phi_tt_red,Phi_tt] = Rob.constr1grad_tt();
[Phi_tr_red,Phi_tr] = Rob.constr1grad_tr(xE);

Phi_tx_red = [Phi_tt_red,Phi_tr_red];
Phi_tx = [Phi_tt, Phi_tr];

PhiDt = Phi_tq * qD + Phi_tx * xDE;
PhiDt_red = Phi_tq_red * qD + Phi_tx_red * xDE(Rob.I_EE);
