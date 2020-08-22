% Ableitung der kinematischen Zwangsbedingungen nach den EE-Koordinaten und
% Ableitung dieser (Gradienten-)Matrix nach der Zeit
%
% Variante 3:
% * Implementierung der Rotation mit Führungs-Beinkette und Folge-Beinketten
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
% PhiD_x_red
%   Ableitung der kinematischen Zwangsbedingungen nach allen Gelenkwinkeln
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% PhiD_x [6xN]
%   Siehe vorher. Hier alle Zeilen der Zwangsbedingungen
%
% Siehe auch: constr1grad_x

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [PhiD_x_red, PhiD_x] = constr3gradD_x(Rob, q, qD, xE, xDE )

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr3gradD_x: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/constr3gradD_x: qD muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr3gradD_x: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr3gradD_x: xDE muss 6x1 sein');

%% Aufruf der Unterfunktionen
% Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
% ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)
[~, PhiD_tt]=Rob.constr2gradD_tt();
[~, PhiD_tr]=Rob.constr2gradD_tr(xE, xDE);
[~, PhiD_rt]=Rob.constr3grad_rt(); % Term und Ableitung Null.
[~, PhiD_rr]=Rob.constr3gradD_rr(q, qD, xE, xDE);

%% Sortierung der ZB-Zeilen in den Matrizen nach Beingruppen, nicht nach ZB-Art
% Initialisierung mit Fallunterscheidung für symbolische Eingabe
if ~Rob.issym
  PhiD_x =     NaN(6*Rob.NLEG,6);
else
  PhiD_x = sym('xx', [6*Rob.NLEG,6]);
  PhiD_x(:)=0;
end

%% Belegung der Ausgabe
for i = 1:Rob.NLEG
  PhiD_x((i-1)*6+1:(i)*6, :) = ...
    [PhiD_tt((i-1)*3+1:(i)*3, :), PhiD_tr((i-1)*3+1:(i)*3, :); ...
     PhiD_rt((i-1)*3+1:(i)*3, :), PhiD_rr((i-1)*3+1:(i)*3, :)];
end
% Stelle Matrix mit reduzierten ZB zusammen
PhiD_x_red = PhiD_x(Rob.I_constr_red, Rob.I_EE);