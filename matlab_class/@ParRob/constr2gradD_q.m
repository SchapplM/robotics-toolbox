% Ableitung der kinematischen Zwangsbedingungen nach den Gelenkwinkeln
% Ableitung dieser (Gradienten-)Matrix nach der Zeit
% Bezeichnungen: 
% * Jacobi-Matrix der inversen Kinematik, 
% * geometrische Matrix der inversen Kinematik
% 
% Variante 2:
% * Absolute Rotation ausgedrückt in XYZ-Euler-Winkeln
% * Rotationsfehler ausgedrückt in ZYX-Euler-Winkeln (und als E(q)-E(x)
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
% PhiD_q_red
%   Ableitung der kinematischen Zwangsbedingungen nach allen Gelenkwinkeln
%   und der Zeit.
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% PhiD_q [6xN]
%   Siehe vorher. Hier alle Zeilen der Zwangsbedingungen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [PhiD_q_red, PhiD_q] = constr2gradD_q(Rob, q, qD, xE, xDE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr2gradD_q: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/constr2gradD_q: qD muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr2gradD_q: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr2gradD_q: xDE muss 6x1 sein');

% Indizes für Reduktion der Zwangsbedingungen bei 3T2R: Nur für
% symmetrische 3T2R-PKM
I_constr_red = 1:6*Rob.NLEG;
if Rob.NJ == 25 % Behelf zur Erkennung symmetrischer 3T2R-PKM
  I_constr_red(4:6:end) = []; % entspricht z-Euler-Winkel
end
%% Aufruf der Unterfunktionen
% Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
% ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)
[~,PhiD_tq] = Rob.constr2gradD_tq(q, qD);
[~,PhiD_rq] = Rob.constr2gradD_rq(q, qD, xE, xDE);

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
% Sortierung der ZB-Zeilen in den Matrizen nach Beingruppen, nicht nach ZB-Art
if ~Rob.issym
  PhiD_q = NaN(6*Rob.NLEG, Rob.NJ);
else
  PhiD_q = sym('xx', [6*Rob.NLEG, Rob.NJ]);
  PhiD_q(:)=0;
end

%% Belegung der Ausgabe
% Entspricht [2_SchapplerTapOrt2019a]/(32)
for i = 1:Rob.NLEG
  PhiD_q((i-1)*6+1:(i)*6, :) = ...
    [PhiD_tq((i-1)*3+1:(i)*3, :); ...
     PhiD_rq((i-1)*3+1:(i)*3, :)];
end
PhiD_q_red = PhiD_q(I_constr_red, :);
