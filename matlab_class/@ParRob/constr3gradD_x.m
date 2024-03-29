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
% platform_frame [1x1 logical]
%   Benutze das Plattform-KS anstatt das EE-KS als Bezugsgröße für x
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

function [PhiD_x_red, PhiD_x] = constr3gradD_x(Rob, q, qD, xE, xDE, platform_frame)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr3gradD_x: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/constr3gradD_x: qD muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr3gradD_x: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr3gradD_x: xDE muss 6x1 sein');
if nargin == 5, platform_frame = false; end
%% Aufruf der Unterfunktionen
% Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
% ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)
[PhiD_tt_red, PhiD_tt]=Rob.constr2gradD_tt();
[PhiD_tr_red, PhiD_tr]=Rob.constr2gradD_tr();
[PhiD_rt_red, PhiD_rt]=Rob.constr3grad_rt(); % Term und Ableitung Null.
[PhiD_rr_red, PhiD_rr]=Rob.constr3gradD_rr(q, qD, xE, xDE, platform_frame);

%% Sortierung der ZB-Zeilen in den Matrizen nach Beingruppen, nicht nach ZB-Art
% Initialisierung mit Fallunterscheidung für symbolische Eingabe
dim_Pq_red=[size(PhiD_tt_red, 1)+size(PhiD_rt_red, 1), ...
  size(PhiD_tt_red, 2)+size(PhiD_tr_red, 2)];
if ~Rob.issym
  PhiD_x =     NaN(6*Rob.NLEG,6);
  PhiD_x_red = NaN(dim_Pq_red);
else
  PhiD_x = sym('xx', [6*Rob.NLEG,6]);
  PhiD_x(:)=0;
  PhiD_x_red = sym('xx', dim_Pq_red);
  PhiD_x_red(:)=0;
end
Leg_I_EE_Task = cat(1,Rob.Leg(:).I_EE_Task);
nPhit = sum(Rob.I_EE_Task(1:3));
K1 = 1;
J1 = 1;
%% Belegung der Ausgabe
for i = 1:Rob.NLEG
  % Zuordnung der reduzierten Zwangsbedingungen zu den Beinketten
  K2 = K1+sum(Leg_I_EE_Task(i,4:6))-1;
  % Eintrag für Beinkette zusammenstellen
  PhiD_tt_red_i = PhiD_tt_red((i-1)*nPhit+1:(i)*nPhit, :);
  PhiD_tr_red_i = PhiD_tr_red((i-1)*nPhit+1:(i)*nPhit, :);
  PhiD_rt_red_i = PhiD_rt_red(K1:K2,:);
  PhiD_rr_red_i = PhiD_rr_red(K1:K2,:);
  PhiD_x_red_i = [PhiD_tt_red_i, PhiD_tr_red_i; PhiD_rt_red_i, PhiD_rr_red_i];
  PhiD_x_red(J1:J1+size(PhiD_x_red_i,1)-1,:) = PhiD_x_red_i;
  J1 = J1 + size(PhiD_x_red_i,1);
  K1 = K2+1;
  PhiD_x((i-1)*6+1:(i)*6, :) = ...
    [PhiD_tt((i-1)*3+1:(i)*3, :), PhiD_tr((i-1)*3+1:(i)*3, :); ...
     PhiD_rt((i-1)*3+1:(i)*3, :), PhiD_rr((i-1)*3+1:(i)*3, :)];
end
% Alternative: Stelle Matrix mit reduzierten ZB direkt zusammen
% PhiD_x_red = PhiD_x(Rob.I_constr_red, Rob.I_EE);