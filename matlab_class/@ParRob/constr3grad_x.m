% Ableitung der kinematischen Zwangsbedingungen nach den EE-Koordinaten
% Bezeichnungen: 
% * Jacobi-Matrix der direkten Kinematik, 
% * geometrische Matrix der direkten Kinematik
% 
% Variante 3:
% * Implementierung der Rotation mit F�hrungs-Beinkette und Folge-Beinketten
% 
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bez�glich des Basis-KS
% platform_frame [1x1 logical]
%   Benutze das Plattform-KS anstatt das EE-KS als Bezugsgr��e f�r x
% 
% Ausgabe:
% Phi_x_red
%   Ableitung der kinematischen Zwangsbedingungen nach allen Gelenkwinkeln
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% Phi_x [6xN]
%   Siehe vorher. Hier alle Zeilen der Zwangsbedingungen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut f�r Mechatronische Systeme, Universit�t Hannover

function [Phi_x_red, Phi_x] = constr3grad_x(Rob, q, xE, platform_frame)
%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr3grad_x: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr3grad_x: xE muss 6x1 sein');
if nargin == 3, platform_frame = false; end
%% Aufruf der Unterfunktionen
% Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
% ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)
[Phi_tt_red, Phi_tt] = Rob.constr2grad_tt(); % Translation identisch ...
[Phi_tr_red, Phi_tr] = Rob.constr2grad_tr(); % ... mit Var. 2
[Phi_rt_red, Phi_rt] = Rob.constr3grad_rt();
[Phi_rr_red, Phi_rr] = Rob.constr3grad_rr(q, xE, platform_frame);
%% Sortierung der ZB-Zeilen in den Matrizen nach Beingruppen, nicht nach ZB-Art
% Initialisierung mit Fallunterscheidung f�r symbolische Eingabe
dim_Pq_red=[size(Phi_tt_red, 1)+size(Phi_rt_red, 1), ...
  size(Phi_tt_red, 2)+size(Phi_tr_red, 2)];
if ~Rob.issym
  Phi_x = NaN(6*Rob.NLEG, 6);
  Phi_x_red = NaN(dim_Pq_red);
else
  Phi_x = sym('xx', [6*Rob.NLEG, 6]);
  Phi_x(:)=0;
  Phi_x_red = sym('xx', dim_Pq_red);
  Phi_x_red(:)=0;
end
Leg_I_EE_Task = cat(1,Rob.Leg(:).I_EE_Task);
nPhit = sum(Rob.I_EE_Task(1:3));
K1 = 1;
J1 = 1;
% Stelle vollst�ndige Matrix zusammen
for i = 1:Rob.NLEG
  % Zuordnung der reduzierten Zwangsbedingungen zu den Beinketten
  K2 = K1+sum(Leg_I_EE_Task(i,4:6))-1;
  % Eintrag f�r Beinkette zusammenstellen
  Phi_tt_red_i = Phi_tt_red((i-1)*nPhit+1:(i)*nPhit, :);
  Phi_tr_red_i = Phi_tr_red((i-1)*nPhit+1:(i)*nPhit, :);
  Phi_rt_red_i = Phi_rt_red(K1:K2,:);
  Phi_rr_red_i = Phi_rr_red(K1:K2,:);
  Phi_x_red_i = [Phi_tt_red_i, Phi_tr_red_i; Phi_rt_red_i, Phi_rr_red_i];
  Phi_x_red(J1:J1+size(Phi_x_red_i,1)-1,:) = Phi_x_red_i;
  J1 = J1 + size(Phi_x_red_i,1);
  K1 = K2+1;
  Phi_x((i-1)*6+1:(i)*6, :) = ...
    [Phi_tt((i-1)*3+1:(i)*3, :), Phi_tr((i-1)*3+1:(i)*3, :); ...
     Phi_rt((i-1)*3+1:(i)*3, :), Phi_rr((i-1)*3+1:(i)*3, :)];
end
% Alternative: Stelle Matrix mit reduzierten ZB direkt zusammen
% Phi_x_red = Phi_x(Rob.I_constr_red, Rob.I_EE);
