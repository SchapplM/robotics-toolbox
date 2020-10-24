% Ableitung der kinematischen Zwangsbedingungen nach den EE-Koordinaten
% Bezeichnungen: 
% * Jacobi-Matrix der direkten Kinematik, 
% * geometrische Matrix der direkten Kinematik
% 
% Variante 3:
% * Implementierung der Rotation mit Führungs-Beinkette und Folge-Beinketten
% 
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% 
% Ausgabe:
% Phi_x_red
%   Ableitung der kinematischen Zwangsbedingungen nach allen Gelenkwinkeln
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% Phi_x [6xN]
%   Siehe vorher. Hier alle Zeilen der Zwangsbedingungen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phi_x_red, Phi_x] = constr3grad_x(Rob, q, xE)
%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr3grad_x: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr3grad_x: xE muss 6x1 sein');

%% Aufruf der Unterfunktionen
% Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
% ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)
[~, Phi_tt] = Rob.constr2grad_tt();  % Translation identisch mit Var. 2
[~, Phi_tr] = Rob.constr2grad_tr(xE);% Translation identisch mit Var. 2
[~, Phi_rt] = Rob.constr3grad_rt();
[~, Phi_rr] = Rob.constr3grad_rr(q, xE);
%% Sortierung der ZB-Zeilen in den Matrizen nach Beingruppen, nicht nach ZB-Art
% Initialisierung mit Fallunterscheidung für symbolische Eingabe
if ~Rob.issym
  Phi_x = NaN(6*Rob.NLEG, 6);
else
  Phi_x = sym('xx', [6*Rob.NLEG, 6]);
  Phi_x(:)=0;
end
% Stelle vollständige Matrix zusammen
for i = 1:Rob.NLEG
  Phi_x((i-1)*6+1:(i)*6, :) = ...
    [Phi_tt((i-1)*3+1:(i)*3, :), Phi_tr((i-1)*3+1:(i)*3, :); ...
     Phi_rt((i-1)*3+1:(i)*3, :), Phi_rr((i-1)*3+1:(i)*3, :)];
end
% Stelle Matrix mit reduzierten ZB zusammen
Phi_x_red = Phi_x(Rob.I_constr_red, Rob.I_EE);
