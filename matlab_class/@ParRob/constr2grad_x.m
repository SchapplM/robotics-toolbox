% Ableitung der kinematischen Zwangsbedingungen nach den EE-Koordinaten
% Bezeichnungen: 
% * Jacobi-Matrix der direkten Kinematik, 
% * geometrische Matrix der direkten Kinematik
% 
% Variante 2:
% * Absolute Rotation ausgedrückt in XYZ-Euler-Winkeln
% * Rotationsfehler ausgedrückt in ZYX-Euler-Winkeln
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

function [Phi_x_red, Phi_x] = constr2grad_x(Rob, q, xE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr2grad_x: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr2grad_x: xE muss 6x1 sein');

% Indizes für Reduktion der Zwangsbedingungen bei 3T2R: Nur für
% symmetrische 3T2R-PKM
I_constr_red = 1:6*Rob.NLEG;
if Rob.NJ == 25 % Behelf zur Erkennung symmetrischer 3T2R-PKM
  I_constr_red(4:6:end) = []; % entspricht z-Euler-Winkel
end
%% Aufruf der Unterfunktionen
% Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
% ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)
[~, Phi_tt] = Rob.constr2grad_tt();
[~, Phi_tr] = Rob.constr2grad_tr(xE);
[~, Phi_rt] = Rob.constr2grad_rt();
[~, Phi_rr] = Rob.constr2grad_rr(q, xE);

%% Sortierung der ZB-Zeilen in den Matrizen nach Beingruppen, nicht nach ZB-Art
% Initialisierung mit Fallunterscheidung für symbolische Eingabe
if ~Rob.issym
  Phi_x = NaN(6*Rob.NLEG, 6);
else
  Phi_x = sym('xx', 6*Rob.NLEG, 6);
  Phi_x(:)= 0;
end

for i = 1:Rob.NLEG
  Phi_x((i-1)*6+1:(i)*6, :) = ...
    [Phi_tt((i-1)*3+1:(i)*3, :), Phi_tr((i-1)*3+1:(i)*3, :); ...
     Phi_rt((i-1)*3+1:(i)*3, :), Phi_rr((i-1)*3+1:(i)*3, :)];
end
Phi_x_red = Phi_x(I_constr_red, Rob.I_EE);
