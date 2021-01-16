% Ableitung der kinematischen Zwangsbedingungen nach den Gelenkwinkeln
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
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% platform_frame [1x1 logical]
%   Benutze das Plattform-KS anstatt das EE-KS als Bezugsgröße für x
% 
% Ausgabe:
% Phi_q_red
%   Ableitung der kinematischen Zwangsbedingungen nach allen Gelenkwinkeln
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% Phi_q [6xN]
%   Siehe vorher. Hier alle Zeilen der Zwangsbedingungen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phi_q_red, Phi_q] = constr2grad_q(Rob, q, xE, platform_frame)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr2grad_q: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr2grad_q: xE muss 6x1 sein');
if nargin == 3, platform_frame = false; end
% Indizes für Reduktion der Zwangsbedingungen bei 3T2R: Nur für
% symmetrische 3T2R-PKM
I_constr_red = 1:6*Rob.NLEG;
if Rob.NJ == 25 % Behelf zur Erkennung symmetrischer 3T2R-PKM
  I_constr_red(4:6:end) = []; % entspricht z-Euler-Winkel
end
%% Aufruf der Unterfunktionen
% Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
% ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)
[~, Phi_tq] = Rob.constr2grad_tq(q, platform_frame);
[~, Phi_rq] = Rob.constr2grad_rq(q, xE, platform_frame);

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
% Sortierung der ZB-Zeilen in den Matrizen nach Beingruppen, nicht nach ZB-Art
if ~Rob.issym
  Phi_q = NaN(6*Rob.NLEG, Rob.NJ);
else
  Phi_q = sym('xx', 6*Rob.NLEG, Rob.NJ);
  Phi_q(:) = 0;
end

%% Belegung der Ausgabe
% Entspricht [2_SchapplerTapOrt2019a]/(32)
for i = 1:Rob.NLEG
  Phi_q((i-1)*6+1:(i)*6, :) = ...
    [Phi_tq((i-1)*3+1:(i)*3, :); ...
     Phi_rq((i-1)*3+1:(i)*3, :)];
end
Phi_q_red = Phi_q(I_constr_red, :);
