% Ableitung der kinematischen Zwangsbedingungen nach den EE-Koordinaten
% Bezeichnungen: 
% * Jacobi-Matrix der direkten Kinematik, 
% * geometrische Matrix der direkten Kinematik
% 
% Variante 3:
% Implementierung mit Führungs-Beinkette und Folge-Beinketten
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
% (C) Institut für mechatronische Systeme, Universität Hannover

function [Phi_x_red, Phi_x] = constr3grad_x(Rob, q, xE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr2grad_x: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr2grad_x: xE muss 6x1 sein');

%% Aufruf der Unterfunktionen
% Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
% ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)
[Phi_tt_red,Phi_tt]=Rob.constr3grad_tt();
[Phi_tr_red,Phi_tr]=Rob.constr3grad_tr(xE);
[Phi_rt_red,Phi_rt]=Rob.constr3grad_rt();
[Phi_rr_red,Phi_rr]=Rob.constr3grad_rr(q, xE);

% Anzahl ZB
nPhit = size(Phi_tt_red,1)/Rob.NLEG;
nPhir = size(Phi_rr_red,1)/Rob.NLEG;
nPhi = nPhit + nPhir;

%% Sortierung der ZB-Zeilen in den Matrizen nach Beingruppen, nicht nach ZB-Art
% Initialisierung mit Fallunterscheidung für symbolische Eingabe
dim_Px =   [size(Phi_tt,    1)+size(Phi_rt,    1), size(Phi_tt,    2)+size(Phi_tr,    2)];
dim_Px_red=[size(Phi_tt_red,1)+size(Phi_rt_red,1), size(Phi_tt_red,2)+size(Phi_tr_red,2)];
if ~Rob.issym
  Phi_x_red = NaN(dim_Px_red);
  Phi_x =     NaN(dim_Px);
else
  Phi_x_red = sym('xx', dim_Px_red);
  Phi_x_red(:)=0;
  Phi_x = sym('xx',     dim_Px);
  Phi_x(:)=0;
end


for i = 1:Rob.NLEG
  % TODO: Die reduzierten ZB sind aktuell nicht konsistent für Roboter mit
  % Beinketten mit fünf Gelenken. Funktionert bspw. nur für 6UPS-3T2R
  Phi_x_red((i-1)*nPhi+1:(i)*nPhi, :) = ...
    [Phi_tt_red((i-1)*nPhit+1:(i)*nPhit, :), Phi_tr_red((i-1)*nPhit+1:(i)*nPhit, :); ...
     Phi_rt_red((i-1)*nPhir+1:(i)*nPhir, :), Phi_rr_red((i-1)*nPhir+1:(i)*nPhir, :)];
  Phi_x((i-1)*6+1:(i)*6, :) = ...
    [Phi_tt((i-1)*3+1:(i)*3, :), Phi_tr((i-1)*3+1:(i)*3, :); ...
     Phi_rt((i-1)*3+1:(i)*3, :), Phi_rr((i-1)*3+1:(i)*3, :)];
end
