% Ableitung der kinematischen Zwangsbedingungen nach den EE-Koordinaten
% Bezeichnungen: 
% * Jacobi-Matrix der direkten Kinematik, 
% * geometrische Matrix der direkten Kinematik
% 
% Variante 1:
% * Absolute Rotation ausgedrückt in XYZ-Euler-Winkeln
% * Rotationsfehler definiert als 0x-0q ausgedrückt in XYZ-Euler-Winkeln
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

% Quelle:
% [2_SchapplerTapOrt2019a] Schappler, M. et al.: Modeling Parallel Robot
% Kinematics for 3T2R and 3T3R Tasks using Reciprocal Sets of Euler Angles
% (Arbeitstitel), Submitted to MDPI Robotics KaRD2, Version of 27.06.2019

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phi_x_red, Phi_x] = constr1grad_x(Rob, q, xE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr1grad_x: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1grad_x: xE muss 6x1 sein');

%% Aufruf der Unterfunktionen
% Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
% ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)
[Phi_tt_red,Phi_tt]=Rob.constr1grad_tt();
[Phi_tr_red,Phi_tr]=Rob.constr1grad_tr(xE);
[Phi_rt_red,Phi_rt]=Rob.constr1grad_rt();
[Phi_rr_red,Phi_rr]=Rob.constr1grad_rr(q, xE);

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

%% Belegung der Ausgabe
% Entspricht [2_SchapplerTapOrt2019a]/(35) (aber hier andere R-Reihenfolge)
Phi_x_red(Rob.I_constr_t_red,:) = [Phi_tt_red,Phi_tr_red];
Phi_x_red(Rob.I_constr_r_red,:) = [Phi_rt_red,Phi_rr_red];
Phi_x(Rob.I_constr_t,:) = [Phi_tt, Phi_tr];
Phi_x(Rob.I_constr_r,:) = [Phi_rt, Phi_rr];
