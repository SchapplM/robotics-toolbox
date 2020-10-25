% Ableitung der kinematischen Zwangsbedingungen nach den EE-Koordinaten und
% Ableitung dieser (Gradienten-)Matrix nach der Zeit
% 
% Variante 1:
% * Absolute Rotation ausgedrückt in XYZ-Euler-Winkeln
% * Rotationsfehler ausgedrückt in XYZ-Euler-Winkeln
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

function [PhiD_x_red, PhiD_x] = constr1gradD_x(Rob, q, qD, xE, xDE, platform_frame)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr1gradD_x: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/constr1gradD_x: qD muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1gradD_x: xE muss 6x1 sein');
assert(isreal(xE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr1gradD_x: xDE muss 6x1 sein');
if nargin == 5, platform_frame = false; end

%% Aufruf der Unterfunktionen
% Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
% ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)
%%% calling of the differentiation of the  kinematic constraint of the
%%% platform can be found in Modelling sets of Euler angles
[PhiD_tt_red,PhiD_tt]=Rob.constr1gradD_tt(); 
[PhiD_tr_red,PhiD_tr]=Rob.constr1gradD_tr(xE, xDE, platform_frame);
[PhiD_rt_red,PhiD_rt]=Rob.constr1grad_rt(); % Term und Ableitung Null.
[PhiD_rr_red,PhiD_rr]=Rob.constr1gradD_rr(q, qD, xE, xDE, platform_frame);

%% Sortierung der ZB-Zeilen in den Matrizen nach Beingruppen, nicht nach ZB-Art
% Initialisierung mit Fallunterscheidung für symbolische Eingabe
dim_Px =   [size(PhiD_tt,    1)+size(PhiD_rt,    1), size(PhiD_tt,    2)+size(PhiD_tr,    2)];
dim_Px_red=[size(PhiD_tt_red,1)+size(PhiD_rt_red,1), size(PhiD_tt_red,2)+size(PhiD_tr_red,2)];
if ~Rob.issym
  PhiD_x_red = NaN(dim_Px_red);
  PhiD_x =     NaN(dim_Px);
else
  PhiD_x_red = sym('xx', dim_Px_red);
  PhiD_x_red(:)=0;
  PhiD_x = sym('xx',     dim_Px);
  PhiD_x(:)=0;
end

%% Belegung der Ausgabe
PhiD_x_red(Rob.I_constr_t_red,:) = [PhiD_tt_red,PhiD_tr_red];
PhiD_x_red(Rob.I_constr_r_red,:) = [PhiD_rt_red,PhiD_rr_red];
PhiD_x(Rob.I_constr_t,:) = [PhiD_tt, PhiD_tr];
PhiD_x(Rob.I_constr_r,:) = [PhiD_rt, PhiD_rr];
