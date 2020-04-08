% Ableitung der kinematischen Zwangsbedingungen nach den EE-Koordinaten und
% Ableitung dieser (Gradienten-)Matrix nach der Zeit
% 
% Variante 4:
% * Bezogen auf Winkelgeschwindigkeit des Koppelpunktes Bi
%   (effektiv werden die Geschw.-ZB nach den Gelenk-Geschw. abgeleitet)
% 
% Eingabe:
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

% Quelle:
% [A] Aufzeichnungen Schappler vom 13.02.2020

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [PhiD_x_red, PhiD_x] = constr4gradD_x(Rob, xE, xDE)

%% Initialisierung
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr4gradD_x: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr4gradD_x: xDE muss 6x1 sein');
%% Aufruf der Unterfunktionen
% Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
% ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)
[Phi_tt_red,Phi_tt]=Rob.constr1gradD_tt(); % Für Translation identisch mit ...
[Phi_tr_red,Phi_tr]=Rob.constr1gradD_tr(xE, xDE); % ... Methode 1
[Phi_rt_red,Phi_rt]=Rob.constr1grad_rt(); % Term und Ableitung Null.
[Phi_rr_red,Phi_rr]=Rob.constr4gradD_rr(xE, xDE); % Methode 4

%% Sortierung der ZB-Zeilen in den Matrizen nach Beingruppen, nicht nach ZB-Art
% Initialisierung mit Fallunterscheidung für symbolische Eingabe
dim_Px =   [size(Phi_tt,    1)+size(Phi_rt,    1), size(Phi_tt,    2)+size(Phi_tr,    2)];
dim_Px_red=[size(Phi_tt_red,1)+size(Phi_rt_red,1), size(Phi_tt_red,2)+size(Phi_tr_red,2)];
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
PhiD_x_red(Rob.I_constr_t_red,:) = [Phi_tt_red,Phi_tr_red];
PhiD_x_red(Rob.I_constr_r_red,:) = [Phi_rt_red,Phi_rr_red];
PhiD_x(Rob.I_constr_t,:) = [Phi_tt, Phi_tr];
PhiD_x(Rob.I_constr_r,:) = [Phi_rt, Phi_rr];
