% Ableitung der kinematischen Zwangsbedingungen nach den EE-Koordinaten
% Ableitung dieser (Gradienten-)Matrix nach der Zeit
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
% qD [Nx1]
%   Geschwindigkeit aller Gelenkwinkel aller serieller Beinketten der PKM
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

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [PhiD_x_red, PhiD_x] = constr2gradD_x(Rob, q, qD, xE, xDE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr1gradD_x: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/constr1gradD_x: qD muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1gradD_x: xE muss 6x1 sein');
assert(isreal(xE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr1gradD_x: xDE muss 6x1 sein');

%% Aufruf der Unterfunktionen
% Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
% ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)
[PhiD_tt_red,PhiD_tt]=Rob.constr2gradD_tt();
[PhiD_tr_red,PhiD_tr]=Rob.constr2gradD_tr(xE, xDE);
[PhiD_rt_red,PhiD_rt]=Rob.constr2grad_rt();
[PhiD_rr_red,PhiD_rr]=Rob.constr2gradD_rr(q, qD, xE ,xDE);

% Anzahl ZB
nPhit = size(PhiD_tt_red,1)/Rob.NLEG;
nPhir = size(PhiD_rr_red,1)/Rob.NLEG;
nPhi = nPhit + nPhir;

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
  PhiD_x(:)= 0;
end


for i = 1:Rob.NLEG
     % Anzahl der Zwangsbedingungen 
     nPhit = floor(size(PhiD_tt_red,1)/Rob.NLEG);
     nPhir = floor((size(PhiD_rr_red ,1))/Rob.NLEG);
     nPhi = nPhit + nPhir;
     
     % oder feste zahlen so(unguenstig)
     % nPhir = 2;  % vielleicht verallgemeinern, Achtung bei nicht ganzen Zahlen
     % nPhit = 3;  % vielleicht verallgemeinern, Achtung bei nicht ganzen Zahlen 

     PhiD_x_red((i-1)*nPhi+1:(i)*nPhi,:) = ...
     [PhiD_tt_red((i-1)*nPhit+1:(i)*nPhit, :), PhiD_tr_red((i-1)*nPhit+1:(i)*nPhit, :); ...
     PhiD_rt_red((i-1)*nPhir+1:(i)*nPhir, :), PhiD_rr_red((i-1)*nPhir+1:(i)*nPhir, :)]; % nur fuer symmetrisch
 
     PhiD_x((i-1)*6+1:(i)*6, :) = ...
    [PhiD_tt((i-1)*3+1:(i)*3, :), PhiD_tr((i-1)*3+1:(i)*3, :); ...
     PhiD_rt((i-1)*3+1:(i)*3, :), PhiD_rr((i-1)*3+1:(i)*3, :)];
end
