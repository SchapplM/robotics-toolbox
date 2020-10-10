% Ableitung der kinematischen Zwangsbedingungen nach den Gelenkwinkeln
% Ableitung dieser (Gradienten-)Matrix nach der Zeit
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
% qD [Nx1]
%   Geschwindigkeit aller Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% xDE [6x1]
%   Zeitableitung der Endeffektorpose des Roboters bezüglich des Basis-KS
% 
% Ausgabe:
% PhiD_q_red
%   Ableitung der kinematischen Zwangsbedingungen nach allen Gelenkwinkeln
%   und der Zeit.
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% PhiD_q [6xN]
%   Siehe vorher. Hier alle Zeilen der Zwangsbedingungen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [PhiD_q_red, PhiD_q] = constr2gradD_q(Rob, q, qD, xE, xDE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr1gradD_q: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/constr1gradD_q: qD muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1gradD_q: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr1gradD_q: xDE muss 6x1 sein');

%% Aufruf der Unterfunktionen
% Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
% ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)
[PhiD_tq_red,PhiD_tq]=Rob.constr2gradD_tq(q, qD);
[PhiD_rq_red,PhiD_rq]=Rob.constr2gradD_rq(q, qD, xE, xDE);

% Anzahl ZB
nPhit = floor(size(PhiD_tq_red,1)/Rob.NLEG);
nPhir = floor((size(PhiD_rq_red ,1))/Rob.NLEG);
nPhi = nPhit + nPhir;

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
% Sortierung der ZB-Zeilen in den Matrizen nach Beingruppen, nicht nach ZB-Art
dim_Pq_red=[size(PhiD_tq_red,1) + size(PhiD_rq_red ,1), size(PhiD_rq_red,2)];
dim_Pq =   [size(PhiD_tq,1)     + size(PhiD_rq,1),      size(PhiD_rq,    2)];

if ~Rob.issym
  PhiD_q_red = NaN(dim_Pq_red);
  PhiD_q =     NaN(dim_Pq);
else
  PhiD_q_red = sym('xx', dim_Pq_red);
  PhiD_q_red(:)=0;
  PhiD_q = sym('xx',     dim_Pq);
  PhiD_q(:)=0;
end

%% Belegung der Ausgabe
% Entspricht [2_SchapplerTapOrt2019a]/(32)
for i = 1:Rob.NLEG
  PhiD_q_red((i-1)*nPhi+1:(i)*nPhi, :) = ...
    [PhiD_tq_red((i-1)*nPhit+1:(i)*nPhit, :); ...
     PhiD_rq_red((i-1)*nPhir+1:(i)*nPhir, :)];
  PhiD_q((i-1)*6+1:(i)*6, :) = ...
    [PhiD_tq((i-1)*3+1:(i)*3, :); ...
     PhiD_rq((i-1)*3+1:(i)*3, :)];
end
