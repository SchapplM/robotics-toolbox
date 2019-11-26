% Ableitung der kinematischen Zwangsbedingungen nach den Gelenkwinkeln und
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
% 
% Ausgabe:
% Phi_q_red
%   Ableitung der kinematischen Zwangsbedingungen nach allen Gelenkwinkeln
%   und der Zeit.
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% Phi_q [6xN]
%   Siehe vorher. Hier alle Zeilen der Zwangsbedingungen
% 
% Siehe auch: constr1grad_q

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [PhiD_q_red, PhiD_q] = constr1gradD_q(Rob, q, qD, xE, xDE)

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
[Phi_tq_red,Phi_tq]=Rob.constr1gradD_tq(q, qD); % calling of the differentiation of the translational kinematic constraints
[Phi_rq_red,Phi_rq]=Rob.constr1gradD_rq(q, qD, xE, xDE); % calling of the differentiation of the rotational kinematic constraints

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
% Sortierung der ZB-Zeilen in den Matrizen nach Beingruppen, nicht nach ZB-Art
dim_Pq_red=[size(Phi_tq_red,1) + size(Phi_rq_red ,1), size(Phi_rq_red,2)];
dim_Pq =   [size(Phi_tq,1)     + size(Phi_rq,1),      size(Phi_rq,    2)];

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
PhiD_q_red(Rob.I_constr_t_red,:) = Phi_tq_red;
PhiD_q_red(Rob.I_constr_r_red,:) = Phi_rq_red;
PhiD_q(Rob.I_constr_t,:) = Phi_tq;
PhiD_q(Rob.I_constr_r,:) = Phi_rq;