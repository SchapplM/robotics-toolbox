% Ableitung der kinematischen Zwangsbedingungen nach den Gelenkwinkeln
% Bezeichnungen: 
% * Jacobi-Matrix der inversen Kinematik, 
% * geometrische Matrix der inversen Kinematik
% 
% Variante 1:
% * Absolute Rotation ausgedrückt in XYZ-Euler-Winkeln
% * Rotationsfehler ausgedrückt in XYZ-Euler-Winkeln
% 
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% qD [Nx1]
%  Velocity of the  Gelenkwinkel aller serieller Beinketten der PKM
%xDE [N x1]
%   Velocity of the Platform Coordinate based on the orientation and rotation
% Ausgabe:
% Phi_q_red
%   Ableitung der kinematischen Zwangsbedingungen nach allen Gelenkwinkeln
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% Phi_q [6xN]
%   Siehe vorher. Hier alle Zeilen der Zwangsbedingungen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phi_q_red, Phi_q] = constr1gradD_q(Rob, q,qD , xE,xDE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr1grad_q: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/constr1grad_qD: qD muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1grad_q: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr1grad_q: xDE muss 6x1 sein');
% assert(all(size(rpy) == [3 1]) && isreal(rpy), ...
%   'ParRob/constr1gradD_q: rpy angles have to be [3x1] (double)'); 
% assert(all(size(rpyD) == [3 1]) && isreal(rpyD), ...
%   'ParRob/constr1gradD_q: rpy angles time derivatives have to be [3x1] (double)');
% assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
%   'ParRob/constrgradD_tq: q muss %dx1 sein', Rob.NJ);

%% Aufruf der Unterfunktionen
% Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
% ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)
[Phi_tq_red,Phi_tq]=Rob.constr1gradD_tq(q,qD); % calling of the differentiation of the translational kinematic constraints
[Phi_rq_red,Phi_rq]=Rob.constr1gradD_rq(q,qD, xE ,xDE); % calling of the differentiation of the rotational kinematic constraints

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
% Sortierung der ZB-Zeilen in den Matrizen nach Beingruppen, nicht nach ZB-Art
dim_Pq_red=[size(Phi_tq_red,1) + size(Phi_rq_red ,1), size(Phi_rq_red,2)];
dim_Pq =   [size(Phi_tq,1)     + size(Phi_rq,1),      size(Phi_rq,    2)];

if ~Rob.issym
  Phi_q_red = NaN(dim_Pq_red);
  Phi_q =     NaN(dim_Pq);
else
  Phi_q_red = sym('xx', dim_Pq_red);
  Phi_q_red(:)=0;
  Phi_q = sym('xx',     dim_Pq);
  Phi_q(:)=0;
end

%% Belegung der Ausgabe
Phi_q_red(Rob.I_constr_t_red,:) = Phi_tq_red;
Phi_q_red(Rob.I_constr_r_red,:) = Phi_rq_red;
Phi_q(Rob.I_constr_t,:) = Phi_tq;
Phi_q(Rob.I_constr_r,:) = Phi_rq;