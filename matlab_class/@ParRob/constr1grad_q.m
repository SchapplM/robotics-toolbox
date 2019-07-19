% Ableitung der kinematischen Zwangsbedingungen nach den Gelenkwinkeln
% Bezeichnungen: 
% * Jacobi-Matrix der inversen Kinematik, 
% * geometrische Matrix der inversen Kinematik
% 
% Variante 1:
% * Positionsfehler als Differenz der Vektoren vom Basis- zum Koppelpunkt-KS
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
% Phi_q_red
%   Ableitung der kinematischen Zwangsbedingungen nach allen Gelenkwinkeln
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% Phi_q [6xN]
%   Siehe vorher. Hier alle Zeilen der Zwangsbedingungen

% Quelle:
% [2_SchapplerTapOrt2019a] Schappler, M. et al.: Modeling Parallel Robot
% Kinematics for 3T2R and 3T3R Tasks using Reciprocal Sets of Euler Angles
% (Arbeitstitel), Submitted to MDPI Robotics KaRD2, Version of 27.06.2019

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phi_q_red, Phi_q] = constr1grad_q(Rob, q, xE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr1grad_q: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1grad_q: xE muss 6x1 sein');

%% Aufruf der Unterfunktionen
% Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
% ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)
[Phi_tq_red,Phi_tq]=Rob.constr1grad_tq(q);
[Phi_rq_red,Phi_rq]=Rob.constr1grad_rq(q, xE);

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
% Entspricht [2_SchapplerTapOrt2019a]/(32)
Phi_q_red(Rob.I_constr_t_red,:) = Phi_tq_red;
Phi_q_red(Rob.I_constr_r_red,:) = Phi_rq_red;
Phi_q(Rob.I_constr_t,:) = Phi_tq;
Phi_q(Rob.I_constr_r,:) = Phi_rq;
