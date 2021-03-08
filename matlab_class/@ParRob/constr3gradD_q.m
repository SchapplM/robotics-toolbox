% Ableitung der kinematischen Zwangsbedingungen nach den Gelenkwinkeln und
% Ableitung dieser (Gradienten-)Matrix nach der Zeit
% 
% Variante 3:
% Implementierung mit Führungs-Beinkette und Folge-Beinketten
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
% PhiD_q_red
%   Ableitung der kinematischen Zwangsbedingungen nach allen Gelenkwinkeln
%   und der Zeit.
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% PhiD_q [6xN]
%   Siehe vorher. Hier alle Zeilen der Zwangsbedingungen
% 
% Siehe auch: constr1grad_q

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [PhiD_q_red, PhiD_q] = constr3gradD_q(Rob, q, qD, xE, xDE, platform_frame)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr3gradD_q: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/constr3gradD_q: qD muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr3gradD_q: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr3gradD_q: xDE muss 6x1 sein');
if nargin == 5, platform_frame = false; end
%% Aufruf der Unterfunktionen
% Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
% ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)
[PhiD_tq_red,PhiD_tq]=Rob.constr2gradD_tq(q, qD, platform_frame); % calling of the differentiation of the translational kinematic constraints
[PhiD_rq_red,PhiD_rq]=Rob.constr3gradD_rq(q, qD, xE, xDE, platform_frame); % calling of the differentiation of the rotational kinematic constraints

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
% Sortierung der ZB-Zeilen in den Matrizen nach Beingruppen, nicht nach ZB-Art
dim_Pq_red=[size(PhiD_tq_red,1) + size(PhiD_rq_red,1), size(PhiD_rq_red,2)];
dim_Pq =   [size(PhiD_tq,1)     + size(PhiD_rq,1),     size(PhiD_rq,2)];

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
Leg_I_EE_Task = cat(1,Rob.Leg(:).I_EE_Task);
nPhit = sum(Rob.I_EE_Task(1:3));
K1 = 1;
J1 = 1;
for i = 1:Rob.NLEG
  % Zuordnung der reduzierten Zwangsbedingungen zu den Beinketten
  K2 = K1+sum(Leg_I_EE_Task(i,4:6))-1;
  % Eintrag für Beinkette zusammenstellen
  PhiD_tq_red_i = PhiD_tq_red((i-1)*nPhit+1:(i)*nPhit, :);
  PhiD_rq_red_i = PhiD_rq_red(K1:K2,:);
  PhiD_q_red_i = [PhiD_tq_red_i; PhiD_rq_red_i];
  PhiD_q_red(J1:J1+size(PhiD_q_red_i,1)-1,:) = PhiD_q_red_i;
  J1 = J1 + size(PhiD_q_red_i,1);
  K1 = K2+1;
  
  % Zuweisung der vollständigen Zwangsbedingungen (6 pro Beinkette)
  PhiD_q((i-1)*6+1:(i)*6, :) = ...
    [PhiD_tq((i-1)*3+1:(i)*3, :); ...
     PhiD_rq((i-1)*3+1:(i)*3, :)];
end