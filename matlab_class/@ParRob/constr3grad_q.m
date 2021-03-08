% Ableitung der kinematischen Zwangsbedingungen nach den Gelenkwinkeln
% Bezeichnungen: 
% * Jacobi-Matrix der inversen Kinematik, 
% * geometrische Matrix der inversen Kinematik
% 
% Variante 3:
% Implementierung mit Führungs-Beinkette und Folge-Beinketten
% 
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% platform_frame [1x1 logical]
%   Benutze das Plattform-KS anstatt das EE-KS als Bezugsgröße für x
% 
% Ausgabe:
% Phi_q_red
%   Ableitung der kinematischen Zwangsbedingungen nach allen Gelenkwinkeln
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% Phi_q [6xN]
%   Siehe vorher. Hier alle Zeilen der Zwangsbedingungen
% 
% Annahme: Funktioniert aktuell wahrscheinlich nur für 3T2R-PKM

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phi_q_red, Phi_q] = constr3grad_q(Rob, q, xE, platform_frame)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr3grad_q: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr3grad_q: xE muss 6x1 sein');
if nargin == 3, platform_frame = false; end
%% Aufruf der Unterfunktionen
% Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
% ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)
[Phi_tq_red,Phi_tq]=Rob.constr2grad_tq(q, platform_frame); % Translation identisch mit Var. 2
[Phi_rq_red,Phi_rq]=Rob.constr3grad_rq(q, xE, platform_frame);

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
Leg_I_EE_Task = cat(1,Rob.Leg(:).I_EE_Task);
nPhit = sum(Rob.I_EE_Task(1:3));
K1 = 1;
J1 = 1;
for i = 1:Rob.NLEG
  % Zuordnung der reduzierten Zwangsbedingungen zu den Beinketten
  K2 = K1+sum(Leg_I_EE_Task(i,4:6))-1;
  % Eintrag für Beinkette zusammenstellen
  Phi_tq_red_i = Phi_tq_red((i-1)*nPhit+1:(i)*nPhit, :);
  Phi_rq_red_i = Phi_rq_red(K1:K2,:);
  Phi_q_red_i = [Phi_tq_red_i; Phi_rq_red_i];
  Phi_q_red(J1:J1+size(Phi_q_red_i,1)-1,:) = Phi_q_red_i;
  J1 = J1 + size(Phi_q_red_i,1);
  K1 = K2+1;

  % Zuweisung der vollständigen Zwangsbedingungen (6 pro Beinkette)
  Phi_q((i-1)*6+1:(i)*6, :) = ...
    [Phi_tq((i-1)*3+1:(i)*3, :); ...
     Phi_rq((i-1)*3+1:(i)*3, :)];
end
% Alternativ: Direkt mit Indizes bestimmen. Das funktioniert aber nur, wenn
% Aufgabenredundanz vorliegt, und die Indizes darauf abgestimmt sind.
% Bei nicht-AR-Fall ist die Reziprozität nicht berücksichtigt.
% Phi_q_red = Phi_q(Rob.I_constr_red, :);
