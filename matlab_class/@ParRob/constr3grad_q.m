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

% Anzahl ZB
% TODO: Das funktioniert wahrscheinlich nicht bei allen asymmetrischen PKM,
% falls planare Roboter modelliert werden.
% nPhit = size(Phi_tq_red,1)/Rob.NLEG; % TODO: Setzt symmetrische PKM vorraus
% nPhir = size(Phi_rq_red,1)/Rob.NLEG; % TODO: Setzt symmetrische PKM vorraus
% nPhi = nPhit + nPhir;
nPhit = floor(size(Phi_tq_red,1)/Rob.NLEG);
nPhir = floor((size(Phi_rq_red ,1))/Rob.NLEG);
nPhi = nPhit + nPhir;

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
for i = 1:Rob.NLEG
  if all(Rob.I_EE_Task == logical([1 1 1 1 1 0]))
    if all(Rob.Leg(i).I_EE_Task == logical([1 1 1 1 1 0])) % Führungskette: Reduzierte FG um Rotation
      Phi_q_red(1:5, :) = ...
        [Phi_tq_red((i-1)*nPhit+1:(i)*nPhit, :); ...
         Phi_rq_red(1:2, :)];
    else % Folgekette: Alle weiteren Ketten 6 Zwangsbedingungen
      Phi_q_red(6+6*(i-2):5+6*(i-1), :) = ...
        [Phi_tq_red((i-1)*nPhit+1:(i)*nPhit, :); ...
         Phi_rq_red(3+3*(i-2):5+3*(i-2), :)];
    end
  else
    Phi_q_red((i-1)*nPhi+1:(i)*nPhi, :) = ...
      [Phi_tq_red((i-1)*nPhit+1:(i)*nPhit, :); ...
       Phi_rq_red((i-1)*nPhir+1:(i)*nPhir, :)];
  end
  Phi_q((i-1)*6+1:(i)*6, :) = ...
    [Phi_tq((i-1)*3+1:(i)*3, :); ...
     Phi_rq((i-1)*3+1:(i)*3, :)];
end
