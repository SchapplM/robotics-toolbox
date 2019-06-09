% Optimierungskriterium für die inverse Kinematik: Gelenkwinkelgrenzen
% Abweichung der Gelenkwinkel von ihrer mittleren Position
% Wird von invkin benutzt
% 
% Eingabe:
% q
%   Gelenkkoordinaten des Roboters
% 
% Ausgabe:
% h [1x1] : Optimierungskriterium
% hdq [1xN]: Ableitung des Kriteriums nach den Gelenkwinkeln

% Quelle:
% [1] Huo, Baron: The joint-limits and singularity avoidance in robotic
% welding (2008)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [h, hdq] = invkin_optimcrit_limits1(q, qlim)

NJ = length(q);

% Mittlere Gelenkposition
qm = (qlim(:,1) + qlim(:,2)) / 2;
% Wichtungsmatrix
W = eye(NJ);

% [1], Gl. (34)
hdq = (q-qm)' * W;

% [1], Gl. (22) (dort doppeltes W entfernt)
h = hdq * (q-qm);
