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

% Quellen:
% [SchapplerTapOrt2019] Schappler, M. et al.: Resolution of Functional
% Redundancy for 3T2R Robot Tasks using Two Sets of Reciprocal Euler
% Angles, Proc. of the 15th IFToMM World Congress, 2019
% [1] Aufzeichnungen Schappler vom 3.8.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-08
% (C) Institut für mechatronische Systeme, Universität Hannover

function [h, hdq] = optimcrit_limits1(Rob, q)

% Mittlere Gelenkposition
qm = (Rob.qlim(:,1) + Rob.qlim(:,2)) / 2;
% Wichtungsmatrix
W = eye(Rob.NQJ);

% [SchapplerTapOrt2019]/(34); [1], Gl. (34)
hdq = (q-qm)' * W;

% [SchapplerTapOrt2019]/33; [1], Gl. (32)
h = hdq * (q-qm);
