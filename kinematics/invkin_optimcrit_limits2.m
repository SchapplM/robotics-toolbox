% Optimierungskriterium für die inverse Kinematik: Gelenkwinkelgrenzen
% Hyperbolische Abweichung der Gelenkwinkel von ihren Grenzen
% 
% Eingabe:
% q
%   Gelenkkoordinaten des Roboters
% 
% Ausgabe:
% h [1x1] : Optimierungskriterium
% hdq [1xN]: Ableitung des Kriteriums nach den Gelenkwinkeln

% Quellen:
% [SchapplerTapOrt2019a] Schappler, M. et al.: Modeling Parallel Robot
% Kinematics for 3T2R and 3T3R Tasks using Reciprocal Sets of Euler Angles;
% MDPI Robotics KaRD2, 2019
% [ZhuQuCaoYan2013]: An off-line programming system for robotic drilling
% in aerospace manufacturing (2013)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-06
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [h, hdq] = invkin_optimcrit_limits2(q, qlim)

n = length(q);
h = 0;
hdq = zeros(1,n);

for i = 1:n
   % Bei Grenzen unendlich sind Kriterium und Gradient Null
  if any(isinf(qlim(i,:))), continue; end
  if q(i) <= qlim(i,1) % Unterschreitung der unteren Grenze
    h = h + Inf;
    % Benutze bei Überschreitung der Grenzen den Gradienten einer linearen
    % Funktion mit sehr starker Steigung
    hdq(i) = -1e10; % untere Grenze -> h fällt
  elseif q(i) >= qlim(i,2) % Überschreitung der oberen Grenze
    h = h + Inf;
    hdq(i) = 1e10; % obere Grenze -> h steigt
  else
    % [ZhuQuCaoYan2013], Gl. 4; [SchapplerTapOrt2019a]/(45)
    h = h + (qlim(i,2)-qlim(i,1))^2/(8*n) * (1/(q(i)-qlim(i,1))^2+...
                                             1/(q(i)-qlim(i,2))^2);
    % Gradient: Berechnung Schappler vom 01.06.2019
    hdq(i) = -(qlim(i,2)-qlim(i,1))^2/(4*n) * (1/(q(i)-qlim(i,1))^3+...
                                               1/(q(i)-qlim(i,2))^3);
  end
end

% Herleitung der Ableitung:
% q = sym('q'); qmin = sym('qmin'); qmax = sym('qmax');
% h = 1/(q-qmin)^2 + 1/(q-qmax)^2;
% diff(h, q)