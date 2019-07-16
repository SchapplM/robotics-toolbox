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

% Quelle:
% [ZhuQuCaoYan2013]: An off-line programming system for robotic drilling
% in aerospace manufacturing (2013)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-06
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [h, hdq] = invkin_optimcrit_limits2(q, qlim)

n = length(q);
h = 0;
hdq = zeros(1,n);

% Der Gradient darf nur für Bereiche definiert sein, wo die Winkel in ihrem
% Toleranzbereich liegen. Ansonsten können die Gelenkwinkel nicht mehr
% zurückkehren
I_viol = q<qlim(:,1) | q>qlim(:,2);

for i = 1:n
  if ~I_viol(i)
    % [ZhuQuCaoYan2013], Gl. 4
    h = h + (qlim(i,2)-qlim(i,1))^2/(8*n) * (1/(q(i)-qlim(i,1))^2+...
                                             1/(q(i)-qlim(i,2))^2);
    % Gradient: Berechnung Schappler vom 01.06.2019
    hdq(i) = -(qlim(i,2)-qlim(i,1))/(4*n) * (1/(q(i)-qlim(i,1))^3+...
                                             1/(q(i)-qlim(i,2))^3);
  else
    
    h = Inf;
    % Keine Berechnung von hdq: Bleibt Null
  end
end

