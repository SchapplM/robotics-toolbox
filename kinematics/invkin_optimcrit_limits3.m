% Optimierungskriterium für die inverse Kinematik: Kollisionsabstand (z.B.)
% Hyperbolische Abweichung eines Werts zu einer einseitigen Grenze.
% Zur anderen Seite des Schwellwertes bleibt der Wert Null (der linke
% Hyperbel-Ast wird abgeschnitten)
% 
% Eingabe:
% q [Nx1]
%   Gelenkkoordinaten (z.B.)
% qlim [Nx2]
%   Gelenkgrenzen (z.B.) (erste Spalte untere Grenze, zweite Spalte obere)
%   Die untere Grenze dient nur der Skalierung der Hyperbel-Funktion
% qlim_thr [Nx1]
%   Gelenkgrenzen als Schwellwert für (stetige) Abschaltung der Funktion.
%   Der Schwellwert bezieht sich nur auf die obere Grenze
%   Die Hälfte des Bereichs zur oberen Grenze wird als kubischer Spline realisiert.
% 
% Ausgabe:
% h [1x1] : Optimierungskriterium
% hdq [1xN]: Ableitung des Kriteriums nach den Gelenkwinkeln
% 
% Siehe auch: invkin_optimcrit_limits2

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [h, hdq] = invkin_optimcrit_limits3(q, qlim, qlim_thr)

n = length(q);
h = 0;
hdq = zeros(1,n);

if nargin < 3
  % Keinen Spline benutzen. Grenzen so einstellen, dass inaktiv
  qlim_thr = NaN(n,1);
end

for i = 1:n % Gelenke durchgehen
  % Schwellwerte für Spline-Interpolation bestimmen
  qi_thr = qlim_thr(i);
  qi_sw_max = (qlim(i,2)+qi_thr)/2;
  % Bei Grenzen unendlich sind Kriterium und Gradient Null
  if any(isinf(qlim(i,:))), continue; end
  % Fälle der stückweise definierten Funktion prüfen
  if q(i) <= qi_thr % Unterschreitung des Schwellwerts
    % Zielfunktion ist Null
    h = h + 0;
  elseif q(i) >= qlim(i,2) % Überschreitung der oberen Grenze
    h = h + Inf;
    hdq(i) = 1e10; % obere Grenze -> h steigt
  elseif qi_thr < q(i) && q(i) < qi_sw_max % in oberem Spline-Bereich
    % Interpolation mit Spline bei oberer Grenze
    % Bestimme Spline-Koeffizienten
    t5 = qi_sw_max-qi_thr;
    t2 = t5*t5; t4 = t2;
    t1 = t2*t5;
    ttmax = qi_sw_max-qlim(i,2);
    ttmin = qi_sw_max-qlim(i,1);
    t3 =  1/(ttmax^2) + 1/(ttmin^2);
    t6 = -2/(ttmax^3) - 2/(ttmin^3);
    coeff = [t1, t2; 3*t4, 2*t5] \ ((qlim(i,2)-qlim(i,1))^2/(8*n) * [t3;t6]);
    h = h + coeff(1)*(q(i)-qi_thr)^3 + coeff(2)*(q(i)-qi_thr)^2;
    hdq(i) = 3*coeff(1)*(q(i)-qi_thr)^2 + 2*coeff(2)*(q(i)-qi_thr);
  else % hyperbolische Zielfunktion (nahe an Grenzen)
    % [ZhuQuCaoYan2013], Gl. 4; [SchapplerTapOrt2019a]/(45)
    h = h + (qlim(i,2)-qlim(i,1))^2/(8*n) * (1/(q(i)-qlim(i,1))^2+...
                                             1/(q(i)-qlim(i,2))^2);
    % Gradient: Berechnung Schappler vom 01.06.2019
    hdq(i) = -(qlim(i,2)-qlim(i,1))^2/(4*n) * (1/(q(i)-qlim(i,1))^3+...
                                               1/(q(i)-qlim(i,2))^3);
  end
end
