% Optimierungskriterium für die inverse Kinematik: Gelenkwinkelgrenzen
% Hyperbolische Abweichung der Gelenkwinkel von ihren Grenzen
% 
% Eingabe:
% q [Nx1]
%   Gelenkkoordinaten
% qlim [Nx2]
%   Gelenkgrenzen (erste Spalte untere Grenze, zweite Spalte obere)
% qlim_thr [Nx2]
%   Gelenkgrenzen als Schwellwert für (stetige) Abschaltung der Funktion.
%   Erste Spalte untere Grenze, zweite Spalte obere.
%   Die Hälfte des Bereichs wird als kubischer Spline realisiert.
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
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [h, hdq] = invkin_optimcrit_limits2(q, qlim, qlim_thr)

n = length(q);
h = 0;
hdq = zeros(1,n);

if nargin < 3
  % Keinen Spline benutzen. Grenzen so einstellen, dass inaktiv
  qlim_thr = NaN(n,2);
end

for i = 1:n % Gelenke durchgehen
  % Schwellwerte für Spline-Interpolation bestimmen
  qi_thr_min = qlim_thr(i,1);
  qi_sw_min = (qlim(i,1)+qlim_thr(i,1))/2;
  qi_thr_max = qlim_thr(i,2);
  qi_sw_max = (qlim(i,2)+qlim_thr(i,2))/2;
  % Bei Grenzen unendlich sind Kriterium und Gradient Null
  if any(isinf(qlim(i,:))), continue; end
  % Fälle der stückweise definierten Funktion prüfen
  if q(i) <= qlim(i,1) % Unterschreitung der unteren Grenze
    h = h + Inf;
    % Benutze bei Überschreitung der Grenzen den Gradienten einer linearen
    % Funktion mit sehr starker Steigung
    hdq(i) = -1e10; % untere Grenze -> h fällt
  elseif q(i) >= qlim(i,2) % Überschreitung der oberen Grenze
    h = h + Inf;
    hdq(i) = 1e10; % obere Grenze -> h steigt
  elseif qi_sw_min < q(i) && q(i) < qi_thr_min % in unterem Spline-Bereich
    % Interpolation mit Spline bei unterer Grenze
    % Bestimme Spline-Koeffizienten. Berechnung Schappler vom 28.04.2021
    t5 = qi_sw_min-qi_thr_min;
    t2 = t5*t5; t4 = t2;
    t1 = t2*t5;
    ttmax = qi_sw_min-qlim(i,2);
    ttmin = qi_sw_min-qlim(i,1);
    t3 =  1/(ttmax^2) + 1/(ttmin^2);
    t6 = -2/(ttmax^3) - 2/(ttmin^3);
    coeff = [t1, t2; 3*t4, 2*t5] \ ((qlim(i,2)-qlim(i,1))^2/(8*n) * [t3;t6]);
    h = h + coeff(1)*(q(i)-qi_thr_min)^3 + coeff(2)*(q(i)-qi_thr_min)^2;
    hdq(i) = 3*coeff(1)*(q(i)-qi_thr_min)^2 + 2*coeff(2)*(q(i)-qi_thr_min);
  elseif qi_thr_min <= q(i) && q(i) <= qi_thr_max % zwischen Spline-Bereichen
    % Zielfunktion bleibt Null
    h = h + 0;
  elseif qi_thr_max < q(i) && q(i) < qi_sw_max % in oberem Spline-Bereich
    % Interpolation mit Spline bei oberer Grenze
    % Bestimme Spline-Koeffizienten
    t5 = qi_sw_max-qi_thr_max;
    t2 = t5*t5; t4 = t2;
    t1 = t2*t5;
    ttmax = qi_sw_max-qlim(i,2);
    ttmin = qi_sw_max-qlim(i,1);
    t3 =  1/(ttmax^2) + 1/(ttmin^2);
    t6 = -2/(ttmax^3) - 2/(ttmin^3);
    coeff = [t1, t2; 3*t4, 2*t5] \ ((qlim(i,2)-qlim(i,1))^2/(8*n) * [t3;t6]);
    h = h + coeff(1)*(q(i)-qi_thr_max)^3 + coeff(2)*(q(i)-qi_thr_max)^2;
    hdq(i) = 3*coeff(1)*(q(i)-qi_thr_max)^2 + 2*coeff(2)*(q(i)-qi_thr_max);
  else % hyperbolische Zielfunktion (nahe an Grenzen)
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