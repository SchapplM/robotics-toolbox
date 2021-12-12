% Optimierungskriterium für die inverse Kinematik: Konditionszahl (mod.)
% Aktivierung durch kubischen Spline. Damit Benutzung als Nebenbedingung
% mit Schwellwert zur Aktivierung anstatt als dauerhaft aktive Zielfunktion
% 
% Eingabe:
% k [1x1]
%   Konditionszahl
% k_thr [1x1]
%   Schwellwert zum Durchschalten der Konditionszahl. Muss größer 1 sein.
% k_act [1x1]
%   Aktivierungsschwelle, ab der die Kennzahl einen Wert >0 bekommt.
%   Falls nicht belegt: k_act = k_thr / 2
%   Muss kleiner als k_thr sein (und größer 1).
% 
% Ausgabe:
% h [1x1]
%   Optimierungskriterium basierend auf Konditionszahl

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function h = invkin_optimcrit_condsplineact(k, k_thr, k_act)
if k > k_thr
  % Oberhalb des Schwellwerts. Benutze Konditionszahl direkt als Kriterium
  h = k;
  return
end
if nargin < 3
  % Aktivierungsschwelle bei der Hälfte
  k_act = 0.5*k_thr;
end
if k <= k_act
  % Unterhalb der Aktivierungsschwelle. Ausgabe ist Null.
  h = 0;
else
  % Spline-Interpolation zwischen beiden Bereichen für stetige
  % Differenzierbarkeit. Ansatz:
  % s = b0 + b1*(k-k_act) + b2*(k-k_act)^2 + b3*(k-k_act)^3
  % Berechnung der Koeffizienten siehe unten in Datei.
  delta = k_thr - k_act;
  b2 = -(delta - 3*k_thr)/delta^2;
  b3 =  (delta - 2*k_thr)/delta^3;
  h = b2 * (k-k_act)^2 + b3*(k-k_act)^3;
end

% Herleitung Spline-Koeffizienten b2 und b3: (b0=b1=0)
% syms k_thr delta
% [delta^2 delta^3; 2*delta 3*delta^2]\[k_thr;1]
