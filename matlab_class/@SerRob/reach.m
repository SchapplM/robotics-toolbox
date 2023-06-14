% Gebe die Reichweite des Roboters zurück. Die Reichweite beschreibt den
% Abstand eines EE-Punktes, der garantiert nicht mehr im Arbeitsraum liegt.
% Die Berechnung erfolgt als Abschätzung der oberen Grenze mittels DH-Par.
% 
% Eingabe:
% qlim_tmp
%   Gelenkpositionsgrenzen zur Berechnung der Länge der Länge
%   (wegen Schubgelenken, Angabe für alle Gelenke)
% 
% Ausgabe:
% r_E
%   Reichweite (Abstand des Endeffektors bezogen auf Basis)
% r_N
%   Reichweite des letzten Körper-KS (ohne zusätzliche EE-Transformation)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [r_E, r_N] = reach(R, qlim_tmp)

% TODO: Funktioniert aktuell nur bei seriellen, nicht bei hybriden Ketten
if R.Type == 1
  error('Funktioniert noch nicht für hybride Ketten');
end
if nargin < 2
  qlim_tmp = R.qlim;
end

% Bestimme den Betrag aller Einzel-Gelenk-Transformationen aus den
% MDH-Parametern
r_N = 0;
for i = 1:R.NJ
  if R.MDH.sigma(i) == 0 % Drehgelenk
    d_max = R.MDH.d(i);
  else % Schubgelenke
    d_max = max(abs(qlim_tmp(i,:)));
  end
  % Erhöhe die Reichweite des Roboters um die maximale Länge aus der
  % aktuellen Gelenk-Transformation
  r_N = r_N + sqrt(R.MDH.a(i)^2 + d_max^2);
end
% Beitrag der EE-Verschiebung: Dient als "Hebelarm" des letzten Gelenks
% (und aller anderen Gelenke)
r_E = r_N + norm(R.T_N_E(1:3,4));