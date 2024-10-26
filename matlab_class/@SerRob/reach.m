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

if nargin < 2
  qlim_tmp = R.qlim;
end
if R.Type == 1
  % Für seriell-hybride Ketten sind nur die Grenzen der aktiven Gelenke
  % gespeichert. Annahme. Keine passiven Schubgelenke (die bleiben NaN).
  qlim_tmp_allcoord = NaN(R.NJ,2);
  qlim_tmp_allcoord(R.MDH.mu==1,:) = qlim_tmp;
else
  % Für Beinketten paralleleler Roboter ist mu=1 und mu=2 gesetzt.
  qlim_tmp_allcoord = qlim_tmp;
end
% Bestimme den Betrag aller Einzel-Gelenk-Transformationen aus den
% MDH-Parametern. Gehe dafür vom EE zur Basis (Haupt-Kette bei
% seriell-hybriden Kinematiken)
r_N = 0;
i = R.I_EElink;
while i ~= 0
  if R.MDH.sigma(i) == 0 % Drehgelenk
    d_max = R.MDH.d(i);
  else % Schubgelenke
    d_max = max(abs(qlim_tmp_allcoord(i,:)));
  end
  % Erhöhe die Reichweite des Roboters um die maximale Länge aus der
  % aktuellen Gelenk-Transformation
  r_N = r_N + sqrt(R.MDH.a(i)^2 + d_max^2);
  % Gehe zum Vorgänger-Segment
  i = R.MDH.v(i);
end
% Beitrag der EE-Verschiebung: Dient als "Hebelarm" des letzten Gelenks
% (und aller anderen Gelenke)
r_E = r_N + norm(R.T_N_E(1:3,4));