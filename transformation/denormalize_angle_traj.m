% Bestimme eine nicht-normalisierte Trajektorie von Winkeln
% Kann benutzt werden um mehrfache Drehungen um eine Achse zu erkennen.
%   
% Eingabe:
% X [n x m]
%   Trajektorie mit `n` Schritten von `m` Winkeln (bspw. auch Euler-Winkel)
%   
% Ausgabe:
% X_dn [n x m]
%   Trajektorie der nicht-normalisierten Winkel

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-06
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function X_dn = denormalize_angle_traj(X)
% Benutze den Altwert von X(k-1) als Mittelpunkt der Denormalisierung. 
% Annahme: Die Schritte von X sind nicht mehr als 2*pi voneinander entfernt
X_dn = NaN(size(X));
if size(X_dn,1)==0, return; end % Sonst Fehler bei leerer Eingabe
X_dn(1,:) = X(1,:);
for k = 2:size(X,1)
  X_dn(k,:) = normalizeAngle(X(k,:), X_dn(k-1,:));
end
