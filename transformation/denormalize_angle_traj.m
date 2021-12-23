% Bestimme eine nicht-normalisierte Trajektorie von Winkeln
% Kann benutzt werden um mehrfache Drehungen um eine Achse zu erkennen.
%   
% Eingabe:
% X
%   Trajektorie von Winkeln (bspw. auch Euler-Winkel)
% XD (optional, Variante 2)
%   Trajektorie von Ableitungen der Winkel. Können bspw. auch
%   Euler-Winkel-Zeitableitungen sein
% T (optional, Variante 2)
%   Zeitstempel der Trajektorie
%   
% Ausgabe:
% X_dn
%   Trajektorie der nicht-normalisierten Winkel

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-06
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function X_dn = denormalize_angle_traj(X, XD, T)

% Ansatz 1: Benutze den Altwert von X(k-1) als Mittelpunkt der
% Denormalisierung. Annahme: Die Schritte von X sind eng genug beieinander
X_dn2 = NaN(size(X));
X_dn2(1,:) = X(1,:);
for k = 2:size(X,1)
  X_dn2(k,:) = normalizeAngle(X(k,:), X_dn2(k-1,:));
end
if nargin == 1
  X_dn = X_dn2;
  return
end
warning('Eingabe mit 3 Argumenten ist veraltet. XD und T sind obsolet');
% Ansatz 2: Benutze die Geschwindigkeit
% Use integration to avoid restriction to +/- pi
X_ist_int = repmat(X(1,:), length(T), 1) + cumtrapz(T, XD);
% Normalize angles from direct calculation using angles from
% integration as center. Gives exact solution without limitation to +/-pi
X_dn = normalizeAngle(X, X_ist_int);
% Testen
assert(all(abs(X_dn2-X_dn)<1e-10), 'Zweite Implementierung stimmt nicht');
