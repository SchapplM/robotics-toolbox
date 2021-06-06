% Bestimme eine nicht-normalisierte Trajektorie von Winkeln
% Kann benutzt werden um mehrfache Drehungen um eine Achse zu erkennen.
%   
% Eingabe:
% X
%   Trajektorie von Winkeln (bspw. auch Euler-Winkel)
% XD
%   Trajektorie von Ableitungen der Winkel. Können bspw. auch
%   Euler-Winkel-Zeitableitungen sein
% T
%   Zeitstempel der Trajektorie
%   
% Ausgabe:
% X_dn
%   Trajektorie der nicht-normalisierten Winkel

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-06
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function X_dn = denormalize_angle_traj(X, XD, T)

% Use integration to avoid restriction to +/- pi
X_ist_int = repmat(X(1,:), length(T), 1) + cumtrapz(T, XD);
% Normalize angles from direct calculation using angles from
% integration as center. Gives exact solution without limitation to +/-pi
X_dn = normalizeAngle(X, X_ist_int);
