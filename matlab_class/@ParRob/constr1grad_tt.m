% Ableitung der Translationskomponente der kinematischen ZB nach der EE-Position
% 
% Ausgabe:
% Phi_x_red
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% Phi_x [3xN]
%   Ableitung der Translations-ZB nach der EE-Position. Ergebnis ist
%   Einheitsmatrix, da die EE-Position lineare in den Positionsfehler
%   eingeht

% Quellen:
% [A] Aufzeichnungen Schappler vom 15.06.2018
% [B] Aufzeichnungen Schappler vom 22.06.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phi_x_red, Phi_x] = constr1grad_tt(Rob)

%% Initialisierung
NLEG = Rob.NLEG;

Phi_x = zeros(3*NLEG,3);
Phi_x_red = zeros(sum(Rob.I_EE(1:3))*NLEG, sum(Rob.I_EE(1:3)));

%% Belegung der Ausgabevariablen
% Plattform-Koppelpunkt-Jacobi
for i = 1:NLEG
  I1 = 3*(i-1)+1;
  % Gl. (A.37, B.24)
  % negatives Vorzeichen, siehe Definition der Zwangsbedingungen
  phi = -eye(3);
  Phi_x(I1:I1+2,:) = phi;
  
  J1 = sum(Rob.I_EE(1:3))*(i-1)+1;
  Phi_x_red(J1:J1+sum(Rob.I_EE(1:3))-1,:) = phi(Rob.I_EE(1:3), Rob.I_EE(1:3));
end
