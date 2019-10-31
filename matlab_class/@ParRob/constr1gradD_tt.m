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
% [2_SchapplerTapOrt2019a] Schappler, M. et al.: Modeling Parallel Robot
% Kinematics for 3T2R and 3T3R Tasks using Reciprocal Sets of Euler Angles
% (Arbeitstitel), Submitted to MDPI Robotics KaRD2, Version of 27.06.2019
% [A] Aufzeichnungen Schappler vom 15.06.2018
% [B] Aufzeichnungen Schappler vom 22.06.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

%% No need change in the code 

function [Phi_x_red, Phi_x] = constr1gradD_tt(Rob)

%% Initialisierung
NLEG = Rob.NLEG;

Phi_x = zeros(3*NLEG,3);
Phi_x_red = zeros(length(Rob.I_constr_t_red), sum(Rob.I_EE(1:3)));

%% Belegung der Ausgabevariablen
% Plattform-Koppelpunkt-Jacobi
for i = 1:NLEG
  I1 = 3*(i-1)+1;
  % Gl. (A.37, B.24); entspricht [2_SchapplerTapOrt2019a]/35 (oben links)
  % negatives Vorzeichen, siehe Definition der Zwangsbedingungen
  phi = -zeros(3);
  Phi_x(I1:I1+2,:) = phi;
  
  J1 = sum(Rob.Leg(i).I_EE(1:3))*(i-1)+1; % TODO: Anpassung an unterschiedliche Beine
  Phi_x_red(J1:J1+sum(Rob.I_EE(1:3))-1,:) = phi(Rob.Leg(i).I_EE(1:3), Rob.I_EE(1:3));
end
