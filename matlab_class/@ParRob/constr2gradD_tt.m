% Ableitung der Translationskomponente der kinematischen ZB nach der
% EE-Position und Ableitung dieser (Gradienten-)Matrix nach der Zeit
% 
% Ausgabe:
% PhiD_x_red
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% PhiD_x [3xN]
%   Ableitung der Translations-ZB nach der EE-Position und der Zeit. Ergebnis ist
%   Null-Matrix, da die EE-Position linear in den Positionsfehler eingeht

% Siehe auch: constr2grad_tt

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [PhiD_x_red, PhiD_x] = constr2gradD_tt(Rob)

%% Initialisierung
NLEG = Rob.NLEG;

PhiD_x = zeros(3*NLEG,3);
PhiD_x_red = zeros(length(Rob.I_constr_t_red), sum(Rob.I_EE(1:3)));

%% Belegung der Ausgabevariablen
for i = 1:NLEG
  I1 = 3*(i-1)+1;
  % Term in constr1grad_tt ist Einheitsmatrix -> Zeitableitung ist Null
  phi = zeros(3);
  PhiD_x(I1:I1+2,:) = phi;
  
  J1 = sum(Rob.Leg(i).I_EE(1:3))*(i-1)+1; % TODO: Anpassung an unterschiedliche Beine
  PhiD_x_red(J1:J1+sum(Rob.I_EE(1:3))-1,:) = phi(Rob.Leg(i).I_EE(1:3), Rob.I_EE(1:3));
end
