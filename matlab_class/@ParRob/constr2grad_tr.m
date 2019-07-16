% Ableitung der Translationskomponente der kinematischen ZB nach der EE-Orientierung
% Rotation ausgedrückt in XYZ-Euler-Winkeln
% 
% 
% Eingabe:
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% 
% Ausgabe:
% Phix_phi_red
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% Phix_phi [3xN]
%   Ableitung der Translations-ZB nach der EE-Orientierung.

% Quelle:
% [A] Aufzeichnungen Schappler vom 15.06.2018 und 19.06.2018
% [B] Aufzeichnungen Schappler vom 21.06.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phix_phi_red, Phix_phi] = constr2grad_tr(Rob, xE)

%% Initialisierung
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr2grad_tr: xE muss 6x1 sein');
NLEG = Rob.NLEG;

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
dim_P_tr = [3*NLEG,3];
dim_P_tr_red = [sum(Rob.I_EE(1:3))*NLEG, sum(Rob.I_EE(4:6))];
if ~Rob.issym
  Phix_phi = zeros(dim_P_tr);
  Phix_phi_red = zeros(dim_P_tr_red);
else
  Phix_phi = sym('xx', dim_P_tr);
  Phix_phi_red = sym('xx', dim_P_tr_red);
  Phix_phi(:)=0;
  Phix_phi_red(:) = 0;
end

% Plattform-Koppelpunkt-Jacobi
 for i = 1:NLEG
  I1 = 3*(i-1)+1;
  Phix_phi(I1:I1+2,:) = zeros(3,3);
end
