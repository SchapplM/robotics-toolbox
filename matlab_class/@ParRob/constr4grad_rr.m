% Ableitung der Rotationskomponente der kinematischen ZB nach der EE-Orientierung
% 
% Variante 4:
% * Bezogen auf Winkelgeschwindigkeit des Koppelpunktes Bi
%   (effektiv werden die Geschw.-ZB nach den Gelenk-Geschw. abgeleitet)
% 
% Eingabe:
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% 
% Ausgabe:
% Phi_rr_red
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% Phi_rr [3xN]
%   Ableitung der kinematischen Zwangsbedingungen nach der EE-Orientierung
%   Rotatorischer Teil

% Quelle:
% [A] Aufzeichnungen Schappler vom 13.02.2020

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phi_rr_red, Phi_rr] = constr4grad_rr(Rob,xE)

%% Initialisierung
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr4grad_rr: xE muss 6x1 sein');
NLEG = Rob.NLEG;

Phi_rr = zeros(3*NLEG,3);
Phi_rr_red = zeros(length(Rob.I_constr_r_red), sum(Rob.I_EE(4:6)));

phi = xE(4:6); % Euler-Winkel
Jw = euljac(phi, Rob.phiconv_W_E); % Euler-Jacobi-Matrix für EE-Orientierung
%% Belegung der Ausgabevariablen
% Plattform-Koppelpunkt-Jacobi
for i = 1:NLEG
  I1 = 3*(i-1)+1;
  % [A], Gl. 11
  phi = -Jw; % Term entspricht direkt dem Vorfaktor der Winkelgeschwindigkeit
  Phi_rr(I1:I1+2,:) = phi;
  
  J1 = sum(Rob.Leg(i).I_EE(4:6))*(i-1)+1;
  Phi_rr_red(J1:J1+sum(Rob.I_EE(4:6))-1,:) = phi(Rob.Leg(i).I_EE(4:6), Rob.I_EE(4:6));
end
