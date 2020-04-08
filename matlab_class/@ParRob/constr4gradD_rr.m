% Ableitung der Rotationskomponente der kinematischen ZB nach der
% EE-Orientierung und Ableitung dieser (Gradienten-)Matrix nach der Zeit
% 
% Variante 4:
% * Bezogen auf Winkelgeschwindigkeit des Koppelpunktes Bi
%   (effektiv werden die Geschw.-ZB nach den Gelenk-Geschw. abgeleitet)
% 
% Eingabe:
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% xDE [6x1]
%   Zeitableitung der Endeffektorpose des Roboters bezüglich des Basis-KS
% 
% Ausgabe:
% PhiD_rr_red
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% PhiD_rr [3xN]
%   Ableitung der kinematischen Zwangsbedingungen nach der EE-Orientierung
%   Rotatorischer Teil

% Quelle:
% [A] Aufzeichnungen Schappler vom 13.02.2020

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [PhiD_rr_red, PhiD_rr] = constr4gradD_rr(Rob,xE,xDE)

%% Initialisierung
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr4gradD_rr: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr4gradD_rr: xED xDE 6x1 sein');
NLEG = Rob.NLEG;

PhiD_rr = zeros(3*NLEG,3);
PhiD_rr_red = zeros(length(Rob.I_constr_r_red), sum(Rob.I_EE(4:6)));

% Zeitableitung der Euler-Jacobi-Matrix für EE-Orientierung
JwD = euljacD(xE(4:6), xDE(4:6), Rob.phiconv_W_E);
%% Belegung der Ausgabevariablen
% Plattform-Koppelpunkt-Jacobi
for i = 1:NLEG
  I1 = 3*(i-1)+1;
  % [A], Gl. 11
  phi = -JwD; % Zeitableitung des entsprechenden Terms aus constr4grad_rr
  PhiD_rr(I1:I1+2,:) = phi;
  
  J1 = sum(Rob.Leg(i).I_EE_Task(4:6))*(i-1)+1;
  PhiD_rr_red(J1:J1+sum(Rob.Leg(i).I_EE_Task(4:6))-1,:) = phi(Rob.Leg(i).I_EE_Task(4:6), Rob.I_EE(4:6));
end