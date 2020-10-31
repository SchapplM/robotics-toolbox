% Ableitung der Translationskomponente der kinematischen ZB nach der
% EE-Orientierung und Ableitung dieser (Gradienten-)Matrix nach der Zeit
% Rotation ausgedrückt in XYZ-Euler-Winkeln
% 
% Eingabe:
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% xDE [6x1]
%   Zeitableitung der Endeffektorpose des Roboters bezüglich des Basis-KS
% platform_frame [1x1 logical]
%   Benutze das Plattform-KS anstatt das EE-KS als Bezugsgröße für x
% 
% Ausgabe:
% PhiDx_phi_red
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% PhiDx_phi [3xN]
%   Ableitung der Translations-ZB nach der EE-Orientierung und nach der Zeit.
% 
% Siehe auch: constr1grad_tr

% Quellen:
% [A] Aufzeichnungen Moritz Schappler vom 31.10. und 1.11.2019


% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [PhiDx_phi_red, PhiDx_phi] = constr1gradD_tr(Rob, xE, xDE, platform_frame)
%% Initialisierung
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1gradD_tr: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr1gradD_tr: xDE muss 6x1 sein');
if nargin == 3, platform_frame = false; end
NLEG = Rob.NLEG;

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
dim_P_tr = [3*NLEG,3];
dim_P_tr_red = [length(Rob.I_constr_t_red), sum(Rob.I_EE(4:6))];
if ~Rob.issym
  PhiDx_phi = zeros(dim_P_tr);
  PhiDx_phi_red = zeros(dim_P_tr_red);
else
  PhiDx_phi = sym('xx', dim_P_tr);
  PhiDx_phi_red = sym('xx', dim_P_tr_red);
  PhiDx_phi(:)=0;
  PhiDx_phi_red(:) = 0;
end

phi = xE(4:6); % Euler-Winkel
phiD = xDE(4:6); % Euler-Winkel-Zeitableitung
R_0_E = eul2r(phi, Rob.phiconv_W_E); % EE-Rotationsmatrix

r_P_B_all = Rob.r_P_B_all;
if platform_frame
  T_P_E = eye(4);
else
  T_P_E = Rob.T_P_E;
end
r_P_P_E = T_P_E(1:3,4);
%% Berechnung
% Plattform-Koppelpunkt-Jacobi
for i = 1:NLEG
  % translatorischer Anteil
  r_P_P_Bi = r_P_B_all(:,i);
  r_E_E_Bi = T_P_E(1:3,1:3)' * (-r_P_P_E + r_P_P_Bi);
  
  % Komponenten der Gradientenmatrix aus constr1grad_tr
  A = skew(R_0_E*r_E_E_Bi);
  Jw = euljac(phi, Rob.phiconv_W_E);
  
  % Zeitableitung der beiden Teil-Matrizen mit Produktregel
  omega_0E  = Jw * phiD;
  RD_0_E =  skew(omega_0E) * R_0_E; % differentiation of the rotation matrix
  AD = skew( RD_0_E * r_E_E_Bi);
  JwD = euljacD(phi, phiD, Rob.phiconv_W_E);
  
  % Auf vorhandene Koordinaten reduzieren:
  % Auswahl [1 2 3]: x-y-z-Komponenten der translatorischen Zwangsbedingungen
  % Auswahl [1 2 3]: phix, phiy, phiz (z.B. für XYZ-Euler-Winkel)
  I1 = 3*(i-1)+1;
  phiD_xp = AD*Jw + A*JwD; % Ergebnis mit Produktregel für Differentiation
  PhiDx_phi(I1:I1+2,:) = phiD_xp;
  
  J1 = sum(Rob.I_EE(1:3))*(i-1)+1;
  PhiDx_phi_red(J1:J1+sum(Rob.Leg(i).I_EE(1:3))-1,:) = phiD_xp(Rob.Leg(i).I_EE(1:3),Rob.I_EE(4:6));
end
