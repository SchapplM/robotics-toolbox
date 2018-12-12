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

function [Phix_phi_red, Phix_phi] = constr1grad_tr(Rob, xE)

%% Initialisierung
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1grad_tr: xE muss 6x1 sein');
NLEG = Rob.NLEG;

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
issym=checksym(xE);
dim_P_tr = [3*NLEG,3];
dim_P_tr_red = [sum(Rob.I_EE(1:3))*NLEG, sum(Rob.I_EE(4:6))];
if ~issym
  Phix_phi = zeros(dim_P_tr);
  Phix_phi_red = zeros(dim_P_tr_red);
else
  Phix_phi = sym('xx', dim_P_tr);
  Phix_phi_red = sym('xx', dim_P_tr_red);
  Phix_phi(:)=0;
  Phix_phi_red(:) = 0;
end

phi = xE(4:6); % Euler-Winkel
R_0_E = eul2r(phi, Rob.phiconv_W_E);
r_P_B_all = Rob.r_P_B_all;
T_P_E = Rob.T_P_E;
r_P_P_E = T_P_E(1:3,4);
%% Berechnung
% Plattform-Koppelpunkt-Jacobi
for i = 1:NLEG
  % translatorischer Anteil
  r_P_P_Bi = r_P_B_all(:,i);
  r_E_E_Bi = T_P_E(1:3,1:3)' * (-r_P_P_E + r_P_P_Bi);
  Jw = euljac(phi, Rob.phiconv_W_E);
  
  % Auf vorhandene Koordinaten reduzieren:
  % Auswahl [1 2 3]: x-y-z-Komponenten der translatorischen Zwangsbedingungen
  % Auswahl [1 2 3]: phix, phiy, phiz (z.B. für XYZ-Euler-Winkel)
  I1 = 3*(i-1)+1;
  % Gl. (A.36-37)
  phi_xp = skew(R_0_E*r_E_E_Bi)*Jw;
  Phix_phi(I1:I1+2,:) = phi_xp;
  
  J1 = sum(Rob.I_EE(1:3))*(i-1)+1;
  Phix_phi_red(J1:J1+sum(Rob.I_EE(1:3))-1,:) = phi_xp(Rob.I_EE(1:3),Rob.I_EE(4:6));
end
