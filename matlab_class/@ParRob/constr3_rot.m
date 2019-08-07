% Rotationskomponente der kinematischen ZB zwischen Ist- und Soll-Konfiguration
% Vollständige Rotations- und Translationskomponenten
% Variante 3:
% Implementierung mit Führungs-Beinkette und Folge-Beinketten 
% 
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% 
% Ausgabe:
% Phi_red
%   Reduzierte kinematische Zwangsbedingungen (siehe folgendes)
%   Die Reduktion folgt aus der Klassenvariablen I_EE
% Phi [6Mx1]
%   Kinematische Zwangsbedingungen des Roboters für alle M Beine: 
%   Maß für den Orientierungsfehler zwischen Ist-Pose aus
%   gegebenen Gelenkwinkeln q und Soll-Pose aus gegebenen EE-Koordinaten x

% [A] Aufzeichnungen Schappler vom 27.07.2018
% [B] Aufzeichnungen Schappler vom 02.02.2019

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phi_red, Phi] = constr3_rot(Rob, q, xE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr1_rot: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr2_rot: xE muss 6x1 sein');
NLEG = Rob.NLEG;
Phi = NaN(3*NLEG,1);
Phi_red = NaN(length(Rob.I_constr_r_red),1);

R_P_E = Rob.T_P_E(1:3,1:3);

%% Berechnung
R_0_E_x = eul2r(xE(4:6), Rob.phiconv_W_E); 
[~,phiconv_W_E_reci] = euler_angle_properties(Rob.phiconv_W_E);
K1 = 1; % Zähler für Zwangsbedingungen
for iLeg = 1:NLEG
  % Anteil der ZB-Gleichung der Gelenkkette
  % (Aus direkter Kinematik)
  IJ_i = Rob.I1J_LEG(iLeg):Rob.I2J_LEG(iLeg);
  qs = q(IJ_i); % Gelenkwinkel dieser Kette
  T_0i_Bi = Rob.Leg(iLeg).fkineEE(qs);
  
  % Fußpunkt-Orientierung
  phi_0_Ai = Rob.Leg(iLeg).phi_W_0;
  R_0_0i = eul2r(phi_0_Ai, Rob.phiconv_W_0);
  
  % Differenz-Rotation (z.B. mit XYZ-Euler-Winkel)
  % Kette: 0 -> 0i -> Bi -> E
  % (Bi und P sind gleich orientiert)

  if iLeg == 1 % Führungskette
    R_0_E_q_L = R_0_0i * T_0i_Bi(1:3,1:3) * R_P_E;
    % [B] Gl. 16
    R_Ex_Eq = R_0_E_x' * R_0_E_q_L;
    phiL = r2eul(R_Ex_Eq, phiconv_W_E_reci); 
    J1 = 1+3*(iLeg-1);
    J2 = J1+2;
    Phi(J1:J2,:) = phiL;
  
  elseif iLeg > 1 % Folge-Kette
    
    R_0_E_q_f = R_0_0i * T_0i_Bi(1:3,1:3) * R_P_E; 
    % [B] Gl. 18
    R_Lq_Eq = R_0_E_q_L' * R_0_E_q_f;
    phif = r2eul(R_Lq_Eq, phiconv_W_E_reci); 
    J1 = 1+3*(iLeg-1);
    J2 = J1+2;
    Phi(J1:J2,:) = phif;   
  end

  % Reduzierte Zwangsbedingungsgleichungen, für reduzierte EE-FG
  % Indizes für volle ZB
  J1 = 1+3*(iLeg-1);
  J2 = J1+2;
  % Indizes für reduzierte ZB
  K2 = K1+sum(Rob.Leg(iLeg).I_EE_Task(4:6))-1; % Anzahl der rot. ZB. dieser Beinkette

  % Auswahl der wirklich benötigten Einträge
  Phi_i = Phi(J1:J2,:);
  if all(Rob.Leg(iLeg).I_EE_Task == logical([1 1 1 1 1 0]))
    Phi_red(K1:K2,:) = Phi_i([2 3]);
  else
    Phi_red(K1:K2,:) = Phi_i(Rob.Leg(iLeg).I_EE_Task(4:6));
  end
  K1 = K2 + 1;
end