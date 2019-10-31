% Rotationskomponente der kinematischen ZB zwischen Ist- und Soll-Konfiguration
% Vollständige Rotations- und Translationskomponenten
% Variante 1:
% * Absolute Rotation ausgedrückt bspw. in XYZ-Euler-Winkeln
%   (statt XYZ wird die Konvention aus `phiconv_W_E` genommen)
% * Rotationsfehler ausgedrückt genauso bspw. in XYZ-Euler-Winkeln
%   Im Gegensatz zu [2_SchapplerTapOrt2019a] hier Rotation 0(q)-0(x)
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

% Quellen:
% [2_SchapplerTapOrt2019a] Schappler, M. et al.: Modeling Parallel Robot
% Kinematics for 3T2R and 3T3R Tasks using Reciprocal Sets of Euler Angles
% (Arbeitstitel), Submitted to MDPI Robotics KaRD2, Version of 27.06.2019
% [A] Aufzeichnungen Schappler vom 21.06.2018
% [B] Aufzeichnungen Schappler vom 22.06.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phi_red, Phi] = constr1D_rot(Rob, q,qD, xE ,xDE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr1D_rot: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/constr1D_rot: qD muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1D_rot: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr1D_rot: xDE muss 6x1 sein');
% assert(all(size(rpy) == [3 1]) && isreal(rpy), ...
%   'ParRob/constr1D_rot: rpy angles have to be [3x1] (double)'); 
% assert(all(size(rpyD) == [3 1]) && isreal(rpyD), ...
%   'ParRob/constr1D_rot: rpy angles time derivatives have to be [3x1] (double)'); 

NLEG = Rob.NLEG;
Phi = NaN(length(Rob.I_constr_r),1);
Phi_red = NaN(length(Rob.I_constr_r_red),1);

R_P_E = Rob.T_P_E(1:3,1:3);

%% Berechnung
R_0_E_x = eul2r(xE(4:6), Rob.phiconv_W_E);

 JR  = euljac (xE(4:6), Rob.phiconv_W_E) ;
  omega  = JR * xDE(4:6) ;
  RD_0_E_x =   R_0_E_x * skew(omega);


for iLeg = 1:NLEG
  % Anteil der ZB-Gleichung der Gelenkkette
  % (Aus direkter Kinematik)
  IJ_i = Rob.I1J_LEG(iLeg):Rob.I2J_LEG(iLeg);
  qs = q(IJ_i);% Gelenkwinkel dieser Kette
  qds= qD(IJ_i);
  T_0i_Bi = Rob.Leg(iLeg).fkineEE(qs);
  
  % Fußpunkt-Orientierung
  phi_0_Ai = Rob.Leg(iLeg).phi_W_0;
  R_0_0i = eul2r(phi_0_Ai, Rob.Leg(iLeg).phiconv_W_0);


  
  % Differenz-Rotation (z.B. mit XYZ-Euler-Winkel)
  % Kette: 0 -> 0i -> Bi -> E
  % (Bi und P sind gleich orientiert)
  R_0_E_q = R_0_0i * T_0i_Bi(1:3,1:3) * R_P_E;
 %R_0_E_qD = R_0_0i * T_0i_Bi(1:3,1:3) * R_P_E    ;
  
  %R_0q_0x = R_0_E_q * R_0_E_x'; % Unterschied zu [2_SchapplerTapOrt2019a]
  
   R_Bi_P  = 1 ;
  omegajr   = Rob.Leg(iLeg).jacobiw(qs) * qds ;
  RD_0i_Bi = T_0i_Bi(1:3,1:3) * skew (omegajr) ;
  RD_0_E_q = R_0_0i * RD_0i_Bi * R_Bi_P * R_P_E ;
  

  
  RD_0q_0x = (R_0_E_q * RD_0_E_x') + (RD_0_E_q * R_0_E_x') ;
  

  % Gl. (A.1, B.25); ähnlich wie [2_SchapplerTapOrt2019a]/(19)
  %phi = r2eul(R_0q_0x, Rob.phiconv_W_E);
   phi = r2eul(RD_0q_0x, Rob.phiconv_W_E);
  J1 = 1+3*(iLeg-1);
  J2 = J1+2;
  Phi(J1:J2,:) = phi;
end

% Reduzierte Zwangsbedingungsgleichungen, für reduzierte EE-FG
K1 = 1; % Zähler für Zwangsbedingungen
for iLeg = 1:NLEG
  % Indizes für volle ZB
  J1 = 1+3*(iLeg-1);
  J2 = J1+2;
  % Indizes für reduzierte ZB
  K2 = K1+sum(Rob.Leg(iLeg).I_EE_Task(4:6))-1; % Anzahl der rot. ZB. dieser Beinkette

  % Auswahl der wirklich benötigten Einträge
  Phi_i = Phi(J1:J2,:);
  Phi_red(K1:K2,:) = Phi_i(Rob.Leg(iLeg).I_EE_Task(4:6));
  K1 = K2 + 1;
end
