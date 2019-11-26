% Ableitung der Rotationskomponente der kinematischen ZB nach der
% EE-Orientierung und Ableitung dieser (Gradienten-)Matrix nach der Zeit
% 
% Variante 1:
% * Absolute Rotation (Basis->PKM-EE)ausgedrückt in XYZ-Euler-Winkeln
% * Rotationsfehler (Basis-PKM-EE) ausgedrückt in XYZ-Euler-Winkeln
% 
% Implementierung mit Einzelmatrizen (nicht mit Endergebnis aus Maple)
% 
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% qD [Nx1]
%   Geschwindigkeit aller Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% xDE [6x1]
%   Zeitableitung der Endeffektorpose des Roboters bezüglich des Basis-KS
% 
% Ausgabe:
% PhiDpphi_red
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% PhiDpphi [3xN]
%   Ableitung der kinematischen Zwangsbedingungen nach der EE-Orientierung
%   Rotatorischer Teil
% 
% Siehe auch: constr1grad_rr.m, constr1gradD_rq.m

% Quellen:
% [Sriram2019_S876] Implementation of a Structurally Independant Dynamics
% Modelling of Parallel Kinematic Machines based on Full Kinematic
% Constraints (Studienarbeit bei Moritz Schappler)
% [A] Aufzeichnungen Moritz Schappler vom 31.10. und 1.11.2019


% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [PhiDpphi_red, PhiDpphi] = constr1gradD_rr(Rob, q, qD, xE ,xDE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr1gradD_rr: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/constr1gradD_rr: qD muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1gradD_rr: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr1gradD_rr: xDE muss 6x1 sein');

NLEG = Rob.NLEG;

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
% Endergebnis: Gl. (C.35)
% Initialisierung mit Fallunterscheidung für symbolische Eingabe

if ~Rob.issym
  PhiDpphi = zeros(3*NLEG,3);
  PhiDpphi_red = zeros( length(Rob.I_constr_r_red), sum(Rob.I_EE(4:6)) );
else
  PhiDpphi = sym('xx',     [3*NLEG,3]);
  PhiDpphi(:)=0;
  PhiDpphi_red = sym('xx', [length(Rob.I_constr_r_red), sum(Rob.I_EE(4:6))]);
  PhiDpphi_red(:)=0;
end

%% Berechnung
R_P_E = Rob.T_P_E(1:3,1:3);
R_0_E_x = eul2r(xE(4:6), Rob.phiconv_W_E); % the p
omega_0_Ex  = euljac (xE(4:6), Rob.phiconv_W_E) * xDE(4:6);
RD_0_E_x =  R_0_E_x * skew(omega_0_Ex);
R_Bi_P = eye(3,3);

K1 = 1;
for iLeg = 1:NLEG
  IJ_i = Rob.I1J_LEG(iLeg):Rob.I2J_LEG(iLeg);
  q_i = q(IJ_i); % Gelenkwinkel dieser Kette
  qD_i= qD(IJ_i);
  phi_0_Ai = Rob.Leg(iLeg).phi_W_0;
  R_0_0i = eul2r(phi_0_Ai, Rob.Leg(iLeg).phiconv_W_0);
  
  % Definitionen, Laden der Kinematik
  T_0i_Bi = Rob.Leg(iLeg).fkineEE(q_i);
  R_0i_E_q = T_0i_Bi(1:3,1:3) * R_P_E;
  R_0_E_q = R_0_0i * R_0i_E_q;
  
  % Rotationsmatrix des Orientierungsfehlers
  R_0x_0q = (R_0_E_q * R_0_E_x');
  
  % Ableitungen der Rotationsmatrizen berechnen
  omega_0i_Bi   = Rob.Leg(iLeg).jacobiw(q_i) *qD_i;
  RD_0i_Bi = T_0i_Bi(1:3,1:3) * skew (omega_0i_Bi);
  RD_0_E_q = R_0_0i * RD_0i_Bi * R_Bi_P * R_P_E;
  RD_0x_0q = (R_0_E_q * RD_0_E_x') + (RD_0_E_q * R_0_E_x');

  %% (III) Ableitung des Matrixproduktes
  % Setze EE-Rotationsmatrix transponiert ein
  b11=R_0_E_q(1,1);b21=R_0_E_q(1,2);b31=R_0_E_q(1,3);
  b12=R_0_E_q(2,1);b22=R_0_E_q(2,2);b32=R_0_E_q(2,3);
  b13=R_0_E_q(3,1);b23=R_0_E_q(3,2);b33=R_0_E_q(3,3);
  dPidR1b = [b11 0 0 b21 0 0 b31 0 0; 0 b11 0 0 b21 0 0 b31 0; 0 0 b11 0 0 b21 0 0 b31; b12 0 0 b22 0 0 b32 0 0; 0 b12 0 0 b22 0 0 b32 0; 0 0 b12 0 0 b22 0 0 b32; b13 0 0 b23 0 0 b33 0 0; 0 b13 0 0 b23 0 0 b33 0; 0 0 b13 0 0 b23 0 0 b33;];
  
  % Berechnung der Zeitableitung:
  % Setze Zeitableitung der EE-Rotationsmatrix transponiert in Matrix ein.
  % Da die Elemente der Rotationsmatrix nur eingesetzt werden, wird
  % elementweise abgeleitet
  c11=RD_0_E_q(1,1);c21=RD_0_E_q(1,2);c31=RD_0_E_q(1,3);
  c12=RD_0_E_q(2,1);c22=RD_0_E_q(2,2);c32=RD_0_E_q(2,3);
  c13=RD_0_E_q(3,1);c23=RD_0_E_q(3,2);c33=RD_0_E_q(3,3);
  dDPidR1b = [c11 0 0 c21 0 0 c31 0 0; 0 c11 0 0 c21 0 0 c31 0; 0 0 c11 0 0 c21 0 0 c31; c12 0 0 c22 0 0 c32 0 0; 0 c12 0 0 c22 0 0 c32 0; 0 0 c12 0 0 c22 0 0 c32; c13 0 0 c23 0 0 c33 0 0; 0 c13 0 0 c23 0 0 c33 0; 0 0 c13 0 0 c23 0 0 c33;];
  
  %% (IV) Ableitung von R_0_E nach den Euler-Winkeln
  dR0Ebdphi  = rotmat_diff_eul(xE(4:6), Rob.phiconv_W_E);
  dR0EbdphiD = rotmatD_diff_eul(xE(4:6), xDE(4:6), Rob.phiconv_W_E);
  
  %% (II) Transpositions-Matrix; Gl (C.29), [2_SchapplerTapOrt2019a]/(A19)
  P_T = [1 0 0 0 0 0 0 0 0; 0 0 0 1 0 0 0 0 0; 0 0 0 0 0 0 1 0 0; 0 1 0 0 0 0 0 0 0; 0 0 0 0 1 0 0 0 0; 0 0 0 0 0 0 0 1 0; 0 0 1 0 0 0 0 0 0; 0 0 0 0 0 1 0 0 0; 0 0 0 0 0 0 0 0 1;];
  
  %% (I) Ableitung der Euler-Winkel nach der Rotationsmatrix
  dphidRb = eul_diff_rotmat(R_0x_0q,  Rob.phiconv_W_E);
  dDphidRb = eulD_diff_rotmat(R_0x_0q, RD_0x_0q,  Rob.phiconv_W_E);
  
  %% Gesamtergebnis
  % Produktregel des Ergebnisses aus constr1grad_rr
  Phi_phi_i_GradxD = dDphidRb * P_T * dPidR1b  * dR0Ebdphi ...% AD P B  C
                   + dphidRb  * P_T * dDPidR1b * dR0Ebdphi... % A  P BD C
                   + dphidRb  * P_T * dPidR1b  * dR0EbdphiD;  % A  P B  CD
                 
  %% Einsetzen in Ausgabevariable
  J1 = 1+3*(iLeg-1);
  J2 = J1+2;
  PhiDpphi(J1:J2,:) = Phi_phi_i_GradxD;
  
  % Ausgabe mit reduzierter Dimension
  K2 = K1+sum(Rob.Leg(iLeg).I_EE_Task(4:6))-1; % drei rotatorische Einträge
  PhiDpphi_red( K1:K2, 1:sum(Rob.I_EE(4:6)) ) = Phi_phi_i_GradxD(Rob.Leg(iLeg).I_EE_Task(4:6),Rob.I_EE(4:6));
  K1 = K2+1;
end
