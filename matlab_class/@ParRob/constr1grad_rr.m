% Ableitung der Rotationskomponente der kinematischen ZB nach der EE-Orientierung
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
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% 
% Ausgabe:
% Phipphi_red
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% Phipphi [3xN]
%   Ableitung der kinematischen Zwangsbedingungen nach der EE-Orientierung
%   Rotatorischer Teil

% Quellen:
% [A] Aufzeichnungen Schappler vom 19.06.2018
% [B] Aufzeichnungen Schappler vom 21.06.2018
% [C] Aufzeichnungen Schappler vom 13.07.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phipphi_red, Phipphi] = constr1grad_rr(Rob, q, xE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr1grad_rr: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1grad_rr: xE muss 6x1 sein');
NLEG = Rob.NLEG;

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
% Endergebnis: Gl. (C.35)
% Initialisierung mit Fallunterscheidung für symbolische Eingabe

if ~Rob.issym
  Phipphi = zeros(3*NLEG,3);
  Phipphi_red = zeros( length(Rob.I_constr_r_red), sum(Rob.I_EE(4:6)) );
else
  Phipphi = sym('xx',     [3*NLEG,3]);
  Phipphi(:)=0;
  Phipphi_red = sym('xx', [length(Rob.I_constr_r_red), sum(Rob.I_EE(4:6))]);
  Phipphi_red(:)=0;
end

%% Berechnung
R_P_E = Rob.T_P_E(1:3,1:3);
R_0_E_x = eul2r(xE(4:6), Rob.phiconv_W_E);

for iLeg = 1:NLEG
  IJ_i = Rob.I1J_LEG(iLeg):Rob.I2J_LEG(iLeg);
  qs = q(IJ_i); % Gelenkwinkel dieser Kette
  
  phi_0_Ai = Rob.Leg(iLeg).phi_W_0;
  R_0_0i = eul2r(phi_0_Ai, Rob.Leg(iLeg).phiconv_W_0);
  
  % Definitionen, Laden der Kinematik
  T_0i_Bi = Rob.Leg(iLeg).fkineEE(qs);
  R_0i_E_q = T_0i_Bi(1:3,1:3) * R_P_E;
  R_0_E_q = R_0_0i * R_0i_E_q;

  % Rotationsmatrix Differenz-Rotation. So Definiert wie in den
  % Zwangsbedingungen bsp6uPs_ZB_phi
  R_0x_0q = (R_0_E_q * R_0_E_x');

  % Dritter Termin in Gl. (C.35): Ableitung des Matrixproduktes
  % Gl. (B.12)
  % Unabhängig vom Roboter (Rotationsmatrix aus Kinematik wird in
  % Platzhalterelemente eingesetzt)
  % Aus rmatvecprod_diff_rmatvec1_matlab.m
  b11=R_0_E_q(1,1);b21=R_0_E_q(1,2);b31=R_0_E_q(1,3);
  b12=R_0_E_q(2,1);b22=R_0_E_q(2,2);b32=R_0_E_q(2,3);
  b13=R_0_E_q(3,1);b23=R_0_E_q(3,2);b33=R_0_E_q(3,3);
  dPidR1b = [b11 0 0 b21 0 0 b31 0 0; 0 b11 0 0 b21 0 0 b31 0; 0 0 b11 0 0 b21 0 0 b31; b12 0 0 b22 0 0 b32 0 0; 0 b12 0 0 b22 0 0 b32 0; 0 0 b12 0 0 b22 0 0 b32; b13 0 0 b23 0 0 b33 0 0; 0 b13 0 0 b23 0 0 b33 0; 0 0 b13 0 0 b23 0 0 b33;];

  % vierter Termin in Gl. (C.35): Ableitung von R_0_E nach den Euler-Winkeln
  % Unabhängig vom Roboter (nur von Orientierungsdarstellung)
  dR0Ebdphi = rotmat_diff_eul(xE(4:6), Rob.phiconv_W_E);
  
  % Transpositions-Matrix; Gl (C.29)
  % zweiter Term in Gl. (C.35)
  % Aus permat_rvec_transpose_matlab.m
  P_T = [1 0 0 0 0 0 0 0 0; 0 0 0 1 0 0 0 0 0; 0 0 0 0 0 0 1 0 0; 0 1 0 0 0 0 0 0 0; 0 0 0 0 1 0 0 0 0; 0 0 0 0 0 0 0 1 0; 0 0 1 0 0 0 0 0 0; 0 0 0 0 0 1 0 0 0; 0 0 0 0 0 0 0 0 1;];
  
  % Erster Term in Gl. (C.35),(C.28),(B.16): Ableitung der Euler-Winkel nach der
  % Rotationsmatrix
  % Aus P_3RRR/phixyz_diff_rmatvec_matlab.m
  % Unabhängig vom Roboter (nur von Orientierungsdarstellung)
  dphidRb = eul_diff_rotmat(R_0x_0q, Rob.phiconv_W_E);
  
  % Gl. (C.35)
  Phi_phi_i_Gradx = dphidRb * P_T*dPidR1b * dR0Ebdphi;
  
  J1 = 1+3*(iLeg-1);
  J2 = J1+2;
  Phipphi(J1:J2,:) = Phi_phi_i_Gradx;
  
  % Ausgabe mit reduzierter Dimension
  K1 = 1+sum(Rob.I_EE(4:6))*(iLeg-1);
  K2 = K1+sum(Rob.I_EE(4:6))-1;
  Phipphi_red( K1:K2, 1:sum(Rob.I_EE(4:6)) ) = Phi_phi_i_Gradx(Rob.Leg(iLeg).I_EE(4:6),Rob.I_EE(4:6));
end
