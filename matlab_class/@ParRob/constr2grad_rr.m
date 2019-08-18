% Ableitung der Rotationskomponente der kinematischen ZB nach der EE-Orientierung
% 
% Variante 2:
% * Absolute Rotation (Basis->PKM-EE)ausgedrückt in XYZ-Euler-Winkeln
% * Rotationsfehler (Basis-PKM-EE) ausgedrückt in ZYX-Euler-Winkeln
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
% [2_SchapplerTapOrt2019a] Schappler, M. et al.: Modeling Parallel Robot
% Kinematics for 3T2R and 3T3R Tasks using Reciprocal Sets of Euler Angles
% (Arbeitstitel), Submitted to MDPI Robotics KaRD2, Version of 27.06.2019
% [A] Aufzeichnungen Schappler vom 27.07.2018
% [B] Aufzeichnungen Schappler vom 21.06.2018
% [C] Aufzeichnungen Schappler vom 13.07.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phipphi_red, Phipphi] = constr2grad_rr(Rob, q, xE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr2grad_rr: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr2grad_rr: xE muss 6x1 sein');
NLEG = Rob.NLEG;

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
% Endergebnis: [2_SchapplerTapOrt2019a]/(36); Gl. (C.35)

% Initialisierung mit Fallunterscheidung für symbolische Eingabe
if ~Rob.issym
  Phipphi = zeros(3*NLEG,3);
  Phipphi_red = zeros( sum(Rob.I_EE(4:6))*NLEG, sum(Rob.I_EE(4:6)) );
else
  Phipphi = sym('xx',     [3*NLEG,3]);
  Phipphi(:)=0;
  Phipphi_red = sym('xx', [sum(Rob.I_EE(4:6))*NLEG, sum(Rob.I_EE(4:6))]);
  Phipphi_red(:)=0;
end

%% Berechnung
R_P_E = Rob.T_P_E(1:3,1:3);
R_0_E_x = eul2r(xE(4:6), Rob.phiconv_W_E);
[~,phiconv_W_E_reci] = euler_angle_properties(Rob.phiconv_W_E);

for iLeg = 1:NLEG
  IJ_i = Rob.I1J_LEG(iLeg):Rob.I2J_LEG(iLeg);
  qs = q(IJ_i); % Gelenkwinkel dieser Kette
  
  phi_0_Ai = Rob.Leg(iLeg).phi_W_0;
  R_0_0i = eul2r(phi_0_Ai, Rob.Leg(iLeg).phiconv_W_0);
  
  % Definitionen, Laden der Kinematik
  T_0i_Bi = Rob.Leg(iLeg).fkineEE(qs);
  R_0i_E_q = T_0i_Bi(1:3,1:3) * R_P_E;
  R_0_E_q = R_0_0i * R_0i_E_q;

  % Rotationsmatrix Differenz-Rotation. 
  R_Ex_Eq = R_0_E_x' * R_0_E_q; % Argument in [2_SchapplerTapOrt2019a]/(19)

  %% (III) Ableitung des Matrixproduktes
  % Dritter Term in [2_SchapplerTapOrt2019a]/(36) bzw. Gl. (A.50): 
  % [2_SchapplerTapOrt2019a]/(A23); Gl. (B.12)
  % Unabhängig vom Roboter (Rotationsmatrix aus Kinematik wird in
  % Platzhalterelemente eingesetzt)
  % Aus rmatvecprod_diff_rmatvec1_matlab.m
  a11=R_0_E_q(1,1);a21=R_0_E_q(1,2);a31=R_0_E_q(1,3);
  a12=R_0_E_q(2,1);a22=R_0_E_q(2,2);a32=R_0_E_q(2,3);
  a13=R_0_E_q(3,1);a23=R_0_E_q(3,2);a33=R_0_E_q(3,3);
  dPidR2b = [a11 a12 a13 0 0 0 0 0 0; a21 a22 a23 0 0 0 0 0 0; a31 a32 a33 0 0 0 0 0 0; 0 0 0 a11 a12 a13 0 0 0; 0 0 0 a21 a22 a23 0 0 0; 0 0 0 a31 a32 a33 0 0 0; 0 0 0 0 0 0 a11 a12 a13; 0 0 0 0 0 0 a21 a22 a23; 0 0 0 0 0 0 a31 a32 a33;];

  %% (IV) Ableitung von R_0_E nach den Euler-Winkeln
  % Vierter Term in [2_SchapplerTapOrt2019a]/(36) bzw. Gl. (A.50)
  % (XYZ-Euler-Winkel der Endeffektor-Orientierung)
  % Unabhängig vom Roboter (nur von Orientierungsdarstellung)
  dR0Ebdphi = rotmat_diff_eul(xE(4:6), Rob.phiconv_W_E);
  
  %% (II) Transpositions-Matrix;
  % Gl (C.29), [2_SchapplerTapOrt2019a]/(A19)
  % zweiter Term in [2_SchapplerTapOrt2019a]/(36) bzw. (A.50)
  % Aus permat_rvec_transpose_matlab.m
  P_T = [1 0 0 0 0 0 0 0 0; 0 0 0 1 0 0 0 0 0; 0 0 0 0 0 0 1 0 0; 0 1 0 0 0 0 0 0 0; 0 0 0 0 1 0 0 0 0; 0 0 0 0 0 0 0 1 0; 0 0 1 0 0 0 0 0 0; 0 0 0 0 0 1 0 0 0; 0 0 0 0 0 0 0 0 1;];
  
  %% (I) Ableitung der Euler-Winkel nach der Rotationsmatrix
  % Erster Term in Gl. (A.50): 
  % (ZYX-Euler-Winkel des Orientierungsfehlers)
  % Aus eulzyx_diff_rmatvec_matlab.m
  % Unabhängig vom Roboter (nur von Orientierungsdarstellung) 
  dphidRb = eul_diff_rotmat(R_Ex_Eq, phiconv_W_E_reci);
  
  %% Gesamtergebnis
  % [2_SchapplerTapOrt2019a]/(36) bzw. Gl. (A.50)
  Phi_phi_i_Gradx = dphidRb * P_T*dPidR2b * dR0Ebdphi;
  
  %% Einsetzen in Ausgabevariable
  J1 = 1+3*(iLeg-1);
  J2 = J1+2;
  Phipphi(J1:J2,:) = Phi_phi_i_Gradx;
  
  % Ausgabe mit reduzierter Dimension
  K1 = 1+sum(Rob.I_EE(4:6))*(iLeg-1);
  K2 = K1+sum(Rob.I_EE(4:6))-1;
  Phipphi_red( K1:K2, 1:sum(Rob.I_EE(4:6)) ) = Phi_phi_i_Gradx(Rob.I_EE(4:6),Rob.I_EE(4:6));
end
