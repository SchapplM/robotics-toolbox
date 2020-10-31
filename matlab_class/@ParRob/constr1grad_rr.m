% Ableitung der Rotationskomponente der kinematischen ZB nach der EE-Orientierung
% 
% Variante 1:
% * Absolute Rotation (Basis->PKM-EE)ausgedrückt in XYZ-Euler-Winkeln
% * Rotationsfehler (Basis-PKM-EE) ausgedrückt in XYZ-Euler-Winkeln
%   Im Gegensatz zu [2_SchapplerTapOrt2019a] hier Rotation 0(q)-0(x)
% 
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% platform_frame [1x1 logical]
%   Benutze das Plattform-KS anstatt das EE-KS als Bezugsgröße für x
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
% [A] Aufzeichnungen Schappler vom 19.06.2018
% [B] Aufzeichnungen Schappler vom 21.06.2018
% [C] Aufzeichnungen Schappler vom 13.07.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phipphi_red, Phipphi] = constr1grad_rr(Rob, q, xE, platform_frame)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr1grad_rr: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1grad_rr: xE muss 6x1 sein');
if nargin == 3, platform_frame = false; end
NLEG = Rob.NLEG;

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
% Endergebnis: Gl. (C.35); entspricht [2_SchapplerTapOrt2019a]/(36)
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
if platform_frame
  R_P_E = eye(3);
else
  R_P_E = Rob.T_P_E(1:3,1:3);
end
R_0_E_x = eul2r(xE(4:6), Rob.phiconv_W_E);

K1 = 1;
for iLeg = 1:NLEG
  IJ_i = Rob.I1J_LEG(iLeg):Rob.I2J_LEG(iLeg);
  qs = q(IJ_i); % Gelenkwinkel dieser Kette
  
  phi_0_Ai = Rob.Leg(iLeg).phi_W_0;
  R_0_0i = eul2r(phi_0_Ai, Rob.Leg(iLeg).phiconv_W_0);
  R_P_Bi = eulxyz2r(Rob.phi_P_B_all(:,iLeg));
  R_Bi_P = R_P_Bi.';
  
  % Definitionen, Laden der Kinematik
  T_0i_Bi = Rob.Leg(iLeg).fkineEE(qs);
  R_0i_E_q = T_0i_Bi(1:3,1:3) * R_Bi_P * R_P_E;
  R_0_E_q = R_0_0i * R_0i_E_q;

  % Rotationsmatrix Differenz-Rotation.
  % Unterschied zu [2_SchapplerTapOrt2019a]
  R_0x_0q = (R_0_E_q * R_0_E_x');

  %% (III) Ableitung des Matrixproduktes
  % Dritter Term in (C.35); entspricht [2_SchapplerTapOrt2019a]/(36)/III
  % Hier aber andere Reihenfolge als in [2_SchapplerTapOrt2019a].
  % Gl. (B.12) bzw. [2_SchapplerTapOrt2019a]/(A24)
  % Unabhängig vom Roboter (Rotationsmatrix aus Kinematik wird in
  % Platzhalterelemente eingesetzt)
  % Aus rmatvecprod_diff_rmatvec1_matlab.m
  b11=R_0_E_q(1,1);b21=R_0_E_q(1,2);b31=R_0_E_q(1,3);
  b12=R_0_E_q(2,1);b22=R_0_E_q(2,2);b32=R_0_E_q(2,3);
  b13=R_0_E_q(3,1);b23=R_0_E_q(3,2);b33=R_0_E_q(3,3);
  dPidR1b = [b11 0 0 b21 0 0 b31 0 0; 0 b11 0 0 b21 0 0 b31 0; 0 0 b11 0 0 b21 0 0 b31; b12 0 0 b22 0 0 b32 0 0; 0 b12 0 0 b22 0 0 b32 0; 0 0 b12 0 0 b22 0 0 b32; b13 0 0 b23 0 0 b33 0 0; 0 b13 0 0 b23 0 0 b33 0; 0 0 b13 0 0 b23 0 0 b33;];

  %% (IV) Ableitung von R_0_E nach den Euler-Winkeln
  % Vierter Term in Gl. (C.35) bzw. in [2_SchapplerTapOrt2019a]/(36)
  % Unabhängig vom Roboter (nur von Orientierungsdarstellung)
  dR0Ebdphi = rotmat_diff_eul(xE(4:6), Rob.phiconv_W_E);
  
  %% (II) Transpositions-Matrix; Gl (C.29), [2_SchapplerTapOrt2019a]/(A19)
  % zweiter Term in Gl. (C.35) bzw. in [2_SchapplerTapOrt2019a]/(36)
  % Aus permat_rvec_transpose_matlab.m
  P_T = [1 0 0 0 0 0 0 0 0; 0 0 0 1 0 0 0 0 0; 0 0 0 0 0 0 1 0 0; 0 1 0 0 0 0 0 0 0; 0 0 0 0 1 0 0 0 0; 0 0 0 0 0 0 0 1 0; 0 0 1 0 0 0 0 0 0; 0 0 0 0 0 1 0 0 0; 0 0 0 0 0 0 0 0 1;];
  
  %% (I) Ableitung der Euler-Winkel nach der Rotationsmatrix
  % Erster Term in Gl. (C.35),(C.28),(B.16)
  % Es wird eine andere Matrix als in [2_SchapplerTapOrt2019a]/(36)
  % eingesetzt.
  % Aus P_3RRR/phixyz_diff_rmatvec_matlab.m
  % Unabhängig vom Roboter (nur von Orientierungsdarstellung)
  dphidRb = eul_diff_rotmat(R_0x_0q, Rob.phiconv_W_E);
  
  %% Gesamtergebnis
  % Gl. (C.35); entspricht [2_SchapplerTapOrt2019a]/(36)
  Phi_phi_i_Gradx = dphidRb * P_T*dPidR1b * dR0Ebdphi;
  
  %% Einsetzen in Ausgabevariable
  J1 = 1+3*(iLeg-1);
  J2 = J1+2;
  Phipphi(J1:J2,:) = Phi_phi_i_Gradx;
  
  % Ausgabe mit reduzierter Dimension
  K2 = K1+sum(Rob.Leg(iLeg).I_EE_Task(4:6))-1;
  Phipphi_red( K1:K2, 1:sum(Rob.I_EE(4:6)) ) = Phi_phi_i_Gradx(Rob.Leg(iLeg).I_EE_Task(4:6),Rob.I_EE(4:6));
  K1 = K2+1;
end

