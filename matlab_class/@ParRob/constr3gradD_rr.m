% Ableitung der Rotationskomponente der kinematischen ZB nach der EE-Orientierung
% 
% Variante 3:
% * Absolute Rotation (Basis->PKM-EE)ausgedrückt in XYZ-Euler-Winkeln
% * Rotationsfehler (Basis-PKM-EE) ausgedrückt in ZYX-Euler-Winkeln
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
% % [2_SchapplerTapOrt2019a] Schappler, M. et al.: Modeling Parallel Robot
% Kinematics for 3T2R and 3T3R Tasks using Reciprocal Sets of Euler Angles
% (Arbeitstitel), Submitted to MDPI Robotics KaRD2, Version of 27.06.2019
% [A] Aufzeichnungen Schappler vom 27.07.2018
% [B] Aufzeichnungen Schappler vom 21.06.2018
% [C] Aufzeichnungen Schappler vom 13.07.2018
% [D] Aufzeichnungen Schappler vom 03.02.2019
% 
% Siehe auch: constr2grad_rr.m (für Führungskette identisch)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [PhiDpphi_red, PhiDpphi] = constr3gradD_rr(Rob, q, qD, xE, xDE, platform_frame)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr3gradD_rr: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/constr3gradD_rr: qD muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr3gradD_rr: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr3gradD_rr: xDE muss 6x1 sein');
if nargin == 5, platform_frame = false; end
NLEG = Rob.NLEG;

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
Leg_I_EE_Task = cat(1,Rob.Leg(:).I_EE_Task);
rownum_Phipq_red = sum(Leg_I_EE_Task(1,4:6))+... % für Führungskette
  sum(sum(Leg_I_EE_Task(2:end,4:6))); % für Folgeketten
% Endergebnis: [2_SchapplerTapOrt2019a]/(36, 37); Gl. (C.35)

% Initialisierung mit Fallunterscheidung für symbolische Eingabe
if ~Rob.issym
  PhiDpphi = zeros(3*NLEG,3);
  PhiDpphi_red = zeros( rownum_Phipq_red, sum(Rob.I_EE(4:6)) );
else
  PhiDpphi = sym('xx',     [3*NLEG,3]);
  PhiDpphi(:)=0;
  PhiDpphi_red = sym('xx', [rownum_Phipq_red, sum(Rob.I_EE(4:6))]);
  PhiDpphi_red(:)=0;
end

%% Berechnung
if ~platform_frame
  R_P_E = Rob.T_P_E(1:3,1:3);
else
  R_P_E = eye(3);
end
R_0_E_x = eul2r(xE(4:6), Rob.phiconv_W_E);
[~,phiconv_W_E_reci] = euler_angle_properties(Rob.phiconv_W_E);
omega_0_Ex  = euljac (xE(4:6), Rob.phiconv_W_E) * xDE(4:6);
RD_0_E_x =  skew(omega_0_Ex) * R_0_E_x;
K1 = 1;
for iLeg = 1 % nur Führungskette hat Einfluss (siehe Gl. D.47), [2_SchapplerTapOrt2019a]/(37)
  IJ_i = Rob.I1J_LEG(iLeg):Rob.I2J_LEG(iLeg);
  q_i = q(IJ_i); % Gelenkwinkel dieser Kette
  qD_i= qD(IJ_i);
  phi_0_Ai = Rob.Leg(iLeg).phi_W_0;
  R_0_0i = eul2r(phi_0_Ai, Rob.Leg(iLeg).phiconv_W_0);
  R_P_Bi = eulxyz2r(Rob.phi_P_B_all(:,iLeg));
  R_Bi_P = R_P_Bi.';
  
  % Definitionen, Laden der Kinematik
  T_0i_Bi = Rob.Leg(iLeg).fkineEE(q_i);
  R_0i_E_q = T_0i_Bi(1:3,1:3) * R_Bi_P * R_P_E;
  R_0_E_q = R_0_0i * R_0i_E_q;

  % Rotationsmatrix Differenz-Rotation. So Definiert wie in den
  % Zwangsbedingungen bsp6uPs_ZB_phi
  R_Ex_Eq = R_0_E_x' * R_0_E_q; % Argument in [2_SchapplerTapOrt2019a]/(19)
  % Ableitungen der Rotationsmatrizen berechnen
  omega_0i_Bi   = Rob.Leg(iLeg).jacobiw(q_i) *qD_i;
  RD_0i_Bi = skew (omega_0i_Bi) * T_0i_Bi(1:3,1:3);
  RD_0_E_q = R_0_0i * RD_0i_Bi * R_Bi_P * R_P_E;
  RD_Ex_Eq = (RD_0_E_x' * R_0_E_q) + (R_0_E_x' * RD_0_E_q);
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
  % Berechnung der Zeitableitung:
  % Setze Zeitableitung der EE-Rotationsmatrix transponiert in Matrix ein.
  % Da die Elemente der Rotationsmatrix nur eingesetzt werden, wird
  % elementweise abgeleitet
  a11=RD_0_E_q(1,1);a21=RD_0_E_q(1,2);a31=RD_0_E_q(1,3);
  a12=RD_0_E_q(2,1);a22=RD_0_E_q(2,2);a32=RD_0_E_q(2,3);
  a13=RD_0_E_q(3,1);a23=RD_0_E_q(3,2);a33=RD_0_E_q(3,3);
  dDPidR2b = [a11 a12 a13 0 0 0 0 0 0; a21 a22 a23 0 0 0 0 0 0; a31 a32 a33 0 0 0 0 0 0; 0 0 0 a11 a12 a13 0 0 0; 0 0 0 a21 a22 a23 0 0 0; 0 0 0 a31 a32 a33 0 0 0; 0 0 0 0 0 0 a11 a12 a13; 0 0 0 0 0 0 a21 a22 a23; 0 0 0 0 0 0 a31 a32 a33;];

  %% (IV) Ableitung von R_0_E nach den Euler-Winkeln
  % Vierter Term in [2_SchapplerTapOrt2019a]/(36) bzw. Gl. (A.50)
  % (XYZ-Euler-Winkel der Endeffektor-Orientierung)
  % Unabhängig vom Roboter (nur von Orientierungsdarstellung)
  dR0Ebdphi = rotmat_diff_eul(xE(4:6), Rob.phiconv_W_E);
  dR0EbdphiD = rotmatD_diff_eul(xE(4:6), xDE(4:6), Rob.phiconv_W_E);
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
  dDphidRb = eulD_diff_rotmat(R_Ex_Eq, RD_Ex_Eq, phiconv_W_E_reci);
  %% Gesamtergebnis
  % [2_SchapplerTapOrt2019a]/(36) bzw. Gl. (A.50)
  % Gl. (A.50)
  Phi_phi_i_GradxD = dDphidRb * P_T * dPidR2b  * dR0Ebdphi ...% AD P B  C
                   + dphidRb  * P_T * dDPidR2b * dR0Ebdphi... % A  P BD C
                   + dphidRb  * P_T * dPidR2b  * dR0EbdphiD;  % A  P B  CD
  %% Einsetzen in Ausgabevariable
  J1 = 1+3*(iLeg-1);
  J2 = J1+2;
  PhiDpphi(J1:J2,:) = Phi_phi_i_GradxD;
  
  % Ausgabe mit reduzierter Dimension
  K2 = K1+sum(Leg_I_EE_Task(iLeg,4:6))-1;
  if all(Leg_I_EE_Task(iLeg,4:6) == [1 1 0]) % 3T2R mit Aufg.-Red
    PhiDpphi_red( K1:K2, 1:sum(Rob.I_EE(4:6)) ) = Phi_phi_i_GradxD([2 3],Rob.I_EE(4:6)); % Nur 2 Komponenten: 2(Y) und 3(X)
  elseif all(Leg_I_EE_Task(iLeg,4:6) == [0 1 1]) % 3T1R-PKM ohne Aufg.-Red.
    % Nehme an, dass bei 3T1R-PKM die ZB erfüllbar sind und dass das
    % System überbestimmt ist (Beinkette mit 5 FG). Nehme daher Y- und X-
    % Zwangsbedingungen gemeinsam.
    sign_alpha = sign(r2eul(R_Ex_Eq, phiconv_W_E_reci));
    sign_alpha(sign_alpha==0)=1;
    Phipphi_red( K1:K2,1:sum(Rob.I_EE(4:6)) ) = [Phi_phi_i_GradxD(1,Rob.I_EE(4:6)); ... % Z
                              sign_alpha(2)*Phi_phi_i_GradxD(2,Rob.I_EE(4:6)) + ... % |Y|
                              sign_alpha(3)*Phi_phi_i_GradxD(3,Rob.I_EE(4:6))]; % |X|
  elseif all(Leg_I_EE_Task(iLeg,4:6) == [0 1 0]) % 3T1R-PKM mit Aufg.-Red.
    sign_alpha = sign(r2eul(R_Ex_Eq, phiconv_W_E_reci));
    sign_alpha(sign_alpha==0)=1;
    Phipphi_red( K1:K2,1:sum(Rob.I_EE(4:6)) ) = ...
                              sign_alpha(2)*Phi_phi_i_GradxD(2,Rob.I_EE(4:6)) + ... % |Y|
                              sign_alpha(3)*Phi_phi_i_GradxD(3,Rob.I_EE(4:6)); % |X|
  elseif all(Leg_I_EE_Task(iLeg,4:6) == [0 0 0])% 2T1R mit Aufg.-Red
    % ebene Rotation (redundanter Fall). Keinen Eintrag für Führungskette
  elseif all(Leg_I_EE_Task(iLeg,4:6) == [0 0 1]) % 2T1R ohne Aufg.-Red
    % ebene Rotation (nicht-redundanter Fall).
    PhiDpphi_red( K1:K2, 1:sum(Rob.I_EE(4:6)) ) = Phi_phi_i_GradxD(1,Rob.I_EE(4:6)); % nur 1. Eintrag (Z)
  elseif all(Leg_I_EE_Task(iLeg,4:6) == [1 1 1]) % allgemeiner Fall (3T3R-PKM, aber auch 3T0R-PKM mit 3T3R-ZB der Beine)
    PhiDpphi_red( K1:K2, 1:sum(Rob.I_EE(4:6)) ) = Phi_phi_i_GradxD(:,Rob.I_EE(4:6)); % alle drei Einträge
  else
    error('Fall nicht vorgesehen');
  end
  K1 = K2+1;
end
