% Ableitung der Rotationskomponente kinematischen ZB nach den Gelenkwinkeln
% Bezeichnungen: Rotatorischer Teil der ...
% * Jacobi-Matrix der inversen Kinematik, 
% * geometrische Matrix der inversen Kinematik
% (ist in der Literatur wenig gebräuchlich)
% 
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
% Phi_q_red_l
%   Ableitung der kinematischen Zwangsbedingungen nach allen Gelenkwinkeln
%   der Führungs-Beinkette
% Phi_q_red_f
%   Ableitung der kinematischen Zwangsbedingungen nach allen Gelenkwinkeln
%   der Folge-Beinkette
%   Rotatorischer Teil
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% Phi_q_l, Phi_q_f [3xN]
%   Siehe vorher. Hier alle Zeilen der Zwangsbedingungen

% Quellen:
% [A] Aufzeichnungen Schappler vom 21.06.2018
% [B] Aufzeichnungen Schappler vom 13.07.2018
% [C] Aufzeichnungen Schappler vom 27.07.2018
% [D] Aufzeichnungen Schappler vom 02.02.2019

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phipq_red, Phipq] = constr3grad_rq(Rob, q, xE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr2grad_rq: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr2grad_rq: xE muss 6x1 sein');
NLEG = Rob.NLEG;
NJ = Rob.NJ;

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
% Endergebnis, siehe Gl. (B.30)

if ~Rob.issym
  Phipq = zeros(3*NLEG,NJ);
  Phipq_red = zeros(sum(Rob.I_EE(4:6))*NLEG,NJ);
else
  Phipq = sym('phi', [3*NLEG,NJ]);
  Phipq(:)=0;
  Phipq_red = sym('phi', [sum(Rob.I_EE(4:6))*NLEG,NJ]);
  Phipq_red(:)=0;
end

%% Berechnung
R_P_E = Rob.T_P_E(1:3,1:3);
R_0_E_x = eul2r(xE(4:6), Rob.phiconv_W_E);
[~,phiconv_W_E_reci] = euler_angle_properties(Rob.phiconv_W_E);

for iLeg = 1:NLEG
  % Anteil der ZB-Gleichung der Gelenkkette
  IJ_i = Rob.I1J_LEG(iLeg):Rob.I2J_LEG(iLeg);
  qs = q(IJ_i); % Gelenkwinkel dieser Kette
  T_0i_Bi = Rob.Leg(iLeg).fkineEE(qs);
  
  phi_0_Ai = Rob.Leg(iLeg).phi_W_0;
  R_0_0i = eul2r(phi_0_Ai, Rob.Leg(iLeg).phiconv_W_0);
  
  % Ableitung der Koppel-KS-Rotationsmatrix der aktuellen Beinkette nach
  % den Gelenkwinkeln der aktuellen Beinkette
  % Term III aus Gl. (D.22) bzw. (D.24)
  % Term III aus Gl. (B.31): Ableitung der Rotationsmatrix R_0_E nach q
  % Berücksichtigung der zusätzlichen Transformation RS.T_N_E: Gl. (C.7)
  % (jacobiR berücksichtigt diese Rotation nicht; ist definiert zum letzten
  % Körper-KS N der seriellen Kette. Hier werden dadurch direkt zwei
  % Transformationen berücksichtigt: N->Bi->E
  % RS.T_N_E bezieht sich auf den "Endeffektor" der Beinkette, der "Bi" ist.
  % Gl. (B.33)
  R_Ni_E = Rob.Leg(iLeg).T_N_E(1:3,1:3) * R_P_E;
  b11=R_Ni_E(1,1);b12=R_Ni_E(1,2);b13=R_Ni_E(1,3);
  b21=R_Ni_E(2,1);b22=R_Ni_E(2,2);b23=R_Ni_E(2,3);
  b31=R_Ni_E(3,1);b32=R_Ni_E(3,2);b33=R_Ni_E(3,3);
  dPidRb1 = [b11 0 0 b21 0 0 b31 0 0; 0 b11 0 0 b21 0 0 b31 0; 0 0 b11 0 0 b21 0 0 b31; b12 0 0 b22 0 0 b32 0 0; 0 b12 0 0 b22 0 0 b32 0; 0 0 b12 0 0 b22 0 0 b32; b13 0 0 b23 0 0 b33 0 0; 0 b13 0 0 b23 0 0 b33 0; 0 0 b13 0 0 b23 0 0 b33;];
  dRb_0iN_dq = Rob.Leg(iLeg).jacobiR(qs);
  dRb_0iE_dq = dPidRb1 * dRb_0iN_dq;
  
  % Bezug auf Basis-KS der PKM und nicht Basis des seriellen Roboters
  % Matrix-Produkt aus rmatvecprod_diff_rmatvec2_matlab.m, siehe Gl. (A.13)
  a11=R_0_0i(1,1);a12=R_0_0i(1,2);a13=R_0_0i(1,3);
  a21=R_0_0i(2,1);a22=R_0_0i(2,2);a23=R_0_0i(2,3);
  a31=R_0_0i(3,1);a32=R_0_0i(3,2);a33=R_0_0i(3,3);
  dPidRb2 = [a11 a12 a13 0 0 0 0 0 0; a21 a22 a23 0 0 0 0 0 0; a31 a32 a33 0 0 0 0 0 0; 0 0 0 a11 a12 a13 0 0 0; 0 0 0 a21 a22 a23 0 0 0; 0 0 0 a31 a32 a33 0 0 0; 0 0 0 0 0 0 a11 a12 a13; 0 0 0 0 0 0 a21 a22 a23; 0 0 0 0 0 0 a31 a32 a33;];
  dRb_0E_dq = dPidRb2 * dRb_0iE_dq; 
  
  if iLeg == 1
    % Kinematik, Definitionen
    R_0_L_q_L = R_0_0i * T_0i_Bi(1:3,1:3) * R_P_E;
    R_Ex_Eq = R_0_E_x' * R_0_L_q_L;

    % Abspeichern der für jede Beinkette berechneten Ableitung für die
    % Führungskette (siehe unten)
    dRb_0L_dq = dRb_0E_dq;

    % Term II aus Gl. (C.49): Innere Ableitung des Matrix-Produktes
    % Die Matrix R_0_E_x wird transponiert eingesetzt.
    % aus rmatvecprod_diff_rmatvec2_matlab.m
    a11=R_0_E_x(1,1);a12=R_0_E_x(2,1);a13=R_0_E_x(3,1);
    a21=R_0_E_x(1,2);a22=R_0_E_x(2,2);a23=R_0_E_x(3,2);
    a31=R_0_E_x(1,3);a32=R_0_E_x(2,3);a33=R_0_E_x(3,3);
    dPidRb2 = [a11 a12 a13 0 0 0 0 0 0; a21 a22 a23 0 0 0 0 0 0; a31 a32 a33 0 0 0 0 0 0; 0 0 0 a11 a12 a13 0 0 0; 0 0 0 a21 a22 a23 0 0 0; 0 0 0 a31 a32 a33 0 0 0; 0 0 0 0 0 0 a11 a12 a13; 0 0 0 0 0 0 a21 a22 a23; 0 0 0 0 0 0 a31 a32 a33;];

    % Term I aus Gl. (C.49): Ableitung der Euler-Winkel nach der Rot.-matrix
    % (ZYX-Euler-Winkel des Orientierungsfehlers)
    % Aus eulzyx_diff_rmatvec_matlab.m
    % Unabhängig vom Roboter (nur von Orientierungsdarstellung)
    dphidRb = eul_diff_rotmat(R_Ex_Eq, phiconv_W_E_reci);

    Phi_phi_i_Gradq = dphidRb * dPidRb2 * dRb_0E_dq; % Gl. (C.49)  
    % In Endergebnis einsetzen
    I1 = 1+3*(iLeg-1); % I: Zeilen der Ergebnisvariable: Alle rotatorischen ZB
    I2 = I1+2; % drei rotatorische Einträge
    J1 = Rob.I1J_LEG(iLeg); % J: Spalten der Ergebnisvariable: Alle Gelenke
    J2 = Rob.I2J_LEG(iLeg); % so viele Einträge wie Beine in der Kette
    Phipq(I1:I2,J1:J2) = Phi_phi_i_Gradq;

    K1 = 1+sum(Rob.I_EE(4:6))*(iLeg-1); % I: Zeilen der Ergebnisvariable: Alle rotatorischen ZB
    K2 = K1+sum(Rob.I_EE(4:6))-1; % drei rotatorische Einträge
    Phipq_red(K1:K2,J1:J2) = Phi_phi_i_Gradq(Rob.I_EE(4:6),:);
   
  elseif iLeg > 1
    R_0_E_q_f = R_0_0i * T_0i_Bi(1:3,1:3) * R_P_E; 
    % Gl. D.18, D.21 
    R_Lq_Eq_f = R_0_L_q_L' * R_0_E_q_f;

    % Ableitung der EE-Rotation der Folge-Kette nach den Gelenkwinkeln der
    % Folge-Kette (Term III in D.22)
    dRb_0E_dq_f = dRb_0E_dq;
    % Das gleiche für die Führungskette (Term III in D.24)
    dRb_0L_dq_L = dRb_0L_dq;

    % Term II aus Gl. (D.24)
    % Term II aus Gl. (C.49): Innere Ableitung des Matrix-Produktes
    % Die Matrix R_0_E_x wird transponiert eingesetzt.
    % aus rmatvecprod_diff_rmatvec2_matlab.m
    a11=R_0_E_q_f(1,1);a12=R_0_E_q_f(2,1);a13=R_0_E_q_f(3,1);
    a21=R_0_E_q_f(1,2);a22=R_0_E_q_f(2,2);a23=R_0_E_q_f(3,2);
    a31=R_0_E_q_f(1,3);a32=R_0_E_q_f(2,3);a33=R_0_E_q_f(3,3);
    dPidRb2f = [a11 a12 a13 0 0 0 0 0 0; a21 a22 a23 0 0 0 0 0 0; a31 a32 a33 0 0 0 0 0 0; 0 0 0 a11 a12 a13 0 0 0; 0 0 0 a21 a22 a23 0 0 0; 0 0 0 a31 a32 a33 0 0 0; 0 0 0 0 0 0 a11 a12 a13; 0 0 0 0 0 0 a21 a22 a23; 0 0 0 0 0 0 a31 a32 a33;];

    % Term II aus Gl. (D.22)
    a11=R_0_L_q_L(1,1);a12=R_0_L_q_L(2,1);a13=R_0_L_q_L(3,1);
    a21=R_0_L_q_L(1,2);a22=R_0_L_q_L(2,2);a23=R_0_L_q_L(3,2);
    a31=R_0_L_q_L(1,3);a32=R_0_L_q_L(2,3);a33=R_0_L_q_L(3,3);
    dPidRb2L = [a11 a12 a13 0 0 0 0 0 0; a21 a22 a23 0 0 0 0 0 0; a31 a32 a33 0 0 0 0 0 0; 0 0 0 a11 a12 a13 0 0 0; 0 0 0 a21 a22 a23 0 0 0; 0 0 0 a31 a32 a33 0 0 0; 0 0 0 0 0 0 a11 a12 a13; 0 0 0 0 0 0 a21 a22 a23; 0 0 0 0 0 0 a31 a32 a33;];

    % Term I aus Gl. (D.22): 
    % Term I aus Gl. (C.49): Ableitung der Euler-Winkel nach der Rot.-matrix
    % (ZYX-Euler-Winkel des Orientierungsfehlers)
    % Aus eulzyx_diff_rmatvec_matlab.m
    % Unabhängig vom Roboter (nur von Orientierungsdarstellung)
    dphidRbf = eul_diff_rotmat(R_Lq_Eq_f, phiconv_W_E_reci);
    % Term I aus Gl. D.24
    dphidRbL = eul_diff_rotmat(R_Lq_Eq_f, phiconv_W_E_reci);

    P_T = [1 0 0 0 0 0 0 0 0; 0 0 0 1 0 0 0 0 0; 0 0 0 0 0 0 1 0 0; 0 1 0 0 0 0 0 0 0; 0 0 0 0 1 0 0 0 0; 0 0 0 0 0 0 0 1 0; 0 0 1 0 0 0 0 0 0; 0 0 0 0 0 1 0 0 0; 0 0 0 0 0 0 0 0 1;];

    % Gl. (D.24); Ableitung nach Gelenkwinkeln der Führungskette
    Phi_phi_i_Gradq_L = dphidRbL * P_T*dPidRb2f * dRb_0L_dq_L;
    
    % Gl. (D.22); Ableitung nach Gelenkwinkeln der Folgekette
    Phi_phi_i_Gradq_f = dphidRbf * dPidRb2L * dRb_0E_dq_f; 

    % In Endergebnis einsetzen
    I1 = 1+3*(iLeg-1); % I: Zeilen der Ergebnisvariable: Alle rotatorischen ZB
    I2 = I1+2; % drei rotatorische Einträge
    G1 = 1; % G: Spalten der Ergebnisvariable (Führungskette ist Nr. 1)
    G2 = Rob.I2J_LEG(1); % So viele Spalten wie Gelenke in Führungskette
    % Ableitung nach den Gelenkwinkeln der Führungskette
    Phipq(I1:I2,G1:G2) = Phi_phi_i_Gradq_L; 
    J1 = Rob.I1J_LEG(iLeg); % J: Spalten der Ergebnisvariable: Alle Gelenke
    J2 = Rob.I2J_LEG(iLeg); % so viele Einträge wie Beine in der Kette
    % Ableitung nach den Gelenkwinkeln der Folgekette
    Phipq(I1:I2,J1:J2) = Phi_phi_i_Gradq_f;

    K1 = 1+sum(Rob.I_EE(4:6))*(iLeg-1); % K: Zeilen der Ergebnisvariable: Alle rotatorischen ZB
    K2 = K1+sum(Rob.I_EE(4:6))-1; % drei rotatorische Einträge
    Phipq_red(K1:K2,G1:G2) = Phi_phi_i_Gradq_L(Rob.I_EE(4:6),:);
    Phipq_red(K1:K2,J1:J2) = Phi_phi_i_Gradq_f(Rob.I_EE(4:6),:);
  end
end
