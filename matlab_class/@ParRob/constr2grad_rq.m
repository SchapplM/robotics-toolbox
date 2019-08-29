% Ableitung der Rotationskomponente kinematischen ZB nach den Gelenkwinkeln
% Bezeichnungen: Rotatorischer Teil der ...
% * Jacobi-Matrix der inversen Kinematik, 
% * geometrische Matrix der inversen Kinematik
%   (ist in der Literatur wenig gebräuchlich)
% 
% Variante 2:
% * Absolute Rotation ausgedrückt in XYZ-Euler-Winkeln
% * Rotationsfehler als E(q)-E(x) ausgedrückt in ZYX-Euler-Winkeln
% 
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% 
% Ausgabe:
% Phi_q_red
%   Ableitung der kinematischen Zwangsbedingungen nach allen Gelenkwinkeln
%   Rotatorischer Teil
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% Phi_q [3xN]
%   Siehe vorher. Hier alle Zeilen der Zwangsbedingungen

% Quellen:
% [2_SchapplerTapOrt2019a] Schappler, M. et al.: Modeling Parallel Robot
% Kinematics for 3T2R and 3T3R Tasks using Reciprocal Sets of Euler Angles
% (Arbeitstitel), Submitted to MDPI Robotics KaRD2, Version of 27.06.2019
% [SchapplerTapOrt2019] Schappler, M. et al.: Resolution of Functional
% Redundancy for 3T2R Robot Tasks using Two Sets of Reciprocal Euler
% Angles, Proc. of the 15th IFToMM World Congress, 2019
% [A] Aufzeichnungen Schappler vom 21.06.2018
% [B] Aufzeichnungen Schappler vom 13.07.2018
% [C] Aufzeichnungen Schappler vom 27.07.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phipq_red, Phipq] = constr2grad_rq(Rob, q, xE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr2grad_rq: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr2grad_rq: xE muss 6x1 sein');
NLEG = Rob.NLEG;
NJ = Rob.NJ;

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
% Endergebnis, siehe [2_SchapplerTapOrt2019a]/34 oder Gl. (B.30)

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
  
  phi_0_Ai = Rob.Leg(iLeg).phi_W_0;
  R_0_0i = eul2r(phi_0_Ai, Rob.Leg(iLeg).phiconv_W_0);
  
  % Kinematik, Definitionen
  T_0i_Bi = Rob.Leg(iLeg).fkineEE(qs);
  R_0i_E_q = T_0i_Bi(1:3,1:3) * R_P_E;
  R_0_E_q = R_0_0i * R_0i_E_q;
  R_Ex_Eq = R_0_E_x' * R_0_E_q; % Argument in [2_SchapplerTapOrt2019a]/(19)

  %% (III) Ableitung der Rotationsmatrix R_0_E nach q
  % Term III aus [2_SchapplerTapOrt2019a]/(34) oder Gl. (B.31)
  % Berücksichtigung der zusätzlichen Transformation RS.T_N_E: Gl. (C.7)
  % bzw. [SchapplerTapOrt2019]/(32)
  % (jacobiR berücksichtigt diese Rotation nicht; ist definiert zum letzten
  % Körper-KS N der seriellen Kette. Hier werden dadurch direkt zwei
  % Transformationen berücksichtigt: N->Bi->E
  % RS.TNE bezieht sich auf den "Endeffektor" der Beinkette, der "Bi" ist.
  % Gl. (B.33)
  R_Ni_E = Rob.Leg(iLeg).T_N_E(1:3,1:3) * R_P_E;
  b11=R_Ni_E(1,1);b12=R_Ni_E(1,2);b13=R_Ni_E(1,3);
  b21=R_Ni_E(2,1);b22=R_Ni_E(2,2);b23=R_Ni_E(2,3);
  b31=R_Ni_E(3,1);b32=R_Ni_E(3,2);b33=R_Ni_E(3,3);
  % Matrix-Produkt-Gradient siehe [2_SchapplerTapOrt2019a]/(A24)
  dPidRb1 = [b11 0 0 b21 0 0 b31 0 0; 0 b11 0 0 b21 0 0 b31 0; 0 0 b11 0 0 b21 0 0 b31; b12 0 0 b22 0 0 b32 0 0; 0 b12 0 0 b22 0 0 b32 0; 0 0 b12 0 0 b22 0 0 b32; b13 0 0 b23 0 0 b33 0 0; 0 b13 0 0 b23 0 0 b33 0; 0 0 b13 0 0 b23 0 0 b33;];
  dRb_0iN_dq = Rob.Leg(iLeg).jacobiR(qs);
  dRb_0iE_dq = dPidRb1 * dRb_0iN_dq;

  % Bezug auf Basis-KS der PKM und nicht Basis des seriellen Roboters
  % Matrix-Produkt aus rmatvecprod_diff_rmatvec2_matlab.m, siehe Gl. (A.13)
  a11=R_0_0i(1,1);a12=R_0_0i(1,2);a13=R_0_0i(1,3);
  a21=R_0_0i(2,1);a22=R_0_0i(2,2);a23=R_0_0i(2,3);
  a31=R_0_0i(3,1);a32=R_0_0i(3,2);a33=R_0_0i(3,3);
  % Matrix-Produkt-Gradient siehe [2_SchapplerTapOrt2019a]/(A23)
  dPidRb2 = [a11 a12 a13 0 0 0 0 0 0; a21 a22 a23 0 0 0 0 0 0; a31 a32 a33 0 0 0 0 0 0; 0 0 0 a11 a12 a13 0 0 0; 0 0 0 a21 a22 a23 0 0 0; 0 0 0 a31 a32 a33 0 0 0; 0 0 0 0 0 0 a11 a12 a13; 0 0 0 0 0 0 a21 a22 a23; 0 0 0 0 0 0 a31 a32 a33;];
  dRb_0E_dq = dPidRb2 * dRb_0iE_dq;

  %% (II) Innere Ableitung des Matrix-Produktes
  % Term II aus Gl. (C.49) bzw. aus [2_SchapplerTapOrt2019a]/(34)
  % Die Matrix R_0_E_x wird transponiert eingesetzt.
  % aus rmatvecprod_diff_rmatvec2_matlab.m
  a11=R_0_E_x(1,1);a12=R_0_E_x(2,1);a13=R_0_E_x(3,1);
  a21=R_0_E_x(1,2);a22=R_0_E_x(2,2);a23=R_0_E_x(3,2);
  a31=R_0_E_x(1,3);a32=R_0_E_x(2,3);a33=R_0_E_x(3,3);
  % Matrix-Produkt-Gradient siehe [2_SchapplerTapOrt2019a]/(A23)
  dPidRb2 = [a11 a12 a13 0 0 0 0 0 0; a21 a22 a23 0 0 0 0 0 0; a31 a32 a33 0 0 0 0 0 0; 0 0 0 a11 a12 a13 0 0 0; 0 0 0 a21 a22 a23 0 0 0; 0 0 0 a31 a32 a33 0 0 0; 0 0 0 0 0 0 a11 a12 a13; 0 0 0 0 0 0 a21 a22 a23; 0 0 0 0 0 0 a31 a32 a33;];
 
  %% (I) Ableitung der Euler-Winkel nach der Rot.-matrix
  % Term I aus Gl. (C.49) bzw. aus [2_SchapplerTapOrt2019a]/(34) 
  % (ZYX-Euler-Winkel des Orientierungsfehlers)
  % Aus eulzyx_diff_rmatvec_matlab.m
  % Unabhängig vom Roboter (nur von Orientierungsdarstellung)
  dphidRb = eul_diff_rotmat(R_Ex_Eq, phiconv_W_E_reci);
  
  %% Gesamtergebnis
  % Gl. [2_SchapplerTapOrt2019a]/(34) bzw. (C.49)
  Phi_phi_i_Gradq = dphidRb * dPidRb2 * dRb_0E_dq;

  %% In Endergebnis einsetzen
  I1 = 1+3*(iLeg-1); % I: Zeilen der Ergebnisvariable: Alle rotatorischen ZB
  I2 = I1+2; % drei rotatorische Einträge
  J1 = Rob.I1J_LEG(iLeg); % J: Spalten der Ergebnisvariable: Alle Gelenke
  J2 = Rob.I2J_LEG(iLeg); % so viele Einträge wie Beine in der Kette
  Phipq(I1:I2,J1:J2) = Phi_phi_i_Gradq;
  
  K1 = 1+sum(Rob.I_EE(4:6))*(iLeg-1); % I: Zeilen der Ergebnisvariable: Alle rotatorischen ZB
  K2 = K1+sum(Rob.I_EE(4:6))-1; % drei rotatorische Einträge
  % TODO: Die Auswahl der ZB muss an die jeweilige Aufgabe angepasst
  % werden (3T1R, 3T3R); wegen der Reziprozität EE-FG / Residuum
  Phipq_red(K1:K2,J1:J2) = Phi_phi_i_Gradq(Rob.I_EE(4:6),:);
end
