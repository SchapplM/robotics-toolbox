% Ableitung der Rotationskomponente kinematischen ZB nach den Gelenkwinkeln
% Bezeichnungen: Rotatorischer Teil der ...
% * Jacobi-Matrix der inversen Kinematik, 
% * geometrische Matrix der inversen Kinematik
%   (ist in der Literatur wenig gebräuchlich)
% 
% Variante 1:
% * Absolute Rotation ausgedrückt in XYZ-Euler-Winkeln
% * Rotationsfehler ausgedrückt genauso bspw. in XYZ-Euler-Winkeln
%   Rotationsfehler wird als R_0_E * R_D_0 angenommen (also 0(q)->0(x))
%   (anders herum als in [2_SchapplerTapOrt2019a])
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
% [C] Aufzeichnungen Schappler vom 21.08.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phipq_red, Phipq] = constr1grad_rq(Rob, q, xE, platform_frame)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr1grad_rq: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1grad_rq: xE muss 6x1 sein');
if nargin == 3, platform_frame = false; end

NLEG = Rob.NLEG;
NJ = Rob.NJ;

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
% Endergebnis, siehe Gl. (B.30); entspricht [2_SchapplerTapOrt2019a]/(34)

if ~Rob.issym
  Phipq = zeros(3*NLEG,NJ);
  Phipq_red = zeros(length(Rob.I_constr_r_red),NJ);
else
  Phipq = sym('phi', [3*NLEG,NJ]);
  Phipq(:)=0;
  Phipq_red = sym('phi', [length(Rob.I_constr_r_red),NJ]);
  Phipq_red(:)=0;
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
  % Anteil der ZB-Gleichung der Gelenkkette
  IJ_i = Rob.I1J_LEG(iLeg):Rob.I2J_LEG(iLeg);
  qs = q(IJ_i); % Gelenkwinkel dieser Kette
  
  phi_0_Ai = Rob.Leg(iLeg).phi_W_0;
  R_0_0i = eul2r(phi_0_Ai, Rob.Leg(iLeg).phiconv_W_0);
  R_P_Bi = eulxyz2r(Rob.phi_P_B_all(:,iLeg));
  R_Bi_P = R_P_Bi.';
  
  % Kinematik, Definitionen
  T_0i_Bi = Rob.Leg(iLeg).fkineEE(qs);
  R_0i_E_q = T_0i_Bi(1:3,1:3) * R_Bi_P * R_P_E;
  R_0_E_q = R_0_0i * R_0i_E_q;
  R_0x_0q = R_0_E_q * R_0_E_x';

  %% (III) Ableitung der Rotationsmatrix R_0_E nach q
  % Term III aus Gl. (B.31); oder ähnlich aus [2_SchapplerTapOrt2019a]/(34)
  % Berücksichtigung der zusätzlichen Transformation RS.T_N_E: Gl. (C.7);
  % bzw. [SchapplerTapOrt2019]/(32)
  % (jacobiR berücksichtigt diese Rotation nicht; ist definiert zum letzten
  % Körper-KS N der seriellen Kette. Hier werden dadurch direkt zwei
  % Transformationen berücksichtigt: N->Bi->E
  % RS.TNE bezieht sich auf den "Endeffektor" der Beinkette, der "Bi" ist.
  % Gl. (B.33)
  R_Ni_E = Rob.Leg(iLeg).T_N_E(1:3,1:3) * R_Bi_P* R_P_E; % R_Bi_P hinzufügen
  b11=R_Ni_E(1,1);b12=R_Ni_E(1,2);b13=R_Ni_E(1,3);
  b21=R_Ni_E(2,1);b22=R_Ni_E(2,2);b23=R_Ni_E(2,3);
  b31=R_Ni_E(3,1);b32=R_Ni_E(3,2);b33=R_Ni_E(3,3);
  dPidRb1 = [b11 0 0 b21 0 0 b31 0 0; 0 b11 0 0 b21 0 0 b31 0; 0 0 b11 0 0 b21 0 0 b31; b12 0 0 b22 0 0 b32 0 0; 0 b12 0 0 b22 0 0 b32 0; 0 0 b12 0 0 b22 0 0 b32; b13 0 0 b23 0 0 b33 0 0; 0 b13 0 0 b23 0 0 b33 0; 0 0 b13 0 0 b23 0 0 b33;];
  dRb_0iN_dq = Rob.Leg(iLeg).jacobiR(qs);
  dRb_0iE_dq = dPidRb1 * dRb_0iN_dq;

  % Bezug auf Basis-KS der PKM und nicht Basis des seriellen Roboters
  % Matrix-Produkt aus rmatvecprod_diff_rmatvec2_matlab.m, siehe Gl. (A.13)
  % oder [2_SchapplerTapOrt2019a]/(A23)
  % Entspricht [2_SchapplerTapOrt2019a]/(34)/II
  a11=R_0_0i(1,1);a12=R_0_0i(1,2);a13=R_0_0i(1,3);
  a21=R_0_0i(2,1);a22=R_0_0i(2,2);a23=R_0_0i(2,3);
  a31=R_0_0i(3,1);a32=R_0_0i(3,2);a33=R_0_0i(3,3);
  dPidRb2 = [a11 a12 a13 0 0 0 0 0 0; a21 a22 a23 0 0 0 0 0 0; a31 a32 a33 0 0 0 0 0 0; 0 0 0 a11 a12 a13 0 0 0; 0 0 0 a21 a22 a23 0 0 0; 0 0 0 a31 a32 a33 0 0 0; 0 0 0 0 0 0 a11 a12 a13; 0 0 0 0 0 0 a21 a22 a23; 0 0 0 0 0 0 a31 a32 a33;];
  dRb_0E_dq = dPidRb2 * dRb_0iE_dq;

  %% (II) Innere Ableitung des Matrix-Produktes. Die Matrix R_0_E_x wird
  % transponiert eingesetzt.
  % Term II aus Gl. (B.31)
  % aus rmatvecprod_diff_rmatvec1_matlab.m; [2_SchapplerTapOrt2019a]/(A24)
  b11=R_0_E_x(1,1);b12=R_0_E_x(2,1);b13=R_0_E_x(3,1);
  b21=R_0_E_x(1,2);b22=R_0_E_x(2,2);b23=R_0_E_x(3,2);
  b31=R_0_E_x(1,3);b32=R_0_E_x(2,3);b33=R_0_E_x(3,3);
  dPidRb1 = [b11 0 0 b21 0 0 b31 0 0; 0 b11 0 0 b21 0 0 b31 0; 0 0 b11 0 0 b21 0 0 b31; b12 0 0 b22 0 0 b32 0 0; 0 b12 0 0 b22 0 0 b32 0; 0 0 b12 0 0 b22 0 0 b32; b13 0 0 b23 0 0 b33 0 0; 0 b13 0 0 b23 0 0 b33 0; 0 0 b13 0 0 b23 0 0 b33;];
 
  %% (I) Ableitung der Euler-Winkel nach der Rot.-matrix
  % Term I aus Gl. (B.31) bzw. aus [2_SchapplerTapOrt2019a]/(34)
  % Euler-Winkel-Konvention von Trafo W->E
  % Aus codeexport/eulxyz_diff_rmatvec_matlab.m
  dphidRb = eul_diff_rotmat(R_0x_0q, Rob.phiconv_W_E);
  
  %% Gesamtergebnis
  % Gl. (B.31) bzw. ähnlich in [2_SchapplerTapOrt2019a]/(34)
  Phi_phi_i_Gradq = dphidRb * dPidRb1 * dRb_0E_dq;

  %% In Endergebnis einsetzen
  I1 = 1+3*(iLeg-1); % I: Zeilen der Ergebnisvariable: Alle rotatorischen ZB
  I2 = I1+2; % drei rotatorische Einträge
  J1 = Rob.I1J_LEG(iLeg); % J: Spalten der Ergebnisvariable: Alle Gelenke
  J2 = Rob.I2J_LEG(iLeg); % so viele Einträge wie Beine in der Kette
  Phipq(I1:I2,J1:J2) = Phi_phi_i_Gradq;
  
  K2 = K1+sum(Rob.Leg(iLeg).I_EE_Task(4:6))-1;
  Phipq_red(K1:K2,J1:J2) = Phi_phi_i_Gradq(Rob.Leg(iLeg).I_EE_Task(4:6),:);
  K1 = K2+1;
end

