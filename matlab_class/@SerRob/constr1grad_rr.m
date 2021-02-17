% Ableitung der Rotationskomponente der kinematischen Zwangsbedingungen
% nach der EE-Orientierung
% Dieser Term dient nur zur Kontrolle und ist für die inverse Kinematik
% serieller kinematischer Ketten nicht notwendig.
% Variante 1:
% * Absolute Rotation ausgedrückt z.B. in XYZ-Euler-Winkeln
% * Rotationsfehler genauso ausgedrückt z.B. in XYZ-Euler-Winkeln
%   (statt XYZ wird die Konvention aus `phiconv_W_E` genommen)
%   Rotationsfehler wird als R_0_E * R_0_D angenommen (also 0(q)->0(x))
%   Unterschiedlich zu [SchapplerTapOrt2019a], Berechnung aber ähnlich
% 
% Eingabe:
% q
%   Gelenkkoordinaten des Roboters
% Tr0Ex
%   Endeffektorpose des Roboters bezüglich des Basis-KS
%   Homogene Transformationsmatrix ohne letzte Zeile.
% 
% Ausgabe:
% Phi_rr [3x3]
%   Matrix mit Ableitungen der 3 rotatorischen Zwangsbedingungskomponenten
%   (in den Zeilen) nach den 3 EE-Rotationskoordinaten (in den Spalten)
% 
% Siehe auch: ParRob/constr1grad_rr

% Quellen:
% [SchapplerTapOrt2019a] Schappler, M. et al.: Modeling Parallel Robot
% Kinematics for 3T2R and 3T3R Tasks using Reciprocal Sets of Euler Angles,
% MDPI Robotics KaRD2, 2019
% [A] Aufzeichnungen Schappler vom 21.06.2018
% [B] Aufzeichnungen Schappler vom 13.07.2018
% [C] Aufzeichnungen Schappler vom 27.07.2018
% [D] Aufzeichnungen Schappler vom 21.08.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-01
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Phi_rr = constr1grad_rr(Rob, q, Tr0Ex)

assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'SerRob/constr1grad_rr: q muss %dx1 sein', Rob.NJ);

% Endergebnis, siehe [B]/Gl.(30) bzw. [SchapplerTapOrt2019a]/(36)

R_0_E_x = Tr0Ex(1:3,1:3);

% Kinematik, Definitionen
T_0_E = Rob.fkineEE(q);
R_0_E_q = T_0_E(1:3,1:3);

% Rotationsmatrix Differenz-Rotation. So Definiert wie in den
% Zwangsbedingungen bsp6uPs_ZB_phi (abweichend zu Publikation)
R_0x_0q = (R_0_E_q * R_0_E_x');

% Ableitung des Matrixproduktes
% Dritter Term in Gl. (C.35) bzw. in [SchapplerTapOrt2019a]/(36)
% Gl. (B.12)
% Unabhängig vom Roboter (Rotationsmatrix aus Kinematik wird in
% Platzhalterelemente eingesetzt)
% Aus rmatvecprod_diff_rmatvec1_matlab.m
b11=R_0_E_q(1,1);b21=R_0_E_q(1,2);b31=R_0_E_q(1,3);
b12=R_0_E_q(2,1);b22=R_0_E_q(2,2);b32=R_0_E_q(2,3);
b13=R_0_E_q(3,1);b23=R_0_E_q(3,2);b33=R_0_E_q(3,3);
dPidR1b = [b11 0 0 b21 0 0 b31 0 0; 0 b11 0 0 b21 0 0 b31 0; 0 0 b11 0 0 b21 0 0 b31; b12 0 0 b22 0 0 b32 0 0; 0 b12 0 0 b22 0 0 b32 0; 0 0 b12 0 0 b22 0 0 b32; b13 0 0 b23 0 0 b33 0 0; 0 b13 0 0 b23 0 0 b33 0; 0 0 b13 0 0 b23 0 0 b33;];

% Ableitung von R_0_E nach den Euler-Winkeln
% Vierter Term in Gl. (C.35) bzw. in [SchapplerTapOrt2019a]/(36)
% Unabhängig vom Roboter (nur von Orientierungsdarstellung)
dR0Ebdphi = rotmat_diff_eul(r2eul(R_0_E_x, Rob.phiconv_W_E), Rob.phiconv_W_E);

% Transpositions-Matrix; Gl (C.29) bzw. [SchapplerTapOrt2019a]/(A23)
% zweiter Term in Gl. (C.35) bzw. [SchapplerTapOrt2019a]/(36)
% Aus permat_rvec_transpose_matlab.m
P_T = [1 0 0 0 0 0 0 0 0; 0 0 0 1 0 0 0 0 0; 0 0 0 0 0 0 1 0 0; 0 1 0 0 0 0 0 0 0; 0 0 0 0 1 0 0 0 0; 0 0 0 0 0 0 0 1 0; 0 0 1 0 0 0 0 0 0; 0 0 0 0 0 1 0 0 0; 0 0 0 0 0 0 0 0 1;];

% Erster Term in Gl. (C.35),(C.28),(B.16): Ableitung der Euler-Winkel nach der
% Rotationsmatrix
% Aus P_3RRR/phixyz_diff_rmatvec_matlab.m
% Unabhängig vom Roboter (nur von Orientierungsdarstellung)
dphidRb = eul_diff_rotmat(R_0x_0q, Rob.phiconv_W_E);

% Gl. (C.35) bzw. [SchapplerTapOrt2019a]/(36)
Phi_rr = dphidRb * P_T*dPidR1b * dR0Ebdphi;
