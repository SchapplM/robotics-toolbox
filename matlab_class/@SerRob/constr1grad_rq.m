% Ableitung der Rotationskomponente der kinematischen Zwangsbedingungen nach den Gelenkwinkeln
% Variante 1:
% * Absolute Rotation ausgedrückt z.B. in XYZ-Euler-Winkeln
% * Rotationsfehler genauso ausgedrückt z.B. in XYZ-Euler-Winkeln
%   (statt XYZ wird die Konvention aus `phiconv_W_E` genommen)
%   Rotationsfehler wird als R_0_E * R_0_D angenommen (also 0(q)->0(x))
%   (anders herum als in [SchapplerTapOrt2019])
% 
% Eingabe:
% q
%   Gelenkkoordinaten des Roboters
% xE
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% 
% Ausgabe:
% Phi_phi_i_Gradq [3xN]
%   Matrix mit Ableitungen der 3 rotatorischen Zwangsbedingungskomponenten
%   (in den Zeilen) nach den N Gelenkwinkeln (in den Spalten)

% Quellen:
% [SchapplerTapOrt2019] Schappler, M. et al.: Resolution of Functional
% Redundancy for 3T2R Robot Tasks using Two Sets of Reciprocal Euler
% Angles, Proc. of the 15th IFToMM World Congress, 2019
% [A] Aufzeichnungen Schappler vom 21.06.2018
% [B] Aufzeichnungen Schappler vom 13.07.2018
% [C] Aufzeichnungen Schappler vom 27.07.2018
% [D] Aufzeichnungen Schappler vom 21.08.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Phi_phi_i_Gradq = constr1grad_rq(Rob, q, xE)

assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'SerRob/constr1grad_rq: q muss %dx1 sein', Rob.NJ);

% Endergebnis, siehe [SchapplerTapOrt2019] Gl. 31 oder [B] Gl. 30

R_0_E_x = eul2r(xE(4:6), Rob.phiconv_W_E);

% Kinematik, Definitionen
T_0_E = Rob.fkineEE(q);
R_0_E_q = T_0_E(1:3,1:3);
R_0x_0q = R_0_E_q * R_0_E_x';
R_N_E = Rob.T_N_E(1:3,1:3); % Siehe Text vor [SchapplerTapOrt2019]/(32)

% Ableitung der Rotationsmatrix R_0_E nach q
% Term III aus [B]/(31) bzw. III aus [SchapplerTapOrt2019]/(31)
% Gl. D.7
b11=R_N_E(1,1);b12=R_N_E(1,2);b13=R_N_E(1,3);
b21=R_N_E(2,1);b22=R_N_E(2,2);b23=R_N_E(2,3);
b31=R_N_E(3,1);b32=R_N_E(3,2);b33=R_N_E(3,3);
dPidRb1 = [b11 0 0 b21 0 0 b31 0 0; 0 b11 0 0 b21 0 0 b31 0; 0 0 b11 0 0 b21 0 0 b31; b12 0 0 b22 0 0 b32 0 0; 0 b12 0 0 b22 0 0 b32 0; 0 0 b12 0 0 b22 0 0 b32; b13 0 0 b23 0 0 b33 0 0; 0 b13 0 0 b23 0 0 b33 0; 0 0 b13 0 0 b23 0 0 b33;];
dRb_0N_dq = Rob.jacobiR(q);
% [SchapplerTapOrt2019]/(32) für III in [SchapplerTapOrt2019]/(31) einsetzen
% Gl. D.7 für Term III in Gl. (C.49) einsetzen
dRb_0E_dq = dPidRb1 * dRb_0N_dq;

% Innere Ableitung des Matrix-Produktes
% Term II aus [B]/(31); entspricht II aus [SchapplerTapOrt2019]/(31); aber
% hier andere Reihenfolge der Multiplikation.
% Die Matrix R_0_E_x wird transponiert eingesetzt.
% aus rmatvecprod_diff_rmatvec1_matlab.m
b11=R_0_E_x(1,1);b12=R_0_E_x(2,1);b13=R_0_E_x(3,1);
b21=R_0_E_x(1,2);b22=R_0_E_x(2,2);b23=R_0_E_x(3,2);
b31=R_0_E_x(1,3);b32=R_0_E_x(2,3);b33=R_0_E_x(3,3);
dPidRb1 = [b11 0 0 b21 0 0 b31 0 0; 0 b11 0 0 b21 0 0 b31 0; 0 0 b11 0 0 b21 0 0 b31; b12 0 0 b22 0 0 b32 0 0; 0 b12 0 0 b22 0 0 b32 0; 0 0 b12 0 0 b22 0 0 b32; b13 0 0 b23 0 0 b33 0 0; 0 b13 0 0 b23 0 0 b33 0; 0 0 b13 0 0 b23 0 0 b33;];

% Ableitung der Euler-Winkel nach der Rot.-Matrix
% Term I aus [B]/(31) bzw. I aus [SchapplerTapOrt2019]/(31)
% Aus codeexport/eulxyz_diff_rmatvec_matlab.m
dphidRb = eul_diff_rotmat(R_0x_0q, Rob.phiconv_W_E);

% Gl. [B]/(31) bzw. [SchapplerTapOrt2019]/(31)
Phi_phi_i_Gradq = dphidRb * dPidRb1 * dRb_0E_dq;
