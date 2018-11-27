% Ableitung der Rotationskomponente der ZB nach den Gelenkwinkeln
% Variante 2:
% * Absolute Rotation ausgedrückt z.B. in XYZ-Euler-Winkeln
% * Rotationsfehler ausgedrückt z.B. in dazu reziproken ZYX-Euler-Winkeln
%   (statt XYZ wird die Konvention aus `phiconv_W_E` genommen)
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
% [A] Aufzeichnungen Schappler vom 21.06.2018
% [B] Aufzeichnungen Schappler vom 13.07.2018
% [C] Aufzeichnungen Schappler vom 27.07.2018
% [D] Aufzeichnungen Schappler vom 21.08.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für mechatronische Systeme, Universität Hannover

function Phi_phi_i_Gradq = constr2grad_rq(Rob, q, xE)

assert(isreal(q) && all(size(q) == [Rob.NQJ 1]), ...
  'SerRob/constr2grad_rq: q muss %dx1 sein', Rob.NQJ);

% Endergebnis, siehe Gl. (B.30)

R_0_E_x = eul2r(xE(4:6), Rob.phiconv_W_E);
[~,phiconv_W_E_reci] = euler_angle_properties(Rob.phiconv_W_E);

% Kinematik, Definitionen
T_0_E = Rob.fkineEE(q);
R_0_E_q = T_0_E(1:3,1:3);
R_Ex_Eq = R_0_E_x' * R_0_E_q;
R_N_E = Rob.T_N_E(1:3,1:3);

% Term III aus Gl. (C.49): Ableitung der Rotationsmatrix R_0_E nach q
% Gl. D.7
b11=R_N_E(1,1);b12=R_N_E(1,2);b13=R_N_E(1,3);
b21=R_N_E(2,1);b22=R_N_E(2,2);b23=R_N_E(2,3);
b31=R_N_E(3,1);b32=R_N_E(3,2);b33=R_N_E(3,3);
dPidRb1 = [b11 0 0 b21 0 0 b31 0 0; 0 b11 0 0 b21 0 0 b31 0; 0 0 b11 0 0 b21 0 0 b31; b12 0 0 b22 0 0 b32 0 0; 0 b12 0 0 b22 0 0 b32 0; 0 0 b12 0 0 b22 0 0 b32; b13 0 0 b23 0 0 b33 0 0; 0 b13 0 0 b23 0 0 b33 0; 0 0 b13 0 0 b23 0 0 b33;];
dRb_0N_dq = Rob.jacobiR(q);
% Gl. D.7 für Term III in Gl. (C.49) einsetzen
dRb_0E_dq = dPidRb1 * dRb_0N_dq;

% Term II aus Gl. (C.49): Innere Ableitung des Matrix-Produktes
% aus ZB_diff_q_rmatvecprod_diff_rmatvec2_matlab
% (Matrix R_0_E_x wird transponiert in Vorlage eingesetzt)
a11=R_0_E_x(1,1);a12=R_0_E_x(2,1);a13=R_0_E_x(3,1);
a21=R_0_E_x(1,2);a22=R_0_E_x(2,2);a23=R_0_E_x(3,2);
a31=R_0_E_x(1,3);a32=R_0_E_x(2,3);a33=R_0_E_x(3,3);
dPidRb2 = [a11 a12 a13 0 0 0 0 0 0; a21 a22 a23 0 0 0 0 0 0; a31 a32 a33 0 0 0 0 0 0; 0 0 0 a11 a12 a13 0 0 0; 0 0 0 a21 a22 a23 0 0 0; 0 0 0 a31 a32 a33 0 0 0; 0 0 0 0 0 0 a11 a12 a13; 0 0 0 0 0 0 a21 a22 a23; 0 0 0 0 0 0 a31 a32 a33;];

% Term I aus Gl. (C.49): Ableitung der Euler-Winkel nach der Rot.-matrix
% (ZYX-Euler-Winkel des Orientierungsfehlers)
% Unabhängig vom Roboter (nur von Orientierungsdarstellung)
ddeltaRdRb = eul_diff_rotmat(R_Ex_Eq,phiconv_W_E_reci);

% Gl. (C.49) (dort Vertauschung der Reihenfolge)
Phi_phi_i_Gradq = ddeltaRdRb * dPidRb2 * dRb_0E_dq;

