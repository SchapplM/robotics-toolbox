% Rotatorische Komponente der Zwangsbedingungen 
% Variante 2:
% * Absolute Rotation ausgedrückt z.B. in XYZ-Euler-Winkeln
% * Rotationsfehler ausgedrückt z.B. in dazu reziproken ZYX-Euler-Winkeln
% (statt XYZ wird die Konvention aus `phiconv_W_E` genommen)
% 
% Eingabe:
% qJ
%   Gelenkkoordinaten des Roboters
% xE
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% 
% Ausgabe:
% Phi [3x1]
%   Kinematische Zwangsbedingungen des Roboters: Maß für Orientierungsfehler 
%   zwischen Ist-Pose aus gegebenen Gelenkwinkeln q und
%   Soll-Pose aus gegebenen EE-Koordinaten x

% Quellen:
% [A] Aufzeichnungen Schappler vom 27.07.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für mechatronische Systeme, Universität Hannover

function Phi = constr2_rot(Rob, qJ, xE)

assert(isreal(qJ) && all(size(qJ) == [Rob.NQJ 1]), ...
  'SerRob/constr2_rot: qJ muss %dx1 sein', Rob.NQJ);

R_0_E_x = eul2r(xE(4:6), Rob.phiconv_W_E);
[~,phiconv_W_E_reci] = euler_angle_properties(Rob.phiconv_W_E);

T_0_E_q = Rob.fkineEE(qJ);
  
R_0_E_q = T_0_E_q(1:3,1:3);

% Gl. (A.47)  
R_Ex_Eq = R_0_E_x' * R_0_E_q;

% Differenz-Rotation mit z.B. mit ZYX-Euler-Winkel
% Gl. (A.30) (dort Vertauschung der Reihenfolge, hier nicht)
phiR = r2eul(R_Ex_Eq, phiconv_W_E_reci);

Phi = phiR;
