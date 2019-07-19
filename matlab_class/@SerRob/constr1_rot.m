% Rotatorische Komponente der Zwangsbedingungen 
% Variante 1:
% * Absolute Rotation ausgedrückt z.B. in XYZ-Euler-Winkeln
% * Rotationsfehler genauso ausgedrückt z.B. in XYZ-Euler-Winkeln
%   (statt XYZ wird die Konvention aus `phiconv_W_E` genommen)
%   Rotationsfehler wird als R_0_E * R_0_D angenommen (also 0(q)->0(x))
%   Unterschiedlich zu [SchapplerTapOrt2019], Berechnung aber ähnlich
% 
% Eingabe:
% qJ
%   Gelenkkoordinaten des Roboters
% xE
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% 
% Ausgabe:
% Phi
%   Kinematische Zwangsbedingungen des Roboters: Maß für Orientierungsfehler 
%   zwischen Ist-Pose aus gegebenen Gelenkwinkeln q und
%   Soll-Pose aus gegebenen EE-Koordinaten x

% Quellen:
% [SchapplerTapOrt2019] Schappler, M. et al.: Resolution of Functional
% Redundancy for 3T2R Robot Tasks using Two Sets of Reciprocal Euler
% Angles, Proc. of the 15th IFToMM World Congress, 2019
% [A] Aufzeichnungen Schappler vom 21.06.2018
% [B] Aufzeichnungen Schappler vom 22.06.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Phi = constr1_rot(Rob, qJ, xE)

assert(isreal(qJ) && all(size(qJ) == [Rob.NQJ 1]), ...
  'SerRob/constr1_rot: qJ muss %dx1 sein', Rob.NQJ);

R_0_E_x = eul2r(xE(4:6), Rob.phiconv_W_E);

T_0_E_q = Rob.fkineEE(qJ);
  
% Differenz-Rotation mit Euler-Winkeln
% (selbe Konvention wie für absolute Orientierung)
R_0_E_q = T_0_E_q(1:3,1:3);
R_0q_0x = R_0_E_q * R_0_E_x';

% Gl. (A.1, B.25) bzw. [SchapplerTapOrt2019]/(9) mit anderer Reihenfolge
phi = r2eul(R_0q_0x, Rob.phiconv_W_E);

Phi = phi;
