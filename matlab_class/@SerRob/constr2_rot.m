% Rotatorische Komponente der Zwangsbedingungen 
% Variante 2:
% * Absolute Rotation ausgedrückt z.B. in XYZ-Euler-Winkeln
% * Rotationsfehler ausgedrückt in Euler-Winkeln (um raumfeste Achsen), je
%   nach Eingabeargument `reci` (entspricht teilweise PKM-Variante 2)
% 
% Eingabe:
% qJ
%   Gelenkkoordinaten des Roboters
% xE
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% reci (Optional)
%   true: Nehme reziproke Euler-Winkel für Orientierungsfehler (z.B.
%   ZYX-Orientierungsfehler für XYZ-Absolutorientierung)
%   false: Gleiche Euler-Winkel für Fehler und Absolut [Standard]
% 
% Ausgabe:
% Phi [3x1]
%   Kinematische Zwangsbedingungen des Roboters: Maß für Orientierungsfehler 
%   zwischen Ist-Pose aus gegebenen Gelenkwinkeln q und
%   Soll-Pose aus gegebenen EE-Koordinaten x

% Quellen:
% [SchapplerTapOrt2019] Schappler, M. et al.: Resolution of Functional
% Redundancy for 3T2R Robot Tasks using Two Sets of Reciprocal Euler
% Angles, Proc. of the 15th IFToMM World Congress, 2019
% [A] Aufzeichnungen Schappler vom 27.07.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Phi_r = constr2_rot(Rob, qJ, xE, reci)

assert(isreal(qJ) && all(size(qJ) == [Rob.NQJ 1]), ...
  'SerRob/constr2_rot: qJ muss %dx1 sein', Rob.NQJ);
if nargin < 4
  reci = false;
end

R_0_E_x = eul2r(xE(4:6), Rob.phiconv_W_E);
if reci
  [~,phiconv_delta] = euler_angle_properties(Rob.phiconv_W_E);
else
  phiconv_delta = Rob.phiconv_W_E;
end

T_0_E_q = Rob.fkineEE(qJ);
  
R_0_E_q = T_0_E_q(1:3,1:3);

% Argument in [SchapplerTapOrt2019]/(9) bzw. Gl. (A.47)  
R_Ex_Eq = R_0_E_x' * R_0_E_q;

% Differenz-Rotation mit z.B. mit ZYX-Euler-Winkel
% [SchapplerTapOrt2019]/(9) mit (11)
% Gl. (A.30) (dort Vertauschung der Reihenfolge, hier nicht)
Phi_r = r2eul(R_Ex_Eq, phiconv_delta);
