% Translatorische Komponente der Zwangsbedingungen
% Variante 1:
% * Vektor vom Basis- zum EE-KS (unterschiedlich zur Variante 1 bei PKM)
% 
% Eingabe:
% qJ
%   Gelenkkoordinaten des Roboters
% Tr0Ex
%   Endeffektorpose des Roboters bezüglich des Basis-KS
%   Homogene Transformationsmatrix ohne letzte Zeile.
% 
% Ausgabe:
% Phix [3x1]
%   Kinematische Zwangsbedingungen des Roboters: Maß für Positionsfehler 
%   zwischen Ist-Pose aus gegebenen Gelenkwinkeln q und
%   Soll-Pose aus gegebenen EE-Koordinaten x

% Quellen:
% [SchapplerTapOrt2019] Schappler, M. et al.: Resolution of Functional
% Redundancy for 3T2R Robot Tasks using Two Sets of Reciprocal Euler
% Angles, Proc. of the 15th IFToMM World Congress, 2019
% [A] Aufzeichnungen Schappler vom 15.06.2018
% [B] Aufzeichnungen Schappler vom 22.06.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für mechatronische Systeme, Universität Hannover

function Phix = constr1_trans(Rob, qJ, Tr0Ex)

assert(isreal(qJ) && all(size(qJ) == [Rob.NQJ 1]), ...
  'SerRob/constr1_trans: q muss %dx1 sein', Rob.NQJ);

r_0_E_x = Tr0Ex(1:3,4);

% Direkte Kinematik der Beinkette
T_0_E = Rob.fkineEE(qJ);
r_0_E_q = T_0_E(1:3,4);

% [SchapplerTapOrt2019]/(8) bzw. Gl. (A.23, B.22)
Phix = r_0_E_q - r_0_E_x;
