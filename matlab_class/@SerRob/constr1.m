% Kinematische Zwangsbedingungen zwischen Ist- und Soll-Konfiguration
% Vollständige Rotations- und Translationskomponenten
% Variante 1:
% * Vektor vom Basis- zum EE-KS (unterschiedlich zur Variante 1 bei PKM)
% * Absolute Rotation ausgedrückt bspw. in XYZ-Euler-Winkeln
%   (statt XYZ wird die Konvention aus `phiconv_W_E` genommen)
% * Rotationsfehler ausgedrückt genauso bspw. in XYZ-Euler-Winkeln
%   Rotationsfehler wird als R_0_E * R_D_0 angenommen (also 0(q)->0(x))
%   (anders herum als in [SchapplerTapOrt2019])
% 
% Eingabe:
% q
%   Gelenkwinkel des Roboters
% Tr0Ex
%   Endeffektorpose des Roboters bezüglich des Basis-KS
%   Homogene Transformationsmatrix ohne letzte Zeile.
% 
% Ausgabe:
% Phi [6x1]
%   Kinematische Zwangsbedingungen des Roboters: Maß für den Positions- und
%   Orientierungsfehler zwischen Ist-Pose aus gegebenen Gelenkwinkeln q und
%   Soll-Pose aus gegebenen EE-Koordinaten x
% 
% Quelle:
% [SchapplerTapOrt2019] Schappler, M. et al.: Resolution of Functional
% Redundancy for 3T2R Robot Tasks using Two Sets of Reciprocal Euler
% Angles, Proc. of the 15th IFToMM World Congress, 2019

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Phi = constr1(R, q, Tr0Ex)

% [SchapplerTapOrt2019], Gl. 8
Phi_t = R.constr1_trans(q, Tr0Ex);
% Entspricht [SchapplerTapOrt2019], Gl. 9 mit unterschiedlicher Rotation
Phi_r = R.constr1_rot(q, Tr0Ex);
% Entspricht [SchapplerTapOrt2019], Gl. 7
Phi = [Phi_t; Phi_r];