% Kinematische Zwangsbedingungen zwischen Ist- und Soll-Konfiguration
% Vollständige Rotations- und Translationskomponenten
% Variante 1:
% * Vektor vom Basis- zum Koppelpunkt-KS (unterschiedlich zur Variante 1
%   bei seriellen Robotern; dort Basis bis EE)
% * Absolute Rotation ausgedrückt bspw. in XYZ-Euler-Winkeln
%   (statt XYZ wird die Konvention aus `phiconv_W_E` genommen)
% * Rotationsfehler definiert als 0x-0q ausgedrückt in XYZ-Euler-Winkeln
% 
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% 
% Ausgabe:
% Phi_red
%   Reduzierte kinematische Zwangsbedingungen (siehe folgendes)
%   Die Reduktion folgt aus der Klassenvariablen I_EE
% Phi [6x1]
%   Kinematische Zwangsbedingungen des Roboters: Maß für den Positions- und
%   Orientierungsfehler zwischen Ist-Pose aus gegebenen Gelenkwinkeln q und
%   Soll-Pose aus gegebenen EE-Koordinaten x

% Quelle:
% [2_SchapplerTapOrt2019a] Schappler, M. et al.: Modeling Parallel Robot
% Kinematics for 3T2R and 3T3R Tasks using Reciprocal Sets of Euler Angles
% (Arbeitstitel), Submitted to MDPI Robotics KaRD2, Version of 27.06.2019

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phi_red, Phi] = constr1D(R, q, qD, xE,xDE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [R.NJ 1]), ...
  'ParRob/constr1D: q muss %dx1 sein', R.NJ);
assert(isreal(qD) && all(size(qD) == [R.NJ 1]), ...
  'ParRob/constr1D: qD muss %dx1 sein', R.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1D: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr1D: xDE muss 6x1 sein');
% assert(all(size(rpy) == [3 1]) && isreal(rpy), ...
%   'ParRob/constr1D_trans: rpy angles have to be [3x1] (double)'); 
% assert(all(size(rpyD) == [3 1]) && isreal(rpyD), ...
%   'ParRob/constr1D_trans: rpy angles time derivatives have to be [3x1] (double)'); 

% rotatorischer und translatorischer Teil der ZB
% [Phit_red, Phi] = R.constr1D_rot(q, xE ,xDE, rpy, rpyD) ;
% [Phi_red, Phi] = R.constr1D_trans(q, xE ,xDE,rpy,rpyD);
[Phir_red_D, PhirD] = R.constr1D_rot (q,qD, xE,xDE);
[Phit_red_D, PhitD] = R.constr1D_trans(q, xE,xDE);
% Sortierung der ZB-Zeilen in den Matrizen nach Beingruppen, nicht nach ZB-Art
% Indizierung auch mit Klassenvariablen I_constr_t, I_constr_r, ...
Phi_red = NaN(size(Phit_red_D,1)+size(Phir_red_D,1), 1);
Phi =     NaN(size(PhitD,1)    +size(PhirD ,1),    1);
Phi_red(R.I_constr_t_red) = Phit_red_D;
Phi_red(R.I_constr_r_red) = Phir_red_D;
Phi(R.I_constr_t) = PhitD;
Phi(R.I_constr_r) = PhirD;

