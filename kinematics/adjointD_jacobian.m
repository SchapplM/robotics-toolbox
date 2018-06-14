% Zeitableitung der Adjungierten Matrix aus einem Positionsvektor erzeugen
% Mit der adjungierten können Jacobi-Matrizen auf unterschiedliche Punkte
% umgerechnet werden
% 
% Eingabe:
% r_i_B_C [3x1]
%   Vektor von Punkt B zu Punkt C ausgedrückt im KS i
%   Der Vektor muss konstant bezüglich des KS i sein.
% R_W_i [3x3]
%   Rotationsmatrix vom KS i ins KSW (rotiert einen Vektor vom KS i ins KS W)
% omega_W_i [3x1]
%   Winkelgeschwindigkeit des KS i bezogen auf KS W und ausgedrückt im KS W
% 
% Ausgabe:
% AD_C_B [6x6]
%   Zeitableitung der Adjungierten Matrix (Matrix ist im KS W)
% 
% Siehe auch:
% [VorndammeSchHad2017] Jonathan Vorndamme, Moritz Schappler, Sami
% Haddadin: Collision Detection, Isolation and Identification for Humanoids
% (2017)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2017-11
% (C) Institut fuer Regelungstechnik, Universitaet Hannover

function AD_C_B = adjointD_jacobian(r_i_B_C, R_W_i, omega_W_i)
%#codegen
%#cgargs {zeros(3,1),zeros(3,3),zeros(3,1)}
assert(isreal(r_i_B_C) && all(size(r_i_B_C) == [3 1]), ...
  'adjointD_jacobian: r_i_B_C has to be [3x1] (double)');
assert(isreal(R_W_i) && all(size(R_W_i) == [3 3]), ...
  'adjointD_jacobian: R_W_i has to be [3x3] (double)');
assert(isreal(omega_W_i) && all(size(omega_W_i) == [3 1]), ...
  'adjointD_jacobian: omega_W_i has to be [3x1] (double)');

% Siehe auch: adjoint_jacobian.m
r_W_B_C = R_W_i * r_i_B_C;
rD_W_B_C = cross(omega_W_i, r_W_B_C);
AD_C_B = [zeros(3,3), -skew(rD_W_B_C); zeros(3,3), zeros(3,3)];
