% Adjungierte Matrix aus einem Positionsvektor erzeugen
% Mit der adjungierten können Jacobi-Matrizen auf unterschiedliche Punkte
% umgerechnet werden
% 
% Eingabe:
% r_B_C [3x1]
%   Vektor von Punkt B zu Punkt C
% 
% Ausgabe:
% A_C_B [6x6]
%   Adjungierte Matrix
% 
% Beispiel mit Rechenregeln für adjungierte Matrizen:
% r_B_C = rand(3,1);
% A_C_B = adjoint_jacobian(r_B_C)
% inv(A_C_B) - adjoint_jacobian(-r_B_C)
% 
% Beispiel zur Anwendung auf Jacobi-Matrizen:
% J(On,n): Jacobi-Matrix zum Ursprung des letzten KS eines Roboters
% J(E,n)=A(E,On)*J(On,n): Jacobi-Matrix zum Endeffektor des Roboters
% (anderer Punkt auf dem letzten bewegten Körper der seriellen Kette)
% mit A(E,On) = adjoint_jacobian(r_0_On_E)
% 
% Siehe auch:
% [VorndammeSchHad2017] Jonathan Vorndamme, Moritz Schappler, Sami
% Haddadin: Collision Detection, Isolation and Identification for Humanoids
% (2017)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2017-11
% (C) Institut fuer Regelungstechnik, Universitaet Hannover

function A_C_B = adjoint_jacobian(r_B_C)
%#codegen
assert(isa(r_B_C,'double') && isreal(r_B_C) && all(size(r_B_C) == [3 1]), ...
  'adjoint_jacobian: r_B_C has to be [3x1] double');

% [VorndammeSchHad2017], Gl. (24)
A_C_B = [eye(3), -skew(r_B_C); zeros(3,3), eye(3)];
