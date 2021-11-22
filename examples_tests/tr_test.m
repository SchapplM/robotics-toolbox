% Teste Funktionen für homogene Transformationsmatrizen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-11
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

for i = 1:100
  Tr_1 = transl(rand(3,1))*r2t(eulxyz2r(rand(3,1)));
  Tr_2 = transl(-0.5+rand(3,1))*r2t(eulxyz2r(-0.5+rand(3,1)));
  Tr_p1 = trprod(Tr_1, Tr_2);
  Tr_p2 = Tr_1 * Tr_2;
  test_Tr_p = Tr_p1 - Tr_p2;
  assert(all(abs(test_Tr_p(:)) < 1e-10), 'Funktion trprod stimmt nicht');
end