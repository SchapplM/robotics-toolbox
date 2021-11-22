% Produkt zweier homogener Transformationsmatrizen
% Keine Matrix-Multiplikation, sondern Aufteilung Rotation/Translation.
% Kann nützlich sein, wenn die Translation NaN enthält
% 
% Eingabe:
% Tr_1, Tr_2 [4x4], SE(3)
% 
% Ausgabe:
% Tr_p = Tr_1 * Tr_2 [4x4], SE(3)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-11
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function Tr_p = trprod(Tr_1, Tr_2)

Tr_p = [Tr_1(1:3,1:3)*Tr_2(1:3,1:3), Tr_1(1:3,1:3)*Tr_2(1:3,4)+Tr_1(1:3,4);...
        [0 0 0 1]];