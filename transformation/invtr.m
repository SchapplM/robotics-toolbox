% Inverse Transformationsmatrix berechnen
% 
% Quelle:
% [1] Ortmaier,T.: Robotik I Vorlesungsskript

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-07
% (c) Institut für Regelungstechnik, Universität Hannover

function invTrM = invtr(TrM)
%% Init
% Coder Information
%#codegen
%$cgargs {zeros(4,4)}
assert(isreal(TrM) && all(size(TrM) == [4 4]), ...
  'invtr: Transformationsmatrix muss [4x4] (double) sein');   

%% Berechnung
R = TrM(1:3,1:3);
p = TrM(1:3,4);

invTrM = [R.'    , -R.' * p; ...
          [0 0 0], 1];
