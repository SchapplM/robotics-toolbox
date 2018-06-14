% Transformation zwischen Winkelgeschwindigkeit und Ableitung der
% Orientierungsdarstellung für RPY-Winkel (=XYZ-Euler-Winkel).
% 
% Eingabe:
% phi_rpy
%   Euler-Winkel für XYZ-Darstellung
% 
% Ausgabe:
% T
%   Transformationsmatrix (3x3) zwischen Winkelgeschwindigkeit im
%   Basiskoordinatensystem und den Zeitableitungen der
%   Rotationsdarstellungen (w = T*rpyD)
% 
% Source:
% [1] Natale 2003: Interaction Control of Robot Manipulators:
% Six-Degrees-of-Freedom Tasks 
% [2] Ortmaier: Robotik I Skript WS 2014/15
% [3] Corke: Robotics Toolbox

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-08
% (c) Institut für Regelungstechnik, Universität Hannover

function T = angvelotrans_rpy(phi_rpy)
%% Init
%#codegen
%#cgargs {zeros(3,1)}
assert(isreal(phi_rpy) && all(size(phi_rpy) == [3 1]), ...
      'angvelotrans_rpy: phi_rpy = [3x1] (double)');  
alpha = phi_rpy(1);
beta = phi_rpy(2);
gamma = phi_rpy(3);
%% Calculation
% Definition: [1], (2.34)
% Herleitung: [2], S.52, (4.23)
T = [1, 0,          sin(beta);
     0, cos(alpha), -sin(alpha)*cos(beta);
     0, sin(alpha), cos(alpha)*cos(beta)];

% Implementierung für andere Euler-Winkel: [3], eul2jac
% Die Implementierung in [3], rpy2jac wurde in einer neuen Version
% geändert. Vorher war sie identisch mit obiger...
