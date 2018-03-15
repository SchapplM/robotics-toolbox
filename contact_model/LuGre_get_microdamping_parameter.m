% Berechne den LuGre-Parameter für Mikrodämpfung (sigma1) nach [AstromWit2008]
% 
% Eingabe:
% zeta 
%   Modale Dämpfung (sinnvoll: 0.0 bis 1.0)
% sigma0, sigma2
%   LuGre-Parameter nach [AstromWit2008]
% m
%   Masse des Körpers, auf den die Reibung wirkt
% 
% Ausgabe:
% sigma1
%   Berechnetes sigma1, dass bei einem Einmassensystem mit vorgegebenen
%   Eingangsdaten die gewünschte modale Dämpfung erzeugt.
% 
% Quelle:
% [AstromWit2008] K.J. Astrom and C. Canudas-de-Wit: Revisiting the LuGre
% friction model (2008)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-12
% (c) Institut für Regelungstechnik, Universität Hannover

function sigma1 = LuGre_get_microdamping_parameter(zeta, sigma0, sigma2, m)

% [AstromWit2008], S.104 oben, rechte Spalte
sigma1 = 2*zeta*sqrt(sigma0*m) - sigma2;