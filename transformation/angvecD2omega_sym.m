% Berechnung der Winkelgeschwindigkeit aus der Zeitableitung der
% Achse-Winkel-Notation
% 
% Eingabe:
% theta [1x1]
%   Drehwinkel 
% thetaD [1x1]
%   Zeitableitung des Drehwinkels
% k [3x1]
%   Drehachse
% 
% Ausgabe:
% omega [3x1]
%   Winkelgeschwindigkeit
% R [3x3]
%   Rotationsmatrix
% RD [3x3]
%   Zeitableitung der Rotationsmatrix
% 
% Quelle:
% Berechnungen Schappler, 06.03.2017
% 
% Siehe: angvec2r, angvec2r_sym

% Moritz Schappler, schappler@irt.uni-hannover.de, 2017-03
% (c) Institut für Regelungstechnik, Universität Hannover

function [omega, R, RD] = angvecD2omega_sym(u, theta, thetaD)

ux = u(1);
uy = u(2);
uz = u(3);

R = angvec2r_sym(theta, u);

%% Symbolisch generierter Code
% Rotationsmatrix-Zeitableitung
% Siehe rotation_rpy_omega.mw
t24 = sin(theta);
t30 = t24 * uz;
t25 = cos(theta);
t29 = ux * t25;
t28 = uy * t25;
t27 = uz * t25;
t26 = t24 * thetaD;
t23 = ux * uy * t24;
t22 = ux * t30;
t21 = uy * t30;
t1 = [(ux ^ 2 - 0.1e1) * t26 thetaD * (t23 - t27) thetaD * (t22 + t28); thetaD * (t23 + t27) (uy ^ 2 - 0.1e1) * t26 thetaD * (t21 - t29); thetaD * (t22 - t28) thetaD * (t21 + t29) (uz ^ 2 - 0.1e1) * t26;];

RD = t1;

%% Umrechnung in Winkelgeschwindigkeit
omega_skew = RD*R';
% Siehe: skew.m
omega = [omega_skew(3,2);omega_skew(1,3);omega_skew(2,1)];