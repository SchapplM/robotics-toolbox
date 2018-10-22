% Jacobi-Matrix für zyx-Euler-Winkel
% Zur Umrechnung zwischen Euler-Winkel-Ableitung und Winkelgeschwindigkeit
% Konvention: R = rotz(phi1) * roty(phi2) * rotx(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   zyx-Euler-Winkel
%
% Ausgabe:
% J [3x3]:
%   Euler-Jacobimatrix: Linearer Zusammenhang zwischen
%   Winkelgeschwindigkeit und Euler-Winkel-Zeitableitung
%   w = J * phiD
%
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover
%
function J = eulzyxjac(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulzyxjac: phi has to be [3x1] (double)');
phi1_s=phi(1); phi2_s=phi(2); phi3_s=phi(3);
%% Berechnung
% aus codeexport/eulzyxjac_matlab.m (euler_angle_calculations.mw)
t465 = cos(phi1_s);
t464 = cos(phi2_s);
t463 = sin(phi1_s);
t1 = [0 -t463 t465 * t464; 0 t465 t463 * t464; 1 0 -sin(phi2_s);];
J = t1;
