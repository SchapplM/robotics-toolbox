% Jacobi-Matrix für xzy-Euler-Winkel
% Zur Umrechnung zwischen Euler-Winkel-Ableitung und Winkelgeschwindigkeit
% Konvention: R = rotx(phi1) * rotz(phi2) * roty(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   xzy-Euler-Winkel
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
function J = eulxzyjac(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulxzyjac: phi has to be [3x1] (double)');
phi1_s=phi(1); phi2_s=phi(2); phi3_s=phi(3);
%% Berechnung
% aus codeexport/eulxzyjac_matlab.m (euler_angle_calculations.mw)
t444 = cos(phi1_s);
t443 = cos(phi2_s);
t442 = sin(phi1_s);
t1 = [1 0 -sin(phi2_s); 0 -t442 t444 * t443; 0 t444 t442 * t443;];
J = t1;
