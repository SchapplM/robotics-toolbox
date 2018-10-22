% Jacobi-Matrix für yzy-Euler-Winkel
% Zur Umrechnung zwischen Euler-Winkel-Ableitung und Winkelgeschwindigkeit
% Konvention: R = roty(phi1) * rotz(phi2) * roty(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   yzy-Euler-Winkel
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
function J = eulyzyjac(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulyzyjac: phi has to be [3x1] (double)');
phi1_s=phi(1); phi2_s=phi(2); phi3_s=phi(3);
%% Berechnung
% aus codeexport/eulyzyjac_matlab.m (euler_angle_calculations.mw)
t456 = cos(phi1_s);
t455 = sin(phi1_s);
t454 = sin(phi2_s);
t1 = [0 t455 -t456 * t454; 1 0 cos(phi2_s); 0 t456 t455 * t454;];
J = t1;
