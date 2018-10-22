% Jacobi-Matrix für xyx-Euler-Winkel
% Zur Umrechnung zwischen Euler-Winkel-Ableitung und Winkelgeschwindigkeit
% Konvention: R = rotx(phi1) * roty(phi2) * rotx(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   xyx-Euler-Winkel
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
function J = eulxyxjac(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulxyxjac: phi has to be [3x1] (double)');
phi1_s=phi(1); phi2_s=phi(2); phi3_s=phi(3);
%% Berechnung
% aus codeexport/eulxyxjac_matlab.m (euler_angle_calculations.mw)
t435 = cos(phi1_s);
t434 = sin(phi1_s);
t433 = sin(phi2_s);
t1 = [1 0 cos(phi2_s); 0 t435 t434 * t433; 0 t434 -t435 * t433;];
J = t1;
