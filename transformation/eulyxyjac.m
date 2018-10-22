% Jacobi-Matrix für yxy-Euler-Winkel
% Zur Umrechnung zwischen Euler-Winkel-Ableitung und Winkelgeschwindigkeit
% Konvention: R = roty(phi1) * rotx(phi2) * roty(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   yxy-Euler-Winkel
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
function J = eulyxyjac(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulyxyjac: phi has to be [3x1] (double)');
phi1_s=phi(1); phi2_s=phi(2); phi3_s=phi(3);
%% Berechnung
% aus codeexport/eulyxyjac_matlab.m (euler_angle_calculations.mw)
t447 = cos(phi1_s);
t446 = sin(phi1_s);
t445 = sin(phi2_s);
t1 = [0 t447 t446 * t445; 1 0 cos(phi2_s); 0 -t446 t447 * t445;];
J = t1;
