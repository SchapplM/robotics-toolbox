% Zeitableitung der Jacobi-Matrix für yzy-Euler-Winkel
% Zur Umrechnung zwischen zweiter Euler-Winkel-Zeitableitung und Winkelbeschleunigung
% Konvention: R = roty(phi1) * rotz(phi2) * roty(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   yzy-Euler-Winkel
% phiD [3x1]:
%   Zeitableitung der yzy-Euler-Winkel
%
% Ausgabe:
% JD [3x3]:
%   Zeitableitung der Euler-Jacobimatrix: Zusammenhang zwischen
%   Winkelbeschleunigung und zweiter Euler-Winkel-Zeitableitung
%   wD = J * phiDD + JD * phiD
%
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-11
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover
%
function JD = eulyzyjacD(phi, phiD)
%% Init
%#codegen
%$cgargs {zeros(3,1),zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulyzyjacD: phi has to be [3x1] (double)');
assert(isreal(phiD) && all(size(phiD) == [3 1]), 'eulyzyjacD: phiD has to be [3x1] (double)');
phi1_s=phi(1); phi2_s=phi(2); phi3_s=phi(3);
phi1D_s=phiD(1); phi2D_s=phiD(2); phi3D_s=phiD(3);
%% Berechnung
% aus codeexport/eulyzyjacD_matlab.m (euler_angle_calculations.mw)
t519 = sin(phi1_s);
t524 = phi1D_s * t519;
t521 = cos(phi1_s);
t523 = phi1D_s * t521;
t522 = phi2D_s * cos(phi2_s);
t518 = sin(phi2_s);
t1 = [0 t523 t518 * t524 - t521 * t522; 0 0 -phi2D_s * t518; 0 -t524 t518 * t523 + t519 * t522;];
JD = t1;
