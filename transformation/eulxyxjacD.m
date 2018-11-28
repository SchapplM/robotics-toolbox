% Zeitableitung der Jacobi-Matrix für xyx-Euler-Winkel
% Zur Umrechnung zwischen zweiter Euler-Winkel-Zeitableitung und Winkelbeschleunigung
% Konvention: R = rotx(phi1) * roty(phi2) * rotx(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   xyx-Euler-Winkel
% phiD [3x1]:
%   Zeitableitung der xyx-Euler-Winkel
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
function JD = eulxyxjacD(phi, phiD)
%% Init
%#codegen
%$cgargs {zeros(3,1),zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulxyxjacD: phi has to be [3x1] (double)');
assert(isreal(phiD) && all(size(phiD) == [3 1]), 'eulxyxjacD: phiD has to be [3x1] (double)');
phi1_s=phi(1); phi2_s=phi(2); phi3_s=phi(3);
phi1D_s=phiD(1); phi2D_s=phiD(2); phi3D_s=phiD(3);
%% Berechnung
% aus codeexport/eulxyxjacD_matlab.m (euler_angle_calculations.mw)
t470 = sin(phi1_s);
t475 = phi1D_s * t470;
t472 = cos(phi1_s);
t474 = phi1D_s * t472;
t473 = phi2D_s * cos(phi2_s);
t469 = sin(phi2_s);
t1 = [0 0 -phi2D_s * t469; 0 -t475 t469 * t474 + t470 * t473; 0 t474 t469 * t475 - t472 * t473;];
JD = t1;
