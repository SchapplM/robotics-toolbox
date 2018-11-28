% Zeitableitung der Jacobi-Matrix für xzx-Euler-Winkel
% Zur Umrechnung zwischen zweiter Euler-Winkel-Zeitableitung und Winkelbeschleunigung
% Konvention: R = rotx(phi1) * rotz(phi2) * rotx(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   xzx-Euler-Winkel
% phiD [3x1]:
%   Zeitableitung der xzx-Euler-Winkel
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
function JD = eulxzxjacD(phi, phiD)
%% Init
%#codegen
%$cgargs {zeros(3,1),zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulxzxjacD: phi has to be [3x1] (double)');
assert(isreal(phiD) && all(size(phiD) == [3 1]), 'eulxzxjacD: phiD has to be [3x1] (double)');
phi1_s=phi(1); phi2_s=phi(2); phi3_s=phi(3);
phi1D_s=phiD(1); phi2D_s=phiD(2); phi3D_s=phiD(3);
%% Berechnung
% aus codeexport/eulxzxjacD_matlab.m (euler_angle_calculations.mw)
t484 = sin(phi1_s);
t489 = phi1D_s * t484;
t486 = cos(phi1_s);
t488 = phi1D_s * t486;
t487 = phi2D_s * cos(phi2_s);
t483 = sin(phi2_s);
t1 = [0 0 -phi2D_s * t483; 0 -t488 -t483 * t489 + t486 * t487; 0 -t489 t483 * t488 + t484 * t487;];
JD = t1;
