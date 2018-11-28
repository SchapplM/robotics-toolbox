% Zeitableitung der Jacobi-Matrix für xzy-Euler-Winkel
% Zur Umrechnung zwischen zweiter Euler-Winkel-Zeitableitung und Winkelbeschleunigung
% Konvention: R = rotx(phi1) * rotz(phi2) * roty(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   xzy-Euler-Winkel
% phiD [3x1]:
%   Zeitableitung der xzy-Euler-Winkel
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
function JD = eulxzyjacD(phi, phiD)
%% Init
%#codegen
%$cgargs {zeros(3,1),zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulxzyjacD: phi has to be [3x1] (double)');
assert(isreal(phiD) && all(size(phiD) == [3 1]), 'eulxzyjacD: phiD has to be [3x1] (double)');
phi1_s=phi(1); phi2_s=phi(2); phi3_s=phi(3);
phi1D_s=phiD(1); phi2D_s=phiD(2); phi3D_s=phiD(3);
%% Berechnung
% aus codeexport/eulxzyjacD_matlab.m (euler_angle_calculations.mw)
t491 = sin(phi1_s);
t496 = phi1D_s * t491;
t493 = cos(phi1_s);
t495 = phi1D_s * t493;
t494 = phi2D_s * sin(phi2_s);
t492 = cos(phi2_s);
t1 = [0 0 -phi2D_s * t492; 0 -t495 -t492 * t496 - t493 * t494; 0 -t496 -t491 * t494 + t492 * t495;];
JD = t1;
