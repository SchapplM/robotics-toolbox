% Zeitableitung der Jacobi-Matrix für zxy-Euler-Winkel
% Zur Umrechnung zwischen zweiter Euler-Winkel-Zeitableitung und Winkelbeschleunigung
% Konvention: R = rotz(phi1) * rotx(phi2) * roty(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   zxy-Euler-Winkel
% phiD [3x1]:
%   Zeitableitung der zxy-Euler-Winkel
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
function JD = eulzxyjacD(phi, phiD)
%% Init
%#codegen
%$cgargs {zeros(3,1),zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulzxyjacD: phi has to be [3x1] (double)');
assert(isreal(phiD) && all(size(phiD) == [3 1]), 'eulzxyjacD: phiD has to be [3x1] (double)');
phi1_s=phi(1); phi2_s=phi(2); phi3_s=phi(3);
phi1D_s=phiD(1); phi2D_s=phiD(2); phi3D_s=phiD(3);
%% Berechnung
% aus codeexport/eulzxyjacD_matlab.m (euler_angle_calculations.mw)
t526 = sin(phi1_s);
t531 = phi1D_s * t526;
t528 = cos(phi1_s);
t530 = phi1D_s * t528;
t529 = phi2D_s * sin(phi2_s);
t527 = cos(phi2_s);
t1 = [0 -t531 t526 * t529 - t527 * t530; 0 t530 -t527 * t531 - t528 * t529; 0 0 phi2D_s * t527;];
JD = t1;
