% Zeitableitung der Jacobi-Matrix für xyz-Euler-Winkel
% Zur Umrechnung zwischen zweiter Euler-Winkel-Zeitableitung und Winkelbeschleunigung
% Konvention: R = rotx(phi1) * roty(phi2) * rotz(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   xyz-Euler-Winkel
% phiD [3x1]:
%   Zeitableitung der xyz-Euler-Winkel
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
function JD = eulxyzjacD(phi, phiD)
%% Init
%#codegen
%$cgargs {zeros(3,1),zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulxyzjacD: phi has to be [3x1] (double)');
assert(isreal(phiD) && all(size(phiD) == [3 1]), 'eulxyzjacD: phiD has to be [3x1] (double)');
phi1_s=phi(1); phi2_s=phi(2); phi3_s=phi(3);
phi1D_s=phiD(1); phi2D_s=phiD(2); phi3D_s=phiD(3);
%% Berechnung
% aus codeexport/eulxyzjacD_matlab.m (euler_angle_calculations.mw)
t477 = sin(phi1_s);
t482 = phi1D_s * t477;
t479 = cos(phi1_s);
t481 = phi1D_s * t479;
t480 = phi2D_s * sin(phi2_s);
t478 = cos(phi2_s);
t1 = [0 0 phi2D_s * t478; 0 -t482 t477 * t480 - t478 * t481; 0 t481 -t478 * t482 - t479 * t480;];
JD = t1;
