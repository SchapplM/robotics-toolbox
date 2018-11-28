% Zeitableitung der Jacobi-Matrix für yzx-Euler-Winkel
% Zur Umrechnung zwischen zweiter Euler-Winkel-Zeitableitung und Winkelbeschleunigung
% Konvention: R = roty(phi1) * rotz(phi2) * rotx(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   yzx-Euler-Winkel
% phiD [3x1]:
%   Zeitableitung der yzx-Euler-Winkel
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
function JD = eulyzxjacD(phi, phiD)
%% Init
%#codegen
%$cgargs {zeros(3,1),zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulyzxjacD: phi has to be [3x1] (double)');
assert(isreal(phiD) && all(size(phiD) == [3 1]), 'eulyzxjacD: phiD has to be [3x1] (double)');
phi1_s=phi(1); phi2_s=phi(2); phi3_s=phi(3);
phi1D_s=phiD(1); phi2D_s=phiD(2); phi3D_s=phiD(3);
%% Berechnung
% aus codeexport/eulyzxjacD_matlab.m (euler_angle_calculations.mw)
t512 = sin(phi1_s);
t517 = phi1D_s * t512;
t514 = cos(phi1_s);
t516 = phi1D_s * t514;
t515 = phi2D_s * sin(phi2_s);
t513 = cos(phi2_s);
t1 = [0 t516 -t513 * t517 - t514 * t515; 0 0 phi2D_s * t513; 0 -t517 t512 * t515 - t513 * t516;];
JD = t1;
