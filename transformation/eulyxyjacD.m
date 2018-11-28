% Zeitableitung der Jacobi-Matrix für yxy-Euler-Winkel
% Zur Umrechnung zwischen zweiter Euler-Winkel-Zeitableitung und Winkelbeschleunigung
% Konvention: R = roty(phi1) * rotx(phi2) * roty(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   yxy-Euler-Winkel
% phiD [3x1]:
%   Zeitableitung der yxy-Euler-Winkel
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
function JD = eulyxyjacD(phi, phiD)
%% Init
%#codegen
%$cgargs {zeros(3,1),zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulyxyjacD: phi has to be [3x1] (double)');
assert(isreal(phiD) && all(size(phiD) == [3 1]), 'eulyxyjacD: phiD has to be [3x1] (double)');
phi1_s=phi(1); phi2_s=phi(2); phi3_s=phi(3);
phi1D_s=phiD(1); phi2D_s=phiD(2); phi3D_s=phiD(3);
%% Berechnung
% aus codeexport/eulyxyjacD_matlab.m (euler_angle_calculations.mw)
t498 = sin(phi1_s);
t503 = phi1D_s * t498;
t500 = cos(phi1_s);
t502 = phi1D_s * t500;
t501 = phi2D_s * cos(phi2_s);
t497 = sin(phi2_s);
t1 = [0 -t503 t497 * t502 + t498 * t501; 0 0 -phi2D_s * t497; 0 -t502 -t497 * t503 + t500 * t501;];
JD = t1;
