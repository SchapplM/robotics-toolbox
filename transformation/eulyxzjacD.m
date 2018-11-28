% Zeitableitung der Jacobi-Matrix für yxz-Euler-Winkel
% Zur Umrechnung zwischen zweiter Euler-Winkel-Zeitableitung und Winkelbeschleunigung
% Konvention: R = roty(phi1) * rotx(phi2) * rotz(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   yxz-Euler-Winkel
% phiD [3x1]:
%   Zeitableitung der yxz-Euler-Winkel
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
function JD = eulyxzjacD(phi, phiD)
%% Init
%#codegen
%$cgargs {zeros(3,1),zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulyxzjacD: phi has to be [3x1] (double)');
assert(isreal(phiD) && all(size(phiD) == [3 1]), 'eulyxzjacD: phiD has to be [3x1] (double)');
phi1_s=phi(1); phi2_s=phi(2); phi3_s=phi(3);
phi1D_s=phiD(1); phi2D_s=phiD(2); phi3D_s=phiD(3);
%% Berechnung
% aus codeexport/eulyxzjacD_matlab.m (euler_angle_calculations.mw)
t505 = sin(phi1_s);
t510 = phi1D_s * t505;
t507 = cos(phi1_s);
t509 = phi1D_s * t507;
t508 = phi2D_s * sin(phi2_s);
t506 = cos(phi2_s);
t1 = [0 -t510 -t505 * t508 + t506 * t509; 0 0 -phi2D_s * t506; 0 -t509 -t506 * t510 - t507 * t508;];
JD = t1;
