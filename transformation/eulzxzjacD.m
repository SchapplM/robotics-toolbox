% Zeitableitung der Jacobi-Matrix für zxz-Euler-Winkel
% Zur Umrechnung zwischen zweiter Euler-Winkel-Zeitableitung und Winkelbeschleunigung
% Konvention: R = rotz(phi1) * rotx(phi2) * rotz(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   zxz-Euler-Winkel
% phiD [3x1]:
%   Zeitableitung der zxz-Euler-Winkel
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
function JD = eulzxzjacD(phi, phiD)
%% Init
%#codegen
%$cgargs {zeros(3,1),zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulzxzjacD: phi has to be [3x1] (double)');
assert(isreal(phiD) && all(size(phiD) == [3 1]), 'eulzxzjacD: phiD has to be [3x1] (double)');
phi1_s=phi(1); phi2_s=phi(2); phi3_s=phi(3);
phi1D_s=phiD(1); phi2D_s=phiD(2); phi3D_s=phiD(3);
%% Berechnung
% aus codeexport/eulzxzjacD_matlab.m (euler_angle_calculations.mw)
t533 = sin(phi1_s);
t538 = phi1D_s * t533;
t535 = cos(phi1_s);
t537 = phi1D_s * t535;
t536 = phi2D_s * cos(phi2_s);
t532 = sin(phi2_s);
t1 = [0 -t538 t532 * t537 + t533 * t536; 0 t537 t532 * t538 - t535 * t536; 0 0 -phi2D_s * t532;];
JD = t1;
