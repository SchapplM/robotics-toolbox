% Zeitableitung der Jacobi-Matrix für zyx-Euler-Winkel
% Zur Umrechnung zwischen zweiter Euler-Winkel-Zeitableitung und Winkelbeschleunigung
% Konvention: R = rotz(phi1) * roty(phi2) * rotx(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   zyx-Euler-Winkel
% phiD [3x1]:
%   Zeitableitung der zyx-Euler-Winkel
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
function JD = eulzyxjacD(phi, phiD)
%% Init
%#codegen
%$cgargs {zeros(3,1),zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulzyxjacD: phi has to be [3x1] (double)');
assert(isreal(phiD) && all(size(phiD) == [3 1]), 'eulzyxjacD: phiD has to be [3x1] (double)');
phi1_s=phi(1); phi2_s=phi(2); phi3_s=phi(3);
phi1D_s=phiD(1); phi2D_s=phiD(2); phi3D_s=phiD(3);
%% Berechnung
% aus codeexport/eulzyxjacD_matlab.m (euler_angle_calculations.mw)
t540 = sin(phi1_s);
t545 = phi1D_s * t540;
t542 = cos(phi1_s);
t544 = phi1D_s * t542;
t543 = phi2D_s * sin(phi2_s);
t541 = cos(phi2_s);
t1 = [0 -t544 -t541 * t545 - t542 * t543; 0 -t545 -t540 * t543 + t541 * t544; 0 0 -phi2D_s * t541;];
JD = t1;
