% Zeitableitung der Jacobi-Matrix für zyz-Euler-Winkel
% Zur Umrechnung zwischen zweiter Euler-Winkel-Zeitableitung und Winkelbeschleunigung
% Konvention: R = rotz(phi1) * roty(phi2) * rotz(phi3).
% (mitgedrehte Euler-Winkel; intrinsisch)
%
% Eingabe:
% phi [3x1]:
%   zyz-Euler-Winkel
% phiD [3x1]:
%   Zeitableitung der zyz-Euler-Winkel
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
function JD = eulzyzjacD(phi, phiD)
%% Init
%#codegen
%$cgargs {zeros(3,1),zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulzyzjacD: phi has to be [3x1] (double)');
assert(isreal(phiD) && all(size(phiD) == [3 1]), 'eulzyzjacD: phiD has to be [3x1] (double)');
phi1_s=phi(1); phi2_s=phi(2); phi3_s=phi(3);
phi1D_s=phiD(1); phi2D_s=phiD(2); phi3D_s=phiD(3);
%% Berechnung
% aus codeexport/eulzyzjacD_matlab.m (euler_angle_calculations.mw)
t547 = sin(phi1_s);
t552 = phi1D_s * t547;
t549 = cos(phi1_s);
t551 = phi1D_s * t549;
t550 = phi2D_s * cos(phi2_s);
t546 = sin(phi2_s);
t1 = [0 -t551 -t546 * t552 + t549 * t550; 0 -t552 t546 * t551 + t547 * t550; 0 0 -phi2D_s * t546;];
JD = t1;
