% yxz-Euler-Winkel in eine Rotationsmatrix konvertieren

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function R = eulyxz2r(phi)
%% Init
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi) && all(size(phi) == [3 1]), 'eulyxz2r: phi has to be [3x1] (double)');
phi1=phi(1); phi2=phi(2); phi3=phi(3);
%% Berechnung
% R = roty(phi1) * rotx(phi2) * rotz(phi3):
% aus codeexport/eulyxz2r_matlab.m
t48 = sin(phi3);
t50 = sin(phi1);
t57 = t50 * t48;
t51 = cos(phi3);
t56 = t50 * t51;
t53 = cos(phi1);
t55 = t53 * t48;
t54 = t53 * t51;
t52 = cos(phi2);
t49 = sin(phi2);
t1 = [t49 * t57 + t54 t49 * t56 - t55 t50 * t52; t52 * t48 t52 * t51 -t49; t49 * t55 - t56 t49 * t54 + t57 t53 * t52;];
R = t1;
