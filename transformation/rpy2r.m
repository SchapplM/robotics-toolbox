% RPY-Winkel nach Rotationsmatrix konvertieren

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-05
% (c) Institut für Regelungstechnik, Universität Hannover

function R = rpy2r(phi)
%#codegen
%#cgargs {zeros(3,1)}
assert(all(size(phi) == [3 1]) && isreal(phi), ...
  'RPY angles have to be [3x1] (double)'); 

%% Berechnung
% Quelle:
% maple_codegen/rotation_rpy_omega.mw
% maple_codegen/codeexport/R_from_rpy.m

alpha_s = phi(1);
beta_s = phi(2);
gamma_s = phi(3);


t2 = sin(alpha_s);
t5 = sin(beta_s);
t8 = t2 * t5;
t4 = cos(alpha_s);
t7 = t4 * t5;
t6 = cos(beta_s);
t3 = cos(gamma_s);
t1 = sin(gamma_s);
R = [t6 * t3 -t6 * t1 t5; t4 * t1 + t3 * t8 -t1 * t8 + t4 * t3 -t2 * t6; t2 * t1 - t3 * t7 t1 * t7 + t2 * t3 t4 * t6;];
