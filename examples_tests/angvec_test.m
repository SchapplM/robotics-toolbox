% Teste Rotationen nach der Achse-Winkel-Notation
% 
% Ergebnis:
% * Rotation stimmt mit verschiedenen Funktionen
% * Ergebnisse der Geschwindigkeit stimmen nicht exakt überein. Womöglich
% numerische Fehler. In cone_rotation_trajectory_example.m stimmen die
% Integration der Winkelgeschwindigkeiten aus angvec2r_sym.m und
% angvecD2omega_sym.m überein
% 
% Siehe: cone_rotation_trajectory_example.m
 
% Moritz Schappler, schappler@irt.uni-hannover.de, 2017-03
% (c) Institut für Regelungstechnik, Universität Hannover

clear
clc

n = 100;

%% Teste symbolisch generierte Umwandlung

for i = 1:n
  theta = rand(1);
  if i == n
    theta = 0; % Sonderfall
  end
  
  u = rand(3,1);
  unorm = u / norm(u);
  
  R_num = angvec2r(theta,unorm);
  R_sym = angvec2r_sym(theta,unorm);
  
  RDelta = R_num-R_sym;
  
  if any(abs(RDelta(:)) > 1e-10)
    error('Symbolisch generierte Umwandlung angvec2r stimmt nicht');
  end
end
fprintf('angvec2r symbolisch vs. numerisch für %d Kominationen getestet.\n', n);

%% Teste Rück-Umwandlung

for i = 1:n
  theta = (-1+2*rand(1))*pi; % -pi ... pi
  if i == n
    theta = 0; % Sonderfall
  end
  u = rand(3,1);
  unorm = u / norm(u);
  
  R = angvec2r(theta,unorm);
  
  [k_test, u_test] = r2angvec(R);
  R_test = angvec2r(k_test, u_test);
  
  RDelta = R-R_test;
  
  if any(abs(RDelta(:)) > 1e-10) || any(isnan(RDelta(:)))
    error('Umwandlung angvec2r/r2angvec stimmt nicht');
  end
end
fprintf('angvec2r vs. r2angvec für %d Kominationen getestet.\n', n);

return
%% Teste Zeitableitung
% TODO: Hier stimmt es noch nicht
for i = 1:n
  theta = rand(1);
  kD = 1e-3*rand(1);
  dt = 1e-8;
  u = rand(3,1);
  unorm = u / norm(u);
  
  R1 = angvec2r_sym(theta,unorm);
  R2 = angvec2r_sym(theta+kD*dt,unorm);
  
  RD_numdiff = (R2-R1)/dt;
  omegas_num = RD_numdiff*R1';
  omega_num = [omegas_num(3,2);omegas_num(1,3);omegas_num(2,1)];
  
  [omega_sym, R_sym, RD_sym] = angvecD2omega_sym(u, theta, kD);
  
  % Vergleich
  RD_Diff = RD_sym-RD_numdiff;
  if any(abs(RD_Diff(:)) > 1e-10)
    error('RD stimmt nicht zwischen num/sym');
  end
end
 