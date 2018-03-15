% Teste Transformationsfunktionen für Rotationsvektor
% 
% Ergebnis: 
% Test noch nicht erfolgreich.
% Vorzeichen der Rotation nicht eindeutig. Teilweise werden die
% Rotationsmatrizen durch die Konvertierung transponiert.

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-08
% (c) Institut für Regelungstechnik, Universität Hannover

%% Init
clc
clear

% zufällige Rotationsmatrizen
n = 100;
abc = (0.5-rand(n,3))*pi;
R_ges = NaN(3,3,n);

for i = 1:n
  R_ges(:,:,i) = rotx(abc(i,1))*roty(abc(i,2))*rotz(abc(i,3));
end

%% Teste r2rotvec und rotvec2r
for i = 1:n
  R_i = R_ges(:,:,i);
  rotvec_i = r2rotvec(R_i);
  R_rotvec_i = rotvec2r(rotvec_i);
  
  if any( abs( R_rotvec_i(:) - R_i(:) ) > 1e-10 )
    
  [theta, n] = r2angvec(R_i)
  rotvec = n' * theta;
  
  k = rotvec / theta; % Einheitsvektor, um den rotiert wird
  R = angvec2r(theta, k');
    
    error('i = %d. Umrechnung rotvec2r/r2rotvec stimmt nicht', i);
  end
end
fprintf('%d Umrechnungen mit rotvec2r/r2rotvec getestet\n', n);

