% Teste unterschiedliche Transformationsfunktionen für XYZ-Euler-Winkel

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-05
% (c) Institut für Regelungstechnik, Universität Hannover

%% Init
clc
clear

% zufällige Rotationsmatrizen
n = 10000;
rpy = (0.5-rand(n,3))*pi;
R_ges = NaN(3,3,n);

for i = 1:n
  R_ges(:,:,i) = rotx(rpy(i,1))*roty(rpy(i,2))*rotz(rpy(i,3));
end

%% Teste r2eulxyz
for i = 1:n
  R_i = R_ges(:,:,i);
  rpy_i = r2eulxyz(R_i);
  R_i_test = eulxyz2r(rpy_i);
  
  if any( abs( R_i(:)-R_i_test(:) ) > 1e-10 )
    error('Umrechnung r2eulxyz stimmt nicht');
  end
end
fprintf('%d Umrechnungen mit r2eulxyz getestet\n', n);

%% Teste r2eulxyz
for i = 1:n
  R_i = R_ges(:,:,i);
  rpy_i = r2eulxyz(R_i);
  R_i_test = eulxyz2r(rpy_i);
  
  if any( abs( R_i(:)-R_i_test(:) ) > 1e-10 )
    error('Umrechnung r2eulxyz stimmt nicht');
  end
end
fprintf('%d Umrechnungen mit r2eulxyz getestet\n', n);