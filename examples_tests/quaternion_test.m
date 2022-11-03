% Teste unterschiedliche Transformationsfunktionen für Quaternionen

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-08
% (c) Institut für Regelungstechnik, Universität Hannover

%% Init
clc
clear

% zufällige Rotationsmatrizen
n = 10000;
abc = (0.5-rand(n,3))*pi;
R_ges = NaN(3,3,n);

for i = 1:n
  R_ges(:,:,i) = rotx(abc(i,1))*roty(abc(i,2))*rotz(abc(i,3));
end

%% Teste r2quat_klumpp
for i = 1:n
  R_i = R_ges(:,:,i);
  q_i = r2quat_klumpp(R_i);
  R_i_test = quat2r(q_i);
  
  if any( abs( R_i(:)-R_i_test(:) ) > 1e-10 )
    error('Umrechnung r2quat_klumpp stimmt nicht');
  end
end
fprintf('%d Umrechnungen mit r2quat_klumpp getestet\n', n);

%% Teste r2quat
for i = 1:n
  R_i = R_ges(:,:,i);
  q_i = r2quat(R_i);
  R_i_test = quat2r(q_i);
  
  if any( abs( R_i(:)-R_i_test(:) ) > 1e-10 )
    error('Umrechnung r2quat stimmt nicht');
  end
end
fprintf('%d Umrechnungen mit r2quat getestet\n', n);

%% Teste invquat
for j = 1:n
  R_j = R_ges(:,:,j);
  R_j_inv = R_j';
  
  q_j = r2quat(R_j);
  q_j_inv = invquat(q_j);
  R_j_inv_test = quat2r(q_j_inv);
  
  test = R_j_inv - R_j_inv_test;
  
  if any( abs( test(:)) > 1e-10 )
    error('Umrechnung invquat stimmt nicht');
  end
end
fprintf('%d Umrechnungen mit invquat getestet\n', n);

%% Teste quatprod
for j = 2:n
  R1 = R_ges(:,:,j-1);
  R2 = R_ges(:,:,j);
  R3 = R1*R2;
  
  q1 = r2quat(R1);
  q2 = r2quat(R2);
  q3 = quatprod(q1,q2);
  
  R3_test = quat2r(q3);
  
  test = R3_test - R3;
  if any( abs( test(:) ) > 1e-10 )
    error('Umrechnung quatprod stimmt nicht');
  end
end
fprintf('%d Umrechnungen mit quatprod getestet\n', n);

%% Teste Quaternion-Klasse aus Corke-Robotic-Toolbox
for j = 2:n
  R1 = R_ges(:,:,j-1);
  R2 = R_ges(:,:,j);
  R3 = R1*R2;
  q1 = r2quat(R1);
  q2 = r2quat(R2);
  Q1 = Quaternion(q1(1), q1(2:end));
  Q2 = Quaternion(q2(1), q2(2:end));
  q3 = quatprod(q1, q2);
  Q3 = Q1 * Q2;
  R3_test1 = quat2r(q3);
  R3_test2 = quat2r([Q3.s; Q3.v(:)]);
  test1 = R3_test1 - R3_test2;
  if any( abs( test1(:) ) > 1e-10 )
    error('Umrechnung quat2r/r2quat vs Quaternion-Klasse stimmt nicht');
  end
  test2 = R3 - R3_test2;
  if any( abs( test(:) ) > 1e-10 )
    error('Umrechnung quat2r/r2quat+Quaternion-Klasse stimmt nicht gegen Rotationsmatrix');
  end
end
fprintf('%d Umrechnungen mit Quaternion-Klasse getestet\n', n);