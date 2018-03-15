% Calculate joint inertia based on Composite Rigid Body Algorithm (CRBA) from [Featherstone2008]
% 
% Input:
% T_stack [NJ*3x4]
%   Stacked Transformation Matrices for all joint transformations (last
%   line with [0 0 0 1] cut off.
% RotAx_i [NJx3
%   Rotation Axes of the joints in local frames of the bodies articulated
%   by the joints (Joints have to be 1DoF revolute)
% v [NJx1]
%   Link Indices of predecessing links for the joints (joint i articulates
%   body i, body 0 is the base)
% m_num_mdh [NLx1], rSges_num_mdh [NLx3], Icges_num_mdh [NLx6]
%   dynamic parameters
% 
% Sources:
% [Featherstone2008] Roy Featherstone: Rigid Body Dynamics Algorithms (2008)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-12
% (C) Institut für Regelungstechnik, Universität Hannover

function H = inertia_nCRB_vp1_m(T_stack, RotAx_i, v, m_num, rSges_num_mdh, Icges_num_mdh)

%% Allgemein
% Vorgänger-Segmente der Gelenke, [Featherstone2008] S. NJ1
% p = v_mdh; % 0 ist die Basis (pelvis)
% Nachfolger-Segmente der Gelenke
% s = (1:NJ)';
% parent array (Vorgänger-Segment für alle Segmente)
% [Featherstone2008] Gl. (4.2)
% Hier ist lambda identisch mit v_mdh
% lambda = min([p,s]')'; %#ok<UDIM>
lambda = v;
NJ = length(v);
%% Massenmatrix zusammensetzen
H = zeros(NJ,NJ);
% compose matrix for all links

% Alle Gelenk-Transformationsmatrizen als 4x4xNJ-Array. Die Verwendung von
% T_stack weiter unten führt zu Kompilierfehlern (Dimensionserkennung) in Simulink.
T_mdh = NaN(4,4,NJ);
for i = 1:NJ
  T_mdh(:,:,i) = [T_stack(3*i-2:3*i,:);[0 0 0 1]];
end

I_c_ges = NaN(NJ,36);
for i = 1:NJ % loop through all bodies, [Featherstone2008] T6.2, Line 2
  % Trägheitsmatrix für diesen Körper
  I_c = inertiavector2matrix(Icges_num_mdh(i+1,:)); % Andere Indexnotation beachten
  % [Featherstone2008] Gl. (2.63)
  I_O = [I_c+m_num(i+1)*skew(rSges_num_mdh(i+1,:))*skew(rSges_num_mdh(i+1,:))', m_num(i+1)*skew(rSges_num_mdh(i+1,:)); ...
         m_num(i+1)*skew(rSges_num_mdh(i+1,:))',                                m_num(i+1)*eye(3)];

  % Komposit-Matrix initialisieren
  I_c_ges(i,:) = I_O(:);
end

% loop through all bodies, [Featherstone2008] T6.2, Line 4
% Basis zählt nicht als Körper
for i = NJ:-1:1 % Wird die Schleife mit uint8 definiert, funktioniert der generierte mex-Code nicht (Matlab-Bug)
  % Alt. 1. Berechnung mit kumulativen Transformationsmatrizen (intuitiver):
  % R_0_i = T_c_mdh(1:3,1:3,i+1); % Andere Indexnotation beachten. Matlab: 1=Basis, Algorithmus: 0=Basis
  % % Transformation zum Vorgänger-Segment
  % R_0_li = T_c_mdh(1:3,1:3,lambda(i)+1);
  % R_i_li = R_0_i' * R_0_li;
  % r_i_i_li = R_0_i' * r_0_i_li;
  % r_0_i_li = -T_c_mdh(1:3,4,i+1) + T_c_mdh(1:3,4,lambda(i)+1);
  
  % Alt. 2. Berechnung mit direkten Transformationsmatrizen (schneller):
  % Transformation von diesem zum Vorgänger
  R_i_li = T_mdh(1:3,1:3,i)';
  r_li_li_i = T_mdh(1:3,4,i);
  % Vektor von diesem zum Vorgänger
  r_i_i_li = -R_i_li*r_li_li_i;

  % [Featherstone2008], S. 22
  %   A = i
  %   B = li
  %   r = r_i_i_li
  %   E = R_B_A = R_li_i = R_i_li'
  %  Gl. (2.26)
  X_i_li = [R_i_li, zeros(3,3);skew(r_i_i_li)*R_i_li , R_i_li];
  
  I_i_c = reshape(I_c_ges(i,:), 6, 6);

  % Komposit-Matrix aufbauen
  if lambda(i) > 0
    I_li_c = reshape(I_c_ges(lambda(i),:), 6, 6);

    % [Featherstone2008] T6.2, Line NJ (s steht für Stern)
    % [Featherstone2008] Gl. (2.25) (S. 22, Formelzeichen A,B,r,E s.o.)
    X_li_i_s = [R_i_li(1:3,1:3)', -R_i_li(1:3,1:3)'*skew(r_i_i_li); zeros(3,3), R_i_li(1:3,1:3)']; 
    % Transformiere die Trägheit des aktuellen Körpers i in die
    % Koordinaten des Vorgängers und füge zu dessen Komposit-Trägheit
    % hinzu
    I_li_c_new = I_li_c + X_li_i_s*I_i_c*X_i_li;
    I_c_ges(lambda(i),:) = I_li_c_new(:);
  end

  % Gelenk-Transformationsmatrix
  % Siehe [Featherstone2008] Gl. (3.33), Example 3.2
  S_i = [RotAx_i(i,:)'; 0; 0; 0]; % Allgemeine Drehachse angenommen

  % [Featherstone2008] T6.2, Line 9
  F = I_i_c*S_i;
  
  % [Featherstone2008] T6.2, Line 10
  H(i,i) = S_i' * F; % Diagonalelement der Massenmatrix

  j = uint8(i); % [Featherstone2008] T6.2, Line 11
  for tmp = 1:NJ % Dummy-Schleife für Kompilierbarkeit. Abbruchkriterium s.u.
    if lambda(j) == 0 % [Featherstone2008] T6.2, Line 12, Abbruchbedingung
      break;
    end
    % Alt. 1. Berechnung mit kumulativen Transformationsmatrizen (intuitiver):
    % R_0_j = T_c_mdh(1:3,1:3,j+1);
    % R_0_lj = T_c_mdh(1:3,1:3,lambda(j)+1);
    % R_j_lj = R_0_j' * R_0_lj;    
    % r_0_j_lj = -T_c_mdh(1:3,4,j+1) + T_c_mdh(1:3,4,lambda(j)+1);
    % r_j_j_lj_test = R_0_j' * r_0_j_lj;
    
    % Alt. 2. Berechnung mit direkten Transformationsmatrizen (schneller):
    R_j_lj = T_mdh(1:3,1:3,j)';
    r_lj_lj_j = T_mdh(1:3,4,j);
    r_j_j_lj = -R_j_lj*r_lj_lj_j;
    
    % [Featherstone2008], S. 22
    %   A = j
    %   B = lj
    %   r = r_A_A_B = r_j_j_lj
    %   E = R_B_A = R_lj_j = R_j_lj'
    %  Gl. (2.25): X_B_A_s = X_lj_j_s       
    X_lj_j_s = [R_j_lj(1:3,1:3)', -R_j_lj(1:3,1:3)'*skew(r_j_j_lj); zeros(3,3), R_j_lj(1:3,1:3)'];

    % [Featherstone2008] T6.2, Line 13
    F = X_lj_j_s*F; % F war vorher in Koordinaten j, hiernach in lambda(j) (des Vorgängers)
    % [Featherstone2008] T6.2, Line 14
    j = (lambda(j)); % Kette weiter nach hinten durchgehen (bis zur Basis)

    S_j = [RotAx_i(j,:)'; 0; 0; 0];% Siehe [Featherstone2008] Gl. (3.33), Example 3.2
    % [Featherstone2008] T6.2, Line 15
    H(i,j) = F' * S_j;
    % [Featherstone2008] T6.2, Line 16
    H(j,i) = H(i,j); % Symmetrie ausnutzen
  end
end

