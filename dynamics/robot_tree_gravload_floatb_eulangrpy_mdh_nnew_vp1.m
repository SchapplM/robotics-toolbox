% Calculate vector of gravitational base and joint forces/torques for a
% general floating base robot
% Use numerical implementation of recursive Newton-Euler Algorithm
% 
% Input:
% q [NJx1]
%   Joint Angles [rad]
% phi_base [3x1]
%   Base orientation in world frame. Expressed with RPY Euler angles (xyz)
% alpha_mdh, a_mdh, d_mdh, theta_mdh, q_offset_mdh, b_mdh, beta_mdh, v_mdh, sigma [NJx1]
%   kinematic parameters according to [2]
%   sigma: type of joints (0=rotational, 1=prismatic)
% m_num_mdh [(NJ+1)x1], rSges_num_mdh [(NB+1)x3]
%   dynamic parameters (parameter set 1: center of mass in link frame)
% 
% Output:
% tau_g [(6+NJ)x1]
%   base forces and joint torques required to compensate gravitational forces. 
%   Base moments expressed in generalized coordinates (Euler-RPY)
% 
% Sources:
% [1] Featherstone: Rigid Body Dynamics Algorithms, S.98
% [2] Khalil, W. and Kleinfinger, J.-F.: A new geometric notation for open and closed-loop robots (1986)
% 
% Siehe robot_tree_invdyn_floatb_eulangrpy_nnew_vp1.m

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-06
% (c) Institut für Regelungstechnik, Universität Hannover

function tau_g = robot_tree_gravload_floatb_eulangrpy_mdh_nnew_vp1(q, phi_base, g_world, ...
  alpha_mdh, a_mdh, d_mdh, theta_mdh, q_offset_mdh, b_mdh, beta_mdh, v_mdh, sigma, m_num, rSges_num_mdh)


%% Init
nq = length(q);
nb = nq+1;
tau_J = NaN(nq,1);
R_W_0 = eulxyz2r(phi_base);

%% Vorwärts-Iteration
% Positionen
T_mdh = NaN(4,4,nq); % Alle Gelenk-Transformationsmatrizen
for i = 1:nq
  if sigma(i) == 0 % Rotationsgelenk
    d_i = d_mdh(i);
    theta_i = q(i)+q_offset_mdh(i);
  else % Schubgelenk
    d_i = q(i)+q_offset_mdh(i);
    theta_i = theta_mdh(i);
  end
  T_mdh(:,:,i) = trotz(beta_mdh(i)) * ... 
                     transl([0;0;b_mdh(i)]) * trotx(alpha_mdh(i)) * ...
                     transl([a_mdh(i);0;0]) * trotz(theta_i) ...
                     * transl([0;0;d_i]);
end

vD_i_i_ges = NaN(3,nb);

% Anfangswerte: Geschwindigkeit und Beschleunigung der Basis

vD_i_i_ges(:,1) = -R_W_0'*g_world;

for i = 2:nb
  % Nummer des Vorgänger-Segments
  j = v_mdh(i-1)+1; % Gelenk 1 führt zu Körper 2 (Matlab-Indizes) usw.
  
  % Temporäre Ausdrücke belegen
  vD_j_j = vD_i_i_ges(:,j);
  R_j_i = T_mdh(1:3,1:3,i-1);
  
  % Berechnung
  vD_i_i = R_j_i'*( vD_j_j );
  
  % Ausgabeausdrücke belegen
  vD_i_i_ges(:,i) = vD_i_i;
end


%% Rückwärts-Rekursion
f_i_i_ges = NaN(3,nb);
n_i_i_ges = NaN(3,nb);

for i = nb:-1:1
  % Temporäre Ausdrücke belegen
  vD_i_i = vD_i_i_ges(:,i);
  c_i = rSges_num_mdh(i,:)';
  
  % Dynamik-Terme
  F_i = m_num(i)*vD_i_i;
  
  f_i_i = F_i;
  n_i_i = cross(c_i, F_i);
  
  % Suche alle Nachfolger und addiere das Schnittmoment
  I_nf = find( (v_mdh == (i-1)) )' + 1;
  for j = I_nf % Index des Nachfolgers
    R_i_j = T_mdh(1:3,1:3,j-1);
    f_j_j = f_i_i_ges(:,j);
    n_j_j = n_i_i_ges(:,j);
    r_i_i_j = T_mdh(1:3,4,j-1);
     
    f_i_i = f_i_i + R_i_j*f_j_j;
    n_i_i = n_i_i + R_i_j*n_j_j + cross(r_i_i_j, R_i_j*f_j_j);
  end
  
  % Ausgabeausdrücke belegen
  f_i_i_ges(:,i) = f_i_i;
  n_i_i_ges(:,i) = n_i_i;
end

%% Projektion auf die Gelenke
for i = 2:nb
  if sigma(i-1) == 0 % Drehgelenk
    tau_J(i-1) = [0 0 1] * n_i_i_ges(:,i);
  else % Schubgelenk
    tau_J(i-1) = [0 0 1] * f_i_i_ges(:,i);
  end
end

%% Basis-Kraft
T_basevel = angvelotrans_rpy(phi_base);
tau_B = [R_W_0*f_i_i_ges(:,1); T_basevel' * R_W_0*n_i_i_ges(:,1)]; 

%% Ausgabe
tau_g = [tau_B; tau_J];
