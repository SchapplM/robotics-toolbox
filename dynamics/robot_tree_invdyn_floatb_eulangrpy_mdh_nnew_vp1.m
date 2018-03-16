% Calculate vector of inverse dynamics base and joint forces/torques for a
% general floating base robot
% Use numerical implementation of recursive Newton-Euler Algorithm
% 
% Input:
% q [NJx1]
%   Joint Angles [rad]
% qD [NJx1]
%   Joint Velocities [rad/s]
% qDD [NJx1]
%   Joint Accelerations [rad/s^2]
% phi_base [3x1]
%   Base orientation in world frame. Expressed with RPY Euler angles (xyz)
% xD_base [6x1]
%   time derivative of 
%   r_base (3x1 Base position in world frame) and 
%   phi_base (3x1)
% xDD_base [6x1]
%   second time derivative of r_base (3x1) and phi_base (3x1)
% alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh, v_mdh [NJx1]
%   kinematic parameters according to [2]
% m_num_mdh [(NJ+1)x1], rSges_num_mdh [(NB+1)x3], Icges_num_mdh [(NB+1)x6]
%   dynamic parameters (parameter set 1: center of mass in link frame and 
%   inertia about center of mass. Order: xx, yy, zz, xy, xz, yz)
% 
% Output:
% tau [(6+NJ)x1]
%   base forces and joint torques required to compensate inverse dynamics
%   base moments in generalized coordinates (euler-RPY)
% v_i_i_ges [3xNB]
%   translational velocity of each body frame expressed in the body frame
% w_i_i_ges [3xNB]
%   angular velocity of each body frame expressed in the body frame
% 
% Sources:
% [1] Featherstone: Rigid Body Dynamics Algorithms, S.98
% [2] Khalil, W. and Kleinfinger, J.-F.: A new geometric notation for open and closed-loop robots (1986)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-06
% (c) Institut für Regelungstechnik, Universität Hannover

function [tau, v_i_i_ges, w_i_i_ges] = robot_tree_invdyn_floatb_eulangrpy_mdh_nnew_vp1(q, qD, qDD, phi_base, xD_base, xDD_base, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh, v_mdh, m_num, rSges_num_mdh, Icges_num_mdh)


%% Init
nq = length(q);
nb = nq+1;
tau_J = NaN(nq,1);
R_W_0 = rpy2r(phi_base);

%% Vorwärts-Iteration
% Positionen
T_mdh = NaN(4,4,nq); % Alle Gelenk-Transformationsmatrizen
for i = 1:nq
  T_mdh(:,:,i) = trotz(beta_mdh(i)) * ... 
                     transl([0;0;b_mdh(i)]) * trotx(alpha_mdh(i)) * ...
                     transl([a_mdh(i);0;0]) * trotz(q(i)+q_offset_mdh(i)) ...
                     * transl([0;0;d_mdh(i)]);
end

v_i_i_ges = NaN(3,nb);
w_i_i_ges = NaN(3,nb);
vD_i_i_ges = NaN(3,nb);
wD_i_i_ges = NaN(3,nb);

% Anfangswerte: Geschwindigkeit und Beschleunigung der Basis
v_i_i_ges(:,1) = R_W_0'*xD_base(1:3);
w_i_i_ges(:,1) = R_W_0'*rpyD2omega(phi_base, xD_base(4:6));

vD_i_i_ges(:,1) = R_W_0'*xDD_base(1:3);
wD_i_i_ges(:,1) = R_W_0'*rpyDD2omegaD(phi_base, xD_base(4:6), xDD_base(4:6));

for i = 2:nb
  % Nummer des Vorgänger-Segments
  j = v_mdh(i-1)+1; % Gelenk 1 führt zu Körper 2 (Matlab-Indizes) usw.
  
  % Temporäre Ausdrücke belegen
  v_j_j = v_i_i_ges(:,j);
  w_j_j = w_i_i_ges(:,j);
  vD_j_j = vD_i_i_ges(:,j);
  wD_j_j = wD_i_i_ges(:,j);
  R_j_i = T_mdh(1:3,1:3,i-1);
  r_j_j_i = T_mdh(1:3,4,i-1);
  
  % Berechnung
  w_i_i = R_j_i'*w_j_j + [0;0;1]*qD(i-1);
  v_i_i = R_j_i'*( v_j_j + cross(w_j_j, r_j_j_i) );
  
  wD_i_i = R_j_i'*wD_j_j + [0;0;1]*qDD(i-1) + cross(R_j_i'*w_j_j, [0;0;1]*qD(i-1));
  vD_i_i = R_j_i'*( vD_j_j + cross(wD_j_j, r_j_j_i) +cross(w_j_j, cross(w_j_j, r_j_j_i)) );
  
  % Ausgabeausdrücke belegen
  v_i_i_ges(:,i) = v_i_i;
  w_i_i_ges(:,i) = w_i_i;
  vD_i_i_ges(:,i) = vD_i_i;
  wD_i_i_ges(:,i) = wD_i_i;
end


%% Rückwärts-Rekursion
f_i_i_ges = NaN(3,nb);
n_i_i_ges = NaN(3,nb);

for i = nb:-1:1
  % Temporäre Ausdrücke belegen
  vD_i_i = vD_i_i_ges(:,i);
  w_i_i = w_i_i_ges(:,i);
  wD_i_i = wD_i_i_ges(:,i);
  c_i = rSges_num_mdh(i,:)';
  I_i = inertiavector2matrix(Icges_num_mdh(i,:));
  
  % Dynamik-Terme
  F_i = m_num(i)*(vD_i_i + cross(wD_i_i, c_i) + cross( w_i_i,cross(w_i_i, c_i)) );
  N_i = I_i*wD_i_i + cross(w_i_i, I_i*w_i_i);
  
  f_i_i = F_i;
  n_i_i = N_i + cross(c_i, F_i);
  
  % Suche alle Nachfolger und addiere das Schnittmoment
  I_nf = find( (v_mdh == (i-1)) )' + 1;
  % Wähle diese Konstruktion um Schleifen mit variabler Länge zu vermeiden (Kompilierbarkeit)
  if ~isempty(I_nf)
    for tmp = 1:length(v_mdh) % Index des Nachfolgers
      j = I_nf(tmp);
      R_i_j = T_mdh(1:3,1:3,j-1);
      f_j_j = f_i_i_ges(:,j);
      n_j_j = n_i_i_ges(:,j);
      r_i_i_j = T_mdh(1:3,4,j-1);

      f_i_i = f_i_i + R_i_j*f_j_j;
      n_i_i = n_i_i + R_i_j*n_j_j + cross(r_i_i_j, R_i_j*f_j_j);
      if tmp == length(I_nf)
        break; % Abbruch. Alle Nachfolger untersucht.
      end
    end
  end
  % Ausgabeausdrücke belegen
  f_i_i_ges(:,i) = f_i_i;
  n_i_i_ges(:,i) = n_i_i;
end

%% Projektion auf die Gelenke
for i = 2:nb
  tau_J(i-1) = [0 0 1] * n_i_i_ges(:,i);
end

%% Basis-Kraft
T_basevel = angvelotrans_rpy(phi_base);
tau_B = [R_W_0*f_i_i_ges(:,1); T_basevel' * R_W_0*n_i_i_ges(:,1)]; 

%% Ausgabe
tau = [tau_B; tau_J];
