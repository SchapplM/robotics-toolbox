%% Joint torque of Hunt Crossley joint limit model
% Berechne das Gelenkmoment durch Erreichen der Endanschläge der Gelenke
% 
% Input:
% q [Nx1]
%   Joint positions [rad]
% qD [Nx1]
%   Joint velocities [rad/s]
% HC_params [Nx3]
%   Hunt Crossley Parameters (R,h,alpha)
% q_min [Nx1]
%   Lower joint position limits [rad]
% q_max [Nx1]
%   Upper joint position limits [rad]
% 
% Output:
% tau_s [Nx1]
%   Joint Limit spring torques
% tau_d [Nx1]
%   Joint Limit damping torques
% 
% Source:
% [AzadFeatherstone2010] Morteza Azad and Roy Featherstone: Modeling the
% contact between a rolling sphere and a compliant ground plane (2010)
% [SunBai14] Y. Sun Z.F. Bai, J. Chen. Effects of contact force model on dynamics charac-
% teristics of mechanical system with revolute clearance joints. Transactions of
% Mechanical Engineering, 38(M2):375–388, 2014.

% Niclas Miller, miller@irt.uni-hannover.de
% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-06
% (c) Institut fuer Regelungstechnik, Universitaet Hannover

function [tau_s,tau_d,E_springs]=joint_limit_torques_Hunt_Crossley(q, qD, HC_params, q_min, q_max)

N = length(q);

tau_s=zeros(N,1);
tau_d=zeros(N,1);
E_spring=zeros(N,1);

% [AzadFeatherstone2010] Gl. (10), (12). Kraft aus dem Kontaktmodell ist
% Null, falls kein Kontakt
for i = 1:N
  % Für jedes Gelenk unterschiedliche Kontaktparameter
  R=HC_params(i,1);
  h=HC_params(i,2);
  alpha=HC_params(i,3);
  
  % Prüfe Überschreitung der Winkel-Endanschläge und berechne Kontaktkraft
  if q(i)>q_max(i)
      [tau_s(i),tau_d(i),E_spring(i)]=Hunt_Crossley_contact_model_func(q_max(i)-q(i),qD(i),R,h,alpha);
  elseif q(i)<q_min(i)
      [tau_s(i),tau_d(i),E_spring(i)]=Hunt_Crossley_contact_model_func(q_min(i)-q(i),qD(i),R,h,alpha);
  end
end
E_springs = sum(E_spring);
return;




    
    
