% Direkte Kinematik mit allgemeiner MDH-Notation für Gelenktransformation
% 
% Eingabe:
% q: Gelenkwinkel
% beta_mdh, ...: MDH-Parameter
% 
% Ausgabe:
% T: Transformationsmatrizen
% Tc_0: Kumulierte Transformationsmatrizen von der Basis zu den Körper-KS

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2026-03
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [T_mdh, Tc_0] = robot_fkine_mdh(q, beta_mdh, b_mdh, alpha_mdh, a_mdh, theta_mdh, d_mdh, qoffset_mdh, sigma_mdh, v_mdh)

NJ = length(beta_mdh);

% Einzelne MDH-Transformationen berechnen
T_mdh = NaN(4,4,NJ); % Alle Gelenk-Transformationsmatrizen
for i = 1:NJ
  if sigma_mdh(i) == 0 % Rotationsgelenk
    d_i = d_mdh(i);
    theta_i = q(i)+qoffset_mdh(i);
  else % Schubgelenk
    d_i = q(i)+qoffset_mdh(i);
    theta_i = theta_mdh(i);
  end
  T_mdh(:,:,i) = trotz(beta_mdh(i)) * transl([0;0;b_mdh(i)]) *... 
                 trotx(alpha_mdh(i)) * transl([a_mdh(i);0;0]) *...
                 trotz(theta_i) * transl([0;0;d_i]);
end

if nargout == 1
  return
end
% Transformation von Basis zum jeweiligen MDH-KS
Tc_0 = NaN(4,4,NJ);
 % Basis-Segment auf Einheitsmatrix gesetzt (Konvention, für Kompatibilität
 % mit transformierten Matrizen)
Tc_0(:,:,1) = eye(4);
for i = 1:NJ % Gelenk-Transformation anwenden
  i_pre = v_mdh(i); % Index zum Vorgänger-Koordinatensystem (Baumstruktur)
  Tc_0(:,:,i+1) = Tc_0(:,:,i_pre+1) * T_mdh(:,:,i);
end