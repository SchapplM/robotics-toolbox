% Steifigkeitsmatrix einer PKM bezogen auf die Endeffektor-Plattform
% 
% Eingabe:
% q 
%   Gelenkwinkel
%
% Ausgabe:
% Kx [6x6]
%   Kartesische Steifigkeitsmatrix der PKM
%   Komponenten: [N/m (3x3), Nm/m (3x3); N/rad (3x3), Nm/rad (3x3)]
%   Summe aus KxJ und KxJ
% KxJ [6x6]
%   Kartesische Steifigkeitsmatrix gebildet aus den Steifigkeiten der
%   Gelenke der einzelnen Beinketten (bei Annahme idealer Segmente)
% KxJ [6x6]
%   Kartesische Steifigkeitsmatrix gebildet aus den Steifigkeiten der
%   Segmente der einzelnen Beinketten (bei Annahme idealer Gelenke)
%
% Quellen:
% [Zhao2020] Modellierung und Maßsynthese serieller und paralleler Roboter
% hinsichtlich der strukturellen Steifigkeit (Masterarbeit)
% [Klimchik2011] Enhanced stiffness modeling of serial and parallel
% manipulators for robotic-based processing of high performance materials

% Masterarbeit Yuqi ZHAO, zhaoyuqi.nicolas@gmail.com, 2019-12
% Betreuer: Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Kx, KxJ, KxL] = stiffness(R, q)

Kx = zeros(6,6);
KxJ = zeros(6,6);
KxL = zeros(6,6);

for i = 1:R.NLEG
  q_i = q(R.I1J_LEG(i):R.I2J_LEG(i));
  % Steifigkeit serieller Kette berechnen (bezogen von Basis der
  % Beinkette zum Endpunkt der Beinkette)
  [~, N_LB_LE, N_LN_LE_J, N_LN_LE_L] = R.Leg(i).stiffness(q_i);
  % Rotation von Bein Basis zum Plattform Basis [Klimchik2011 P70]
  R_B0_W = [R.Leg(i).T_W_0(1:3,1:3),zeros(3,3);zeros(3,3),R.Leg(i).T_W_0(1:3,1:3)];
  % Jacobian Matrix vom jedem Bein zum Endeffektor berechnen [Klimchik2011 P71]
  v = -R.r_P_B_all(:,i); % TODO: Hier müsste der EE genommen werden
  VX = skew(v); %Vector mit Kreuzprodukt vom Leg_EE zum Plat_EE
  Jv = [eye(3), zeros(3); VX, eye(3)];
  % Transformation der Steifigkeitsmatrix
  SF_PB_PE = Jv * R_B0_W / N_LB_LE * R_B0_W' * Jv';
  % Steifigkeit der parallelen Struktur ist Summe der einelnen
  % Steifigkeiten
  Kx = Kx + SF_PB_PE;
  
  % Rechnung für Matrizen bezogen auf Segmente oder Gelenke nachholen,
  % falls Ausgabe gefragt. TODO: Gibt noch Probleme, falls
  % Segmentnachgiebigkeit Null ist.
  if nargout > 1
    KxJ = KxJ + Jv*R_B0_W/(N_LN_LE_J)*R_B0_W'*Jv';
  end
  if nargout > 2
    KxL = KxL + Jv*R_B0_W/(N_LN_LE_L)*R_B0_W'*Jv';
  end
end
