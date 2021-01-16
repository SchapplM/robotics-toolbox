% Ableitung der Translationskomponente der kinematischen ZB nach den Gelenkwinkeln
% 
% Variante 2:
% * Andere Implementierung mit Vektor 0-P statt A-B
% * Translationsfehler ist Differenz zwischen Vektor berechnet aus x und q
% 
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% platform_frame [1x1 logical]
%   Benutze das Plattform-KS anstatt das EE-KS als Bezugsgröße für x
% 
% Ausgabe:
% Phi_q_legs_red
%   Ableitung der kinematischen Zwangsbedingungen nach allen Gelenkwinkeln
%   Translatorischer Teil
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% Phi_q_legs [3xN]
%   Siehe vorher. Hier alle Zeilen der Zwangsbedingungen
% 
% Siehe auch: SerRob/constr1grad_tq.m

% Quellen:
% [2_SchapplerTapOrt2019a] Schappler, M. et al.: Modeling Parallel Robot
% Kinematics for 3T2R and 3T3R Tasks using Reciprocal Sets of Euler Angles
% (Arbeitstitel), Submitted to MDPI Robotics KaRD2, Version of 27.06.2019
% [A] Aufzeichnungen Schappler vom 15.06.2018
% [B] Aufzeichnungen Schappler vom 22.06.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phi_q_legs_red, Phi_q_legs] = constr2grad_tq(Rob, q, platform_frame)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr2grad_tq: q muss %dx1 sein', Rob.NJ);
if nargin == 2, platform_frame = false; end
NLEG = Rob.NLEG;
NJ = Rob.NJ;

if ~platform_frame
  r_P_E = Rob.r_P_E;
else
  r_P_E = zeros(3,1);
end
%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
if ~Rob.issym
  Phi_q_legs = NaN(3*NLEG,NJ);
  Phi_q_legs_red = NaN(sum(Rob.I_EE(1:3))*NLEG,NJ);
else
  Phi_q_legs = sym('phi', [3*NLEG,NJ]);
  Phi_q_legs(:)=0;
  Phi_q_legs_red = sym('phi', [sum(Rob.I_EE(1:3))*NLEG,NJ]);
  Phi_q_legs_red(:)=0;
end

%% Berechnung
% Berechnung aus dem translatorischen Teil der Jacobi-Matrix der seriellen
% Beinketten 
for i = 1:NLEG
  IJ_i = Rob.I1J_LEG(i):Rob.I2J_LEG(i);
  qs = q(IJ_i); % Gelenkwinkel dieser Kette
  
  phi_0_Ai = Rob.Leg(i).phi_W_0;
  R_0_0i = eul2r(phi_0_Ai, Rob.Leg(i).phiconv_W_0);
  
  % Jacobi-Matrix der Beinkette, bezogen auf den Koppelpunkt
  J0i_i_trans = Rob.Leg(i).jacobit(qs);
  J0_i_trans = R_0_0i*J0i_i_trans; % Bezug auf das Basis-KS der PKM
  J_Ai_Bi = J0_i_trans; % Nur xyz-Koordinate in ZB.
  % Berücksichtigung des zusätzlichen "Hebelarms" vom Koppelpunkt zum EE
  J0i_i_rot = Rob.Leg(i).jacobiw(qs);
  J0_i_rot = R_0_0i*J0i_i_rot;
 
  T_0i_Bi = Rob.Leg(i).fkineEE(qs);
  R_0i_Bi = T_0i_Bi(1:3,1:3);
  R_Bi_P = eulxyz2r(Rob.phi_P_B_all(:,i)).';
  R_0_Bi = R_0_0i * R_0i_Bi; 
  R_0_P = R_0_Bi * R_Bi_P;
  
  r_P_P_Bi = Rob.r_P_B_all(:,i);
  r_P_Bi_P = -  r_P_P_Bi;
  r_B_E = R_0_P * (r_P_Bi_P + r_P_E);  
  % Umrechnung der vorher auf Koppelpunkt bezogenen Jacobi auf den Endeffektor
  % Siehe dazu adjoint_jacobian.m
  J_0_E = J_Ai_Bi + -skew(r_B_E) * J0_i_rot;
  
  if ~Rob.issym
    dPhidqJi = zeros(3*NLEG,Rob.Leg(i).NQJ);
    dPhidqJi_red = zeros(sum(Rob.I_EE(1:3))*NLEG,Rob.Leg(i).NQJ);
  else
    dPhidqJi = sym('xx', [3*NLEG,Rob.Leg(i).NQJ]);
    dPhidqJi(:)=0;
    dPhidqJi_red = sym('xx', [sum(Rob.I_EE(1:3))*NLEG,Rob.Leg(i).NQJ]);
    dPhidqJi_red(:)=0;
  end
  
  % [2_SchapplerTapOrt2019a]/(A15); Gl. (A.25-26, B.23)
  % Kein negatives Vorzeichen, siehe Definition der Zwangsbedingungen
  dPhidqJi(3*(i-1)+1:3*(i),:) = J_0_E;
  Phi_q_legs(:,IJ_i) = dPhidqJi;
  
  % Eintragen in Ergebnis-Variable
  I1 = sum(Rob.I_EE(1:3))*(i-1)+1;
  I2 = I1+sum(Rob.I_EE(1:3))-1;
  dPhidqJi_red(I1:I2,:) = J_0_E(Rob.I_EE(1:3),:);
  Phi_q_legs_red(:,IJ_i) = dPhidqJi_red;
end
