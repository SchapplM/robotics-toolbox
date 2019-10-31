% Ableitung der Translationskomponente der kinematischen ZB nach den Gelenkwinkeln
% 
% Variante 1:
% * Translation ausgedrückt als Vektor vom Basis-Koppelpunkt A zum
%   Plattform-Koppelpunkt B
% * Translationsfehler ist Differenz zwischen Vektor berechnet aus x und q
% 
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% qD [Nx1]
%  Velocity of the  Gelenkwinkel aller serieller Beinketten der PKM
% Ausgabe:
% Phi_q_legs_red
%   Ableitung der kinematischen Zwangsbedingungen nach allen Gelenkwinkeln
%   Translatorischer Teil
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% Phi_q_legs [3xN]
%   Siehe vorher. Hier alle Zeilen der Zwangsbedingungen
% 
% Siehe auch: SerRob/constr1grad_tq.m

% Quelle:
% [A] Aufzeichnungen Schappler vom 15.06.2018
% [B] Aufzeichnungen Schappler vom 22.06.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phi_q_legs_red, Phi_q_legs] = constr1gradD_tq(Rob, q ,qD)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr1gradD_tq: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/constr1gradD_tq: qD muss %dx1 sein', Rob.NJ);
% assert(all(size(rpy) == [3 1]) && isreal(rpy), ...
%   'ParRob/constr1gradD_tq: rpy angles have to be [3x1] (double)'); 
% assert(all(size(rpyD) == [3 1]) && isreal(rpyD), ...
%   'ParRob/constr1gradD_tq: rpy angles time derivatives have to be [3x1] (double)'); 
NLEG = Rob.NLEG;
NJ = Rob.NJ;

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
if ~Rob.issym
  Phi_q_legs = NaN(3*NLEG,NJ);
  Phi_q_legs_red = NaN(length(Rob.I_constr_t_red),NJ);
else
  Phi_q_legs = sym('phi', [3*NLEG,NJ]);
  Phi_q_legs(:)=0;
  Phi_q_legs_red = sym('phi', [length(Rob.I_constr_t_red),NJ]);
  Phi_q_legs_red(:)=0;
end

%% Berechnung
% Berechnung aus dem translatorischen Teil der Jacobi-Matrix der seriellen
% Beinketten 
% here only the jacobi matrix is differentiated, and the answer of this
% function can be obtained
for i = 1:NLEG
  IJ_i = Rob.I1J_LEG(i):Rob.I2J_LEG(i);
  qs = q(IJ_i); % Gelenkwinkel dieser Kette
  qv = qD(IJ_i);
  
  phi_0_Ai = Rob.Leg(i).phi_W_0;  % phi_W_0 is a empty matrix
  R_0_0i = eul2r(phi_0_Ai, Rob.Leg(i).phiconv_W_0);
  
  J0i_i_trans = Rob.Leg(i).jacobitD(qs,qv);% differntiation of the jacobi matrix
  J0_i_trans = R_0_0i*J0i_i_trans; % Bezug auf das Basis-KS der PKM
  J_Ai_Bi = J0_i_trans; % Nur xyz-Koordinate in ZB.
  if ~Rob.issym
    dPhidqJi = zeros(3*NLEG,Rob.Leg(i).NQJ);
    dPhidqJi_red = zeros(sum(Rob.I_EE(1:3))*NLEG,Rob.Leg(i).NQJ);
  else
    dPhidqJi = sym('xx', [3*NLEG,Rob.Leg(i).NQJ]);
    dPhidqJi(:)=0;
    dPhidqJi_red = sym('xx', [sum(Rob.I_EE(1:3))*NLEG,Rob.Leg(i).NQJ]);
    dPhidqJi_red(:)=0;
  end
  
  % Gl. (A.25-26, B.23)
  % Kein negatives Vorzeichen, siehe Definition der Zwangsbedingungen
  dPhidqJi(3*(i-1)+1:3*(i),:) = J_Ai_Bi;
  Phi_q_legs(:,IJ_i) = dPhidqJi;
  
  % Eintragen in Ergebnis-Variable
  I1 = sum(Rob.I_EE(1:3))*(i-1)+1;
  I2 = I1+sum(Rob.I_EE(1:3))-1;
  if ~isempty(Phi_q_legs_red)
    dPhidqJi_red(I1:I2,:) = J_Ai_Bi(Rob.Leg(i).I_EE(1:3),:);
    Phi_q_legs_red(:,IJ_i) = dPhidqJi_red;
  end
end
