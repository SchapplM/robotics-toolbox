% Ableitung der Translationskomponente der kinematischen ZB nach den
% Gelenkwinkeln und Ableitung dieser (Gradienten-)Matrix nach der Zeit
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
%   Geschwindigkeit aller Gelenkwinkel aller serieller Beinketten der PKM
% 
% Ausgabe:
% PhiD_q_legs_red
%   Ableitung der kinematischen Zwangsbedingungen nach allen Gelenkwinkeln
%   und der Zeit; translatorischer Teil
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% PhiD_q_legs [3xN]
%   Siehe vorher. Hier alle Zeilen der Zwangsbedingungen
% 
% Siehe auch: SerRob/constr1grad_tq.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [PhiD_q_legs_red, PhiD_q_legs] = constr1gradD_tq(Rob, q, qD)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr1gradD_tq: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/constr1gradD_tq: qD muss %dx1 sein', Rob.NJ);

NLEG = Rob.NLEG;
NJ = Rob.NJ;

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
if ~Rob.issym
  PhiD_q_legs = NaN(3*NLEG,NJ);
  PhiD_q_legs_red = NaN(length(Rob.I_constr_t_red),NJ);
else
  PhiD_q_legs = sym('phi', [3*NLEG,NJ]);
  PhiD_q_legs(:)=0;
  PhiD_q_legs_red = sym('phi', [length(Rob.I_constr_t_red),NJ]);
  PhiD_q_legs_red(:)=0;
end

%% Berechnung
% Berechnung aus dem translatorischen Teil der Jacobi-Matrix der seriellen
% Beinketten. Davon muss lediglich die Zeitableitung gebildet werden.
for i = 1:NLEG
  IJ_i = Rob.I1J_LEG(i):Rob.I2J_LEG(i);
  q_i = q(IJ_i); % Gelenkwinkel dieser Kette
  qD_i = qD(IJ_i); % Gelenkgeschw. dieser Kette
  
  phi_0_Ai = Rob.Leg(i).phi_W_0;
  R_0_0i = eul2r(phi_0_Ai, Rob.Leg(i).phiconv_W_0);
  
  JD0i_i_trans = Rob.Leg(i).jacobitD(q_i, qD_i); % Zeitableitung der Jacobi
  JD0_i_trans = R_0_0i*JD0i_i_trans; % Bezug auf das Basis-KS der PKM
  JD_Ai_Bi = JD0_i_trans; % Nur xyz-Koordinate in ZB.
  if ~Rob.issym
    dPhidqJi = zeros(3*NLEG,Rob.Leg(i).NQJ);
    dPhidqJi_red = zeros(sum(Rob.I_EE(1:3))*NLEG,Rob.Leg(i).NQJ);
  else
    dPhidqJi = sym('xx', [3*NLEG,Rob.Leg(i).NQJ]);
    dPhidqJi(:)=0;
    dPhidqJi_red = sym('xx', [sum(Rob.I_EE(1:3))*NLEG,Rob.Leg(i).NQJ]);
    dPhidqJi_red(:)=0;
  end
  
  % Kein negatives Vorzeichen, siehe Definition der Zwangsbedingungen
  dPhidqJi(3*(i-1)+1:3*(i),:) = JD_Ai_Bi;
  PhiD_q_legs(:,IJ_i) = dPhidqJi;
  
  % Eintragen in Ergebnis-Variable
  I1 = sum(Rob.I_EE(1:3))*(i-1)+1;
  I2 = I1+sum(Rob.I_EE(1:3))-1;
  if ~isempty(PhiD_q_legs_red)
    dPhidqJi_red(I1:I2,:) = JD_Ai_Bi(Rob.Leg(i).I_EE(1:3),:);
    PhiD_q_legs_red(:,IJ_i) = dPhidqJi_red;
  end
end
