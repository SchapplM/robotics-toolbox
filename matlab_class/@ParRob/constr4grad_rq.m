% Ableitung der Rotationskomponente der kinematischen ZB nach den Gelenkwinkeln
% 
% Variante 4:
% * Bezogen auf Winkelgeschwindigkeit des Koppelpunktes Bi
%   (effektiv werden die Geschw.-ZB nach den Gelenk-Geschw. abgeleitet)
% 
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% 
% Ausgabe:
% Phi_q_red
%   Ableitung der kinematischen Zwangsbedingungen nach allen Gelenkwinkeln
%   Translatorischer Teil
%   Reduzierte Zeilen: Die Reduktion folgt aus der Klassenvariablen I_EE
% Phi_q [3xN]
%   Siehe vorher. Hier alle Zeilen der Zwangsbedingungen
% 
% Siehe auch: SerRob/constr4grad_rr.m

% Quellen:
% [A] Aufzeichnungen Schappler vom 13.02.2020

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Phi_q_red, Phi_q] = constr4grad_rq(Rob, q)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr4grad_rq: q muss %dx1 sein', Rob.NJ);
NLEG = Rob.NLEG;
NJ = Rob.NJ;

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
if ~Rob.issym
  Phi_q = zeros(3*NLEG,NJ);
  Phi_q_red = zeros(length(Rob.I_constr_r_red),NJ);
else
  Phi_q = sym('phi', [3*NLEG,NJ]);
  Phi_q(:)=0;
  Phi_q_red = sym('phi', [length(Rob.I_constr_r_red),NJ]);
  Phi_q_red(:)=0;
end


%% Berechnung
% Berechnung aus dem translatorischen Teil der Jacobi-Matrix der seriellen
% Beinketten 
for i = 1:NLEG
  IJ_i = Rob.I1J_LEG(i):Rob.I2J_LEG(i);
  qs = q(IJ_i); % Gelenkwinkel dieser Kette
  
  phi_0_Ai = Rob.Leg(i).phi_W_0;
  R_0_0i = eul2r(phi_0_Ai, Rob.Leg(i).phiconv_W_0);
  % Geometrische Jacobi-Matrix (Rotations-Teil)
  J0i_i_rotg = Rob.Leg(i).jacobiw(qs);
  J_Ai_Bi = R_0_0i*J0i_i_rotg; % Bezug auf das Basis-KS der PKM

  if ~Rob.issym
    dPhidqJi = zeros(3*NLEG,Rob.Leg(i).NQJ);
    dPhidqJi_red = zeros(sum(Rob.I_EE(4:6))*NLEG,Rob.Leg(i).NQJ);
  else
    dPhidqJi = sym('xx', [3*NLEG,Rob.Leg(i).NQJ]);
    dPhidqJi(:)=0;
    dPhidqJi_red = sym('xx', [sum(Rob.I_EE(4:6))*NLEG,Rob.Leg(i).NQJ]);
    dPhidqJi_red(:)=0;
  end
  
  % Gl. A.10
  dPhidqJi(3*(i-1)+1:3*(i),:) = J_Ai_Bi;
  Phi_q(:,IJ_i) = dPhidqJi;
  
  % Eintragen in Ergebnis-Variable
  I1 = sum(Rob.I_EE(4:6))*(i-1)+1;
  I2 = I1+sum(Rob.I_EE(4:6))-1;
  if ~isempty(Phi_q_red)
    dPhidqJi_red(I1:I2,:) = J_Ai_Bi(Rob.Leg(i).I_EE(4:6),:);
    Phi_q_red(:,IJ_i) = dPhidqJi_red;
  end
end
