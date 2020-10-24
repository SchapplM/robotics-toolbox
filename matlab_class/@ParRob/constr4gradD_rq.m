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

function [Phi_q_red, Phi_q] = constr4gradD_rq(Rob, q, qD)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/constr4gradD_rq: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/constr4gradD_rq: qD muss %dx1 sein', Rob.NJ);
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
% Berechnung aus dem rotatorischen Teil der Jacobi-Matrix der seriellen
% Beinketten
K1 = 1;
for i = 1:NLEG
  IJ_i = Rob.I1J_LEG(i):Rob.I2J_LEG(i);
  qs = q(IJ_i); % Gelenkwinkel dieser Kette
  qsD = qD(IJ_i); 
  
  phi_0_Ai = Rob.Leg(i).phi_W_0;
  R_0_0i = eul2r(phi_0_Ai, Rob.Leg(i).phiconv_W_0);
  % Geometrische Jacobi-Matrix (Rotations-Teil)
  JD0i_i_rotg = Rob.Leg(i).jacobiwD(qs, qsD);
  JD_Ai_Bi = R_0_0i*JD0i_i_rotg; % Bezug auf das Basis-KS der PKM
  
  %% In Endergebnis einsetzen
  I1 = 1+3*(i-1); % I: Zeilen der Ergebnisvariable: Alle rotatorischen ZB
  I2 = I1+2; % drei rotatorische Einträge
  % Gl. A.10
  Phi_q(I1:I2,IJ_i) = JD_Ai_Bi;
  
  % Eintragen in Ergebnis-Variable
  if ~isempty(Phi_q_red)
    K2 = K1+sum(Rob.Leg(i).I_EE_Task(4:6))-1;
    Phi_q_red(K1:K2,IJ_i) = JD_Ai_Bi(Rob.Leg(i).I_EE_Task(4:6),:);
    K1 = K2+1;
  end
end
