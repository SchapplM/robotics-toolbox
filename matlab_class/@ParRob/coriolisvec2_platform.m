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

% Quelle:
% [A] Aufzeichnungen Schappler vom 15.06.2018
% [B] Aufzeichnungen Schappler vom 22.06.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Cred = coriolisvec2_platform(Rob,q ,qD, xE, xDE)
phi_base = xE(4:6) ;
m = Rob.DynPar.mges(end);
%Rob.DynPar.mode = 4 ;
mrSges = Rob.DynPar.mrSges(end,:);
Ifges = Rob.DynPar.Ifges(end,:);
G_q = Rob.constr1grad_q(q, xE);
G_x = Rob.constr1grad_x(q, xE);
G_qd = Rob.constr1gradD_q(q,qD , xE,xDE);
G_xd = Rob.constr1gradD_x(q, qD , xE ,xDE );
%% Aufruf der Funktion Mred
Mred = Rob.inertia2_platform(q ,xE) ;
M_plf = Mred.M_plf ;

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/coriolisvec_platform2: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/coriolisvec_platform2: qD muss %dx1 sein', Rob.NJ);
NLEG = Rob.NLEG;
NJ = Rob.NJ;

%% Berechnung der Parameter f�r Coriolis Korrektur
J =   - G_q \ G_x;



%% Aufruf der Unterfunktionen
% Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
% ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)

%Fc = rigidbody_coriolisvecB_floatb_eulxyz_slag_vp1(phi_base, xD_base, m, rSges, Icges)

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
%if NLEG == 6
  if ~Rob.issym
    C_plf = NaN(NJ+NLEG,1);
    %C_plf = NaN(NJ+NLEG+3,1);
  else
    %Phi_q_legs_red = NaN(length(Rob.I_constr_t_red),NJ);
    printf('The value is not possible')
    
  end
  
  
  K1   = eye ((NLEG+1)*NLEG  );
  R1    = K1  * [ J',eye(NLEG)']' ;
  
  a = inv(G_q)*G_qd*inv(G_q)*G_x - inv(G_q)*G_xd ;
  b =  0 ;
  c =  [ a',zeros(NLEG)']';
  
  
  if Rob.DynPar.mode==2
    
    Fc1 = rigidbody_coriolisvecB_floatb_eulxyz_slag_vp2(phi_base, xDE, m ,mrSges,Ifges) ;
    Fc  = Fc1(Rob.I_EE) ;
    
  else  Rob.DynPar.mode == 4
    
    testset = pkm_example_fullparallel_parroblib_test_setting(Rob) ;
    delta = testset.delta
    tauc_reg = rigidbody_coriolisvecB_floatb_eulxyz_reg2_slag_vp(phi_base, xDE);
    Fc  =  tauc_reg * delta ;
    %
  end
  
  
  %% Berechnung
  
  for n = 1:NLEG
    C_plf((n-1)*NLEG+1:NLEG*n) = Rob.Leg(n).corvec(q(Rob.I1J_LEG(n):Rob.I2J_LEG(n)),qD(Rob.I1J_LEG(n):Rob.I2J_LEG(n)));
  end
  C_plf(NJ+1:end) = Fc;
  
  
% else  NLEG == 3
%   
%   if ~Rob.issym
%     C_plf = NaN(NJ+NLEG+3,1);
%   else
%     %Phi_q_legs_red = NaN(length(Rob.I_constr_t_red),NJ);
%     printf('The value is not possible')
%     
%   end
%   
%   K1   = eye (((NLEG+1)*NLEG)+3 );
%   E1 =  zeros(NLEG+NLEG) ;
%   R1    =   K1 * [ J',E1(Rob.I_EE ,Rob.I_EE)',zeros(NLEG)']' ;% K * [ J',E(Rob.I_EE ,Rob.I_EE)',eye(NLEG)']' ;
% 
%   a = inv(G_q)*G_qd*inv(G_q)*G_x - inv(G_q)*G_xd ;
%   b =  0 ;
%   c =  [ a', E1(Rob.I_EE ,Rob.I_EE)',zeros(NLEG)']';
%   
%   
%   if Rob.DynPar.mode==2
%     
%     Fc = rigidbody_coriolisvecB_floatb_eulxyz_slag_vp2(phi_base, xDE, m ,mrSges,Ifges) ;
%     
%     
%   else Rob.DynPar.mode == 4
%     
%   
%     testset = pkm3_example_test_setting (Rob);
%     delta = testset.delta
%     tauc_reg = rigidbody_coriolisvecB_floatb_eulxyz_reg2_slag_vp(phi_base, xDE);
%     Fc  =  tauc_reg * delta ;
%     
%   end
%   
%   %% Berechnung
%   
%   for n = 1:NLEG
%     C_plf((n-1)*NLEG+1:NLEG*n) = Rob.Leg(n).corvec(q(Rob.I1J_LEG(n):Rob.I2J_LEG(n)),qD(Rob.I1J_LEG(n):Rob.I2J_LEG(n)));
%   end
%   C_plf(NJ+1:end) = Fc;
%   
  
%end

Cred  = transpose(R1)* C_plf +  transpose(R1)*M_plf*c*xDE(Rob.I_EE) ;


