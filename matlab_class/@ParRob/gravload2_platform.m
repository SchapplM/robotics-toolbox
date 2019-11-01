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






function gred = gravload_platform_2(Rob,q ,xE, g)
phi = xE(4:6) ;
m = Rob.DynPar.mges(end);
mrSges = Rob.DynPar.mrSges(end,:);
%Rob.DynPar.mode = 4
G_q = Rob.constr1grad_q(q, xE);
G_x = Rob.constr1grad_x(q, xE);

J1=   - G_q \ G_x;

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/gravload_platform2: q muss %dx1 sein', Rob.NJ);
NLEG = Rob.NLEG;
NJ = Rob.NJ;
%% Aufruf der Unterfunktionen
% Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
% ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)
%g2 = rigidbody_pkm_pf_gravload_vp1(phi, g, m_num, rSges_num_mdh);
%Tw = eulxyzjac(phi);
%H = [eye(3), zeros(3,3); zeros(3,3), Tw];
%tau =  H' * g2 ;

%% Initialisierung mit Fallunterscheidung für symbolische Eingabe

%if NLEG == 6
  
  if ~Rob.issym
    %G_plf = NaN(NJ+NLEG+3,1);
    G_plf = NaN(NJ+NLEG,1);
    
  else
    %Phi_q_legs_red = NaN(length(Rob.I_constr_t_red),NJ);
    printf('The value is not possible')
    
  end
  
  K1   = eye ((NLEG+1)*NLEG  );
  R1   = K1  * [ J1',eye(NLEG)']' ;
  Tw = eulxyzjac(phi);
  H = [eye(3), zeros(3,3); zeros(3,3), Tw];
  if Rob.DynPar.mode == 2
    
    %     gtau = H\rigidbody_pkm_pf_gravload_vp1(phi, g, m, rSges_num_mdh);
    gtau1  =rigidbody_gravloadB_floatb_eulxyz_slag_vp2(phi, g, m, mrSges) ;
    gtau   = gtau1(Rob.I_EE)
    
    
  else Rob.DynPar.mode == 4
    
    testset = pkm_example_fullparallel_parroblib_test_setting(Rob) ;
    delta  = testset.delta ;
    %
    tau = rigidbody_gravloadB_floatb_eulxyz_reg2_slag_vp(phi, g);
    gtau = tau * delta ;
    %
  end
  %
  
  %% Berechnung
  
  
  for i = 1:NLEG
    q_i = q(Rob.I1J_LEG(i):Rob.I2J_LEG(i));
    G_plf((i-1)*NLEG+1:NLEG*i) = Rob.Leg(i).gravload(q_i);
  end
  
  G_plf(NJ+1:end) = gtau;
  
  
  
% else  NLEG == 3
%   if ~Rob.issym
%     G_plf = NaN(NJ+NLEG+3,1);
%     
%   else
%     %Phi_q_legs_red = NaN(length(Rob.I_constr_t_red),NJ);
%     printf('The value is not possible')
%     
%   end
%   
%   K1   = eye (((NLEG+1)*NLEG)+3 );
%   E1 =  zeros(NLEG+NLEG) ;
%   R1    =   K1 * [ J1',E1(Rob.I_EE ,Rob.I_EE)',zeros(NLEG)']';
%   
%   %% Berechnung
%   if Rob.DynPar.mode == 2
%     
%     %g2 = rigidbody_pkm_pf_gravload_vp1(phi, g, m_num, rSges_num_mdh);
%     gtau  =rigidbody_gravloadB_floatb_eulxyz_slag_vp2(phi, g, m, mrSges) ;
%     
%     
%     
%   else Rob.DynPar.mode == 4
%     
%     testset = pkm3_example_test_setting(Rob) ;
%     delta  = testset.delta ;
%     
%     tau = rigidbody_gravloadB_floatb_eulxyz_reg2_slag_vp(phi, g);
%     gtau = tau * delta ;
%     %
%   end
%   
%   for i = 1:NLEG
%     q_i = q(Rob.I1J_LEG(i):Rob.I2J_LEG(i));
%     G_plf((i-1)*NLEG+1:NLEG*i) = Rob.Leg(i).gravload(q_i);
%   end
%   G_plf(NJ+1:end) = gtau;
%   
% end
gred = transpose(R1)* G_plf ;
