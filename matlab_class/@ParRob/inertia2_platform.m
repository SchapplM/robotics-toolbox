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

function Mred = inertia_platform_2(Rob,q ,xE)
phi = xE(4:6) ;
m = Rob.DynPar.mges(end);
mrSges = Rob.DynPar.mrSges(end,:);
qs   = q(Rob.I_EE_Task)'
%Rob.DynPar.mode= 4 ;
Ifges = Rob.DynPar.Ifges(end,:);
rSges = Rob.DynPar.rSges(end,:);
Icges = Rob.DynPar.Icges(end,:);

G_q = Rob.constr1grad_q(q, xE);
G_x = Rob.constr1grad_x(q, xE);

J1=   - G_q \ G_x;
%J2 = J1(Rob.I_EE_Task)'
%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/inertia_platform2: q muss %dx1 sein', Rob.NJ);

NLEG = Rob.NLEG;
NJ = Rob.NJ;

%if  NLEG == 6
  %% Aufruf der Unterfunktionen
  K1   = eye ((NLEG+1)*NLEG  );
  R1   = K1  * [ J1',eye(NLEG)']' ;
  % Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
  % ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)
  % R.DynPar.mges, R.DynPar.rSges, R.DynPar.Icges
  if Rob.DynPar.mode == 2
       % Mtmp1 = rigidbody_pkm_pf_inertia_vp1(phi, m, rSges, Icges);
    Mtmp =  rigidbody_inertiaB_floatb_eulxyz_slag_vp2(phi, m, mrSges, Ifges);
     M1 = Mtmp (Rob.I_EE, Rob.I_EE);
  
%     Tw = eulxyzjac(phi);
%     H = [eye(3), zeros(3,3); zeros(3,3), Tw];    % on changing the xyz the value of Mcomp also changes .
    M = M1;
    
%     M =  H*Mtmp;% * H  ;                 %H' * Mtmp * H ;
    %     %
   else Rob.DynPar.mode == 4
    
    
    testset = pkm_example_fullparallel_parroblib_test_setting(Rob) ;
    MM_reg = rigidbody_inertiaB_floatb_eulxyz_reg2_slag_vp(phi);
    delta  = testset.delta
    Mvec_slag    =  MM_reg * delta ;
    M = vec2symmat(Mvec_slag);
    
   end
  % TODO:
  % R.DynPar.mges, R.DynPar.mrSges, R.DynPar.Ifges
  % M = rigidbody_pkm_pf_inertia_vp2(phi, m_num, mrSges_num_mdh, Ifges_num_mdh);
  
  %% Initialisierung mit Fallunterscheidung für symbolische Eingabe
  if ~Rob.issym
    
    % M_plf  = NaN((NLEG+1)*NLEG+3, (NLEG+1)*NLEG+3);  % +3 is done only for 3fhg system
    M_plf  = NaN((NLEG+1)*NLEG, (NLEG+1)*NLEG);
  else
    %Phi_q_legs_red = NaN(length(Rob.I_constr_t_red),NJ);
    printf('The value is not possible')
    
  end
  
  
  %% Berechnung
  
  
  for i = 1:NLEG
    % M_plf((i-1)*NLEG+1:NLEG*i,1:NJ+NLEG+3) =   [zeros(NLEG,(NLEG*(i-1))) ,Rob.Leg(i).inertia(q(Rob.I1J_LEG(i):Rob.I2J_LEG(i))),zeros(NLEG,NJ -(NLEG*(i-1))+3)];
    M_plf((i-1)*NLEG+1:NLEG*i,1:NJ+NLEG) =   [zeros(NLEG,(NLEG*(i-1))) ,Rob.Leg(i).inertia(q(Rob.I1J_LEG(i):Rob.I2J_LEG(i))),zeros(NLEG,NJ -(NLEG*(i-1)))];
  end
  M_plf(NJ+1:end,1:NJ+NLEG) = [zeros(NLEG,NJ),M];
  
% else   NLEG == 3
%   %% Aufruf der Unterfunktionen
%   K1   = eye (((NLEG+1)*NLEG)+3 );
%   E1 =  eye(NLEG) ;
%   
%   R1    =   [ J1',E1',zeros(NLEG)']' ;
%   R2    = R1(Rob.I_EE)'
%   % Die Unterfunktionen sind nach ZB-Art sortiert, in der Ausgabevariablen
%   % ist die Sortierung nach Beingruppen (ZB Bein 1, ZB Bein 2, ...)
%   % TODO: rigidbody_inertiaB_floatb_eulxyz_slag_vp2
%   %Mtmp = rigidbody_pkm_pf_inertia_vp1(phi, m_num, rSges_num_mdh, Icges_num_mdh);
%   %
%   %
%   % Tw = eulxyzjac(phi);
%   % H = [eye(3), zeros(3,3); zeros(3,3), Tw];
%   %
%   % M = H' * Mtmp * H ;
%   if Rob.DynPar.mode == 2
%     
%     M  =  rigidbody_inertiaB_floatb_eulxyz_slag_vp2(phi, m, mrSges, Ifges);
%     M1 = M(Rob.I_EE, Rob.I_EE)
%   else Rob.DynPar.mode == 4
%     
%     testset = pkm3_example_test_setting (Rob) ;
%     delta   = testset.delta  ;
%     
%     MM_reg = rigidbody_inertiaB_floatb_eulxyz_reg2_slag_vp(phi);
%     
%     Mvec_slag    =  MM_reg * delta ;
%     M = vec2symmat(Mvec_slag);
%     
%   end
%   
%   %% Initialisierung mit Fallunterscheidung für symbolische Eingabe
%   if ~Rob.issym
%     
%     M_plf  = NaN((NLEG+1)*NLEG+3, (NLEG+1)*NLEG+3);  % +3 is done only for 3fhg system
%     
%   else
%     %Phi_q_legs_red = NaN(length(Rob.I_constr_t_red),NJ);
%     printf('The value is not possible')
%     
%   end
%   
%   
%   %% Berechnung
%   
%   
%   for i = 1:NLEG
%     M_plf((i-1)*NLEG+1:NLEG*i,1:NJ+NLEG+3) =   [zeros(NLEG,(NLEG*(i-1))) ,Rob.Leg(i).inertia(q(Rob.I1J_LEG(i):Rob.I2J_LEG(i))),zeros(NLEG,NJ -(NLEG*(i-1))+3)];
%     
%   end
%   
%   M_plf(NJ+1:end,1:end) = [zeros(NLEG+3,NJ),M];
% end
Mred = transpose(R1)* M_plf * R1 ;
Mred = struct ( 'M_plf' ,M_plf,.....
                'Mred' ,Mred ) ;
end

