% Fuction for calculating the reduced inertia  form from Do Thanh
%
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
%xE [N x1]
%   Platform Coordinate based on the orientation and rotation
% Ausgabe:
% Reduced Mass Matrix from Do Thanh


% Quelle:
% Do Thanh [A new program
%to Automatically Generate the Kinematic and Dynamic Equations of General
%Parallel Robots in Symbolic Form.]



% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Mred = inertia_platform_2(Rob,q ,xE)
phi = xE(4:6) ;
m = Rob.DynPar.mges(end);
mrSges = Rob.DynPar.mrSges(end,:);
%Rob.DynPar.mode= 4 ;  for regressor form 
Ifges = Rob.DynPar.Ifges(end,:);


G_q = Rob.constr1grad_q(q, xE);
G_x = Rob.constr1grad_x(q, xE);

J1=   - G_q \ G_x;

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/inertia_platform2: q muss %dx1 sein', Rob.NJ);

NLEG = Rob.NLEG;
NJ = Rob.NJ;

%if  NLEG == 6
  %% Aufruf der Unterfunktionen
  K1   = eye ((NLEG+1)*NLEG  );
  R1   = K1  * [ J1',eye(NLEG)']' ; % for calculation of Projection Matrix R from [Do Thanh]

  if Rob.DynPar.mode == 2
       
    Mtmp =  rigidbody_inertiaB_floatb_eulxyz_slag_vp2(phi, m, mrSges, Ifges);
     M1 = Mtmp (Rob.I_EE, Rob.I_EE); % code for the selection of degree of freedom 
    M = M1;   % for Do Thanh approach 
    

   else Rob.DynPar.mode == 4 ;
    
    
    testset = pkm_example_fullparallel_parroblib_test_setting(Rob) ;
    MM_reg = rigidbody_inertiaB_floatb_eulxyz_reg2_slag_vp(phi);
    delta  = testset.delta ;
    Mvec_slag    =  MM_reg * delta ;
    M = vec2symmat(Mvec_slag);  % for Do thanh regreesor dynamic form 
    
   end

  
  %% Initialisierung mit Fallunterscheidung für symbolische Eingabe
  if ~Rob.issym
   
    M_plf  = NaN((NLEG+1)*NLEG, (NLEG+1)*NLEG); % calculation of the mass matrix for the joint coordinates
  else

    printf('The value is not possible')
    
  end
  
  
  %% Berechnung
  
  
  for i = 1:NLEG
 
    M_plf((i-1)*NLEG+1:NLEG*i,1:NJ+NLEG) =   [zeros(NLEG,(NLEG*(i-1))) ,Rob.Leg(i).inertia(q(Rob.I1J_LEG(i):Rob.I2J_LEG(i))),zeros(NLEG,NJ -(NLEG*(i-1)))];
  end
  M_plf(NJ+1:end,1:NJ+NLEG) = [zeros(NLEG,NJ),M]; % calculation of the mass matrix for the platform coordinate
  

Mred = transpose(R1)* M_plf * R1 ;
Mred = struct ( 'M_plf' ,M_plf,.....
                'Mred' ,Mred ) ;
end

