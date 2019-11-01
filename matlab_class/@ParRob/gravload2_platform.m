% Fuction for calculating the reduced gravitational form from Do Thanh
%
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
%xE [N x1]
%   Platform Coordinate based on the orientation and rotation
% Ausgabe:
% Reduced Gravitation Matrix from Do Thanh


% Quelle:
% Do Thanh [A new program
%to Automatically Generate the Kinematic and Dynamic Equations of General
%Parallel Robots in Symbolic Form.]

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover






function gred = gravload_platform_2(Rob,q ,xE, g)
phi = xE(4:6) ;
m = Rob.DynPar.mges(end);
mrSges = Rob.DynPar.mrSges(end,:);
%Rob.DynPar.mode = 4 ; % used for calling the regressor dynamics
G_q = Rob.constr1grad_q(q, xE);
G_x = Rob.constr1grad_x(q, xE);

J1=   - G_q \ G_x; % relation jacobi matrix between the joint coordinate 
% and the platform coordinate which is calculated on the basis of 
% kinematic constraints

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/gravload_platform2: q muss %dx1 sein', Rob.NJ);
NLEG = Rob.NLEG;
NJ = Rob.NJ;



%% Initialisierung mit Fallunterscheidung für symbolische Eingabe


  
  if ~Rob.issym
   
    G_plf = NaN(NJ+NLEG,1);
    
  else
  
    printf('The value is not possible')
    
  end
  
  K1   = eye ((NLEG+1)*NLEG  );
  R1   = K1  * [ J1',eye(NLEG)']' ; % same as the projection matrix in[Do Thanh]source can be seen in the beginning of the code
  
  if Rob.DynPar.mode == 2
    
    % for Do Thanh appproach 
   
    gtau1  =rigidbody_gravloadB_floatb_eulxyz_slag_vp2(phi, g, m, mrSges) ;
    gtau   = gtau1(Rob.I_EE);
    
    
  else Rob.DynPar.mode == 4 ;
    % for regreesor form with Do Thanh approach 
    
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
    G_plf((i-1)*NLEG+1:NLEG*i) = Rob.Leg(i).gravload(q_i); % for the value of the joint coordinates
  end
  
  G_plf(NJ+1:end) = gtau;  % for the value of platform 
  
  
  

% end
gred = transpose(R1)* G_plf ;
