% Fuction for calculating the reduced coriolis matrix  form from Do Thanh
%
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% qD [Nx1]
%  Velocity of the  Gelenkwinkel aller serieller Beinketten der PKM
%xE [N x1]
%   Platform Coordinate based on the orientation and rotation
%xDE [N x1]
%   Velocity of the Platform Coordinate based on the orientation and rotation
% Ausgabe:
% Reduced Coriolis Matrix from Do Thanh


% Quelle:
% Do Thanh [A new program
%to Automatically Generate the Kinematic and Dynamic Equations of General
%Parallel Robots in Symbolic Form.]

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Cred = coriolisvec2_platform(Rob,q ,qD, xE, xDE)
phi_base = xE(4:6) ;
m = Rob.DynPar.mges(end);
%Rob.DynPar.mode = 4 ;  for regressor form 
mrSges = Rob.DynPar.mrSges(end,:);
Ifges = Rob.DynPar.Ifges(end,:);
G_q = Rob.constr1grad_q(q, xE);
G_x = Rob.constr1grad_x(q, xE);
G_qd = Rob.constr1gradD_q(q,qD , xE,xDE); % differentiation of the kinematic constraints of joint coordinate can be found in [Do Thanh]
G_xd = Rob.constr1gradD_x(q, qD , xE ,xDE ); %% differentiation of the kinematic constraints of platform  coordinate can be found in [Do Thanh]
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





%% Initialisierung mit Fallunterscheidung für symbolische Eingabe
  if ~Rob.issym
    C_plf = NaN(NJ+NLEG,1);

  else
    
    printf('The value is not possible')
    
  end
  
  
  K1   = eye ((NLEG+1)*NLEG  );
  R1    = K1  * [ J',eye(NLEG)']' ; % projection matrix R as mention in Do Thanh APPROACH 
  
  a = inv(G_q)*G_qd*inv(G_q)*G_x - inv(G_q)*G_xd ; % formula for the first term in the differentiation of the projection matrix which can be found in [Do Thanh ]
  c =  [ a',zeros(NLEG)']'; % differentiation of the projection matrix Rdot
  
  
  if Rob.DynPar.mode==2
    
    Fc1 = rigidbody_coriolisvecB_floatb_eulxyz_slag_vp2(phi_base, xDE, m ,mrSges,Ifges) ;
    Fc  = Fc1(Rob.I_EE) ;
    
  else  Rob.DynPar.mode == 4 ;
    
    testset = pkm_example_fullparallel_parroblib_test_setting(Rob) ;
    delta = testset.delta ;
    tauc_reg = rigidbody_coriolisvecB_floatb_eulxyz_reg2_slag_vp(phi_base, xDE);
    Fc  =  tauc_reg * delta ;
    %
  end
  
  
  %% Berechnung
  
  for n = 1:NLEG
    C_plf((n-1)*NLEG+1:NLEG*n) = Rob.Leg(n).corvec(q(Rob.I1J_LEG(n):Rob.I2J_LEG(n)),qD(Rob.I1J_LEG(n):Rob.I2J_LEG(n))); % coriolis matrix for the joint coordinate
  end
  C_plf(NJ+1:end) = Fc; % corilis matrix for the platform coordinate
  

Cred  = transpose(R1)* C_plf +  transpose(R1)*M_plf*c*xDE(Rob.I_EE) ;% formula of the reduced coriolis matrix can be found in [Do Thanh]


