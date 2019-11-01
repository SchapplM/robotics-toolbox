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

% function is made for the calculation of the inverse dynamic, later the
% function could be elobrated to directly calculate the inverse dynamic
% function now only used for callin the mass, coriolis and gravitation 
% matrix

% Quelle:
% Do Thanh [A new program
%to Automatically Generate the Kinematic and Dynamic Equations of General
%Parallel Robots in Symbolic Form.]


% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function tau_plf = invdyn_ser_platform (Rob,q,qD,xE,xDE)
%m = Rob.DynPar.mges(end);
%mrSges = Rob.DynPar.mrSges(end,:);
%Ifges = Rob.DynPar.Ifges(end,:);
NLEG = Rob.NLEG;
%NJ = Rob.NJ;
%phi = xE(4:6);
g = Rob.gravity;
%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/invdyn: q muss %dx1 sein',Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/invdyn: qD muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1grad_q: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr1grad_q: xDE muss 6x1 sein');


%% Aufruf der Unterfunktion

Cred = Rob.coriolisvec2_platform(q ,qD,xE, xDE) ;
gred = Rob.gravload2_platform(q ,xE, g) ;
Mred = inertia2_platform(Rob,q ,xE) ;
Mred = Mred.Mred;


%Tw = eulxyzjac(phi);
%H = [eye(3), zeros(3,3); zeros(3,3), Tw]; 
%Cred = Cwow.Cred ;
%Cred1 = Cwow.Cred1 ;% on changing the xyz the value of Mcomp also changes .
%H1 = [eye(3), zeros(3,3); zeros(3,3), Tw];  


%% Start
%tau_plf = NaN(NLEG);
 G_q = Rob.constr1grad_q(q, xE);
 G_x = Rob.constr1grad_x(q, xE);
 J =   - G_q \ G_x;
 K   = eye ((NLEG+1)*NLEG  );
 R   = K  * [ J',eye(NLEG)']' ;


  %taured=  Mred*xDD  + Cred + gred 
  
  %JT   = transpose(R)*eye((NLEG+1)*NLEG,NLEG) ;
  %tau_plf = JT * taured ;
  



tau_plf = struct( 'Mred', Mred, ...
  'gred', gred,....
  'Cred', Cred) ;
