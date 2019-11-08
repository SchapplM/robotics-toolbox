% Fuction for calculating the reduced inertia  form from Do Thanh
%
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS (nicht:
%   Plattform-Pose xP)
% 
% Ausgabe:
% Mred_xD
%   Massenmatrix (bezogen auf EE-Koordinaten der PKM)
% M_plf
%   Vollständige Massenmatrix der PKM (für alle Subsysteme, Kräfte und
%   Beschleunigungenin x-Koordinaten)

% Quelle:
% [DT09] Do Thanh, T. et al: On the inverse dynamics problem of general
% parallel robots, ICM 2009
% [Sriram2019_S876] Sriram, R.: Implementation of a Structurally Independant
% Dynamics Modelling of Parallel Kinematic Machines based on Full Kinematic
% Constraints (Studienarbeit bei Moritz Schappler, 2019).

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Mred_xD, M_plf] = inertia2_platform(Rob, q, xE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/inertia2_platform: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/inertia2_platform: xE muss 6x1 sein');

% Dynamik-Parameter der Endeffektor-Plattform
m_P = Rob.DynPar.mges(end);
mrSges = Rob.DynPar.mrSges(end,:);
Ifges = Rob.DynPar.Ifges(end,:);

NLEG = Rob.NLEG;
NJ = Rob.NJ;

if Rob.issym
  error('Nicht implementiert')
end

% Variable zum Speichern der vollständigen Massenmatrix (Subsysteme)
M_plf  = NaN((NLEG+1)*NLEG, (NLEG+1)*NLEG);

%% Projektionsmatrizen
% Gradientenmatrix der vollständigen Zwangsbedingungen
G_q = Rob.constr1grad_q(q, xE);
G_x = Rob.constr1grad_x(q, xE);
% Inverse Jacobi-Matrix
J1 = - G_q \ G_x; % Siehe: ParRob/jacobi_qa_x

K1 = eye ((NLEG+1)*NLEG); % Reihenfolge der Koordinaten (erst Beine, dann Plattform), [DT09]/(9)
R1 = K1  * [ J1',eye(NLEG)']'; % Projektionsmatrix, [DT09]/(15)

%% Starrkörper-Dynamik der Plattform

if Rob.DynPar.mode == 2
  Mtmp =  rigidbody_inertiaB_floatb_eulxyz_slag_vp2(xE(4:6), m_P, mrSges, Ifges);
  M1 = Mtmp (Rob.I_EE, Rob.I_EE); % code for the selection of degree of freedom 
  M = M1;   % for Do Thanh approach 
else
  error('Regressor noch nicht implementiert');
  testset = pkm_example_fullparallel_parroblib_test_setting(Rob) ;
  MM_reg = rigidbody_inertiaB_floatb_eulxyz_reg2_slag_vp(xE(4:6));
  delta  = testset.delta ;
  Mvec_slag    =  MM_reg * delta ;
  M = vec2symmat(Mvec_slag);  % for Do thanh regreesor dynamic form 
end
  
%% Berechnung der Projektion
% Massenmatrix aller Beinketten berechnen
for i = 1:NLEG
  M_plf((i-1)*NLEG+1:NLEG*i,1:NJ+NLEG) =   [zeros(NLEG,(NLEG*(i-1))) ,Rob.Leg(i).inertia(q(Rob.I1J_LEG(i):Rob.I2J_LEG(i))),zeros(NLEG,NJ -(NLEG*(i-1)))];
end
M_plf(NJ+1:end,1:NJ+NLEG) = [zeros(NLEG,NJ),M];% Massenmatrix der Plattform
  
% Projektion aller Terme der Subsysteme [DT09]/(23)
% Die Momente (Zeilen der Massenmatrix) liegen noch bezogen auf die
% Euler-Winkel vor (entsprechend der EE-Koordinaten)
Mred_sD = transpose(R1)* M_plf * R1;

% Umrechnung der Momente auf kartesische Koordinaten (Basis-KS des
% Roboters)
Tw = euljac(xE(4:6), Rob.phiconv_W_E);
H = [eye(3), zeros(3,3); zeros(3,3), Tw];
Mred_xD = H(Rob.I_EE,Rob.I_EE)'\ Mred_sD;
