% Gravitations-Komponente der Inversen Dynamik für PKM (bezogen auf
% Endeffektor-Plattform)
%
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS (nicht:
%   Plattform-Pose xP)
% 
% Ausgabe:
% Fgx: Gravitationskräfte (bezogen auf EE-Koordinaten der PKM; Kräfte und
% Momente in kartesischen Koordinaten, bezogen auf Basis-KS)

% Quelle:
% [DT09] Do Thanh, T. et al: On the inverse dynamics problem of general
% parallel robots, ICM 2009
% [Sriram2019_S876] Sriram, R.: Implementation of a Structurally Independant
% Dynamics Modelling of Parallel Kinematic Machines based on Full Kinematic
% Constraints (Studienarbeit bei Moritz Schappler, 2019).

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function gred_Fs = gravload2_platform(Rob, q, xE)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/gravload2_platform: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/gravload2_platform: xE muss 6x1 sein');

g = Rob.gravity;
% Dynamik-Parameter der Endeffektor-Plattform
m_P = Rob.DynPar.mges(end);
mrS_P = Rob.DynPar.mrSges(end,:);

NLEG = Rob.NLEG;
NJ = Rob.NJ;

if Rob.issym
  error('Nicht implementiert')
end
% Variable zum Speichern der vollständigen Gravitations-Kräfte (Subsysteme)
G_plf = NaN(NJ+NLEG,1);

%% Projektionsmatrizen
% Gradientenmatrix der vollständigen Zwangsbedingungen
G_q = Rob.constr1grad_q(q, xE);
G_x = Rob.constr1grad_x(q, xE);
% Inverse Jacobi-Matrix
Jinv = - G_q \ G_x; % Siehe: ParRob/jacobi_qa_x

K1 = eye ((NLEG+1)*NLEG); % Reihenfolge der Koordinaten (erst Beine, dann Plattform), [DT09]/(9)
R1 = K1 * [ Jinv',eye(NLEG)']'; % Projektionsmatrix, [DT09]/(15)

%% Starrkörper-Dynamik der Plattform
if Rob.DynPar.mode == 2
  gtau1  =rigidbody_gravloadB_floatb_eulxyz_slag_vp2(xE(4:6), g, m_P, mrS_P) ;
  gtau_P   = gtau1(Rob.I_EE);
else % TODO: Noch nicht implementiert
  error('Regressor noch nicht implementiert');
  % TODO: Code aus Sriram2019_S876 anpassen
  testset = pkm_example_fullparallel_parroblib_test_setting(Rob) ;
  delta  = testset.delta ;
  tau = rigidbody_gravloadB_floatb_eulxyz_reg2_slag_vp(xE(4:6), g);
  gtau_P = tau * delta ;
end

%% Berechnung der Projektion
% Gravitations-Komponente aller Beinketten berechnen [DT09]/(6)
for i = 1:NLEG
  q_i = q(Rob.I1J_LEG(i):Rob.I2J_LEG(i));
  G_plf((i-1)*NLEG+1:NLEG*i) = Rob.Leg(i).gravload(q_i);
end
G_plf(NJ+1:end) = gtau_P; % Gravitationskraft der Plattform
  
% Projektion aller Terme der Subsysteme [DT09]/(23)
% Die Momente liegen noch bezogen auf die Euler-Winkel vor (entsprechend der
% EE-Koordinaten)
gred_Fx = transpose(R1)* G_plf;

% Umrechnung der Momente auf kartesische Koordinaten (Basis-KS des
% Roboters)
Tw = euljac(xE(4:6), Rob.phiconv_W_E);
H = [eye(3), zeros(3,3); zeros(3,3), Tw];
gred_Fs = H(Rob.I_EE,Rob.I_EE)' \  gred_Fx;