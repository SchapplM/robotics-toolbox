% Berechnung der Plattform-Kräfte aufgrund von Flieh- und Coriolis-Kräften
%
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% qD [Nx1]
%   Geschwindigkeit aller Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% xDE [6x1]
%   Zeitableitung der Endeffektorpose des Roboters bezüglich des Basis-KS
% Jinv [N x Nx] (optional zur Rechenersparnis)
%   Vollständige Inverse Jacobi-Matrix der PKM (bezogen auf alle N aktiven
%   und passiven Gelenke und die bewegliche EE-Koordinaten xE)
% JinvD [N x Nx] (optional zur Rechenersparnis)
%   Zeitableitung von Jinv
% M_full (optional zur Rechenersparnis)
%   Vollständige Massenmatrix der PKM (für alle Subsysteme, Kräfte und
%   Beschleunigungen in x-Koordinaten)
%   Siehe auch: ParRob/inertia2_platform_full
% M_full_reg (optional zur Rechenersparnis)
%   Regressormatrix zur zweiten Ausgabe (M_full). Matrix wird 3D gestapelt.
%   (Dimension 1 und 2 sind die ursprüngliche Massenmatrix, Dim. 3 sind die
%   Dynamikparameter 
%   Siehe auch: ParRob/inertia2_platform_full
% 
% Ausgabe:
% Cred_Fs
%   Coriolis-Kräfte (bezogen auf Endeffektor-Plattform der PKM)
%   Das zweite Ausgabeargument ist nur zulässig, wenn DynPar.mode=4 ist
% Cred_Fs_reg
%   Regressormatrix zur Coriolis-Kraft
%   Das zweite Ausgabeargument ist nur zulässig, wenn DynPar.mode=4 ist

% Quelle:
% [DT09] Do Thanh, T. et al: On the inverse dynamics problem of general
% parallel robots, ICM 2009
% [Sriram2019_S876] Sriram, R.: Implementation of a Structurally Independant
% Dynamics Modelling of Parallel Kinematic Machines based on Full Kinematic
% Constraints (Studienarbeit bei Moritz Schappler, 2019).

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Cred_Fs, Cred_Fs_reg] = coriolisvec2_platform(Rob, q, qD, xE, xDE, Jinv, JinvD, M_full, M_full_reg)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/coriolisvec2_platform: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/coriolisvec2_platform: qD muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/coriolisvec2_platform: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/coriolisvec2_platform: xDE muss 6x1 sein');
if nargin >= 6
  assert(isreal(Jinv) && all(size(Jinv) == [Rob.NJ sum(Rob.I_EE)]), ...
    'ParRob/coriolisvec2_platform: Jinv muss %dx%d sein', Rob.NJ, sum(Rob.I_EE));
end
if nargin == 7
  assert(isreal(JinvD) && all(size(JinvD) == [Rob.NJ sum(Rob.I_EE)]), ...
    'ParRob/coriolisvec2_platform: JinvD muss %dx%d sein', Rob.NJ, sum(Rob.I_EE));
end
% Dynamik-Parameter der Endeffektor-Plattform
m_P = Rob.DynPar.mges(end);
mrS_P = Rob.DynPar.mrSges(end,:);
If_P = Rob.DynPar.Ifges(end,:);

NLEG = Rob.NLEG;
NJ = Rob.NJ;

if Rob.issym
  error('Nicht implementiert')
end
% Variable zum Speichern der vollständigen Coriolis-Kräfte (Subsysteme)
C_full = NaN(NJ+NLEG,1);
if nargout == 2
  if Rob.DynPar.mode == 3
    C_full_reg = zeros(NJ+NLEG,length(Rob.DynPar.ipv_n1s));
  elseif Rob.DynPar.mode == 4
    C_full_reg = zeros(NJ+NLEG,length(Rob.DynPar.mpv_n1s));
  else
    error('Ausgabe des Regressors mit Dynamik-Modus %d nicht möglich', Rob.DynPar.mode);
  end
end
%% Projektionsmatrizen
if nargin < 7
  G_q = Rob.constr1grad_q(q, xE);
  G_x = Rob.constr1grad_x(q, xE);
  Jinv = - G_q \ G_x; % Siehe: ParRob/jacobi_qa_x
end
if nargin < 7
  G_qD = Rob.constr1gradD_q(q, qD, xE, xDE);
  G_xD = Rob.constr1gradD_x(q, qD, xE, xDE);
  JinvD = G_q\G_qD/G_q*G_x - G_q\G_xD; % Siehe: ParRob/jacobiD_qa_x
end
% Reihenfolge der Koordinaten (erst Beine, dann Plattform), [DT09]/(9)
% Hier Einheitsmatrix, daher keine Multiplikation notwendig.
R1 =  [Jinv;eye(NLEG)]; % Projektionsmatrix, [DT09]/(15)
R1D = [JinvD; zeros(NLEG)]; % Projektionsmatrix-Zeitableitung, [DT09]/(21)

%% Starrkörper-Dynamik der Plattform
if Rob.DynPar.mode==2
  Fc_plf = rigidbody_coriolisvecB_floatb_eulxyz_slag_vp2_mex(xE(4:6), xDE, m_P ,mrS_P,If_P) ;
else
  Fc_plf_reg = rigidbody_coriolisvecB_floatb_eulxyz_reg2_slag_vp_mex(xE(4:6), xDE);
  if Rob.DynPar.mode == 3
    delta = Rob.DynPar.ipv_n1s(end-sum(Rob.I_platform_dynpar)+1:end);
  else
    delta = Rob.DynPar.mpv_n1s(end-sum(Rob.I_platform_dynpar)+1:end);
  end
  Fc_plf = Fc_plf_reg(:,Rob.I_platform_dynpar) * delta;
end
Fc_plf_red = Fc_plf(Rob.I_EE);
%% Berechnung der Projektion
% Coriolis-Komponente aller Beinketten berechnen [DT09]/(6)
ii = 1;
for i = 1:NLEG
  if Rob.DynPar.mode == 2
    cq_leg = Rob.Leg(i).corvec(q(Rob.I1J_LEG(i):Rob.I2J_LEG(i)),qD(Rob.I1J_LEG(i):Rob.I2J_LEG(i)));
  elseif Rob.DynPar.mode == 3
    % Regressorform mit Inertialparametern
    [~,cq_leg_reg] = Rob.Leg(i).corvec(q(Rob.I1J_LEG(i):Rob.I2J_LEG(i)),qD(Rob.I1J_LEG(i):Rob.I2J_LEG(i)));
    cq_leg = cq_leg_reg*Rob.DynPar.ipv_n1s(1:end-sum(Rob.I_platform_dynpar));
  else
    % Regressorform mit Minimalparametern
    [~,cq_leg_reg] = Rob.Leg(i).corvec(q(Rob.I1J_LEG(i):Rob.I2J_LEG(i)),qD(Rob.I1J_LEG(i):Rob.I2J_LEG(i)));
    cq_leg = cq_leg_reg*Rob.DynPar.mpv_n1s(1:end-sum(Rob.I_platform_dynpar));
  end
  C_full(ii:Rob.Leg(i).NJ+ii-1) = cq_leg;
  
  if nargout == 2
    C_full_reg(ii:Rob.Leg(i).NJ+ii-1,1:end-sum(Rob.I_platform_dynpar)) = cq_leg_reg;
  end
  ii = ii + Rob.Leg(i).NJ;
end
% Corioliskomponente für Plattform eintragen
C_full(NJ+1:end) = Fc_plf_red; % Coriolis-Kraft der Plattform
if nargout == 2
  C_full_reg(NJ+1:end,end-sum(Rob.I_platform_dynpar)+1:end) = Fc_plf_reg(Rob.I_EE,Rob.I_platform_dynpar);
end
% Massenmatrix-Komponenten aller Subsysteme
if nargout == 1 && nargin < 8
  [M_full] = Rob.inertia2_platform_full(q, xE);
elseif nargout == 2 && nargin < 9
  % Regressor-Matrix soll berechnet werden. Benötigt Regressor-Matrix der Massenmatrix
  [M_full, M_full_reg] = Rob.inertia2_platform_full(q, xE);
end
% Projektion aller Terme der Subsysteme [DT09]/(23)
% Die Momente liegen noch bezogen auf die Euler-Winkel vor (entsprechend der
% EE-Koordinaten)
Cred_Fx = R1'* C_full + R1' * M_full * R1D * xDE(Rob.I_EE);
if nargout == 2
  % Erster Summand der vorherigen Gleichung (direkt Spaltenweise auf
  % Regressor-Matrix anwendbar)
  Cred_Fx_reg = R1' * C_full_reg;
  % Zweiter Summand: Führe Projektion für jeden Dynamikparameter einzeln
  % aus
  for jj = 1:size(C_full_reg,2)
    % Volle Massenmatrix aller Subsysteme bezogen auf diesen Parameter `jj`
    M_full_jj = reshape(M_full_reg(:,:,jj), size(M_full));
    % Projektion wie oben
    Cred_regadd = R1' * M_full_jj * R1D * xDE(Rob.I_EE);
    Cred_Fx_reg(:,jj) = Cred_Fx_reg(:,jj) + Cred_regadd;
  end
end

% Umrechnung der Momente auf kartesische Koordinaten (Basis-KS des
% Roboters)
Tw = euljac(xE(4:6), Rob.phiconv_W_E);
H = [eye(3), zeros(3,3); zeros(3,3), Tw];
Cred_Fs = H(Rob.I_EE,Rob.I_EE)'\Cred_Fx;
if nargout == 2
  % Ausführung der Transformation spaltenweise für alle Dynamikparameter
  Cred_Fs_reg = H(Rob.I_EE,Rob.I_EE)' \  Cred_Fx_reg;
end
