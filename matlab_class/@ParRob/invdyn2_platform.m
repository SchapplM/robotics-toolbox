% Berechnung der Plattformkraft aufgrund der Effekte der inversen Dynamik
%
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% qD [Nx1]
%   Geschwindigkeit aller Gelenkwinkel aller serieller Beinketten der PKM
% qDD [Nx1]
%   Beschleunigung aller Gelenkwinkel aller serieller Beinketten der PKM
% xP [6x1]
%   Plattform-Pose des Roboters bezüglich des Basis-KS (Nicht: EE-KS)
% xDP [6x1]
%   Zeitableitung der Plattform-Pose des Roboters bezüglich des Basis-KS
% xDDP [6x1]
%   Zweite Zeitableitung von xP
% Jinv [N x Nx] (optional zur Rechenersparnis)
%   Vollständige Inverse Jacobi-Matrix der PKM (bezogen auf alle N aktiven
%   und passiven Gelenke und die bewegliche Plattform-Koordinaten xP).
%   Nicht bezogen auf die EE-Koordinaten (nicht identisch zu Matrix aus InvKin)
% 
% Ausgabe:
% Fs
%   Invers-Dynamik-Kräfte (bezogen auf Endeffektor-Plattform der PKM)
%   (enthalten alle dynamischen Effekte: Grav., Coriolis, Beschleunigung)
%   Die Momente sind kartesische Momente bezogen auf das Basis-KS der PKM.
% Fs_reg
%   Regressormatrix zur Invers-Dynamik-Kraft. Bezogen auf in DynPar.mode
%   gewählte Regressorform

% Quelle:
% [DT09] Do Thanh, T. et al: On the inverse dynamics problem of general
% parallel robots, ICM 2009
% [Sriram2019_S876] Sriram, R.: Implementation of a Structurally Independant
% Dynamics Modelling of Parallel Kinematic Machines based on Full Kinematic
% Constraints (Studienarbeit bei Moritz Schappler, 2019).

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Fs, Fs_reg] = invdyn2_platform(Rob, q, qD, qDD, xP, xDP, xDDP, Jinv)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/invdyn2_platform: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/invdyn2_platform: qD muss %dx1 sein', Rob.NJ);
assert(isreal(qDD) && all(size(qDD) == [Rob.NJ 1]), ...
  'ParRob/invdyn2_platform: qDD muss %dx1 sein', Rob.NJ);
assert(isreal(xP) && all(size(xP) == [6 1]), ...
  'ParRob/invdyn2_platform: xP muss 6x1 sein');
assert(isreal(xDP) && all(size(xDP) == [6 1]), ...
  'ParRob/invdyn2_platform: xDP muss 6x1 sein');
assert(isreal(xDDP) && all(size(xDDP) == [6 1]), ...
  'ParRob/invdyn2_platform: xDDP muss 6x1 sein');
if nargin == 8
  assert(isreal(Jinv) && all(size(Jinv) == [Rob.NJ sum(Rob.I_EE)]), ...
    'ParRob/invdyn2_platform: Jinv muss %dx%d sein', Rob.NJ, sum(Rob.I_EE));
end
g = Rob.gravity;
% Dynamik-Parameter der Endeffektor-Plattform (bezogen auf Plattform-KS,
% nicht: Endeffektor-KS)
m_P = Rob.DynPar.mges(end);
mrS_P = Rob.DynPar.mrSges(end,:);
If_P = Rob.DynPar.Ifges(end,:);

NLEG = Rob.NLEG;
NJ = Rob.NJ;

if Rob.issym
  error('Nicht implementiert')
end
% Variable zum Speichern der vollständigen Dynamik-Kräfte (Subsysteme)
Tau_full = NaN(NJ+NLEG,1);
if nargout == 2
  if Rob.DynPar.mode == 3
    Tau_full_reg = zeros(NJ+NLEG,length(Rob.DynPar.ipv_n1s));
  elseif Rob.DynPar.mode == 4
    Tau_full_reg = zeros(NJ+NLEG,length(Rob.DynPar.mpv_n1s));
  else
    error('Ausgabe des Regressors mit Dynamik-Modus %d nicht möglich', Rob.DynPar.mode);
  end
end
%% Projektionsmatrizen
if nargin < 8
  G_q = Rob.constr1grad_q(q, xP, true);
  G_x = Rob.constr1grad_x(q, xP, true);
  Jinv = - G_q \ G_x; % Siehe: ParRob/jacobi_qa_x
end

% Reihenfolge der Koordinaten (erst Beine, dann Plattform), [DT09]/(9)
% Hier Einheitsmatrix, daher keine Multiplikation notwendig.
R1 = [Jinv; eye(NLEG)]; % Projektionsmatrix, [DT09]/(15)

%% Starrkörper-Dynamik der Plattform
if Rob.DynPar.mode == 2
  F_plf = rigidbody_invdynB_floatb_eulxyz_slag_vp2_mex(xP(4:6), xDP, xDDP, g, m_P, mrS_P, If_P);
else
  F_plf_reg = rigidbody_invdynB_floatb_eulxyz_reg2_slag_vp_mex(xP(4:6), xDP, xDDP, g);
  delta = Rob.DynPar.mpv_n1s(end-sum(Rob.I_platform_dynpar)+1:end); % unabhängig von gewählter Berechnung verfügbar
  F_plf = F_plf_reg(:,Rob.I_platform_dynpar) * delta;
end
F_plf_red = F_plf(Rob.I_EE);

%% Berechnung der Projektion
% Inversdynamik-Komponente aller Beinketten berechnen; analog zu [DT09]/(6)
ii = 1;
for i = 1:NLEG
  q_i = q(Rob.I1J_LEG(i):Rob.I2J_LEG(i));
  qD_i = qD(Rob.I1J_LEG(i):Rob.I2J_LEG(i));
  qDD_i = qDD(Rob.I1J_LEG(i):Rob.I2J_LEG(i));
  if Rob.DynPar.mode == 2
    tauq_leg = Rob.Leg(i).invdyn(q_i, qD_i, qDD_i);
  elseif Rob.DynPar.mode == 3
    % Regressorform mit Inertialparametern
    [~,tauq_leg_reg] = Rob.Leg(i).invdyn(q_i, qD_i, qDD_i);
    tauq_leg = tauq_leg_reg*Rob.DynPar.ipv_n1s(1:end-sum(Rob.I_platform_dynpar));
  else
    % Regressorform mit Minimalparametern
    [~,tauq_leg_reg] = Rob.Leg(i).invdyn(q_i, qD_i, qDD_i);
    tauq_leg = tauq_leg_reg*Rob.DynPar.mpv_n1s(1:end-sum(Rob.I_platform_dynpar));
  end
  Tau_full(ii:Rob.Leg(i).NJ+ii-1) = tauq_leg;
  if nargout == 2
    Tau_full_reg(ii:Rob.Leg(i).NJ+ii-1,1:end-sum(Rob.I_platform_dynpar)) = tauq_leg_reg;
  end
  ii = ii + Rob.Leg(i).NJ;
end
% Dynamikkomponente für Plattform eintragen
Tau_full(NJ+1:end) = F_plf_red; % Dynamik-Kraft der Plattform
if nargout == 2
  Tau_full_reg(NJ+1:end,end-sum(Rob.I_platform_dynpar)+1:end) = F_plf_reg(Rob.I_EE,Rob.I_platform_dynpar);
end
% Projektion aller Terme der Subsysteme; analog zu [DT09]/(23)
% Die Momente liegen noch bezogen auf die Euler-Winkel vor (entsprechend der
% EE-Koordinaten)
taured_Fx = R1'* Tau_full;
if nargout == 2
  taured_Fx_reg = R1' * Tau_full_reg;
end
% Umrechnung der Momente auf kartesische Koordinaten (Basis-KS des
% Roboters)
Tw = euljac(xP(4:6), Rob.phiconv_W_E);
H = [eye(3), zeros(3,3); zeros(3,3), Tw];
Fs = H(Rob.I_EE,Rob.I_EE)' \  taured_Fx;
if nargout == 2
  Fs_reg = H(Rob.I_EE,Rob.I_EE)' \  taured_Fx_reg;
end

return
%% Alternative Berechnung
% Offizielle Version aus [DT09].
% Vorteil: Keine Notwendigkeit von qDD als Eingabe
% Nachteile: 
% * Wesentlich aufwändigere Berechnung aufgrund der Massenmatrix-Implementierung
% * Notwendigkeit von JinvD
if nargin >= 8 %#ok<UNRCH>
  G_q = Rob.constr1grad_q(q, xP, true);
  G_x = Rob.constr1grad_x(q, xP, true);
end
G_qD = Rob.constr1gradD_q(q, qD, xP, xDP, true);
G_xD = Rob.constr1gradD_x(q, qD, xP, xDP, true);
JinvD = G_q\G_qD/G_q*G_x - G_q\G_xD; % Siehe: ParRob/jacobiD_qa_x
if nargout == 1
  M_full = inertia2_platform_full(Rob, q, xP);
  Fc = Rob.coriolisvec2_platform(q, qD, xP, xDP, Jinv, JinvD, M_full);
  M  = Rob.inertia2_platform(q, xP, Jinv, M_full);
  Fg = Rob.gravload2_platform(q, xP, Jinv);
else
  [M_full, M_full_reg] = inertia2_platform_full(Rob, q, xP);
  [Fc, Fc_reg] = Rob.coriolisvec2_platform(q, qD, xP, xDP, Jinv, JinvD, M_full, M_full_reg);
  [M, M_reg] = Rob.inertia2_platform(q, xP, Jinv, M_full, M_full_reg);
  [Fg, Fg_reg] = Rob.gravload2_platform(q, xP, Jinv);
  % Beschleunigungs-Kraft berechnen: Die Regressor-Werte müssen mit dem
  % jeweiligen Dynamikparameter multipliziert werden.
  Fa_reg = NaN(size(Fc_reg));
  for jj = 1:size(M_reg,2)
    M_jj = reshape(M_reg(:,jj), size(M)); % Massenmatrix für diesen Dyn.Par.
    Fa_jj = M_jj*xDDP(Rob.I_EE); % Beschleunigungskraft für DynPar jj
    Fa_reg(:,jj) = Fa_jj; % Regressor-Spalte ergänzen
  end
  % Regressor
  Fs_reg = Fc_reg + Fg_reg + Fa_reg;
end
% Gesamt-Dynamik
Fs = Fg + Fc + M*xDDP(Rob.I_EE);

