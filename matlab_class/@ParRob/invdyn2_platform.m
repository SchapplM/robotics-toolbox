% Berechnung der Plattformkraft aufgrund der Effekte der inversen Dynamik
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
% 
% Ausgabe:
% F
%   Invers-Dynamik-Kräfte (bezogen auf Endeffektor-Plattform der PKM)
%   (enthalten alle dynamischen Effekte: Grav., Coriolis, Beschleunigung)
%   Die Momente sind kartesische Momente bezogen auf das Basis-KS der PKM.
% F_reg
%   Regressormatrix zur Invers-Dynamik-Kraft

% Quelle:
% [DT09] Do Thanh, T. et al: On the inverse dynamics problem of general
% parallel robots, ICM 2009
% [Sriram2019_S876] Sriram, R.: Implementation of a Structurally Independant
% Dynamics Modelling of Parallel Kinematic Machines based on Full Kinematic
% Constraints (Studienarbeit bei Moritz Schappler, 2019).

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [F, F_reg] = invdyn2_platform(Rob, q, qD, qDD, xE, xDE, xDDE, Jinv, JinvD)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/invdyn2_platform: q muss %dx1 sein', Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/invdyn2_platform: qD muss %dx1 sein', Rob.NJ);
assert(isreal(qDD) && all(size(qDD) == [Rob.NJ 1]), ...
  'ParRob/invdyn2_platform: qDD muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/invdyn2_platform: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/invdyn2_platform: xDE muss 6x1 sein');
assert(isreal(xDDE) && all(size(xDDE) == [6 1]), ...
  'ParRob/invdyn2_platform: xDDE muss 6x1 sein');
if nargin >= 8
  assert(isreal(Jinv) && all(size(Jinv) == [Rob.NJ sum(Rob.I_EE)]), ...
    'ParRob/coriolisvec2_platform: Jinv muss %dx%d sein', Rob.NJ, sum(Rob.I_EE));
end
if nargin == 9
  assert(isreal(JinvD) && all(size(JinvD) == [Rob.NJ sum(Rob.I_EE)]), ...
    'ParRob/coriolisvec2_platform: JinvD muss %dx%d sein', Rob.NJ, sum(Rob.I_EE));
end
%% Projektionsmatrizen
if nargin < 9
  G_q = Rob.constr1grad_q(q, xE);
  G_x = Rob.constr1grad_x(q, xE);
  Jinv = - G_q \ G_x; % Siehe: ParRob/jacobi_qa_x
end
if nargin < 9
  G_qd = Rob.constr1gradD_q(q, qD, xE, xDE);
  G_xd = Rob.constr1gradD_x(q, qD, xE, xDE);
  JinvD = G_q\G_qd/G_q*G_x - G_q\G_xd; % Siehe: ParRob/jacobiD_qa_x
end
%% Funktionsaufrufe
if nargout == 1
  M = Rob.inertia2_platform(q, xE, Jinv);
  Fc = Rob.coriolisvec2_platform(q, qD, xE, xDE, Jinv, JinvD);
  Fg = Rob.gravload2_platform(q, xE, Jinv);
else
  [M, ~, M_reg] = Rob.inertia2_platform(q, xE, Jinv);
  [Fc, Fc_reg] = Rob.coriolisvec2_platform(q, qD, xE, xDE, Jinv, JinvD);
  [Fg, Fg_reg] = Rob.gravload2_platform(q, xE, Jinv);
  % Beschleunigungs-Kraft berechnen: Die Regressor-Werte müssen mit dem
  % jeweiligen Dynamikparameter multipliziert werden.
  Fa_reg = NaN(size(Fc_reg));
  for jj = 1:size(M_reg,2)
    M_jj = reshape(M_reg(:,jj), size(M)); % Massenmatrix für diesen Dyn.Par.
    Fa_jj = M_jj*xDDE(Rob.I_EE); % Beschleunigungskraft für DynPar jj
    Fa_reg(:,jj) = Fa_jj; % Regressor-Spalte ergänzen
  end
  % Regressor
  F_reg = Fc_reg + Fg_reg + Fa_reg;
end
F = Fg + Fc + M*xDDE(Rob.I_EE);