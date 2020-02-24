% Berechnung der vollständigen Subsystem-PKM-Massenmatrix (bezogen auf
% Plattform-Koordinaten x). Die Matrix dient zur Berechnung der
% Massenmatrix und der Coriolis-Kräfte
%
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS (nicht:
%   Plattform-Pose xP)
% Jinv [N x Nx] (optional zur Rechenersparnis)
%   Vollständige Inverse Jacobi-Matrix der PKM (bezogen auf alle N aktiven
%   und passiven Gelenke und die bewegliche EE-Koordinaten xE)
% 
% Ausgabe:
% M_full
%   Vollständige Massenmatrix der PKM (für alle Subsysteme, Kräfte und
%   Beschleunigungen in x-Koordinaten)
% M_full_reg
%   Regressormatrix zur zweiten Ausgabe (M_full). Matrix wird 3D gestapelt.
%   (Dimension 1 und 2 sind die ursprüngliche Massenmatrix, Dim. 3 sind die
%   Dynamikparameter.
%   Das zweite Ausgabeargument ist nur zulässig, wenn DynPar.mode=4 ist

% Quelle:
% [DT09] Do Thanh, T. et al: On the inverse dynamics problem of general
% parallel robots, ICM 2009
% [Sriram2019_S876] Sriram, R.: Implementation of a Structurally Independant
% Dynamics Modelling of Parallel Kinematic Machines based on Full Kinematic
% Constraints (Studienarbeit bei Moritz Schappler, 2019).

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [M_full, M_full_reg] = inertia2_platform_full(Rob, q, xE, Jinv)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/inertia2_platform: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/inertia2_platform: xE muss 6x1 sein');
if nargin == 4
  assert(isreal(Jinv) && all(size(Jinv) == [Rob.NJ sum(Rob.I_EE)]), ...
    'ParRob/inertia2_platform: Jinv muss %dx%d sein', Rob.NJ, sum(Rob.I_EE));
end
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
% M_full = NaN((NLEG+1)*NLEG, (NLEG+1)*NLEG);
M_full = NaN((Rob.Leg(1).NL)*NLEG, (Rob.Leg(1).NL)*NLEG);
if nargout == 2 % Ausgabe der Regressormatrizen
  if Rob.DynPar.mode == 3
    M_full_reg = zeros((NLEG+1)*NLEG, (NLEG+1)*NLEG, length(Rob.DynPar.ipv_n1s));
  elseif Rob.DynPar.mode == 4
    M_full_reg = zeros((NLEG+1)*NLEG, (NLEG+1)*NLEG, length(Rob.DynPar.mpv_n1s));
  else
    error('Ausgabe des Regressors mit Dynamik-Modus %d nicht möglich', Rob.DynPar.mode);
  end
end

%% Starrkörper-Dynamik der Plattform
if Rob.DynPar.mode == 2
  M_plf_full = rigidbody_inertiaB_floatb_eulxyz_slag_vp2_mex(xE(4:6), m_P, mrSges, Ifges);
else
  % Regressor-Matrix für allgemeine Starrkörper (nutze nicht-symmetrische
  % Form der Ausgabe, obwohl die Matrix symm. ist. Macht Rechnung unten
  % einfacher.
  [~,Mvec_plf_reg] = rigidbody_inertiaB_floatb_eulxyz_reg2_slag_vp_mex(xE(4:6));
  if Rob.DynPar.mode == 3
    delta = Rob.DynPar.ipv_n1s(end-sum(Rob.I_platform_dynpar)+1:end);
  else
    delta = Rob.DynPar.mpv_n1s(end-sum(Rob.I_platform_dynpar)+1:end);
  end
  Mvec_plf = Mvec_plf_reg(:,Rob.I_platform_dynpar) * delta;
  M_plf_full = reshape(Mvec_plf, 6, 6);
end
M_plf_red = M_plf_full (Rob.I_EE, Rob.I_EE);

%% Berechnung der Projektion
% Massenmatrix aller Beinketten berechnen
for i = 1:NLEG
  q_i = q(Rob.I1J_LEG(i):Rob.I2J_LEG(i));
  if Rob.DynPar.mode == 2
    Mq_Leg = Rob.Leg(i).inertia(q_i);
  else
    [~,Mvec_Leg_reg] = Rob.Leg(i).inertia(q_i);
    if Rob.DynPar.mode == 3
      % Regressorform mit Inertialparametern
      Mvec_Leg = Mvec_Leg_reg * Rob.DynPar.ipv_n1s(1:end-sum(Rob.I_platform_dynpar));
    else
      % Regressorform mit Minimalparametern
      if Rob.DynPar.mode == 3
        Mvec_Leg = Mvec_Leg_reg * Rob.DynPar.ipv_n1s(1:end-sum(Rob.I_platform_dynpar));
      else
        Mvec_Leg = Mvec_Leg_reg * Rob.DynPar.mpv_n1s(1:end-sum(Rob.I_platform_dynpar));
      end
    end
    Mq_Leg = vec2symmat(Mvec_Leg);
  end
  M_full((i-1)*Rob.Leg(i).NJ+1:Rob.Leg(i).NJ*i,1:NJ+NLEG) = [zeros(Rob.Leg(i).NJ,(Rob.Leg(i).NJ*(i-1))),Mq_Leg, zeros(Rob.Leg(i).NJ,NJ+NLEG -(Rob.Leg(i).NJ*(i)))];
  if nargout == 2
    for jj = 1:size(Mvec_Leg_reg,2)
      Mq_Leg_reg_jj = vec2symmat(Mvec_Leg_reg(:,jj));
      M_full_reg((i-1)*NLEG+1:NLEG*i,1:NJ+NLEG,jj) = [zeros(NLEG,(NLEG*(i-1))),Mq_Leg_reg_jj, zeros(NLEG,NJ -(NLEG*(i-1)))];
    end
  end
end
% Massenmatrix-Terme der Plattform
M_full(NJ+1:end,1:NJ+NLEG) = [zeros(NLEG,NJ),M_plf_red];
if nargout == 2 % Ausgabe der Regressormatrizen
  % Wiederhole die gleiche Zuweisung, aber für alle Dynamikparameter
  % einzeln (die Spalten in der Starrkörper-Regressormatrix sind andere als
  % in der PKM-Massenmatrix, daher andere Indizes).
  jj = size(M_full_reg,3)-sum(Rob.I_platform_dynpar);
  for kk = find(Rob.I_platform_dynpar) % Index über Dynamikparameter des allgemeinen Starrkörpers (für Plattform)
    jj = jj + 1; % Index in den Dynamikparametern der PKM
    M_plf_full_jj = reshape(Mvec_plf_reg(:,kk), 6, 6);
    M_plf_red_jj = M_plf_full_jj (Rob.I_EE, Rob.I_EE);
    M_full_reg(NJ+1:end,1:NJ+NLEG,jj) = [zeros(NLEG,NJ),M_plf_red_jj];
  end
end
