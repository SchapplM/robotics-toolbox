% Teste die in der ParRob und RobBase gespeicherten
% Koordinatentransformationen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

%% Initialisierung des 6UPS-Roboters
% Kinematik ist eigentlich nicht wichtig für folgende Tests
if isempty(which('parroblib_path_init.m'))
  warning('Repo mit parallelen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end
RP = parroblib_create_robot_class('P6RRPRRR14V3G1P4A1', '', 0.5, 0.2);
RP.fill_fcn_handles(false);
RP.align_platform_coupling(4, [0.2;0.1]);

%% Beispiel-Trajektorie
X0 = [ [0;0;0.5]; [0;0;0]*pi/180 ];
% Trajektorie mit beliebigen Bewegungen der Plattform
XL = [X0'+1*[[ 0.0, 0.0, 0.0], [ 0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.1, 0.0, 0.0], [ 0.0, 0.0, 0.0]]; ...
      X0'+1*[[-0.1, 0.0, 0.0], [ 0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.1, 0.0], [ 0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0,-0.1, 0.0], [ 0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.1], [ 0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0,-0.1], [ 0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.1, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [-0.1, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.0, 0.1, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.0,-0.1, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.0, 0.0, 0.1]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.0, 0.0,-0.1]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.0, 0.0, 0.3]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.0, 0.3, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.3, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.2,-0.1, 0.3], [ 0.3, 0.2, 0.1]]; ...
      X0'+1*[[-0.1, 0.2,-0.1], [ 0.5,-0.2,-0.2]]; ...
      X0'+1*[[ 0.2, 0.3, 0.2], [ 0.2, 0.1, 0.3]]];
XL = [XL; XL(1,:)]; % Rückfahrt zurück zum Startpunkt.
[X_t,XD_t,XDD_t,T] = traj_trapez2_multipoint(XL, 1, 0.1, 0.01, 1e-3, 1e-1);

%% Teste Umrechnung zwischen Plattform- und EE-Koordinaten
X_E = X_t;
XD_E = XD_t;
XDD_E = XDD_t;
[X_P, XD_P, XDD_P] = RP.xE2xP_traj(X_E, XD_E, XDD_E);
% Neuen EE festlegen
RP.update_EE(rand(3,1),rand(3,1));
% Geschwindigkeit des neuen EE ausrechnen
[X_E2, XD_E2, XDD_E2] = RP.xP2xE_traj(X_P, XD_P, XDD_P);
% Zurückrechnen auf Plattform
[X_P2, XD_P2, XDD_P2] = RP.xE2xP_traj(X_E2, XD_E2, XDD_E2);
% Prüfen, ob durch Hin- und Herrechnen ein Fehler passiert ist
Test=[X_P;XD_P;XDD_P]-[X_P2;XD_P2;XDD_P2];
assert(all(abs(Test(:))<1e-10), 'Umrechnung Plattform-EE mit xP2xE / xE2xP stimmt nicht.');
fprintf('Klassen-Methode xP2xE_traj und xE2xP_traj in sich getestet\n');
%% Teste Umrechnung der Trajektorie zwischen Basis und Welt
Traj_W = struct('T', T, 'X', X_t, 'XD', XD_t, 'XDD', XDD_t);
for i = 0:4 % drei Fälle durchgehen: mit/ohne Rotation und 180° um x
  fprintf('Prüfe Transformation der Trajektorie, Fall %d\n', i);
  % Beliebige Transformation der Basis einstellen
  if i == 0
    % Keine Transformation durchführen
    RP.update_base(zeros(3,1), zeros(3,1));
  elseif i == 1
    RP.update_base(rand(3,1), rand(3,1));
  elseif i == 2
    RP.update_base(rand(3,1), zeros(3,1)); % Keine Rotation der Basis
  elseif i == 3
    RP.update_base(rand(3,1), [pi;0;0]); % Nur Drehung um x (Deckenmontage)
  elseif i == 4
    RP.update_base(rand(3,1), [0;0;pi/2]); % Nur Drehung um z
  else
    error('Fall nicht definiert');
  end
  T_W_B1 = RP.T_W_0;
  % Trajektorien-Eckpunkte in Basis-KS umrechnen
  Traj_B1 = RP.transform_traj(Traj_W);
  % De-Normalisieren, damit besser gegen Integration von XD vergleichbar
  Traj_B1.X(:,4:6) = denormalize_angle_traj(Traj_B1.X(:,4:6));
  % Inverse Rechnung anstellen. Rechenweg 1.
  Traj_B2_v1 = RP.transform_traj(Traj_B1, false); % über Argument umschalten
  % Rechenweg 2: Transformiere Welt-KS (alte Implementierung)
  T_W_B2 = invtr(T_W_B1);
  RP.update_base(T_W_B2(1:3,4), r2eul(T_W_B2(1:3,1:3), RP.phiconv_W_0));
  Traj_B2_v2 = RP.transform_traj(Traj_B1);
  % Vergleiche beide Rechenwege
  test_X_v12 = Traj_B2_v1.X - Traj_B2_v2.X;
  test_XD_v12 = Traj_B2_v1.XD - Traj_B2_v2.XD;
  test_XDD_v12 = Traj_B2_v1.XDD - Traj_B2_v2.XDD;
  assert(all(abs(test_X_v12(:))<1e-10), 'Fehler bei Umrechnung der Positions-Traj_B1.');
  assert(all(abs(test_XD_v12(:))<1e-10), 'Fehler bei Umrechnung der Geschw.-Traj_B1.');
  assert(all(abs(test_XDD_v12(:))<1e-10), 'Fehler bei Umrechnung der Beschl.-Traj_B1.');
  % Testen: B2 ist eigentlich identisch mit W. Trajektorie muss nach
  % zweifacher Transformation wieder die ursprüngliche sein
  test_X = Traj_W.X - Traj_B2_v2.X;
  assert(all(abs(test_X(:))<1e-10), 'Fehler bei Umrechnung der Positions-Traj_B1.');
  test_XD = Traj_W.XD - Traj_B2_v2.XD;
  assert(all(abs(test_XD(:))<1e-10), 'Fehler bei Umrechnung der Geschw.-Traj_B1.');
  test_XDD = Traj_W.XDD - Traj_B2_v2.XDD;
  assert(all(abs(test_XDD(:))<1e-10), 'Fehler bei Umrechnung der Beschl.-Traj_B1.');
  % Prüfe auch die Konsistenz der Trajektorie mit Integration. Bei Euler-
  % Winkeln muss die Trajektorie auch integrierbar sein
  X_numint = repmat(Traj_B1.X(1,:),size(Traj_B1.X,1),1)+cumtrapz(Traj_B1.T, Traj_B1.XD);
  XD_numint = repmat(Traj_B1.XD(1,:),size(Traj_B1.XD,1),1)+cumtrapz(Traj_B1.T, Traj_B1.XDD);
  corrX = diag(corr(X_numint, Traj_B1.X));
  corrX(all(abs(X_numint-Traj_B1.X)<1e-6)) = 1;
  assert(all(corrX>0.98), 'Trajektorie ist nicht konsistent (X-XD)');
  corrXD = diag(corr(XD_numint, Traj_B1.XD));
  corrXD(all(abs(Traj_B1.XD)<1e-3)) = 1;
  assert(all(corrXD>0.98), 'Trajektorie ist nicht konsistent (XD-XDD)');
  continue
  % Debug (für den Fehlerfall bei der Integration+Korrelation)
  figure(1);clf; %#ok<UNRCH>
  for rr = 1:6
    if rr <4, l = 'trans'; else, l = 'rot'; end
    subplot(3,6,sprc2no(3,6,1,rr)); hold on;
    plot(Traj_B1.T, X_numint(:,rr), '-');
    plot(Traj_B1.T, Traj_B1.X(:,rr), '-');
    if rr==6, legend({'int(xD)', 'x'}); end
    grid on; ylabel(sprintf('x %d (%s)', rr, l));
    subplot(3,6,sprc2no(3,6,2,rr)); hold on;
    plot(Traj_B1.T, XD_numint(:,rr), '-');
    plot(Traj_B1.T, Traj_B1.XD(:,rr), '-');
    grid on; ylabel(sprintf('xD %d (%s)', rr, l));
    subplot(3,6,sprc2no(3,6,3,rr)); hold on;
    plot(Traj_B1.T, Traj_B1.XDD(:,rr), '-');
    grid on; ylabel(sprintf('xDD %d (%s)', rr, l));
  end
  linkxaxes
end

fprintf('Klassen-Methode transform_traj in sich getestet\n');
%% Test the wrench transformation between EE and platform frame
% Deterministic setup that does not depend on the state the sections above
% leave behind (random base transformation and random EE transformation).
RP.update_base(zeros(3,1), zeros(3,1));
RP.update_EE([0.1;-0.2;0.3], [0.3;-0.1;0.2]); % offset and rotation between P and E
% EE trajectory belonging to the (unchanged) platform trajectory X_P
X_EW = RP.xP2xE_traj(X_P);
rng(0);

% Check against the definition: the force is the same in both frames and
% only the moment is shifted by the lever arm from P to E.
for i = [1, round(size(X_EW,1)/2), size(X_EW,1)]
  xE_i = X_EW(i,:)';
  w_E_i = randn(6,1);
  w_P_i = RP.wrench_EE2P(w_E_i, xE_i, true);
  T_0_E_i = RP.x2t(xE_i);
  r_0_P_E_i = T_0_E_i(1:3,1:3) * (RP.T_P_E(1:3,1:3)' * RP.T_P_E(1:3,4));
  assert(all(abs(w_P_i(1:3) - w_E_i(1:3)) < 1e-12), ...
    'The force has to be identical in EE and platform frame.');
  assert(all(abs(w_P_i(4:6) - (w_E_i(4:6) + cross(r_0_P_E_i, w_E_i(1:3)))) < 1e-12), ...
    'The moment w.r.t. the platform does not match the lever-arm shift.');
end
fprintf('Class method wrench_EE2P checked against the moment shift\n');

% Round trip EE -> platform -> EE
xE = X_EW(1,:)';
w_E = [1.0; -2.0; 0.5; 0.1; -0.2; 0.3];
w_P = RP.wrench_EE2P(w_E, xE, true);
w_E_back = RP.wrench_EE2P(w_P, xE, false);
assert(all(abs(w_E - w_E_back) < 1e-10), ...
  'Transformation of a wrench between EE and platform frame is not invertible.');
fprintf('Class method wrench_EE2P tested for consistency\n');

W_E = randn(size(X_EW,1), 6);
W_P = RP.wrench_EE2P_traj(W_E, X_EW, true);
W_E_back = RP.wrench_EE2P_traj(W_P, X_EW, false);
assert(all(abs(W_E(:) - W_E_back(:)) < 1e-10), ['Transformation of wrench ', ...
  'trajectories between EE and platform frame is not invertible.']);
% The trajectory version has to be identical to the single-point version
assert(all(abs(W_P(1,:)' - RP.wrench_EE2P(W_E(1,:)', X_EW(1,:)', true)) < 1e-10), ...
  'wrench_EE2P_traj does not match wrench_EE2P.');
fprintf('Class method wrench_EE2P_traj tested for consistency\n');
%% Test the wrench transformation against the Jacobian matrices
% The wrench is the dual quantity of the twist. Transforming the wrench with
% wrench_EE2P therefore has to be consistent with the Jacobian matrices,
% which relate the actuator velocities to the platform resp. EE twist.
% Joint limits only serve to draw a sensible initial value for the IK, see
% ParRob_class_example_6UPS.m
for i = 1:RP.NLEG
  RP.Leg(i).qlim = repmat([-2*pi, 2*pi], RP.Leg(i).NQJ, 1);
  RP.Leg(i).qlim(3,:) = [0.4, 0.7]; % length of the prismatic actuator
end
qlim_pkm = cat(1, RP.Leg.qlim);
q0 = qlim_pkm(:,1)+rand(RP.NJ,1).*(qlim_pkm(:,2)-qlim_pkm(:,1));
q0(RP.I_qa) = 0.5; % start with positive actuator length (configuration must not flip)
% Only look at a few samples of the trajectory. The IK is computed for each
% of them separately and warm-started with the previous solution.
II = round(linspace(1, size(X_EW,1), 20));
for i = II
  xE_i = X_EW(i,:)';
  xP_i = RP.xE2xP(xE_i); % has to belong to xE_i, do not take it from X_P
  [q_i, Phi_i] = RP.invkin1(xE_i, q0);
  assert(all(abs(Phi_i) < 1e-8), sprintf(...
    'Inverse kinematics did not converge for sample %d', i));
  q0 = q_i; % warm start for the next sample
  % Arbitrary actuator velocity and platform wrench. The identities checked
  % below hold for any value, so no velocity IK is necessary.
  qD_a = randn(sum(RP.I_qa),1);
  w_P_i = randn(6,1);
  % Jacobian related to the platform twist instead of the Euler angle rates
  % and the corresponding actuator forces. See ParRob/invdyn_actjoint
  JinvP_i = RP.jacobi_qa_x(q_i, xP_i, true);
  TeulP = [eye(3,3), zeros(3,3); zeros(3,3), euljac(xP_i(4:6), RP.phiconv_W_E)];
  JinvP_qaD_sD = JinvP_i / TeulP;
  tau_i = JinvP_qaD_sD' \ w_P_i;
  % Check that the mapping into the joint space preserves the power
  p_jointspace = tau_i' * qD_a;
  sDP_i = JinvP_qaD_sD \ qD_a;
  p_platform = w_P_i' * sDP_i;
  assert(abs(p_jointspace-p_platform) < 1e-8*max(1,abs(p_jointspace)), ...
    'Power does not match: platform vs. joint space');
  % Same in the EE frame
  JinvE_i = RP.jacobi_qa_x(q_i, xE_i, false);
  TeulE = [eye(3,3), zeros(3,3); zeros(3,3), euljac(xE_i(4:6), RP.phiconv_W_E)];
  JinvE_qaD_sD = JinvE_i / TeulE;
  w_E_via_jac = JinvE_qaD_sD' * tau_i;
  sDE_i = JinvE_qaD_sD \ qD_a;
  p_endeffector = w_E_via_jac' * sDE_i;
  assert(abs(p_jointspace-p_endeffector) < 1e-8*max(1,abs(p_jointspace)), ...
    'Power does not match: EE (via Jacobian) vs. joint space');
  % The direct transformation has to give the same result as the detour via
  % the Jacobian matrices. This is the actual test of the transformation.
  w_P_direct = RP.wrench_EE2P(w_E_via_jac, xE_i, true);
  assert(all(abs(w_P_direct-w_P_i) < 1e-8*max(1,max(abs(w_P_i)))), ['wrench_EE2P ', ...
    '(EE->platform) does not match the transformation via the Jacobian matrices.']);
  w_E_direct = RP.wrench_EE2P(w_P_i, xE_i, false);
  assert(all(abs(w_E_direct-w_E_via_jac) < 1e-8*max(1,max(abs(w_E_via_jac)))), ...
    ['wrench_EE2P (platform->EE) does not match the transformation via the ', ...
    'Jacobian matrices.']);
end
fprintf('Class method wrench_EE2P tested against the Jacobian matrices\n');
