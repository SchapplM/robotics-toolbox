% Test the minimum-norm solution in the task-redundant trajectory IK for
% serial robots.
%
% Background: with task redundancy the system J_x(I_EE_Task,:)*qD = xD is
% underdetermined. The Matlab operator "\" returns a basic solution with at
% most rank(J) non-zero entries, i.e. at least one joint velocity is exactly
% zero. Which joint that is depends on the QR column pivoting and changes
% with the start pose. The trajectory IK must therefore form the pseudo
% inverse explicitly, so that the task part of the joint velocity is
% orthogonal to the nullspace and consistent with the nullspace projector.
%
% Example (from Redundanz_RRR.m of the lecture material): planar RRR robot
% moving the end effector along a straight line. The task has 2T0R DOF only,
% the robot has three joints.
%
% Steps:
% * initialize robot S3RRR1, compute the start pose in closed form
% * generate a straight line with trapezoidal velocity profile
% * run the trajectory IK for several nullspace criteria
% * part 1: on Jacobian level, show that "\" freezes one joint over the
%   whole trajectory while pinv does not
% * part 2: check that the trajectory IK does not freeze any joint
% * part 3: check that the IK velocity matches the pinv solution and not
%   the basic solution
% * part 4: check that the end effector follows the reference path

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2026-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

if isempty(which('serroblib_path_init.m'))
  warning('Serial robot model repo is not on the path. Test not executable.');
  return
end

%% User input
% Nullspace criteria to check. The defect is most visible without any
% additional optimization ('none'), since nothing then overlays the
% arbitrary nullspace motion of the basic solution.
% Column 1: name. Column 2: P gain. Column 3: D gain.
Variants = { ...
  'none',     0, 0; ...
  'qlim',     1, 0.3; ...
  'jac_cond', 1, 0.3};

l1 = 6; % length of the first link in m
l2 = 5; % length of the second link in m
l3 = 4; % length of the third link in m (via EE transformation)

P_S = [10.0; 1.0]; % start point of the line in m
P_Z = [ 1.5; 6.7]; % target point of the line in m
v0  = 1.0; % path velocity in m/s

% Start pose: the first link is horizontal, which fixes q1. The elbow points
% upwards.
q1_0 = 0;
elbow_up = true;

%% Initialize robot
SName = 'S3RRR1';
RS = serroblib_create_robot_class(SName);

% Kinematic parameters: a2 and a3 are the first two lengths, the third one
% is covered by the EE transformation.
pkin = zeros(length(RS.pkin_names),1);
pkin(strcmp(RS.pkin_names, 'a2')) = l1;
pkin(strcmp(RS.pkin_names, 'a3')) = l2;
RS.update_mdh(pkin);
RS.update_EE([l3;0;0]);

% Set the limits. Without a parameter set from the database they are NaN or
% infinite, but the IK enforces them by default.
RS.qlim = repmat([-2*pi, 2*pi], RS.NQJ, 1);
RS.qDlim = repmat([-4*pi, 4*pi], RS.NQJ, 1);
RS.qDDlim = repmat([-100, 100], RS.NQJ, 1);

% Use compiled functions (much faster). Missing files are generated.
serroblib_update_template_functions({SName});
RS.fill_fcn_handles(true, true);

% Task DOF 2T0R: only the EE position is prescribed, the orientation is
% free. This makes the robot task redundant.
I_EE_Task = logical([1 1 0 0 0 0]);
RS.update_EE_FG([], I_EE_Task);
assert(sum(I_EE_Task) < RS.NQJ, 'Test case is not task redundant');

%% Determine the start pose
% With q1 given, a planar 2R problem from joint 2 to the end effector
% remains, which can be solved in closed form.
r_2S = P_S - l1*[cos(q1_0); sin(q1_0)];
d_2S = norm(r_2S);
assert(d_2S <= l2+l3 && d_2S >= abs(l2-l3), ...
  'P_S is not reachable with the given q1 = %1.1f deg', q1_0*180/pi);
q3_0 = acos((d_2S^2 - l2^2 - l3^2)/(2*l2*l3));
if elbow_up, q3_0 = -q3_0; end
q2_0 = atan2(r_2S(2), r_2S(1)) - ...
  atan2(l3*sin(q3_0), l2 + l3*cos(q3_0)) - q1_0;
q0 = [q1_0; q2_0; q3_0];

T_0_E = RS.fkineEE(q0);
assert(norm(T_0_E(1:2,4) - P_S) < 1e-10, ...
  'Start pose does not reach the start point P_S');

%% Generate the trajectory (straight line with constant velocity)
x0 = RS.t2x(RS.fkineEE(q0));
XL = x0';
XL(2,:) = XL(1,:); XL(2,1:2) = P_Z';
[X, XD, XDD, T] = traj_trapez2_multipoint(XL, v0, 1e-1, 1e-2, 1e-3, 0);
nt = length(T);
% Time steps with motion. At standstill (start/end) qD is trivially zero and
% does not distinguish the two solutions.
I_move = any(abs(XD(:,I_EE_Task)) > 1e-10, 2);
assert(sum(I_move) > 0.5*nt, 'Trajectory has too few time steps with motion');
fprintf(['Trajectory: %d time steps (%d with motion), duration %1.2f s, ', ...
  'path length %1.2f m\n'], nt, sum(I_move), T(end), norm(P_Z-P_S));

%% Run the trajectory IK for all variants
nv = size(Variants,1);
Q_all = cell(nv,1); QD_all = cell(nv,1);
for i = 1:nv
  [crit, wnP, wnD] = Variants{i,:};
  s_traj = struct('I_EE', I_EE_Task, 'wn', zeros(RS.idx_ik_length.wntraj,1));
  switch crit
    case 'none'
      % no additional optimization
    case 'qlim'
      s_traj.wn(RS.idx_iktraj_wnP.qlim_par) = wnP;
      s_traj.wn(RS.idx_iktraj_wnD.qlim_par) = wnD;
    case 'jac_cond'
      s_traj.wn(RS.idx_iktraj_wnP.jac_cond) = wnP;
      s_traj.wn(RS.idx_iktraj_wnD.jac_cond) = wnD;
    otherwise
      error('Unknown nullspace criterion "%s"', crit);
  end
  [Q, QD, ~, PHI] = RS.invkin2_traj(X, XD, XDD, T, q0, s_traj);
  assert(all(abs(PHI(:)) < 1e-6), ...
    'Trajectory IK for criterion "%s" failed (max|Phi|=%1.1e)', ...
    crit, max(abs(PHI(:))));
  Q_all{i} = Q; QD_all{i} = QD;
end
fprintf('Trajectory IK computed for %d variants.\n', nv);

%% Part 1: basic solution ("\") vs. minimum-norm solution (pinv)
% Both solutions are formed directly from the analytic Jacobian for the
% joint trajectory of the variant without nullspace optimization. The
% comparison is therefore independent of the IK implementation.
Q = Q_all{1};
QD_bs = NaN(nt, RS.NQJ); % basic solution from the operator "\"
QD_pi = NaN(nt, RS.NQJ); % minimum-norm solution from pinv
for k = 1:nt
  J_x = RS.jacobia(Q(k,:)');
  J_task = J_x(I_EE_Task,:); % 2x3: underdetermined system
  assert(size(J_task,1) < size(J_task,2), 'System is not underdetermined');
  xD_k = XD(k,I_EE_Task)';
  QD_bs(k,:) = (J_task \ xD_k)';
  QD_pi(k,:) = (pinv(J_task) * xD_k)';
  % Both solutions have to fulfil the task exactly. They only differ by a
  % component in the nullspace.
  assert(all(abs(J_task*QD_bs(k,:)' - xD_k) < 1e-10), ...
    'Basic solution does not fulfil the task (k=%d)', k);
  assert(all(abs(J_task*QD_pi(k,:)' - xD_k) < 1e-10), ...
    'pinv solution does not fulfil the task (k=%d)', k);
end

% The basic solution sets at least one joint velocity to exactly zero in
% every time step (at most rank(J_task)=2 non-zero entries).
n_null_bs = sum(QD_bs(I_move,:) == 0, 2);
assert(all(n_null_bs >= RS.NQJ - sum(I_EE_Task)), ...
  ['Expectation not met: the basic solution of "\\" should have at least ', ...
   '%d joint velocity/velocities exactly zero in every time step. ', ...
   'Minimum was %d.'], RS.NQJ - sum(I_EE_Task), min(n_null_bs));

% It is always the same joint, so it stands still over the whole trajectory
% - exactly the effect that pinv is supposed to avoid.
I_fix_bs = all(QD_bs(I_move,:) == 0, 1);
assert(any(I_fix_bs), ...
  ['Expectation not met: with the basic solution of "\\" at least one ', ...
   'joint should stand still over the whole trajectory.']);
fprintf(['Part 1: basic solution ("\\") freezes joint %sover the whole ', ...
  'trajectory (share of qD==0 per joint: %s %%).\n'], ...
  sprintf('%d ', find(I_fix_bs)), ...
  mat2str(100*sum(QD_bs(I_move,:)==0,1)/sum(I_move), 3));

% The minimum-norm solution does not freeze any joint.
I_fix_pi = all(QD_pi(I_move,:) == 0, 1);
assert(~any(I_fix_pi), ...
  ['Expectation not met: the pinv solution must not freeze any joint over ', ...
   'the whole trajectory. Affected: %s'], mat2str(find(I_fix_pi)));
fprintf(['Part 1: pinv solution does not freeze any joint ', ...
  '(share of qD==0 per joint: %s %%).\n'], ...
  mat2str(100*sum(QD_pi(I_move,:)==0,1)/sum(I_move), 3));

% By definition the norm of the pinv solution is never larger than the one
% of the basic solution (both fulfil the same task).
norm_bs = sqrt(sum(QD_bs(I_move,:).^2, 2));
norm_pi = sqrt(sum(QD_pi(I_move,:).^2, 2));
assert(all(norm_pi <= norm_bs + 1e-10), ...
  'Norm of the pinv solution is larger than the one of the basic solution');
fprintf(['Part 1: norm of the pinv solution is %1.1f %% of the basic ', ...
  'solution on average (minimum %1.1f %%).\n'], ...
  100*mean(norm_pi./norm_bs), 100*min(norm_pi./norm_bs));

%% Part 2: the trajectory IK must not freeze any joint
for i = 1:nv
  crit = Variants{i,1};
  QD = QD_all{i};
  I_fix = all(QD(I_move,:) == 0, 1);
  assert(~any(I_fix), ...
    ['Criterion "%s": joint(s) %sdo not move over the whole trajectory. ', ...
     'Likely cause: the trajectory IK uses the operator "\\" instead of ', ...
     'pinv for the underdetermined system.'], crit, sprintf('%d ', find(I_fix)));
  % In addition, every joint has to move noticeably over the trajectory.
  jointrange = max(Q_all{i}) - min(Q_all{i});
  assert(all(jointrange > 1e-3), ...
    'Criterion "%s": joint(s) %sbarely move (range %s deg).', crit, ...
    sprintf('%d ', find(jointrange <= 1e-3)), mat2str(jointrange*180/pi, 3));
  fprintf(['Part 2: criterion "%s" ok (range per joint: %s deg, ', ...
    'share of qD==0: %s %%).\n'], crit, mat2str(jointrange*180/pi, 3), ...
    mat2str(100*sum(QD(I_move,:)==0,1)/sum(I_move), 3));
end

%% Part 3: the IK velocity has to match the pinv solution
% Without nullspace optimization (variant 1) the joint velocity consists of
% the task part only (plus a correction term for the linearization error).
% It therefore has to match the minimum-norm solution and to differ clearly
% from the basic solution.
QD = QD_all{1};
err_pi = max(max(abs(QD(I_move,:) - QD_pi(I_move,:))));
err_bs = max(max(abs(QD(I_move,:) - QD_bs(I_move,:))));
qD_max = max(max(abs(QD(I_move,:))));
assert(err_pi < 1e-3*qD_max, ...
  ['Joint velocity of the IK does not match the minimum-norm solution ', ...
   '(deviation %1.1e, limit %1.1e).'], err_pi, 1e-3*qD_max);
assert(err_bs > 1e-2*qD_max, ...
  ['Joint velocity of the IK matches the basic solution of "\\". The ', ...
   'change to pinv is apparently not effective (deviation %1.1e).'], err_bs);
fprintf(['Part 3: IK velocity matches the pinv solution (max. deviation ', ...
  '%1.1e) and differs from the basic solution (max. deviation %1.1e). ', ...
  'Maximum joint velocity %1.3f rad/s.\n'], err_pi, err_bs, qD_max);

%% Part 4: consistency of the path
for i = 1:nv
  crit = Variants{i,1};
  X_ist = RS.fkineEE2_traj(Q_all{i});
  delta_x = X_ist(:,1:2) - X(:,1:2);
  assert(max(abs(delta_x(:))) < 1e-6, ...
    'Criterion "%s": path deviation %1.1e m too large', crit, max(abs(delta_x(:))));
  fprintf('Part 4: criterion "%s" - max. path deviation %1.1e m.\n', ...
    crit, max(abs(delta_x(:))));
end

fprintf('Test %s completed successfully.\n', mfilename);
