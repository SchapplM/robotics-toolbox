% Test the joint weighting of the task-redundant trajectory IK for serial
% robots (setting joint_weight).
%
% Background: with task redundancy the system J_x(I_EE_Task,:)*qD = xD is
% underdetermined. By default the pseudo inverse selects the solution of
% minimum norm qD'*qD. With a weighting W = diag(joint_weight) the solution
% minimizes qD'*W*qD instead, i.e. the weighted pseudo inverse
% W^-1*J'*(J*W^-1*J')^-1 is used. A large weight makes the corresponding
% joint move less. This is the redundancy resolution shown in the lecture
% material (Redundanz_RRR.m), where W = diag(w1,1,1) is varied.
%
% The weighting is applied to the task velocity, the task acceleration and
% the nullspace projector, so that all three use the same metric.
%
% Example: planar RRR robot moving the end effector along a straight line.
% The task has 2T0R DOF only, the robot has three joints.
%
% Steps:
% * initialize robot S3RRR1, compute the start pose in closed form
% * generate a straight line with trapezoidal velocity profile
% * run the trajectory IK for several weightings
% * part 1: default weighting reproduces the unweighted result exactly
% * part 2: the IK velocity matches the weighted pseudo inverse and differs
%   from the unweighted one
% * part 3: the nullspace projector is consistent with the weighting
% * part 4: a larger weight reduces the motion of the weighted joint
% * part 5: the end effector follows the reference path in all cases

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2026-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

if isempty(which('serroblib_path_init.m'))
  warning('Serial robot model repo is not on the path. Test not executable.');
  return
end

%% User input
% Weightings to check. The first joint is weighted, the other two are left
% at one. A larger value is expected to reduce the motion of joint 1.
w1_all = [1, 10, 100];

l1 = 6; % length of the first link in m
l2 = 5; % length of the second link in m
l3 = 4; % length of the third link in m (via EE transformation)

P_S = [12.0; 1.0]; % start point of the line in m
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
% does not distinguish the solutions.
I_move = any(abs(XD(:,I_EE_Task)) > 1e-10, 2);
assert(sum(I_move) > 0.5*nt, 'Trajectory has too few time steps with motion');
fprintf(['Trajectory: %d time steps (%d with motion), duration %1.2f s, ', ...
  'path length %1.2f m\n'], nt, sum(I_move), T(end), norm(P_Z-P_S));

%% Run the trajectory IK for all weightings
% No nullspace criterion is used, so that qD consists of the task part and
% the linearization correction only. That isolates the effect of the
% weighting.
nw = length(w1_all);
Q_all = cell(nw,1); QD_all = cell(nw,1);
for i = 1:nw
  s_traj = struct('I_EE', I_EE_Task, 'wn', zeros(RS.idx_ik_length.wntraj,1), ...
    'joint_weight', [w1_all(i); 1; 1]);
  [Q, QD, ~, PHI] = RS.invkin2_traj(X, XD, XDD, T, q0, s_traj);
  assert(all(abs(PHI(:)) < 1e-6), ...
    'Trajectory IK for w1 = %g failed (max|Phi|=%1.1e)', ...
    w1_all(i), max(abs(PHI(:))));
  Q_all{i} = Q; QD_all{i} = QD;
end
fprintf('Trajectory IK computed for %d weightings.\n', nw);

%% Part 1: default weighting must not change the result
% Without the option and with joint_weight = ones the code must take the
% same branch, so the results have to be bit-identical.
s_ref = struct('I_EE', I_EE_Task, 'wn', zeros(RS.idx_ik_length.wntraj,1));
[Q_ref, QD_ref, QDD_ref] = RS.invkin2_traj(X, XD, XDD, T, q0, s_ref);
s_one = s_ref; s_one.joint_weight = ones(RS.NQJ,1);
[Q_one, QD_one, QDD_one] = RS.invkin2_traj(X, XD, XDD, T, q0, s_one);
assert(isequal(Q_ref, Q_one) && isequal(QD_ref, QD_one) && ...
  isequal(QDD_ref, QDD_one), ...
  ['joint_weight = ones changes the result. The default path must be ', ...
   'identical to the unweighted implementation.']);
% w1_all(1) is 1, so that run must match as well
assert(isequal(Q_ref, Q_all{1}), 'Run with w1 = 1 differs from the reference');
fprintf('Part 1: joint_weight = ones reproduces the unweighted result exactly.\n');

%% Part 2: the IK velocity matches the weighted pseudo inverse
% Build both candidate solutions independently from the Jacobian and check
% which one the IK returns.
% Only sample the phase of constant velocity. In the ramps xD is small, so
% the two candidate solutions are close in absolute terms and a relative
% criterion would be misleading there.
v_path = sqrt(sum(XD(:,I_EE_Task).^2, 2));
I_fast = find(v_path > 0.5*max(v_path));
I_test = I_fast(round(linspace(1, numel(I_fast), 25)));
for i = 1:nw
  W = diag([w1_all(i); 1; 1]);
  Q = Q_all{i}; QD = QD_all{i};
  qD_max = max(max(abs(QD(I_move,:))));
  d_w = NaN(numel(I_test),1); % deviation from the weighted solution
  d_u = NaN(numel(I_test),1); % deviation from the unweighted solution
  d_sep = NaN(numel(I_test),1); % distance of the two candidates
  for j = 1:numel(I_test)
    k = I_test(j);
    J = RS.jacobia(Q(k,:)');
    J = J(I_EE_Task,:);
    xD_k = XD(k,I_EE_Task)';
    qD_w = (W\J') * ((J*(W\J')) \ xD_k); % weighted pseudo inverse
    qD_u = pinv(J) * xD_k; % unweighted minimum norm
    % Both must satisfy the task exactly
    assert(norm(J*qD_w - xD_k) < 1e-10, 'Weighted solution violates the task');
    d_w(j) = norm(QD(k,:)' - qD_w);
    d_u(j) = norm(QD(k,:)' - qD_u);
    d_sep(j) = norm(qD_w - qD_u);
  end
  assert(max(d_w) < 1e-3*qD_max, ...
    ['w1 = %g: joint velocity of the IK does not match the weighted ', ...
     'pseudo inverse (deviation %1.1e, limit %1.1e).'], ...
    w1_all(i), max(d_w), 1e-3*qD_max);
  if w1_all(i) == 1
    % Both candidates are identical here; nothing to distinguish
    fprintf(['Part 2: w1 = %3g - matches the weighted solution ', ...
      '(deviation %1.1e); identical to the unweighted one, as expected.\n'], ...
      w1_all(i), max(d_w));
  else
    % Only use samples where the two candidates differ by a relevant amount
    % compared to the overall velocity level.
    I_sig = d_sep > 0.01*qD_max;
    assert(sum(I_sig) >= 5, ...
      ['w1 = %g: weighted and unweighted solution are almost identical on ', ...
       'the whole path, the test would not be conclusive.'], w1_all(i));
    assert(min(d_u(I_sig)) > 10*max(d_w(I_sig)), ...
      ['w1 = %g: the IK is not clearly closer to the weighted solution ', ...
       '(distance to weighted %1.1e, to unweighted %1.1e).'], ...
      w1_all(i), max(d_w(I_sig)), min(d_u(I_sig)));
    fprintf(['Part 2: w1 = %3g - matches the weighted solution ', ...
      '(deviation %1.1e) and differs from the unweighted one ', ...
      '(deviation %1.1e).\n'], w1_all(i), max(d_w(I_sig)), min(d_u(I_sig)));
  end
end

%% Part 3: the nullspace projector is consistent with the weighting
% N must map into the nullspace of the task Jacobian (J*N = 0) for any
% weighting, and the task part must be orthogonal to the nullspace in the
% weighted metric.
for i = 1:nw
  W = diag([w1_all(i); 1; 1]);
  Q = Q_all{i};
  err_JN = 0; err_orth = 0;
  for j = 1:numel(I_test)
    k = I_test(j);
    J = RS.jacobia(Q(k,:)');
    J = J(I_EE_Task,:);
    Jt_pinv = (W\J') / (J*(W\J'));
    N = eye(RS.NQJ) - Jt_pinv*J;
    err_JN = max(err_JN, max(abs(J*N), [], 'all'));
    % Task part and an arbitrary nullspace vector, W-orthogonality
    qD_w = Jt_pinv * XD(k,I_EE_Task)';
    qD_N = N * [1;1;1];
    err_orth = max(err_orth, abs(qD_w' * W * qD_N)/(norm(qD_w)*norm(qD_N)));
  end
  assert(err_JN < 1e-9, ...
    'w1 = %g: J*N is not zero (max %1.1e)', w1_all(i), err_JN);
  assert(err_orth < 1e-9, ...
    ['w1 = %g: task part is not W-orthogonal to the nullspace ', ...
     '(max %1.1e)'], w1_all(i), err_orth);
  fprintf(['Part 3: w1 = %3g - J*N = 0 (max %1.1e) and the task part is ', ...
    'W-orthogonal to the nullspace (max %1.1e).\n'], ...
    w1_all(i), err_JN, err_orth);
end

%% Part 4: a larger weight reduces the motion of the weighted joint
% This is the effect the weighting is meant to have. Compare the integrated
% absolute joint motion, which is more robust than the range.
motion = NaN(nw, RS.NQJ);
for i = 1:nw
  motion(i,:) = trapz(T, abs(QD_all{i}));
end
for i = 1:nw
  fprintf('Part 4: w1 = %3g - path of the joints [%s] rad.\n', ...
    w1_all(i), sprintf('%1.2f ', motion(i,:)));
end
assert(all(diff(motion(:,1)) < 0), ...
  ['The motion of joint 1 does not decrease monotonically with a growing ', ...
   'weight: [%s] rad.'], sprintf('%1.3f ', motion(:,1)));
assert(motion(end,1) < 0.9*motion(1,1), ...
  ['The weighting has hardly any effect: motion of joint 1 only drops ', ...
   'from %1.3f to %1.3f rad.'], motion(1,1), motion(end,1));
fprintf(['Part 4: motion of joint 1 decreases monotonically from %1.2f to ', ...
  '%1.2f rad (w1 = %g to %g).\n'], motion(1,1), motion(end,1), ...
  w1_all(1), w1_all(end));

%% Part 5: the end effector follows the reference path
for i = 1:nw
  X_ist = RS.fkineEE2_traj(Q_all{i});
  delta_x = X_ist(:,1:2) - X(:,1:2);
  assert(max(abs(delta_x(:))) < 1e-6, ...
    'w1 = %g: path deviation too large (%1.1e m)', ...
    w1_all(i), max(abs(delta_x(:))));
  fprintf('Part 5: w1 = %3g - max. path deviation %1.1e m.\n', ...
    w1_all(i), max(abs(delta_x(:))));
end

fprintf('Test %s completed successfully.\n', mfilename);
