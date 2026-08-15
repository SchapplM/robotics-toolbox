% Test the minimum-norm solution in the trajectory IK for a structurally
% redundant serial robot (KUKA LBR 4+, 7 joints, 3T3R task).
%
% Companion to SerRob_taskred_minnorm_test, which covers *task* redundancy on
% a planar 3R robot. Here the redundancy is *structural*: the robot has seven
% joints for a six-DOF task, so J_x(I_EE,:) is 6x7 and underdetermined even
% though the full 3T3R task is prescribed. This exercises two branches of
% robot_invkin_traj.m.template that the planar test cannot reach:
% * redundant       - from NQJ > sum(I_EE) instead of a reduced task
% * redundant_struct - the ik_solution_min_norm = false branch, which solves
%                      with the structural DOF (Rob_I_EE) and is
%                      underdetermined only for robots like this one
%
% The Matlab operator "\" would return a basic solution here as well, i.e. one
% joint velocity exactly zero over the whole trajectory. The IK has to use the
% pseudo inverse instead.
%
% Steps:
% * initialize the LBR from the model database
% * generate a short Cartesian trajectory (translation and rotation)
% * part 1: compare "\" and pinv on Jacobian level
% * part 2: check that the trajectory IK does not freeze a joint, for both
%   settings of ik_solution_min_norm
% * part 3: check the consistency of Q, QD and QDD by cumtrapz integration

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2026-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

if isempty(which('serroblib_path_init.m'))
  warning('Serial robot model repo is not on the path. Test not executable.');
  return
end

%% User input
SName = 'S7RRRRRRR1';        % LBR type
RName = 'S7RRRRRRR1_LWR4P';  % parameters from the data sheet

q0 = pi/180*[45, -60, 0, 60, 0, -60, 0]'; % start pose (as in the LBR example)
% Cartesian displacement of the trajectory. The rotation is important: it
% produces a noticeable nullspace motion, which is what makes the difference
% between the basic and the minimum-norm solution visible.
dx = [0.12, -0.10, 0.10, 0, 0, 30*pi/180];
vmax = 0.30; % path velocity
Ts   = 1e-3; % sample time

%% Initialize robot
serroblib_update_template_functions({SName});
RS = serroblib_create_robot_class(SName, RName);
RS.fill_fcn_handles(true, true);
% The database model has no acceleration limits. They are not enforced below,
% but have to be finite for the IK.
if any(isnan(RS.qDDlim(:)))
  RS.qDDlim = repmat([-100, 100], RS.NQJ, 1);
end
% Full 3T3R task. The redundancy comes from the seventh joint, not from a
% reduced task.
assert(all(RS.I_EE_Task), 'Expected a 3T3R task');
assert(sum(RS.I_EE_Task) < RS.NQJ, ...
  'Robot %s is not structurally redundant (%d DOF, %d joints)', ...
  SName, sum(RS.I_EE_Task), RS.NQJ);

%% Generate the trajectory
x0 = RS.t2x(RS.fkineEE(q0));
[X, XD, XDD, T] = traj_trapez2_multipoint([x0'; x0'+dx], vmax, 0.3, 0.01, Ts, 0);
nt = length(T);
% Time steps with motion. At standstill qD is trivially zero and does not
% distinguish the two solutions.
I_move = any(abs(XD) > 1e-10, 2);
fprintf('Trajectory: %d time steps (%d with motion), duration %1.2f s\n', ...
  nt, sum(I_move), T(end));

%% Run the trajectory IK for both settings of ik_solution_min_norm
s_traj = struct('I_EE', RS.I_EE_Task, 'wn', zeros(RS.idx_ik_length.wntraj,1), ...
  'enforce_qlim', false, 'enforce_qDlim', false, 'enforce_xlim', false);
[Q, QD, QDD, PHI] = RS.invkin2_traj(X, XD, XDD, T, q0, s_traj);
assert(all(abs(PHI(:)) < 1e-6), ...
  'Trajectory IK failed (max|Phi| = %1.1e)', max(abs(PHI(:))));

s_traj_ff = s_traj; s_traj_ff.ik_solution_min_norm = false;
[Q_ff, QD_ff, ~, PHI_ff] = RS.invkin2_traj(X, XD, XDD, T, q0, s_traj_ff);
assert(all(abs(PHI_ff(:)) < 1e-6), ...
  'Trajectory IK with ik_solution_min_norm=false failed (max|Phi| = %1.1e)', ...
  max(abs(PHI_ff(:))));

% The end effector has to follow the reference path in both cases.
for QQ = {Q, Q_ff}
  X_ist = RS.fkineEE2_traj(QQ{1});
  delta_x = X_ist(:,1:3) - X(:,1:3);
  assert(max(abs(delta_x(:))) < 1e-6, ...
    'Path deviation %1.1e m too large', max(abs(delta_x(:))));
end
fprintf('IK successful, max|Phi| = %1.1e, joint range %s deg\n', ...
  max(abs(PHI(:))), mat2str((max(Q)-min(Q))*180/pi, 3));

%% Part 1: basic solution ("\") vs. minimum-norm solution (pinv)
% Both are formed directly from the analytic Jacobian along the computed joint
% trajectory, so the comparison does not depend on the IK implementation.
QD_bs = NaN(nt, RS.NQJ); % basic solution from the operator "\"
QD_pi = NaN(nt, RS.NQJ); % minimum-norm solution from pinv
for k = 1:nt
  J_task = RS.jacobia(Q(k,:)'); % 6x7: underdetermined
  assert(size(J_task,1) < size(J_task,2), 'System is not underdetermined');
  xD_k = XD(k,:)';
  QD_bs(k,:) = (J_task \ xD_k)';
  QD_pi(k,:) = (pinv(J_task) * xD_k)';
  assert(all(abs(J_task*QD_bs(k,:)' - xD_k) < 1e-8), ...
    'Basic solution does not fulfil the task (k=%d)', k);
  assert(all(abs(J_task*QD_pi(k,:)' - xD_k) < 1e-8), ...
    'pinv solution does not fulfil the task (k=%d)', k);
end

% rank(J_task) = 6 < 7, so the basic solution has at least one entry exactly
% zero in every time step, and here it is always the same joint.
assert(all(sum(QD_bs(I_move,:) == 0, 2) >= RS.NQJ - sum(RS.I_EE_Task)), ...
  'Expectation not met: the basic solution of "\\" should have at least one joint velocity exactly zero in every time step');
I_fix_bs = all(QD_bs(I_move,:) == 0, 1);
assert(any(I_fix_bs), ...
  'Expectation not met: with the basic solution of "\\" one joint should stand still over the whole trajectory');
I_fix_pi = all(QD_pi(I_move,:) == 0, 1);
assert(~any(I_fix_pi), ...
  'The pinv solution must not freeze a joint. Affected: %s', mat2str(find(I_fix_pi)));
fprintf(['Part 1: "\\" freezes joint %sover the whole trajectory, pinv ', ...
  'freezes none.\n'], sprintf('%d ', find(I_fix_bs)));

% The pinv solution never has a larger norm than the basic solution.
norm_bs = sqrt(sum(QD_bs(I_move,:).^2, 2));
norm_pi = sqrt(sum(QD_pi(I_move,:).^2, 2));
assert(all(norm_pi <= norm_bs + 1e-10), ...
  'Norm of the pinv solution is larger than the one of the basic solution');
fprintf('Part 1: norm of the pinv solution is %1.1f %% of the basic solution (minimum %1.1f %%).\n', ...
  100*mean(norm_pi./norm_bs), 100*min(norm_pi./norm_bs));

% Without nullspace optimization the IK velocity is the task part plus a small
% correction term, so it has to match pinv and not the basic solution.
err_pi = max(max(abs(QD(I_move,:) - QD_pi(I_move,:))));
err_bs = max(max(abs(QD(I_move,:) - QD_bs(I_move,:))));
assert(err_bs > 100*err_pi, ...
  ['The IK velocity is not clearly closer to the minimum-norm solution than ', ...
   'to the basic solution (deviation %1.1e vs %1.1e).'], err_pi, err_bs);
fprintf(['Part 1: IK velocity matches pinv (max. deviation %1.1e) and differs ', ...
  'from the basic solution (%1.1e), ratio %1.0f.\n'], err_pi, err_bs, err_bs/err_pi);

%% Part 2: neither setting of ik_solution_min_norm may freeze a joint
% The second case runs through the branch with the structural DOF
% (redundant_struct), which is underdetermined for this robot as well. Since
% the task DOF equal the structural DOF here, the numerical result is the same;
% the point of the check is the code path.
for c = {{'true', QD}, {'false', QD_ff}}
  [name, QDc] = c{1}{:};
  I_fix = all(QDc(I_move,:) == 0, 1);
  assert(~any(I_fix), ...
    ['ik_solution_min_norm = %s: joint(s) %sdo not move over the whole ', ...
     'trajectory. Likely cause: "\\" is used instead of pinv for the ', ...
     'underdetermined system.'], name, sprintf('%d ', find(I_fix)));
  fprintf('Part 2: ik_solution_min_norm = %-5s ok, no joint frozen.\n', name);
end

%% Part 3: consistency of Q, QD and QDD by integration
% QD is the exact time derivative of Q, so this check is tight.
Q_int = Q(1,:) + cumtrapz(T, QD);
err_q = max(max(abs(Q_int - Q))) / max(max(abs(Q - Q(1,:))));
assert(err_q < 1e-3, 'Q is not consistent with QD (relative error %1.1e)', err_q);

% QDD is not the exact time derivative of QD: qDD_k_T is the minimum-norm
% acceleration, whereas the time derivative of the minimum-norm velocity would
% contain the derivative of the pseudo inverse as well. For a redundant robot
% both differ by a component in the nullspace, so a residual of a few percent
% is expected here and does not indicate an error. The check therefore only
% catches gross inconsistencies.
QD_int = QD(1,:) + cumtrapz(T, QDD);
err_qD = max(max(abs(QD_int - QD))) / max(abs(QD(:)));
assert(err_qD < 0.15, 'QD is not consistent with QDD (relative error %1.1e)', err_qD);
fprintf(['Part 3: cumtrapz consistency - QD->Q %1.1e (limit 1e-3), ', ...
  'QDD->QD %1.1e (limit 0.15, see comment).\n'], err_q, err_qD);

fprintf('Test %s completed successfully.\n', mfilename);
