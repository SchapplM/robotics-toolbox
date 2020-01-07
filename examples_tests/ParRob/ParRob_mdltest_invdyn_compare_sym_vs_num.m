% Vergleiche die symbolische und numerische Implementierung der Dynamik

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-11
% (C) Institut für Mechatronische Systeme, Universität Hannover

clc
clear

%% Benutzereingaben
usr_only_check_symvsnum = true;
usr_DynParMode = 3;

%% Initialisierung
EEFG_Ges = [1 1 0 0 0 1; ...
            1 1 1 0 0 0; ...
            1 1 1 0 0 1; ...
            1 1 1 1 1 1];
EE_FG_Mask = [1 1 1 1 1 1];
rob_path = fileparts(which('robotics_toolbox_path_init.m'));
% Pfad zum Abspeichern von Maßsynthese-Ergebnissen
tmpdir_params = fullfile(rob_path, 'examples_tests', 'tmp_ParRob', 'param_dimsynthres');
mkdirs(tmpdir_params);
%% Alle Roboter durchgehen

for i_FG = 1:size(EEFG_Ges,1)
  num_robots_tested = 0;
  num_robots_success = 0;
  robot_list_succ = {};
  robot_list_fail = {};
  EE_FG = EEFG_Ges(i_FG,:);
  [PNames_Kin, ~] = parroblib_filter_robots(sum(EE_FG), EE_FG, EE_FG_Mask, 6);
  % III = find(strcmp(PNames_Akt, 'P3RRR1G1P1A1'));
  for ii = 1:length(PNames_Kin)
    PNameA = [PNames_Kin{ii},'A1']; % Nehme nur die erste Aktuierung (ist für Dynamik egal)
    fprintf('Untersuche PKM %s\n', PNameA);
    paramfile_robot = fullfile(tmpdir_params, sprintf('%s_params.mat', PNameA));
    %% Temporäres Filtern
    if contains(PNameA, 'G4')
      fprintf('PKM aussortiert (nicht implementierte Anordnung)\n');
      continue
    end
    %% Roboter Initialisieren
    RP = parroblib_create_robot_class(PNameA, 1, 0.3);
    % Initialisierung der Funktionen: Kompilierte Funktionen nehmen
    files_missing = RP.fill_fcn_handles(true, true);
    if length(files_missing) >= 6 && usr_only_check_symvsnum
      fprintf('Keine symbolisch generierten Dynamik-Funktionen verfügbar. Kein Test Sym vs Num möglich.\n');
      continue
    end
    if any(contains(files_missing, 'para_pf_mdp')) && any(usr_DynParMode == [3,4])
      fprintf('Regressor-Form soll getestet werden, aber Funktion nicht vorhanden\n');
      continue
    end

    %% Kinematikparameter durch Optimierung erzeugen (oder gespeichert laden)
    Set = cds_settings_defaults(struct('DoF', EE_FG));
    Set.task.Ts = 1e-2;
    Set.task.Tv = 1e-1;
    Set.task.profile = 0; % Nur Eckpunkte, kein Zeitverlauf mit Geschwindigkeit
    Set.task.maxangle = 5*pi/180;
    Traj = cds_gen_traj(EE_FG, 1, Set.task);
    % Reduziere Punkte (geht dann schneller, aber auch schlechtere KinPar.
    % Traj = timestruct_select(Traj, [1, 2]);
    params_success = false;
    % Prüfe, ob die Dynamik-Implementierung in symbolischer Form vorliegt
    if exist(paramfile_robot, 'file')
      params = load(paramfile_robot);
      q0 = params.q0;
      for il = 1:RP.NLEG, RP.Leg(il).update_mdh(params.pkin); end
      RP.align_base_coupling(params.DesPar_ParRob.base_method, params.DesPar_ParRob.base_par);
      RP.align_platform_coupling(params.DesPar_ParRob.platform_method, params.DesPar_ParRob.platform_par(1:end-1));
      % Prüfe die Lösbarkeit der IK
      [~,Phi]=RP.invkin(Traj.X(1,:)', q0);
      if all(abs(Phi)<1e-6) && ~any(isnan(Phi))
        params_success = true; % Parameter für erfolgreiche IK geladen.
      end
    end
    if ~params_success
      % Führe Maßsynthese neu aus. Parameter nicht erfolgreich geladen
      Set.optimization.objective = 'valid_act';
      Set.optimization.ee_rotation = false;
      Set.optimization.ee_translation = false;
      Set.optimization.movebase = false;
      Set.optimization.base_size = false;
      Set.optimization.platform_size = false;
      Set.structures.use_parallel_rankdef = 6;
      Set.structures.whitelist = {PNameA}; % nur diese PKM untersuchen
      Set.structures.nopassiveprismatic = false; % Für Dynamik-Test egal 
      Set.structures.maxnumprismatic = 6; % Für Dynamik-Test egal wie viele Schubgelenke
      Set.general.noprogressfigure = true;
      Set.general.verbosity = 3;
      Set.general.nosummary = true;
      cds_start
      if isempty(Structures) || RobotOptRes.fval > 1000
        % Die Methode valid_act nimmt die erstbeste bestimmbare Kinematik.
        % Die Wahl der aktuierten Gelenke muss nicht zu vollem Rang führen.
        % Kriterium ist daher nur die Bestimmbarkeit des Rangs (fval <1000)
        warning('Etwas ist bei der Maßsynthese schiefgelaufen');
        continue
      end
      % Funktionierende Parameter abspeichern (für nächstes Mal)
      RP = RobotOptRes.R;
      pkin = RP.Leg(1).pkin;
      DesPar_ParRob = RP.DesPar;
      q0 = RobotOptRes.q0;
      save(paramfile_robot, 'pkin', 'DesPar_ParRob', 'q0');
      fprintf('Maßsynthese beendet\n');
    end
    

    %% Parameter für Dynamik-Test
    RP.DynPar.mode = usr_DynParMode;
    for il = 1:RP.NLEG, RP.Leg(il).DynPar.mode = usr_DynParMode; end
    mges_PKM = rand(size(RP.DynPar.mges));
    rSges_PKM = rand(size(RP.DynPar.rSges));
    ISges_PKM = rand(size(RP.DynPar.Icges));
    mges_PKM(RP.NQJ_LEG_bc+1:end-1) = 0;
    rSges_PKM(RP.NQJ_LEG_bc+1:end-1,:) = 0;
    ISges_PKM(RP.NQJ_LEG_bc+1:end-1,:) = 0;
%     % Debug: Keine Beine
%     mges_PKM(1:end-1) = 0;
%     rSges_PKM(1:end-1,:) = 0;
%     ISges_PKM(1:end-1,:) = 0;
%     % Debug: Keine Plattform
%     mges_PKM(end) = 0;
%     rSges_PKM(end,:) = 0;
%     ISges_PKM(end,:) = 0;

    RP.update_dynpar1 (mges_PKM, rSges_PKM, ISges_PKM);

    %% Dynamik-Test Symbolisch gegen numerisch
    % Test-Konfiguration
    n = 10;
    X_test = repmat(Traj.X(1,:),n,1) + 1e-1*rand(n,6);
    Q_test = NaN(n, RP.NJ); QD_test = Q_test; QDD_test = Q_test;
    XD_test = rand(n,6);     XDD_test = rand(n,6);
    X_test(:,~RP.I_EE) = 0;
    XD_test(:,~RP.I_EE) = 0;   XDD_test(:,~RP.I_EE) = 0;
    Information_matrix = [];
    % Dynamik berechnen (mit den Zufallswerten)
    n_succ = 0;
    n_fail = 0;
    for i = 1:n
      % IK für Testkonfiguration berechnen
      [q,Phi] = RP.invkin(X_test(i,:)', q0);
      if any(abs(Phi) > 1e-8) || any(isnan(Phi)), continue; end % IK für diese Pose nicht erfolgreich
      Q_test(i,:) = q;
      [~, Jinv] = RP.jacobi_qa_x(q, X_test(i,:)');
      QD_test(i,:) = Jinv*XD_test(i,RP.I_EE)';
      [~, JinvD] = RP.jacobiD_qa_x(q, QD_test(i,:)', X_test(i,:)', XD_test(i,:)');
      QDD_test(i,:) = JinvD*XD_test(i,RP.I_EE)' + Jinv*XDD_test(i,RP.I_EE)';
      
      % Parameter zufällig setzen
      RP.update_gravity(rand(3,1));
      
      % Berechne Dynamik-Terme auf zwei Arten
      Mx2 = RP.inertia2_platform(Q_test(i,:)' , X_test(i,:)');
      Cx2 = RP.coriolisvec2_platform(Q_test(i,:)', QD_test(i,:)', X_test(i,:)', XD_test(i,:)');
      Gx2 = RP.gravload2_platform(Q_test(i,:)', X_test(i,:)');
      Fx2 = RP.invdyn2_platform(Q_test(i,:)', QD_test(i,:)', QDD_test(i,:)', X_test(i,:)', XD_test(i,:)', XDD_test(i,:)');
      Mx1 = RP.inertia_platform(Q_test(i,:)', X_test(i,:)');
      Cx1 = RP.coriolisvec_platform(Q_test(i,:)', X_test(i,:)', XD_test(i,:)');
      Gx1 = RP.gravload_platform(Q_test(i,:)', X_test(i,:)');
      Fx1 = RP.invdyn_platform(Q_test(i,:)', X_test(i,:)', XD_test(i,:)', XDD_test(i,:)');
      test_M = Mx2 - Mx1;
      test_C = Cx2 - Cx1;
      test_G = Gx2 - Gx1;
      test_F = Fx2 - Fx1;
      test_F_sum = Mx2*XDD_test(i,RP.I_EE)'+Cx2+Gx2-Fx2;
      fail = false;
      if any(abs(test_M(:)) > 1e-10)
        warning('Massenmatrix stimmt nicht');fail = true;
      end
      if any(abs(test_C(:)) > 1e-10)
        warning('Coriolis-Terme stimmen nicht');fail = true;
      end
      if any(abs(test_G(:)) > 1e-10)
        warning('Gravitations-Terme stimmen nicht');fail = true;
      end
      if any(abs(test_F(:)) > 1e-10) || any(abs(test_F_sum(:)) > 1e-10)
        warning('Inversdynamik-Terme (gesamt) stimmen nicht');fail = true;
      end
      % Prüfe Aufruf mit gegebener Jacobi-Matrix (sollte schneller sein)
      Mx2 = RP.inertia2_platform(Q_test(i,:)' , X_test(i,:)', Jinv);
      Cx2 = RP.coriolisvec2_platform(Q_test(i,:)', QD_test(i,:)', X_test(i,:)', XD_test(i,:)', Jinv, JinvD);
      Gx2 = RP.gravload2_platform(Q_test(i,:)', X_test(i,:)', Jinv);
      
      % Prüfe Regressor-Form
      if usr_DynParMode == 3 || usr_DynParMode == 4
        if usr_DynParMode == 3 % Inertialparameter-Vektor
          dpv = RP.DynPar.ipv_n1s;
        else % Minimalparameter-Vektor
          dpv = RP.DynPar.mpv_n1s;
        end
        [Gx2,Gx2_reg] = RP.gravload2_platform(Q_test(i,:)', X_test(i,:)');
        test_Greg = Gx2_reg*dpv - Gx2;
        if any(abs(test_Greg(:)) > 1e-10)
          error('Gravitationskraft-Regressor stimmt nicht');
        end
        [M_full,M_full_reg] = RP.inertia2_platform_full(Q_test(i,:)', X_test(i,:)');
        [Mx2,Mx2_reg] = RP.inertia2_platform(Q_test(i,:)' , X_test(i,:)');
        Mx_reg_vec = Mx2_reg*dpv;
        test_Mreg = reshape(Mx_reg_vec, sum(RP.I_EE), sum(RP.I_EE)) - Mx2;
        if any(abs(test_Mreg(:)) > 1e-10)
          error('Massenmatrix-Regressor stimmt nicht');
        end
        M_full_regtest = zeros(size(M_full));
        for jj = 1:length(dpv)
          M_full_regtest = M_full_regtest + M_full_reg(:,:,jj) .* dpv(jj);
        end
        test_Mreg_full = M_full_regtest - M_full;
        if any(abs(test_Mreg_full(:)) > 1e-10)
          error('Massenmatrix-Regressor (vollständige Jacobi vor Projektion) stimmt nicht');
        end
        [Cx2,Cx2_reg] = RP.coriolisvec2_platform(Q_test(i,:)', QD_test(i,:)', X_test(i,:)', XD_test(i,:)') ;
        test_Creg = Cx2_reg*dpv - Cx2;
        if any(abs(test_Creg(:)) > 1e-10)
          error('Coriolis-Kraft-Regressor stimmt nicht');
        end
        [Fx2,Fx2_reg] = RP.invdyn2_platform(Q_test(i,:)', QD_test(i,:)', QDD_test(i,:)', X_test(i,:)', XD_test(i,:)', XDD_test(i,:)');
        test_Freg = Fx2_reg*dpv - Fx2;
        if any(abs(test_Freg(:)) > 1e-10)
          error('Inversdynamik-Kraft-Regressor stimmt nicht');
        end
        % Fülle eine Informationsmatrix um die Konditionierung der
        % Regressormatrix in einem virtuellen Identifikationsproblem zu
        % testen
        Information_matrix = [Information_matrix; Fx2_reg]; %#ok<AGROW>
      end
      % Auswertung
      if fail
        n_fail = n_fail + 1;
      else
        n_succ = n_succ + 1;
      end
    end
    fprintf('%d/%d Kombinationen getestet (%d i.O., %d n.i.O) (bei restlichen %d IK falsch).\n', ...
      n_succ+n_fail, n, n_succ, n_fail, n-(n_succ+n_fail));
    num_robots_tested = num_robots_tested + 1;
    if n_succ == n
      num_robots_success = num_robots_success + 1;
      robot_list_succ = [robot_list_succ(:)', {PNameA}];
    else
      robot_list_fail = [robot_list_fail(:)', {PNameA}];
    end
    if usr_DynParMode == 3 || usr_DynParMode == 4
      % Prüfe Rang der Informationsmatrix
      fprintf(['%s: Die Informationsmatrix der Inversen Dynamik hat Rang %d. \nBei ', ...
        '%d Dynamikparametern der PKM gibt es also %d zu viel. \nSymbolisch ', ...
        'wurden %d Parameter bestimmt\n'], PNameA, rank(Information_matrix), length(dpv), ...
        length(dpv)-rank(Information_matrix),length(dpv));
    end
  end
  fprintf('Dynamik in symbolischer und numerischer Form für %d/%d PKM mit FG [%s] getestet. %d Erfolgreich\n', ...
    num_robots_tested, length(PNames_Kin), disp_array(EE_FG, '%1.0f'), num_robots_success);
  disp('Keine Übereinstimmung bei folgenden Robotern:');
  disp(robot_list_fail(:));
  disp('Übereinstimmung bei folgenden Robotern:');
  disp(robot_list_succ(:));
end
