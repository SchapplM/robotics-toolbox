% Teste die PKM-Dynamik mit Berechnung der Energiekonsistenz Direkten Dynamik

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-11
% (C) Institut für Mechatronische Systeme, Universität Hannover

clc
clear

%% Benutzereingaben
usr_only_check_symvsnum = true;
usr_plot_figures = true;
usr_save_figures = true;
usr_plot_animation = true;
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
% Pfad zum Abspeichern der Ergebnisse
respath = fullfile(rob_path, 'examples_tests', 'results', 'energy_consistency');
mkdirs(respath);

%% Alle Roboter durchgehen

for i_FG = 1:size(EEFG_Ges,1)
  ResStat = table();
  num_robots_tested = 0;
  num_robots_success = 0;
  robot_list_succ = {};
  robot_list_fail = {};
  EE_FG = EEFG_Ges(i_FG,:);
  [PNames_Kin, ~] = parroblib_filter_robots(sum(EE_FG), EE_FG, EE_FG_Mask, 6);
%   III = find(strcmp(PNames_Akt, 'P3PRR1G1P1A1'));
  for ii = 1:length(PNames_Kin) % III%
    PName = [PNames_Kin{ii},'A1']; % Nehme nur die erste Aktuierung (ist egal)
    fprintf('Untersuche PKM %s\n', PName);
    paramfile_robot = fullfile(tmpdir_params, sprintf('%s_params.mat', PName));
    %% Temporäres Filtern
    if contains(PName, 'G4')
      fprintf('PKM aussortiert (nicht implementierte Anordnung)\n');
      continue
    end
    %% Roboter Initialisieren
    RP = parroblib_create_robot_class(PName, 1, 0.3);
    % Initialisierung der Funktionen: Kompilierte Funktionen nehmen
    files_missing = RP.fill_fcn_handles(true, true);

    %% Kinematikparameter durch Optimierung erzeugen (oder gespeichert laden)
    Set = cds_settings_defaults(struct('DoF', EE_FG));
    Set.task.Ts = 1e-2;
    Set.task.Tv = 1e-1;
    Set.task.profile = 1; % Zeitverlauf mit Geschwindigkeit
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
      Set.structures.whitelist = {PName}; % nur diese PKM untersuchen
      Set.structures.nopassiveprismatic = false; % Für Dynamik-Test egal 
      Set.structures.maxnumprismatic = 6; % Für Dynamik-Test egal wie viele Schubgelenke
      Set.general.noprogressfigure = true;
      Set.general.verbosity = 3;
      Set.general.nosummary = true;
      cds_start
      resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
      resfile = fullfile(resmaindir, sprintf('Rob%d_%s_Endergebnis.mat', 1, PName));
      load(resfile, 'RobotOptRes');
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
    mges_PKM = rand(size(RP.DynPar.mges));
    rSges_PKM = rand(size(RP.DynPar.rSges));
    ISges_PKM = rand(size(RP.DynPar.Icges));
    RP.update_dynpar1 (mges_PKM, rSges_PKM, ISges_PKM);

    %% Dynamik-Test Symbolisch gegen numerisch
    % Test-Konfiguration
    n = 1;
    X_test = repmat(Traj.X(1,:),n,1) + 1e-10*rand(n,6);

    Q_test = NaN(n, RP.NJ); QD_test = Q_test; QDD_test = Q_test;
    XD_test = rand(n,6);     XDD_test = rand(n,6);
    XD_test(:,~RP.I_EE) = 0;   XDD_test(:,~RP.I_EE) = 0;
    % IK für Testkonfiguration berechnen
    for i = 1:n
      [q,Phi] = RP.invkin(X_test(i,:)', q0);
      if any(abs(Phi) > 1e-8) || any(isnan(Phi)), X_test(i,:)=X_test(i,:)*NaN; continue; end
      Q_test(i,:) = q;
      [~, Jinv_voll] = RP.jacobi_qa_x(q, X_test(i,:)');
      QD_test(i,:) = Jinv_voll*XD_test(i,RP.I_EE)';
      [~, JinvD_voll] = RP.jacobiD_qa_x(q, QD_test(i,:)', X_test(i,:)', XD_test(i,:)');
    end
    % Dynamik berechnen (mit den Zufallswerten)
    n_succ = 0;
    n_fail = 0;
    for i_tk = 1:n
      if any(isnan(X_test(i_tk,:))), continue; end % IK für diese Pose nicht erfolgreich
      
%       if i_tk > 0
        RP.update_base([0;0;1], rand(3,1));
%       end
      RP.update_gravity([0;0;-9.81]);
      % Initialisierung
      dt = 1000e-6;
      T_end = 2;
      nt = T_end/dt;
      Tges = zeros(nt,1);
      xD_red_old = XD_test(i_tk,RP.I_EE)';
      E_ges = NaN(nt,3+2+RP.NLEG*2); % 3 gesamt, 2 Plattform, 2 für jede Beinkette
      X_ges = NaN(nt,6);
      XD_ges = NaN(nt,6);
      XDD_ges = NaN(nt,6);
      PHI_ges = NaN(nt, RP.I2constr_red(end));
      PHI_ges2 = NaN(nt, RP.I2constr_red(end));
      Q_ges = NaN(nt,RP.NJ);
      QD_ges = NaN(nt,RP.NJ);
      SingDet = NaN(nt,2);
      Facc_ges = NaN(nt,6);
      P_diss_ges = zeros(nt,2);

      % Anfangswerte für iterativen Algorithmus
      q = Q_test(i_tk,:)';
      q_old = q;
      qD = QD_test(i_tk,:)';
      x = X_test(i_tk,:)';
      x_red_old = x(RP.I_EE);
      xD = XD_test(i_tk,:)';
      t1 = tic;
      s = struct( ...
             'n_min', 25, ... % Minimale Anzahl Iterationen
             'n_max', 1000, ... % Maximale Anzahl Iterationen
             'Phit_tol', 1e-12, ... % Toleranz für translatorischen Fehler
             'Phir_tol', 1e-12); % Toleranz für rotatorischen Fehler
      
      for i = 1:nt
        Tges(i) = (i-1)*dt;
  
        % Dynamik-Terme berechnen
        Mx = RP.inertia2_platform(q, x, Jinv_voll);
        Gx = RP.gravload2_platform(q, x, Jinv_voll);
        Cx = RP.coriolisvec2_platform(q, qD, x, xD, Jinv_voll, JinvD_voll);
        Facc_ges(i,RP.I_EE) = -Gx - Cx;
  
        % Beschleunigung der Plattform berechnen
        xDD_red = Mx \ (-Gx - Cx);
        
        % Integration der Beschleunigung
        xD_red = xD_red_old+ xDD_red*dt;
        
        % Integration der Geschwindigkeit
        % TODO: Hier muss eventuell noch eine Anpassung an die
        % Winkelgeschwindigkeit vorgenommen werden
        x_red = x_red_old + xD_red_old*dt + 0.5*xDD_red*dt^2;
        
        XDD = zeros(6,1);
        XDD(RP.I_EE) = xDD_red;
        xD = zeros(6,1);
        xD(RP.I_EE) = xD_red;
        x = zeros(6,1);
        x(RP.I_EE) = x_red;
        
        % Berechnung der Gelenk-Geschwindigkeit
        Phiq_red = RP.constr1grad_q(q, x);
        Phix_red = RP.constr1grad_x(q, x);
        Jinv_voll = -Phiq_red\Phix_red; % Jacobi-Matrix als Hilfe für Dynamik-Fkt speichern
        [~,JinvD_voll] = RP.jacobiD_qa_x(q, qD, x, xD); % effizienter hier zu berechnen als in Dynamik
        qD = Jinv_voll*xD_red;
  
        % Kennzahlen für Singularitäten
        SingDet(i,1) = cond(Phiq_red);
        SingDet(i,2) = cond(Phix_red);

        % Integration der Gelenk-Geschwindigkeit und IK
        q_dik = q_old + qD*dt;
        [q_ik,Phi] = RP.invkin_ser(x, q_dik, s);
        if any(isnan(q_ik)) || any(abs(Phi) > 1e-6)
          warning('i=%d/%d: IK kann nicht berechnet werden. Roboter verlässt vermutlich den Arbeitsraum.', i, nt);
          break
        end
        
        % Kinematische Zwangsbedingungen prüfen
        PHI_ges(i,:) = RP.constr1(q_dik, x);
        PHI_ges2(i,:) = Phi;

        q = q_ik;
        
        % Berechnung der Systemenergie
        [T_i, T_legs_i, T_platform_i] = RP.ekin(q, qD, x, xD);
        [U_i, U_legs_i, U_platform_i] = RP.epot(q, x);

        % Werte für diesen Zeitschritt abspeichern
        E_ges(i,1:3) = [T_i, U_i, T_i+U_i];
        E_ges(i,4:4+RP.NLEG) = [T_platform_i; T_legs_i];
        E_ges(i,4+RP.NLEG+1:4+RP.NLEG*2+1) = [U_platform_i; U_legs_i];
        X_ges(i,:) = x;
        XD_ges(i,:) = xD;
        XDD_ges(i,:) = XDD;
        Q_ges(i,:) = q;
        QD_ges(i,:) = qD;
  
        % Berechne die zwischen den Teilsystemen umgesetzte Energie als Leistung
        % Der numerische Fehler entspricht Dissipation
        if i > 1
          P_diff = (E_ges(i, :) - E_ges(i-1, :)) / dt;
          p_komp = sum(abs(P_diff([1 2 4:end])));
          p_sys = abs(P_diff(3));
          P_diss_ges(i,:) = [p_sys, p_komp];
        end

        % Altwerte abspeichern für nächsten Zeitschritt
        x_red_old = x_red;
        xD_red_old = xD_red;
        q_old = q;
      end
      Ip_end = i;
      fprintf('Dauer für %1.0f Simulationsschritte (%1.2fs simulierte Zeit): %1.2fs (%1.0f%% Echtzeit)\n', ...
        i, Tges(Ip_end), toc(t1), 100*Tges(Ip_end)/toc(t1));
      if usr_plot_figures
        %% Plot: Energie
        change_current_figure(1);clf;
        kompleg = {'Plattform'};
        for ileg = 1:RP.NLEG
          kompleg = {kompleg{:}, sprintf('Beinkette %d', ileg)}; %#ok<CCAT>
        end
        subplot(2,3,1);hold on;
        plot(Tges(1:Ip_end), E_ges(1:Ip_end,1));
        plot(Tges(1:Ip_end), E_ges(1:Ip_end,2));
        plot(Tges(1:Ip_end), E_ges(1:Ip_end,3));
        legend({'Kinetisch', 'Potentiell', 'Gesamt'});
        grid on; ylabel('Teil-Energien in J');
        subplot(2,3,2);
        plot(Tges(1:Ip_end), E_ges(1:Ip_end,3)-E_ges(1,3));
        grid on; ylabel('Energiedifferenz in J');
        subplot(2,3,3);
        plot(Tges(1:Ip_end), E_ges(1:Ip_end,4:4+RP.NLEG));
        grid on; ylabel('Kinetische Energie der Komponenten in J');
        legend(kompleg);
        subplot(2,3,4);
        plot(Tges(1:Ip_end), E_ges(1:Ip_end,4+RP.NLEG+1:4+RP.NLEG*2+1));
        grid on; ylabel('Potentielle Energie der Komponenten in J');
        legend(kompleg);
        subplot(2,3,5);
        plot(Tges(1:Ip_end), P_diss_ges(1:Ip_end,:));
        grid on; ylabel('Übertragene Leistung in W');
        legend({'Gesamt (Dissipation)', 'zwischen Komponenten'});
        subplot(2,3,6);
        plot(Tges(1:Ip_end), 100*P_diss_ges(1:Ip_end,1)./P_diss_ges(1:Ip_end,2));
        grid on; ylabel('Anteil Dissipation an Leistungsfluss in Prozent');
        linkxaxes
        sgtitle(sprintf('%s: Energie', PName));
        set(1, 'Name', 'Energie', 'NumberTitle', 'off');
        if usr_save_figures
          name = fullfile(respath, sprintf('%s_Energie_dt%dus', PName, 1e6*dt));
          export_fig([name, '.png']);
          saveas(gcf, [name, '.png']);
        end
        %% Plot: Gelenke
        change_current_figure(2);clf;
        for ileg = 1:RP.NLEG
          subplot(RP.NLEG,2,sprc2no(RP.NLEG,2,ileg,1));
          plot(Tges, Q_ges(:,RP.I1J_LEG(ileg):RP.I2J_LEG(ileg)));
          grid on; ylabel(sprintf('Beinkette %d', ileg));

          title('Gelenkposition');
          subplot(RP.NLEG,2,sprc2no(RP.NLEG,2,ileg,2));
          plot(Tges, QD_ges(:,RP.I1J_LEG(ileg):RP.I2J_LEG(ileg)));
          grid on; 
          title('Gelenkgeschwindigkeit');
        end
        linkxaxes
        sgtitle(sprintf('%s: Gelenke', PName));
        set(2, 'Name', 'Beingelenk-Kinematik', 'NumberTitle', 'off');
        if usr_save_figures
          name = fullfile(respath, sprintf('%s_Gelenke_dt%dus', PName, 1e6*dt));
          export_fig([name, '.png']);
          saveas(gcf, [name, '.png']);
        end
        %% Plot: Plattform
        change_current_figure(3);clf;
        subplot(2,2,1);
        xpos_leg = {'rx', 'ry', 'rz', 'phix', 'phiy', 'phiz'};
        plot(Tges, X_ges(:,RP.I_EE));
        grid on; ylabel('Plattform-Position (im Basis-KS)');
        legend(xpos_leg(RP.I_EE));
        subplot(2,2,2);
        xvel_leg = {'vx', 'vy', 'vz', 'phiDx', 'phiDy', 'phiDz'};
        plot(Tges, XD_ges(:,RP.I_EE));
        grid on; ylabel('Plattform-Geschwindigkeit (im Basis-KS)');
        legend(xvel_leg(RP.I_EE));
        subplot(2,2,3);
        plot(Tges, XDD_ges(:,RP.I_EE));
        grid on; ylabel('Plattform-Beschleunigung (im Basis-KS)');
        subplot(2,2,4);
        Facc_leg = {'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'};
        plot(Tges, Facc_ges(:,RP.I_EE));
        grid on; ylabel('Plattform-Kraft (im Basis-KS)');
        legend(Facc_leg(RP.I_EE));
        linkxaxes
        sgtitle(sprintf('%s: Plattform', PName));
        set(3, 'Name', 'Plattform-Kinematik', 'NumberTitle', 'off');
        if usr_save_figures
          name = fullfile(respath, sprintf('%s_Plattform_dt%dus', PName, 1e6*dt));
          export_fig([name, '.png']);
          saveas(gcf, [name, '.png']);
        end
        
        %% Plot: Diverses
        change_current_figure(4);clf;
        subplot(2,1,1);
        title('Singularitäts-Kennzahlen');
        plot(Tges, SingDet);
        legend({'cond(JIK)', 'cond(JDK)'});
        grid on;
        subplot(2,2,3); hold on;
        title('Prüfung der Kinematik-Konsistenz');
        plot(Tges, PHI_ges(:, RP.I_constr_t_red));
        set(gca, 'ColorOrderIndex', 1);
        plot(Tges, PHI_ges2(:, RP.I_constr_t_red), '--');
        grid on; ylabel('Translatorische Zwangsbed.');
        subplot(2,2,4); hold on;
        if ~isempty(RP.I_constr_r_red)
        plot(Tges, PHI_ges(:, RP.I_constr_r_red));
        set(gca, 'ColorOrderIndex', 1);
        plot(Tges, PHI_ges2(:, RP.I_constr_r_red), '--');
        grid on;ylabel('Rotatorische Zwangsbed.');
        end
        linkxaxes
        sgtitle(sprintf('%s: Validierung/Test', PName));
        set(4, 'Name', 'Testen', 'NumberTitle', 'off');
        if usr_save_figures
          name = fullfile(respath, sprintf('%s_Test_dt%dus', PName, 1e6*dt));
          export_fig([name, '.png']);
          saveas(gcf, [name, '.png']);
        end
      end
      
      if usr_plot_animation && i > 1
        %% Plot des Roboters in Startpose
        change_current_figure(5);clf;hold all;
        xlabel('x in m');ylabel('y in m');zlabel('z in m'); view(3);
        axis auto
        hold on;grid on;
        s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
        RP.plot(Q_ges(1,:)', X_ges(1,:)', s_plot);
        title(sprintf('%s in Startkonfiguration',PName));
        %% Animation des bewegten Roboters
        i_end_vis = Ip_end-1;
        i_diff = ceil(i_end_vis / 25);
        s_anim = struct( 'gif_name', fullfile(respath, sprintf('%s_energy_test_dt%dus.gif', PName, 1e6*dt)));
        s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
        change_current_figure(6);clf;hold all;
        view(3);
        axis auto
        hold on;grid on;
        xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
        title(sprintf('%s Dynamik-Simulation ',PName));
        RP.anim( Q_ges(1:i_diff:i_end_vis,:), X_ges(1:i_diff:i_end_vis,:), s_anim, s_plot);
        fprintf('Animation der Bewegung gespeichert: %s\n', s_anim.gif_name);
      end
      %% Auswertung
      % Validiere bei 80% der Zeit. Annahme: Wenn eine Singularität die
      % Simulation abbricht, ist sie hier noch nicht wirksam.
      % Starte Validierung erst nach 20% der Zeit. Annahme: Beim Start ist
      % die Bezugsgröße noch zu klein für einen relativen Fehler
      if i == 1
        I_test = 1;
      else
        I_test = floor(Ip_end*0.20):floor(Ip_end*0.8);
      end
      E_error = max(abs(E_ges(1:Ip_end,3)-E_ges(1,3)));
      % Nehme den Anteil der dissipierten Energie an der umgesetzten Energie
      Pdiss_relerror = max(abs(P_diss_ges(I_test,1)./P_diss_ges(I_test,2)));
      fail = false;
      if Pdiss_relerror > 10e-2 || isnan(Pdiss_relerror)
        fail = true;
      end
      if fail
        n_fail = n_fail + 1;
      else
        n_succ = n_succ + 1;
      end
      TrajAchieved = i/nt; % Anteil der Simulationsdauer, die geschafft wurde
      % Ergebnis in Tabelle abspeichern
      ResStat = [ResStat; {PName, Pdiss_relerror, TrajAchieved}]; %#ok<AGROW>
    end
    fprintf('%d/%d Kombinationen getestet (%d i.O., %d n.i.O) (bei restlichen %d IK falsch).\n', ...
      n_succ+n_fail, n, n_succ, n_fail, n-(n_succ+n_fail));
    num_robots_tested = num_robots_tested + 1;
    if n_succ == n
      num_robots_success = num_robots_success + 1;
      robot_list_succ = [robot_list_succ(:)', {PName}];
    else
      robot_list_fail = [robot_list_fail(:)', {PName}];
    end
  end
  ResStat.Properties.VariableNames = {'Name', 'RelError20_80', 'ProzentTraj'};
  save(fullfile(respath, sprintf('ResStat_%dT%dR_FG.mat', sum(EE_FG(1:3)), sum(EE_FG(4:6)))), 'ResStat');
  fprintf('Energiekonsistenz für %d PKM mit FG [%s] getestet. %d Erfolgreich\n', ...
    num_robots_tested, disp_array(EE_FG, '%1.0f'), num_robots_success);
  disp('Keine Energiekonsistenz bei folgenden Robotern:');
  disp(robot_list_fail(:));
  disp('Energiekonsistenz bei folgenden Robotern:');
  disp(robot_list_succ(:));
  disp('Ergebnistabelle für FG:');
  disp(ResStat);
end
