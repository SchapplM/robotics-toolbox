% Teste die PKM-Dynamik mit Berechnung der Energiekonsistenz Direkten Dynamik
% Zusätzlich wird eine Form der kinematischen Zwangsbedingungen getestet
% (da beim Energiekonsistenz-Test sowieso brauchbare E-/A-Daten der Kinematik entstehen)
% 
% Erstellt (und speichert) Bilder:
%  1 - Diagnosebild des Roboters in Startpose
%  2 - Diagnosebild des Roboters in Startpose
%  3 - Energiekonsistenz (Betrachtung von Energien und Leistungen)
%  4 - Gelenkpositionen der Beinketten
%  5 - Gelenke: Konsistenz (Position vs Geschwindigkeit)
%  6 - Gelenke: Konsistenz (Position vs Geschwindigkeit)
%  7 - Plattform
%  8 - Testen: Singularitätskennzahlen
%  9 - Roboter in Startpose
% 10 - Animation des Roboters
% Bilder werden in Unterordner für EE-FG und PKM gespeichert.
% Zusätzlich Speicherung einer Ergebnis-Tabelle
% 
% Siehe auch: ParRob_mdltest_invdyn_compare_sym_vs_num.m (ähnlich)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-11
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

%% Benutzereingaben
usr_plot_figures = true;
usr_save_figures = true;
usr_testselection = false;
usr_plot_animation = false;
usr_debug = true;
usr_jointspring = true; % Setze Feder in passive Gelenke, damit PKM um Arbeitspunkt schwingt (kein freier Fall)
usr_num_tests_per_dof = 5;
usr_test_constr4_JinvD = false; % zum Debuggen der constr4gradD-Funktionen
usr_shuffle_pkm_selection = true; % Zufällige Auswahl der PKM.
%% Initialisierung
EEFG_Ges = [1 1 0 0 0 1; ...
            1 1 1 0 0 0; ...
            1 1 1 0 0 1; ...
            1 1 1 1 1 0; ...
            1 1 1 1 1 1];
rob_path = fileparts(which('robotics_toolbox_path_init.m'));
% Pfad zum Abspeichern von Maßsynthese-Ergebnissen
tmpdir_params = fullfile(rob_path, 'examples_tests', 'tmp_ParRob', 'param_dimsynthres');
mkdirs(tmpdir_params);
% Pfad zum Abspeichern der Ergebnisse
respath = fullfile(rob_path, 'examples_tests', 'results', 'energy_consistency');
mkdirs(respath);

s = struct( ... % Einstellung für IK
  'normalize', false, ... % Winkel nicht normalisieren, da sonst Federmoment falsch.
  'n_min', 25, ... % Minimale Anzahl Iterationen: Damit möglichst Residuum wirklich Null (bzw. 1e-15)
  'n_max', 1000, ... % Standard-Wert
  'Phit_tol', 1e-12, ... % Sehr hohe Genauigkeit (Ausschluss als ...
  'Phir_tol', 1e-12); % ... Fehlerquelle der Dynamik);
%% Alle Roboter durchgehen

for i_FG = 1:size(EEFG_Ges,1)
  ResStat = table();
  num_robots_tested = 0;
  num_robots_success = 0;
  robot_list_succ = {};
  robot_list_fail = {};
  EE_FG = EEFG_Ges(i_FG,:);
  fprintf('Untersuche Energiekonsistenz für %dT%dR-PKM\n', sum(EE_FG(1:3)), sum(EE_FG(4:6)));
  [PNames_Kin, ~] = parroblib_filter_robots(EE_FG, 6);
  if isempty(PNames_Kin)
    continue % Es gibt keine PKM mit diesen FG.
  end
  if usr_testselection % Debug
    switch i_FG
      case 1
        III = find(strcmp(PNames_Kin, 'P3RRR1G1P1'));
      case 2
        III = find(strcmp(PNames_Kin, 'P3PRRRR8V2G1P2'));
      case 3
        III = find(strcmp(PNames_Kin, 'P4PRRRR8V1G3P1'));
      case 4
        III = find(strcmp(PNames_Kin, 'P5RRRPR4V1G9P8'));
      case 5
        III = find(strcmp(PNames_Kin, 'P6PRRRRR6V2G8P1'));
    end
  else
    III = 1:length(PNames_Kin);
    if usr_shuffle_pkm_selection
      III = III(randperm(length(III)));
    end
  end
  for ii = III
    PName = [PNames_Kin{ii},'A1']; % Nehme nur die erste Aktuierung (ist egal)
    fprintf('Untersuche PKM %d/%d: %s\n', ii, length(PNames_Kin), PName);
    paramfile_robot = fullfile(tmpdir_params, sprintf('%s_params.mat', PName));

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
    if i_FG == 1,    trajno = 2; %#ok<ALIGN>
    elseif i_FG < 4, trajno = 1;
    else,            trajno = 3; end
    Traj_W = cds_gen_traj(EE_FG, trajno, Set.task);
    % Reduziere Punkte (geht dann schneller, aber auch schlechtere KinPar.)
    % Traj = timestruct_select(Traj, [1, 2]);
    % Lade die bestehenden Parameter und prüfe, ob sie gültig sind oder mit
    % einer veralteten Version der Maßsynthese erstellt wurden.
    params_valid = false;
    if exist(paramfile_robot, 'file')
      params = load(paramfile_robot);
      if length(intersect(fieldnames(params), {'pkin', 'qlim', 'r_W_0', ...
          'phi_W_0', 'r_P_E', 'phi_P_E', 'DesPar_ParRob', 'q0'})) == 8
        params_valid = true;
      end
    end
    % params_valid = false; % Debug: neue Maßsynthese erzwingen.
    params_success = false;
    if params_valid % Parameter sind gültig: Lade die alten Parameter und teste sie
      q0 = params.q0;
      for il = 1:RP.NLEG
        RP.Leg(il).update_mdh(params.pkin); 
        RP.Leg(il).qlim = params.qlim(RP.I1J_LEG(il):RP.I2J_LEG(il),:);
      end
      RP.update_base(params.r_W_0, params.phi_W_0);
      RP.update_EE(params.r_P_E, params.phi_P_E);
      RP.align_base_coupling(params.DesPar_ParRob.base_method, params.DesPar_ParRob.base_par);
      RP.align_platform_coupling(params.DesPar_ParRob.platform_method, params.DesPar_ParRob.platform_par(1:end-1));
      Traj_0 = cds_transform_traj(RP, Traj_W);
      % Prüfe die Lösbarkeit der IK (Startpunkt und Trajektorie)
      [q_test,Phi]=RP.invkin_ser(Traj_0.X(1,:)', q0);
      [Q_test, PHI] = RP.invkin_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q0);
      if all(abs(Phi)<1e-6) && ~any(isnan(Phi)) && all(abs(PHI(:))<1e-6) && ~any(isnan(PHI(:)))
        fprintf('IK erfolgreich mit abgespeicherten Parametern gelöst\n');
        params_success = true; % Parameter für erfolgreiche IK geladen.
      else
        warning('IK mit abgespeicherten Parametern nicht lösbar.');
        if usr_debug
          % Erzeuge ein Bild zur Diagnose der Ursache der falschen Kinematik
          q_test(isnan(q_test)) = q0(isnan(q_test));
          change_current_figure(1);clf;hold all;
          xlabel('x in m');ylabel('y in m');zlabel('z in m'); view(3);
          axis auto
          hold on;grid on;
          s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
          RP.plot(q_test, Traj_0.X(1,:)', s_plot);
          title(sprintf('%s in Startkonfiguration',PName));
        end
      end
    else
      fprintf('Es gibt keine (gültigen) abgespeicherten Parameter. Führe Maßsynthese durch\n');
    end
    if ~params_success
      % Führe Maßsynthese neu aus. Parameter nicht erfolgreich geladen
      Set.optimization.objective = 'condition';
      % Führe Maßsynthese ohne EE-Transformation durch. Hat für dieses
      % Skript aber keine Auswirkungen.
      Set.optimization.ee_rotation = false; % darf beliebige Werte einnehmen ....
      % Set.optimization.ee_translation = false; % ... muss für Dynamik egal sein
      % Set.optimization.ee_translation_only_serial = false; % damit obige Funktion wirkt
      Set.general.debug_calc = true; % Dann auch Abbruch, wenn 3T2R nicht funktioniert.
      Set.optimization.movebase = false;
      Set.optimization.base_size = false;
      Set.optimization.platform_size = false;
      Set.optimization.obj_limit = 1e3; % Sofort abbrechen, falls Ergebnis irgendwie funktionierend (hinsichtlich IK der Beingelenke)
      Set.structures.use_parallel_rankdef = 6;
      Set.structures.whitelist = {PName}; % nur diese PKM untersuchen
      Set.structures.nopassiveprismatic = false; % Für Dynamik-Test egal 
      Set.structures.maxnumprismatic = 6; % Für Dynamik-Test egal wie viele Schubgelenke
      Set.general.noprogressfigure = true;
      Set.general.verbosity = 3;
      Set.general.nosummary = true;
      Set.optimization.optname = sprintf('dimsynth_energy_consistency_DoF_%dT%dR', sum(EE_FG(1:3)), sum(EE_FG(4:6)));
      % Set.general.create_template_functions = true; % Debug
      Traj = Traj_W;
      cds_start
      resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
      i_select = 0;
      for i = 1:length(Structures) % alle Ergebnisse durchgehen (falls mehrere theta-Varianten)
        resfile1 = fullfile(resmaindir, sprintf('Rob%d_%s_Details.mat', Structures{i}.Number, PName));
        tmp1 = load(resfile1, 'RobotOptDetails');
        resfile1 = fullfile(resmaindir, sprintf('Rob%d_%s_Endergebnis.mat', Structures{i}.Number, PName));
        tmp2 = load(resfile1, 'RobotOptRes');
        if isfield(tmp1, 'RobotOptDetails') && tmp2.RobotOptRes.fval < 1000
          i_select = i;
          RobotOptDetails = tmp1.RobotOptDetails;
          RobotOptRes = tmp2.RobotOptRes;
          break;
        end
      end
      if isempty(Structures) || i_select == 0
        % Die Methode valid_act nimmt die erstbeste bestimmbare Kinematik.
        % Die Wahl der aktuierten Gelenke muss nicht zu vollem Rang führen.
        % Kriterium ist daher nur die Bestimmbarkeit des Rangs (fval <1000)
        warning('Etwas ist bei der Maßsynthese schiefgelaufen. Keine Lösung.');
        continue
      end
      % Funktionierende Parameter abspeichern (für nächstes Mal)
      RP = RobotOptDetails.R;
      r_W_0 = RP.r_W_0;
      phi_W_0 = RP.phi_W_0;
      phi_P_E = RP.phi_P_E;
      r_P_E = RP.r_P_E;
      pkin = RP.Leg(1).pkin;
      DesPar_ParRob = RP.DesPar;
      q0 = RobotOptRes.q0;
      qlim = cat(1, RP.Leg.qlim); % Wichtig für Mehrfach-Versuche der IK
      save(paramfile_robot, 'pkin', 'DesPar_ParRob', 'q0', 'r_W_0', 'phi_W_0', 'qlim', 'r_P_E', 'phi_P_E');
      fprintf('Maßsynthese beendet\n');
      Traj_0 = cds_transform_traj(RP, Traj_W);
    end
    respath_rob = fullfile(respath, sprintf('%dT%dR', sum(EE_FG(1:3)), sum(EE_FG(4:6))), PName);
    mkdirs(respath_rob);

    %% Parameter für Dynamik-Test
    mges_PKM = rand(size(RP.DynPar.mges));
    rSges_PKM = rand(size(RP.DynPar.rSges));
    ISges_PKM = rand(size(RP.DynPar.Icges));
    RP.update_dynpar1 (mges_PKM, rSges_PKM, ISges_PKM);

    %% Dynamik-Test: Energiekonsistenz
    % Test-Konfiguration
    n = 2;
    % Gebe die Startpose der Plattform in EE-Koordinaten vor, da diese
    % Werte direkt aus den geladenen Daten der Maßsynthese kommen
    XE_test_tmp = repmat(Traj_0.XE,ceil(n/size(Traj_0.XE,1)),1);
    XE_test_tmp = XE_test_tmp(1:n,:);
    XED_test = NaN(n, 6); % wird später belegt
    % Rechne in Plattform-KS um (welches für die Dynamik benutzt wird).
    % Benutze dafür noch die ursprüngliche Transformation P-E
    XP_test = RP.xE2xP_traj(XE_test_tmp);
    % Setze eine beliebige EE-Transformation. Diese Werte haben keinen
    % Einfluss auf die weitere Berechnung, da die Dynamik nur mit dem
    % Plattform-KS berechnet wird.
    r_P_E   = [0.1;0.2;0.3];
    phi_P_E = [45; 35; 10]*pi/180;
    RP.update_EE(r_P_E, phi_P_E);
    % Setze eine beliebige Basis-Transformation. Dürfte auch keinen
    % Einfluss haben. Nur gering, damit Plot noch erkennbar.
    phi_W_0 = [5;-8;12]*pi/180;
    r_W_0 = RP.r_W_0;
    RP.update_base(r_W_0, phi_W_0)
    % Berechne die EE-Transformation neu (mit eventuell geänderter
    % EE-Trafo P-E)
    XE_test = RP.xP2xE_traj(XP_test);
    XP_test2 = RP.xE2xP_traj(XE_test);
    if any(abs(XP_test2(:)-XP_test(:))>1e-6)
      error('Hin- und Rücktransformation KS P/E fehlerhaft');
    end
    Q_test = NaN(n, RP.NJ);
    QD_test = NaN(n, RP.NJ);
    % Definiere eine beliebige Geschwindigkeit, aber im Plattform-KS
    XPD_test = rand(n, 6);
    XPD_test(:,~RP.I_EE) = 0; % nicht belegte FG haben keine Geschw.
    % IK für Testkonfiguration berechnen
    for i = 1:n
      % IK mit EE-Pose berechnen
      [q, Phi, ~, Stats] = RP.invkin_ser(XP_test(i,:)', q0, s, ...
        struct('platform_frame', true));
      
      if any(abs(Phi) > 1e-8) || any(isnan(Phi))
        XE_test(i,:) = NaN; XP_test(i,:) = NaN;
        continue;
      end
      % Debug: Optische Prüfung, ob Ergebnis richtig ist:
%       change_current_figure(6666); clf; hold all;
%       s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], ...
%         'ks_platform', RP.NLEG+1, 'straight', 0);
%       xlabel('x in m');ylabel('y in m');zlabel('z in m'); view(3);
%       axis auto; hold on; grid on;
%       RP.plot(q, XE_test(i,:)', s_plot);

      if all(RP.I_EE == [1 1 1 1 1 0])
        % Plattform- und EE-Pose neu berechnen (wegen drittem Euler-Winkel,
        % der sich evtl ändert). Die jeweils andere Pose (Position und
        % Orientierung) ändert sich vollständig (alle sechs Komponenten).
        % Ursache: Die freie Drehung um die z-Achse wirkt mit dem Hebel
        % r_P_E auf den Endeffektor
        xE_i = RP.fkineEE_traj(q', [], [], 1, false);
        XE_test(i,:) = xE_i(:);
        xP_i = RP.fkineEE_traj(q', [], [], 1, true);
        if any(abs(xP_i(1:5) - XP_test(i,1:5)) > 1e-6)
          error('Plattform-Position und Zeigerichtung hat sich durch EE-Trafo geändert');
        end
        % Der dritte Euler-Winkel kann sich gegenüber der Vorgabe ändern;
        % sollte aber konstant bleiben (keine Geschwindigkeit)
        XP_test(i,6) = xP_i(6);
      end
      % Prüfe, ob direkte Kinematik übereinstimmt
      [Tc_Pges_0,Tc_Pges_W] = RP.fkine_platform(XE_test(i,:)');
      [TcLges_0,TcLges_W] = RP.fkine_legs(q);
      Tc_B_q = NaN(4,4,RP.NLEG);
      k = 0;
      for iLeg = 1:RP.NLEG
        NLL = RP.Leg(iLeg).NL;
        k = k + NLL+1;
        T_B_q = TcLges_0(:,:,k);
        T_B_x = Tc_Pges_0(:,:,iLeg);
        test_T_B = T_B_q\T_B_x-eye(4);
        if any(abs(test_T_B(:))>1e-6)
          error('Direkte Kinematik der Beinketten stimmt nicht mit Plattform überein');
        end
      end
      % Testweise inverse Kinematik mit Plattform-Pose berechnen. Ziel:
      % Dynamik-Funktionen sollen mit Größen im  Plattform-KS funktionieren.
      % (Benötigt Aktualisierung des Plattform-KS bei 3T2R)
      [q2, Phi2, ~, Stats2] = RP.invkin_ser(XE_test(i,:)', q, s, ...
        struct('platform_frame', false));
      if any(abs(q-q2)>1e-8) % Es muss das gleiche Ergebnis rauskommen.
        change_current_figure(5342);clf;
        s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], ...
          'ks_platform', RP.NLEG+1, 'straight', 0);
        subplot(1,2,1); hold all;
        xlabel('x in m');ylabel('y in m');zlabel('z in m'); view(3);
        axis auto; hold on; grid on;
        RP.plot(q, XE_test(i,:)', s_plot);
        title('IK mit Plattform-KS-KS (q)');
        subplot(1,2,2); hold all;
        xlabel('x in m');ylabel('y in m');zlabel('z in m'); view(3);
        axis auto; hold on; grid on;
        RP.plot(q2, XE_test(i,:)', s_plot);
        title('IK mit Endeffektor (q2)');
        error('IK mit KS P oder KS E gibt anderes Ergebnis. Fehler: %1.1e', max(abs(q-q2)));
      end

      Q_test(i,:) = q;
      % Berechne die Geschwindigkeit mit der Jacobi-Matrix. Beziehe die
      % Matrix dafür auf das Plattform-KS (und nicht das EE-KS)
      if ~all(RP.I_EE == [1 1 1 1 1 0])
        [~, Jinv_voll] = RP.jacobi_qa_x(q, XP_test(i,:)', true);
      else % 3T2R: Eigene Modellierung
        G2_q = RP.constr2grad_q(q, XP_test(i,:)', true);
        G2_x = RP.constr2grad_x(q, XP_test(i,:)', true);
        Jinv_voll = -G2_q\G2_x;
      end
      QD_test(i,:) = Jinv_voll*XPD_test(i,RP.I_EE)';
      if all(RP.I_EE == [1 1 1 1 1 0])
        % Berechne die EE-Geschwindigkeiten neu (notwendig)
        [xE_i, xDE_i] = RP.fkineEE_traj(q', QD_test(i,:), [], 1, false);
        XE_test(i,:) = xE_i;
        XED_test(i,:) = xDE_i;
        [XP_i, xDP_i] = RP.fkineEE_traj(q', QD_test(i,:), [], 1, true);
        if any(abs(XP_test(i,1:6) - XP_i(1:6)) > 1e-6) % 6. Eintrag wurde oben schon geändert
          error('Plattform-Lage xP hat sich verändert nach Geschw.-Berechnung');
        end
        % Trage die Zeitableitung des dritten Euler-Winkels ein. Kann sich
        % ändern, da bei fünf FG nicht steuerbar. Ist notwendig für eine
        % der folgenden Prüfungen, die auf 3T3R basieren.
        XPD_test(i,6) = xDP_i(6);
      end
      % Die Berechnung der Geschwindigkeit kann durch Singularitäten
      % numerische schwierig sein. Nur auf Fehler testen, falls Geschw.
      % klein.
      if all(RP.I_EE == [1 1 1 1 1 0]) && max(abs(xDP_i)) < 1e6 && ...
          max(abs(QD_test(i,:))) < 1e6
        test_xPD = XPD_test(i,1:5) - xDP_i(1:5);
        if any(abs(test_xPD) > 1e-5)
          error(['Geschwindigkeit xPD hat sich verändert nach Geschw.-', ...
            'Berechnung. Fehler: %1.1e'], max(abs(test_xPD)));
        end
        % Ab hier Debuggen für 3T2R-PKM
        % Direkte Berechnung mit Jacobi-Matrix, die zur Herleitung diente
        Phi2D_test = G2_q*QD_test(i,:)' + G2_x*XPD_test(i,RP.I_EE)';
        if any(abs(Phi2D_test) > 1e-10)
          error('Herleitung qD/xD stimmt nicht. Fehler %1.1e', max(abs(Phi2D_test)));
        end
        % Geschwindigkeit nochmal neu mit ZB-Def. 2 (Plattform-KS)
        G2P_q = RP.constr2grad_q(q, XP_test(i,:)', true);
        G2P_x = RP.constr2grad_x(q, XP_test(i,:)', true);
        Jinv2P_voll = -G2P_q\G2P_x;
        qD2P = Jinv2P_voll*XPD_test(i,RP.I_EE)';
        % Geschwindigkeit auch mit ZB-Def. 2 und EE-KS
        [~, G2E_q_voll] = RP.constr2grad_q(q, XE_test(i,:)', false);
        [~, G2E_x_voll] = RP.constr2grad_x(q, XE_test(i,:)', false);
        Jinv2E_voll = -G2E_q_voll\G2E_x_voll;
        qD2E = Jinv2E_voll*xDE_i';
        if any(abs(qD2P - qD2E) > 1e-6) || any(isnan([qD2P;qD2E]))
          error('Gelenk-Geschwindigkeit stimmt nicht zwischen ZB Methode 2P vs 2E.');
        end
        % Geschwindigkeit aus anderer ZB-Definition (Plattform-KS)
        [G3P_q, G3P_q_voll] = RP.constr3grad_q(q, XP_test(i,:)', true);
        [G3P_x, G3P_x_voll] = RP.constr3grad_x(q, XP_test(i,:)', true);
        Jinv3P_voll = -G3P_q_voll\G3P_x_voll;
        qD3P = Jinv3P_voll*xDP_i';
        if any(abs(qD3P - qD2P) > 1e-6) || any(isnan([qD3P;qD2P]))
          error('Gelenk-Geschwindigkeit stimmt nicht zwischen ZB Methode 2P und 3P. 3T2R-PKM ungültig.');
        end
        Jinv3P_voll2 = -G3P_q\G3P_x(:,RP.I_EE);
        qD3P2 = Jinv3P_voll2*XPD_test(i,RP.I_EE)';
        if any(abs(qD3P - qD3P2) > 1e-6) || any(isnan([qD3P;qD3P2]))
          error('Gelenk-Geschwindigkeit stimmt nicht mit Methode 3P.');
        end
        % Geschwindigkeit aus anderer ZB-Definition (EE-KS). Hier muss der
        % vollständige EE-Geschwindigkeitsvektor benutzt werden. Es können
        % nicht die sechsten Komponenten von xP und xE Null sein.
        [G3E_q, G3E_q_voll] = RP.constr3grad_q(q, XE_test(i,:)', false);
        [G3E_x, G3E_x_voll] = RP.constr3grad_x(q, XE_test(i,:)', false);
        Jinv3E_voll = -G3E_q_voll\G3E_x_voll;
        qD3E = Jinv3E_voll*xDE_i';
        if any(abs(qD3E - qD2E) > 1e-6) || any(isnan([qD3E;qD2E]))
          error('Gelenk-Geschwindigkeit stimmt nicht zwischen ZB Methode 2E und 3E. 3T2R-PKM ungültig.');
        end
        Jinv3E_voll2 = -G3E_q\G3E_x;
        qD3E2 = Jinv3E_voll2*XED_test(i,RP.I_EE)';
        if any(abs(qD3P - qD3E) > 1e-6) || any(isnan([qD3P;qD3E]))
          error('Gelenk-Geschwindigkeit stimmt nicht zwischen ZB Methode 3P vs 3E.');
        end
      end
    end
    % Bei singulären PKM ist bereits die Gelenkgeschwindigkeit unendlich,
    % obwohl die Plattform-Geschwindigkeit klein ist. Damit ist es schwer
    % unten weiterzurechnen.
    if any(abs(QD_test(:)) > 1e8)
      warning('Geschwindigkeit wird zu groß. Keine Anfangsgeschw. setzen.');
      XPD_test(:) = 0;
      XED_test(:) = 0;
      QD_test(:) = 0;
    end
    % Teste, ob die Start-Konfiguration überhaupt richtig ist. Der dritte
    % Euler-Winkel wurde oben schon korrigiert.
    if all(RP.I_EE_Task == [1 1 1 1 1 0])
      for j = 1:RP.NLEG
        [XP_korr, XPD_korr] = RP.fkineEE2_traj(Q_test, QD_test, 0*QD_test, j, true);
        [XE_korr, XED_korr] = RP.fkineEE2_traj(Q_test, QD_test, 0*QD_test, j, false);
        % Vergleiche diese Beinkette gegen die Plattform (bzw. den EE)
        test_XP = XP_korr(:,1:6) - XP_test(:,1:6);
        test_XPD = XPD_korr(:,1:6) - XPD_test(:,1:6);
        test_XE = XE_korr(:,1:6) - XE_test(:,1:6);
        test_XED = XED_korr(:,1:6) - XED_test(:,1:6);
        test_XP([false(size(test_XP,1),3),abs(abs(test_XP(:,4:end))-2*pi)<1e-3]) = 0; % 2pi-Fehler entfernen
        test_XE([false(size(test_XE,1),3),abs(abs(test_XE(:,4:end))-2*pi)<1e-3]) = 0;
        if max(abs(test_XP(:))) > 1e-6
          error('Plattform-Trajektorie (X) stimmt nicht zwischen Beinkette %d und Referenz überein', j);
        end
        if max(abs(test_XPD(:))) > 1e-6
          error('Plattform-Trajektorie (XD) stimmt nicht zwischen Beinkette %d und Referenz überein', j);
        end
        if max(abs(test_XE(:))) > 1e-6
          error('Endeffektor-Trajektorie (X) stimmt nicht zwischen Beinkette %d und Referenz überein', j);
        end
        if max(abs(test_XED(:))) > 1e-6
          error('Endeffektor-Trajektorie (XD) stimmt nicht zwischen Beinkette %d und Referenz überein', j);
        end
      end
    end
    % Prüfe mit anderer Methode, ob Geschwindigkeit korrekt berechnet wurde
    PHID_testP = RP.constr4D2_traj(Q_test, QD_test, XP_test, XPD_test, true);
    PHID_testE = RP.constr4D2_traj(Q_test, QD_test, XE_test, XED_test);
    if any(abs(PHID_testP(:)) > 1e-6) || any(abs(PHID_testE(:)) > 1e-6)
      error(['Anfangswerte der Vorwärtsdynamik sind nicht konsistent. ', ...
        'Fehler in constr4D max. %1.1e (KS P) bzw. %1.1e (KS E)'], ...
        max(abs(PHID_testP(:))), max(abs(PHID_testE(:))));
    end
    if usr_debug
      change_current_figure(2);clf;hold all;
      xlabel('x in m');ylabel('y in m');zlabel('z in m'); view(3);
      axis auto
      hold on;grid on;
      s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], ...
        'ks_platform', RP.NLEG+1, 'straight', 0);
      RP.plot(Q_test(1,:)', XE_test(1,:)', s_plot);
      title(sprintf('%s in Testkonfiguration 1',PName));
    end
    if ~all(EE_FG == [1 1 1 1 1 0])
      % Benutze für normale PKM die Zwangsbedingungs-Modellierungen Nr. 1
      % und 4.
      jacobi_modes = [1 2];
    else
      % Benutze für 3T2R-PKM nur die Jacobi-Matrix aus Zwangsbedingung Nr.
      % 2. Alle anderen führen zu einer nicht-konsistenten Kinematik
      jacobi_modes = 3;
    end
    for jm = jacobi_modes % Dynamik mit verschiedenen Arten von Jacobi-Matrix berechnen
    % Dynamik berechnen (mit den Zufallswerten)
    n_succ = 0;
    n_fail = 0;
    for i_tk = 1:n % Testkonfigurationen ("tk")
      if any(isnan(XE_test(i_tk,:))), continue; end % IK für diese Pose nicht erfolgreich
      
      if i_tk > 1
        RP.update_base([0;0;1], rand(3,1));
      end
      RP.update_gravity([0;0;-9.81]);
      % Anfangswerte für iterativen Algorithmus
      q0 = Q_test(i_tk,:)'; % Anfangswert für t=0 bzw. i=0
      qD0 = NaN*q0; % wird in fdyn neu berechnet.
      xP = XP_test(i_tk,:)'; % Anfangswert (passend zu q)
      if any(abs(xP(~RP.I_EE)) > 1e-7) && ~all(EE_FG == [1 1 1 1 1 0])
        % Bei 3T2R-PKM kann der sechste FG einen konstanten Wert haben.
        % Hängt davon ab, ob die Montage der Beinketten verdreht ist.
        error('EE-Pose zwischen EE und Plattform-KS ist nicht konsistent');
      end
      xP_red0 = xP(RP.I_EE);
      xPD = XPD_test(i_tk,:)'; % Anfangswert (Geschwindigkeit t=0)
      xPD_red0 = xPD(RP.I_EE);
      T_end = 0.5;
      % Feder in passive Gelenke einsetzen
      if usr_jointspring
        for k = 1:RP.NLEG
          I_k = RP.I1J_LEG(k):RP.I2J_LEG(k);
          % Ruhelage der Feder ist die Startkonfiguration
          RP.Leg(k).DesPar.joint_stiffness_qref = q(I_k);
          % Federsteifigkeit auf moderaten Wert
          RP.Leg(k).DesPar.joint_stiffness = ones(length(I_k),1)*100;
        end
      end
      dt = 1000e-6;
      %% Vorwärtsdynamik mit ode45 berechnen
      fprintf('Berechne Vorwärtsdynamik mit ode45\n'); t1=tic();
      fdynstruct = struct( ...
        'q0', q0, ...
        'x0red', xP_red0, ...
        'qD0', qD0, ...
        'xD0', xPD_red0, ...
        'dtmax', dt, ...
        't_End', T_end, ...
        'J_method', jm);
      fdynoutput = RP.fdyn(fdynstruct);
      nt = length(fdynoutput.Tges);
      fprintf(['Vorwärtsdynamik mit ode45 gerechnet. %d Schritte in %1.3fs. ', ...
        'Echtzeitfaktor %1.1f%%.\n'], nt, toc(t1), 100*T_end/toc(t1));

      %% Ergebnisse auslesen und weitere Variablen initialisieren
      Tges = fdynoutput.Tges;     
      E_ges = NaN(nt,4+2+RP.NLEG*3); % 4 gesamt, 2 Plattform, 3 für jede Beinkette (s.u.)
      Q_ges = fdynoutput.Qges;
      QD_ges = fdynoutput.QDges;
      XP_ges = zeros(nt,6);
      XPD_ges = zeros(nt,6);
      XP_ges(:,RP.I_EE) = fdynoutput.XPredges;
      XPD_ges(:,RP.I_EE) = fdynoutput.XPDredges;
      XPDD_ges = NaN(nt,6);
      PHI_ges = NaN(nt, RP.I2constr_red(end));
      PHI_ges2 = NaN(nt, RP.I2constr_red(end));
      QDD_ges = NaN(nt,RP.NJ);
      SingDet = NaN(nt,6);
      Facc_ges = NaN(nt,6);
      P_diss_ges = zeros(nt,3);
      %% Nachverarbeitung der Ergebnisse
      t2 = tic();
      for i = 1:nt
        q_ode = Q_ges(i,:)';
        qD_ode = QD_ges(i,:)';
        xP = XP_ges(i,:)';
        xPD = XPD_ges(i,:)';
        xPD_red = xPD(RP.I_EE);
        if any(isnan([q_ode;qD_ode;xP;xPD]))
          warning(['i=%d/%d: Abbruch der Berechnung in ode45. Vermutlich ', ...
            'aufgrund von IK-Inkonsistenz.'], i, nt);
          nt = i - 1;
          break
        end   
        % Prüfe, ob Zwangsbedingungen noch stimmen und berechne korrigierte
        % Gelenkwinkel, da in ODE Integrationsfehler auftreten können (und
        % dort intern auch mit anderen Winkeln gerechnet wird)
        [q, Phi] = RP.invkin_ser(xP, q_ode, s, struct('platform_frame',true));
        % Plattform-Pose neu berechnen (3. Euler-Winkel; eventuell bei
        % manchen PKM abhängig und ungleich Null)
        xP = RP.fkineEE_traj(q', [], [], 1, true)';
        if any(isnan(q)) || any(abs(Phi) > 1e-6) || any(isnan(Phi))
          warning('i=%d/%d: IK kann nicht berechnet werden. Roboter verlässt vermutlich den Arbeitsraum.', i, nt);
          nt = i - 1;
          break
        end
        if i == 1 && any(abs(xP - XP_test(i_tk,:)') > 1e-10)
          error('Anfangswerte in der Vorwärtsdynamik stimmen nicht');          
        end
        
        % Kinematik für Zeitschritt i berechnen:
        % Berechnung der Gelenk-Geschwindigkeit und Jacobi-Matrix
        [G1_q, G1_q_voll] = RP.constr1grad_q(q, xP, true); % Für Konditionszahl weiter unten
        [G1_x, G1_x_voll] = RP.constr1grad_x(q, xP, true);
        G2_q = RP.constr2grad_q(q, xP, true);
        G2_x = RP.constr2grad_x(q, xP, true);
        [G4_q, G4_q_voll] = RP.constr4grad_q(q);
        [G4_x, G4_x_voll] = RP.constr4grad_x(xP, true);
        if jm == 1 % Nehme Euler-Winkel-Jacobi für Dynamik
          Jinv_voll = -G1_q\G1_x; % Jacobi-Matrix als Hilfe für Dynamik-Fkt speichern
        elseif jm == 2  % Nehme neue Modellierung der Jacobi für die Dynamik
          Jinv_voll = -G4_q\G4_x;
        else % Modellierung für 3T2R-PKM
          % Kann nicht erkennen, wenn die PKM ungültig ist. Dafür wird
          % constr3 benötigt. Annahme: Alle PKM die hier aus der PKM-Daten-
          % bank genommen werden, funktionieren. Nehmer daher constr2.
          Jinv_voll = -G2_q\G2_x;
        end
        % Berechne Gelenkwinkel neu (und ignoriere die aus der ode45).
        qD = Jinv_voll*xPD_red;
        % Berechne Plattform-Geschwindigkeit neu (für 3T2R). Zusätzlicher Test.
        if all(RP.I_EE_Task == [1 1 1 1 1 0])
          for j = 1:RP.NLEG
            [xP_korrT, xPD_korrT] = RP.fkineEE_traj(q', qD', 0*qD', j, true); 
            if j == 1
              test_x = xP_korrT(1:5)' - xP(1:5);
              test_xD = xPD_korrT(1:5)' - xPD(1:5);
              xP = xP_korrT';
              xPD = xPD_korrT';
            else
              test_x = xP_korrT(1:6)' - xP(1:6);
              test_xD = xPD_korrT(1:6)' - xPD(1:6);
            end
            test_x([false(size(test_x,1),3),abs(abs(test_x(:,4:end))-2*pi)<1e-3]) = 0; % 2pi-Fehler entfernen
            if max(abs(test_x(:))) > 1e-6 || max(abs(test_xD(:))) > 1e-6
              error('Plattform-Trajektorie stimmt nicht zwischen verschiedenen Beinkette %d und Referenz überein', j);
            end
          end
        end
        % Berechne Jacobi-Zeitableitung (für Dynamik-Berechnung des nächsten Zeitschritts)
        if jm == 1
          GD1_q = RP.constr1gradD_q(q, qD, xP, xPD, true);
          GD1_x = RP.constr1gradD_x(q, qD, xP, xPD, true);
          JinvD_voll = G1_q\(GD1_q*(G1_q\G1_x)) - G1_q\GD1_x; % effizienter hier zu berechnen als in Dynamik
        elseif jm == 2
          % Nehme Modellierung 4 der Jacobi für die Dynamik
          GD4_q = RP.constr4gradD_q(q, qD);
          GD4_x = RP.constr4gradD_x(xP, xPD, true);
          JinvD_voll = G4_q\(GD4_q*(G4_q\G4_x)) - G4_q\GD4_x;
        else
          GD2_q = RP.constr2gradD_q(q, qD, xP, xPD, true);
          GD2_x = RP.constr2gradD_x(q, qD, xP, xPD, true);
          JinvD_voll = G2_q\(GD2_q*(G2_q\G2_x)) - G2_q\GD2_x;
        end

        % Kennzahlen für Singularitäten
        if ~all(EE_FG == [1 1 1 1 1 0])
          SingDet(i,1) = cond(G1_q);
          SingDet(i,2) = cond(G1_x);
          SingDet(i,3) = cond(G4_q);
          SingDet(i,4) = cond(G4_x);
        else
          % Für 3T2R-PKM sind die reduzierten ZB der Var. 1 und 4 nicht
          % definiert.
          SingDet(i,1) = cond(G1_q_voll);
          SingDet(i,2) = cond(G1_x_voll);
          SingDet(i,3) = cond(G4_q_voll);
          SingDet(i,4) = cond(G4_x_voll);
        end
        SingDet(i,5) = cond(G2_q);
        SingDet(i,6) = cond(G2_x);
        %% Dynamik und Energie
        % Dynamik-Terme berechnen (Beschleunigung und Kraft im Zeitschritt i)
        Mx = RP.inertia2_platform(q, xP, Jinv_voll);
        Gx = RP.gravload2_platform(q, xP, Jinv_voll);
        Cx = RP.coriolisvec2_platform(q, qD, xP, xPD, Jinv_voll, JinvD_voll);
        Kx = RP.jointtorque_platform(q, xP, RP.springtorque(q), Jinv_voll);
        Facc_ges(i,RP.I_EE) = -Gx - Cx - Kx;
  
        % Beschleunigung der Plattform berechnen
        xPDD_red = Mx \ (-Gx - Cx - Kx);
        XPDD = zeros(6,1); % 6x1-Vektor, falls reduzierte Plattform-FG
        XPDD(RP.I_EE) = xPDD_red;
        % Beschleunigung der Beingelenke
        qDD = Jinv_voll*xPDD_red + JinvD_voll*xPD_red;
        
        % Berechnung der Systemenergie
        [T_i, T_legs_i, T_platform_i] = RP.ekin(q, qD, xP, xPD);
        [Ugrav_i, Ugrav_legs_i, Ugrav_platform_i] = RP.epot(q, xP);
        [Uspr_i, Uspr_legs_i] = RP.epotspring(q);
        %% Werte für diesen Zeitschritt abspeichern
        E_ges(i,1:4) = [T_i, Ugrav_i, Uspr_i, T_i+Ugrav_i+Uspr_i];
        E_ges(i,5:5+RP.NLEG) = [T_platform_i; T_legs_i];
        E_ges(i,5+RP.NLEG+1:5+RP.NLEG*2+1) = [Ugrav_platform_i; Ugrav_legs_i];
        E_ges(i,5+RP.NLEG*2+2:5+RP.NLEG*3+1) = Uspr_legs_i;
        XP_ges(i,:) = xP;
        XPD_ges(i,:) = xPD;
        XPDD_ges(i,:) = XPDD;
        Q_ges(i,:) = q;
        QD_ges(i,:) = qD;
        QDD_ges(i,:) = qDD;
        
        % Berechne die zwischen den Teilsystemen umgesetzte Energie als Leistung
        % Der numerische Fehler entspricht Dissipation
        if i > 1
          P_diff = (E_ges(i, :) - E_ges(i-1, :)) / (Tges(i)-Tges(i-1)); % Leistung aus Energiedifferenz
          p_komp = sum(abs(P_diff([1 2 3 5:end]))); % Summe der Differenzen aller Teilsysteme (Beinketten, Plattform (kinetisch/potentiell)); entspricht umgesetzter Leistung
          p_sys = abs(P_diff(4)); % Differenz der Gesamt-Energie (entspricht Dissipation)
          P_diss_ges(i,1:2) = [p_sys, p_komp];
          % Gleitender Maximalwert des Leistungsflusses zwischen den
          % Komponenten. Dadurch wird die fehlerbedingte Dissipation ins
          % Verhältnis gesetzt.
          P_diss_ges(i,3) = max(P_diss_ges(1:i,2));
        end
        
        % Kinematische Zwangsbedingungen prüfen
        if ~all(EE_FG == [1 1 1 1 1 0])
          PHI_ges(i,:) = RP.constr1(q, xP, true);
        else
          % Nehme constr3, um sicher festzustellen, ob die PKM noch konsistent ist.
          PHI_ges(i,:) = RP.constr3(q, xP, true);
        end
        PHI_ges2(i,:) = Phi; % Residuum aus IK oben
        
        %% Zusätzliche Prüfung für constr4 vs constr1 (Optional)
        if ~all(EE_FG == [1 1 1 1 1 0]) && ... % nicht für 3T2R-PKM testen
          cond(G1_q) < 1e2 && usr_test_constr4_JinvD % nicht bei Singularität testen
             
          % Zusätzlicher Test: Prüfe andere Berechnung der Jacobi-Matrix
          % Dafür müssen die korrigierten IK-Gelenkwinkel genommen werden
          % Berechne Einfache Gradienten und Jacobi-Matrizen mit beiden
          % Methoden:
          G4_q = RP.constr4grad_q(q);
          G4_x = RP.constr4grad_x(xP);
          Jinv_voll4 = -G4_q\G4_x;
          G1_q = RP.constr1grad_q(q,xP);
          G1_x = RP.constr1grad_x(q,xP);
          Jinv_voll1 = -G1_q\G1_x;
          if ~all(EE_FG==[1 1 1 1 1 0])
          % Vergleiche gegen symbolische Herleitung der Jacobi-Matrix
          % (existiert nicht für 3T2R)
          Jinv_sym_qa_x = RP.jacobi_qa_x(q, xP);
          Jinv4_num_qa_x = Jinv_voll4(RP.I_qa,:);
          Jinv1_num_qa_x = Jinv_voll1(RP.I_qa,:);
          if any(abs(Jinv_sym_qa_x(:)-Jinv1_num_qa_x(:))>1e-2) % TODO: Noch hohe Toleranz
            error('Jacobi-Matrix nach Methode 4 stimmt nicht gegen symbolisch generierte');
          end
          if any(abs(Jinv_sym_qa_x(:)-Jinv4_num_qa_x(:))>1e-2)
            error('Jacobi-Matrix nach Methode 1 stimmt nicht gegen symbolisch generierte');
          end
          end
          % Vergleiche absoluten und relativen Fehler (bei kleinen
          % absoluten Werten größerer Einfluss von Rundungsfehlern)
          test_J_abs = abs(Jinv_voll4 - Jinv_voll1);
          test_J_rel = test_J_abs ./ Jinv_voll1;
          I_relerr = abs(test_J_rel) > 5e-2; % Indizes mit Fehler größer 5%
          I_abserr = test_J_abs > 2e10*eps(1+max(abs(Jinv_voll1(:)))); % Absoluter Fehler über Toleranz von min. 1e-4
          if any( I_relerr(:) & I_abserr(:) ) % Fehler bei Überschreitung von absolutem und relativem Fehler
            error('Modell 4 für die Jacobi-Matrix stimmt nicht gegen Modell 1');
          end
          % Alle Zeitableitungen der ZB-Gradienten mit Methode 1 und 4
          GD4_q = RP.constr4gradD_q(q, qD);
          GD4_x = RP.constr4gradD_x(xP, xPD);
          GD1_q = RP.constr1gradD_q(q, qD, xP, xPD);
          GD1_x = RP.constr1gradD_x(q, qD, xP, xPD);
          JinvD_voll1 = G1_q\(GD1_q*(G1_q\G1_x)) - G1_q\GD1_x;
          JinvD_voll4 = G4_q\(GD4_q*(G4_q\G4_x)) - G4_q\GD4_x;
          % Vergleiche Jacobi-Zeitableitung nach beiden Methoden
          JinvD_voll1(abs(JinvD_voll1(:))<1e-10) = 0;
          JinvD_voll4(abs(JinvD_voll4(:))<1e-10) = 0;
          test_JD_abs = JinvD_voll4 - JinvD_voll1;
          test_JD_rel = test_JD_abs ./ JinvD_voll1;
          test_JD_abs(abs(test_JD_abs(:))<1e-4) = 0; % zur einfacheren Anzeige
          % Absoluter Fehler bis 2e-3 und relativer Fehler 5% ist kritisch
          % (gleichzeitig. Ansonsten bei kleinen Zahlenwerten hoher
          % relativer Fehler oder bei großen Zahlenwerten hoher absoluter)
          test_JD = abs(test_JD_abs) > 2e-3 & abs(test_JD_rel) > 5e-2;
          if any(test_JD(:))
            error('i=%d/%d: Modell 4 für die Jacobi-Matrix stimmt nicht gegen Modell 1', i, nt);
          end        
        end
      end
      if nt == 0
        warning('Bereits der Startwert konnte nicht berechnet werden');
        continue;
      end
      Ip_end = nt;
      RTratio = Tges(Ip_end)/toc(t2);
      fprintf(['Dauer für Nachverarbeitung von %d Simulationsschritten (%1.2fs simulierte Zeit): ', ...
        '%1.2fs (%1.0f%% Echtzeit)\n'], Ip_end, Tges(Ip_end), toc(t2), 100*RTratio);
      %% Konsistenz der PKM berechnen
      % Falls es hier einen Fehler gibt, hat die differentielle Kinematik
      % oben nicht funktioniert. PKM-EE für jede Beinkette einzeln berechnen
      X_fromlegs = NaN(Ip_end, 6, RP.NLEG);
      XD_fromlegs = NaN(Ip_end, 6, RP.NLEG);
      XDD_fromlegs = NaN(Ip_end, 6, RP.NLEG);
      for j = 1:RP.NLEG
        [X_fromlegs(1:Ip_end,:,j),XD_fromlegs(1:Ip_end,:,j),XDD_fromlegs(1:Ip_end,:,j)] = ...
          RP.fkineEE_traj(Q_ges(1:Ip_end,:), QD_ges(1:Ip_end,:), QDD_ges(1:Ip_end,:), j);
        if j > 1
          % Teste nur bis zum fünftletzten Zeitpunkt. Die letzten können
          % schon durch eine Singularität beeinträchtigt sein
          Ip_testend = max(1,Ip_end-5);
          test_X = X_fromlegs(1:Ip_testend,1:5, 1) - X_fromlegs(1:Ip_testend,1:5, j);
          test_X(abs(abs(test_X)-2*pi) < 1e-2) = 0; %2pi-Fehler entfernen
          if any(abs(test_X(:))>1e-6)
            Ifirst = find(any(abs(test_X)>1e-6,2),1,'first');
            error(['Die Endeffektor-Trajektorie X aus Beinkette %d stimmt nicht ', ...
              'gegen Beinkette 1. Zuerst in Zeitschritt %d/%d. Fehler dort %1.2e'], j, ...
              Ifirst, nt, max(abs(test_X(Ifirst,:))));
          end
          test_XD_abs = XD_fromlegs(1:Ip_testend,1:6,1) - XD_fromlegs(1:Ip_testend,1:6,j);
          test_XD_rel = test_XD_abs ./ XD_fromlegs(1:Ip_testend,1:6,1);
          if any(abs(test_XD_abs(:))>1e-6 & abs(test_XD_rel(:))>1e-2)
            error(['Die Endeffektor-Trajektorie XD aus Beinkette %d stimmt nicht ', ...
              'gegen Beinkette 1. Zuerst in Zeitschritt %d/%d.'], j, ...
              find(any(abs(test_XD_abs)>1e-6,2),1,'first'), nt);
          end
          test_XDD_abs = XDD_fromlegs(1:Ip_testend,1:6,1) - XDD_fromlegs(1:Ip_testend,1:6,j);
          test_XDD_rel = test_XDD_abs ./ XDD_fromlegs(1:Ip_testend,1:6,1);
          if any(abs(test_XDD_abs(:))>1e-6 & abs(test_XDD_rel(:))>1e-2)
            error(['Die Endeffektor-Trajektorie XDD aus Beinkette %d stimmt nicht ', ...
              'gegen Beinkette 1. Zuerst in Zeitschritt %d/%d.'], j, ...
              find(any(abs(test_XDD_abs)>1e-6,2),1,'first'), nt);
          end
        end
      end
      % Prüfe, ob die Integration der Zeitableitungen konsistent ist
      Q_ges_num = repmat(Q_ges(1,:),size(Q_ges,1),1)+cumtrapz(Tges, QD_ges);
      QD_ges_num = zeros(size(Q_ges));
      QD_ges_num(1,:) = qD0;
      QD_ges_num(2:end,RP.MDH.sigma==1) = diff(Q_ges(:,RP.MDH.sigma==1))./...
        repmat(diff(Tges), 1, sum(RP.MDH.sigma==1)); % Differenzenquotient
      QD_ges_num(2:end,RP.MDH.sigma==0) = (mod(diff(Q_ges(:,RP.MDH.sigma==0))+pi, 2*pi)-pi)./...
        repmat(diff(Tges), 1, sum(RP.MDH.sigma==0)); % Siehe angdiff.m
      QD_ges_num2 = repmat(QD_ges(1,:),size(QD_ges,1),1)+cumtrapz(Tges, QDD_ges);
      %% Weitere Berechnungen
      % Gleitender Maximalwert des Leistungsflusses zwischen den
      % Komponenten. Dadurch wird die fehlerbedingte Dissipation ins
      % Verhältnis gesetzt. Funktioniert sonst nicht, wenn der Wert gegen
      % Null geht.
      for i = 1:nt
        P_diss_ges(i,3) = max(P_diss_ges(1:i,2));
      end

      if usr_plot_figures
        %% Plot: Energie
        change_current_figure(3);clf;
        kompleg = {'Plattform'};
        for ileg = 1:RP.NLEG
          kompleg = {kompleg{:}, sprintf('Beinkette %d', ileg)}; %#ok<CCAT>
        end
        subplot(2,3,1);hold on;
        plot(Tges(1:Ip_end), E_ges(1:Ip_end,1));
        plot(Tges(1:Ip_end), E_ges(1:Ip_end,2));
        plot(Tges(1:Ip_end), E_ges(1:Ip_end,3));
        plot(Tges(1:Ip_end), E_ges(1:Ip_end,4));
        legend({'Kinetisch', 'Grav-Pot.', 'Feder-Pot.', 'Gesamt'});
        grid on; ylabel('Teil-Energien in J');
        subplot(2,3,2);
        plot(Tges(1:Ip_end), E_ges(1:Ip_end,4)-E_ges(1,4));
        grid on; ylabel('Energiedifferenz in J (bezgl t=0)');
        subplot(2,3,3);
        plot(Tges(1:Ip_end), E_ges(1:Ip_end,5:5+RP.NLEG));
        grid on; ylabel('Kinetische Energie der Komponenten in J');
        legend(kompleg);
        subplot(2,3,4);
        plot(Tges(1:Ip_end), E_ges(1:Ip_end,5+RP.NLEG+1:5+RP.NLEG*2+1));
        grid on; ylabel('Potentielle Energie der Komponenten in J');
        legend(kompleg);
        subplot(2,3,5);
        plot(Tges(1:Ip_end), P_diss_ges(1:Ip_end,:));
        grid on; ylabel('Übertragene Leistung in W');
        legend({'Gesamt (Dissipation)', 'zwischen Komponenten', 'zw. Kom. (max. bis jetzt)'});
        subplot(2,3,6); hold on;
        plot(Tges(1:Ip_end), 100*P_diss_ges(1:Ip_end,1)./P_diss_ges(1:Ip_end,2));
        plot(Tges(1:Ip_end), 100*P_diss_ges(1:Ip_end,1)./P_diss_ges(1:Ip_end,3));
        legend({'momentan', 'bzgl Max. bis jetzt'});
        grid on; ylabel('Anteil Dissipation an Leistungsfluss in Prozent');
        linkxaxes
        sgtitle(sprintf('%s: Energie', PName));
        set(3, 'Name', 'Energie', 'NumberTitle', 'off');
        if usr_save_figures
          name = fullfile(respath_rob, sprintf('%s_Energie_dt%dus_Jac%d', PName, 1e6*dt, jm));
          export_fig([name, '.png']);
          saveas(gcf, [name, '.fig']);
        end
        %% Plot: Gelenke
        change_current_figure(4);clf;
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
        set(4, 'Name', 'Beingelenk-Kinematik', 'NumberTitle', 'off');
        if usr_save_figures
          name = fullfile(respath_rob, sprintf('%s_Gelenke_dt%dus_Jac%d', PName, 1e6*dt, jm));
          export_fig([name, '.png']);
          saveas(gcf, [name, '.fig']);
        end
        %% Plot: Gelenke: Konsistenz
        change_current_figure(5);clf;
        for i = 1:RP.NJ
          legnum = find(i>=RP.I1J_LEG, 1, 'last');
          legjointnum = i-(RP.I1J_LEG(legnum)-1);
          subplot(ceil(sqrt(RP.NJ)), ceil(RP.NJ/ceil(sqrt(RP.NJ))), i);
          hold on; grid on;
          hdl1=plot(Tges, Q_ges(:,i), '-');
          hdl3=plot(Tges, Q_ges_num(:,i), '--');
          title(sprintf('q %d L%d,J%d', i, legnum, legjointnum));
          if i == length(q), legend([hdl1;hdl3], {'q','int(qD)'}); end
        end
        sgtitle(sprintf('%s: Gelenke Konsistenz q', PName));
        set(5, 'Name', 'Gelenk-Kons.-q', 'NumberTitle', 'off');
        if usr_save_figures
          name = fullfile(respath_rob, sprintf('%s_Gelenke_Konsistenz_q_dt%dus_Jac%d', PName, 1e6*dt, jm));
          export_fig([name, '.png']);
          saveas(gcf, [name, '.fig']);
        end
        change_current_figure(6);clf;
        for i = 1:RP.NJ
          legnum = find(i>=RP.I1J_LEG, 1, 'last');
          legjointnum = i-(RP.I1J_LEG(legnum)-1);
          subplot(ceil(sqrt(RP.NJ)), ceil(RP.NJ/ceil(sqrt(RP.NJ))), i);
          hold on; grid on;
          hdl1=plot(Tges, QD_ges(:,i), '-');
          hdl2=plot(Tges, QD_ges_num(:,i), '--');
          hdl3=plot(Tges, QD_ges_num2(:,i), ':');
          title(sprintf('qD %d L%d,J%d', i, legnum, legjointnum));
          if i == length(qD), legend([hdl1;hdl2;hdl3], {'qD','diff(q)', 'int(qDD)'}); end
        end
        sgtitle(sprintf('%s: Gelenke Konsistenz qD', PName));
        set(6, 'Name', 'Gelenk-Kons.-qD', 'NumberTitle', 'off');
        if usr_save_figures
          name = fullfile(respath_rob, sprintf('%s_Gelenke_Konsistenz_qD_dt%dus_Jac%d', PName, 1e6*dt, jm));
          export_fig([name, '.png']);
          saveas(gcf, [name, '.fig']);
        end

        %% Plot: Plattform
        change_current_figure(7);clf;
        subplot(2,2,1);
        xpos_leg = {'rx', 'ry', 'rz', 'phix', 'phiy', 'phiz'};
        plot(Tges, XP_ges(:,RP.I_EE));
        grid on; ylabel('Plattform-Position (im Basis-KS)');
        legend(xpos_leg(RP.I_EE));
        subplot(2,2,2);
        xvel_leg = {'vx', 'vy', 'vz', 'phiDx', 'phiDy', 'phiDz'};
        plot(Tges, XPD_ges(:,RP.I_EE));
        grid on; ylabel('Plattform-Geschwindigkeit (im Basis-KS)');
        legend(xvel_leg(RP.I_EE));
        subplot(2,2,3);
        plot(Tges, XPDD_ges(:,RP.I_EE));
        grid on; ylabel('Plattform-Beschleunigung (im Basis-KS)');
        subplot(2,2,4);
        Facc_leg = {'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'};
        plot(Tges, Facc_ges(:,RP.I_EE));
        grid on; ylabel('Plattform-Kraft (im Basis-KS)');
        legend(Facc_leg(RP.I_EE));
        linkxaxes
        sgtitle(sprintf('%s: Plattform', PName));
        set(7, 'Name', 'Plattform-Kinematik', 'NumberTitle', 'off');
        if usr_save_figures
          name = fullfile(respath_rob, sprintf('%s_Plattform_dt%dus_Jac%d', PName, 1e6*dt, jm));
          export_fig([name, '.png']);
          saveas(gcf, [name, '.fig']);
        end
        
        %% Plot: Diverses
        change_current_figure(8);clf;
        subplot(2,1,1); hold on;
        title('Singularitäts-Kennzahlen');
        plot(Tges, SingDet(:,1:2));
        plot(Tges, SingDet(:,3:4), '-');
        plot(Tges, SingDet(:,5:6), '-');
        legend({'cond(JIK) (var.1)', 'cond(JDK) (var.1)', ...
                'cond(JIK) (var.4)', 'cond(JDK) (var.4)', ...
                'cond(JIK) (var.2)', 'cond(JDK) (var.2)'});
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
        set(8, 'Name', 'Testen', 'NumberTitle', 'off');
        if usr_save_figures
          name = fullfile(respath_rob, sprintf('%s_Test_dt%dus_Jac%d', PName, 1e6*dt, jm));
          export_fig([name, '.png']);
          saveas(gcf, [name, '.fig']);
        end
      end
      save(fullfile(respath_rob, sprintf('%s_Test_dt%dus_Jac%d_data.mat', PName, 1e6*dt, jm)));
      
      if usr_plot_animation && Ip_end > 1
        %% Plot des Roboters in Startpose
        change_current_figure(9);clf;hold all;
        xlabel('x in m');ylabel('y in m');zlabel('z in m'); view(3);
        axis auto
        hold on;grid on;
        s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
        RP.plot(Q_ges(1,:)', XP_ges(1,:)', s_plot);
        title(sprintf('%s in Startkonfiguration',PName));
        %% Animation des bewegten Roboters
        i_end_vis = Ip_end-1;
        i_diff = ceil(i_end_vis / 25);
        s_anim = struct( 'gif_name', fullfile(respath_rob, sprintf('%s_energy_test_dt%dus.gif', PName, 1e6*dt)));
        s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
        change_current_figure(10);clf;hold all;
        set(10, 'name', 'Anim', ...
          'color','w', 'NumberTitle', 'off', 'units','normalized',...
          'outerposition',[0 0 1 1]); % Vollbild, damit GIF größer wird
        view(3);
        axis auto
        hold on;grid on;
        xlabel('x in m');ylabel('y in m');zlabel('z in m');
        title(sprintf('%s Dynamik-Simulation ',PName));
        RP.anim( Q_ges(1:i_diff:i_end_vis,:), XP_ges(1:i_diff:i_end_vis,:), s_anim, s_plot);
        fprintf('Animation der Bewegung gespeichert: %s\n', s_anim.gif_name);
      end
      %% Auswertung
      % Validiere bei 80% der Zeit. Annahme: Wenn eine Singularität die
      % Simulation abbricht, ist sie hier noch nicht wirksam.
      % Starte Validierung erst nach 20% der Zeit. Annahme: Beim Start ist
      % die Bezugsgröße noch zu klein für einen relativen Fehler
      if Ip_end == 1
        I_test2080 = 1;
        I_test0530 = 1;
        I_till1stsing = 1;
      else
        I_test2080 = max([floor(Ip_end*0.20);1]):max([floor(Ip_end*0.8);1]);
        I_test0530 = max([floor(Ip_end*0.05);1]):max([floor(Ip_end*0.30);1]);
        % Annahme: Überhöhung der Konditionszahl deutet auf Singularität
        % Bestimme die Dissipation nur bis dahin.
        I_till1stsing = 1:min([find(SingDet(:,1)/SingDet(1,1) > 3, 1, 'first');Ip_end]);
      end
      E_error = max(abs(E_ges(1:Ip_end,3)-E_ges(1,3)));
      % Nehme den Anteil der dissipierten Energie an der umgesetzten Energie
      % als Kriterium, ob die Berechnung der Vorwärtsdynamik erfolgreich war.
      % Benutze als Bezugsgröße die maximal umgesetzte Energie bis zum
      % Betrachtungszeitpunkt. Dadurch führt der Rückgang des Energie-
      % flusses bei gleichzeitiger Beibehaltung der Dissipation (durch
      % numerische Fehler) nicht zum Fehlerfall.
      Pdiss_relerror2080 = max(abs(P_diss_ges(I_test2080,1)./P_diss_ges(I_test2080,3)));
      Pdiss_relerror0530 = max(abs(P_diss_ges(I_test0530,1)./P_diss_ges(I_test0530,3)));
      Pdiss_relerrori1stsing = max(abs(P_diss_ges(I_till1stsing,1)./P_diss_ges(I_till1stsing,3)));
      fail = false;
      if Pdiss_relerrori1stsing > 10e-2 || isnan(Pdiss_relerrori1stsing)
        fail = true;
      end
      if fail
        n_fail = n_fail + 1;
      else
        n_succ = n_succ + 1;
      end
      TrajUntil1stSing = I_till1stsing(end) / Ip_end;
      TrajAchieved = Ip_end/nt; % Anteil der Simulationsdauer, die geschafft wurde
      % Ergebnis in Tabelle abspeichern
      ResStat = [ResStat; {PName, jm, Pdiss_relerrori1stsing, ...
        Pdiss_relerror2080, Pdiss_relerror0530, 100*TrajUntil1stSing, ...
        100*TrajAchieved, 100*RTratio, double(~fail)}]; %#ok<AGROW>
    end
    fprintf(['%d/%d Testkombinationen für %s getestet (%d i.O., %d n.i.O) ', ...
      '(bei restlichen %d IK falsch).\n'], n_succ+n_fail, n, PName, n_succ, n_fail, n-(n_succ+n_fail));
    num_robots_tested = num_robots_tested + 1;
    if n_succ == n
      num_robots_success = num_robots_success + 1;
      robot_list_succ = [robot_list_succ(:)', {PName}];
    else
      robot_list_fail = [robot_list_fail(:)', {PName}];
    end
    end % Schleife über jacobi_mode
    if num_robots_tested >= usr_num_tests_per_dof*2 % da jede PKM zwei mal getestet wird
      break; % Testen aller Roboter dauert sehr lange. Ergebnis für wenige reicht aus.
    end
  end % Schleife über Verschiedene PKM
  if isempty(ResStat) % Dieser Fall kann nur bei manuellem Test einer PKM vorkommen.
    warning('Leere Ergebnistabelle für FG-Kombination Nr. %d. Keiner der PKM konnte erfolgreich getestet werden', i_FG);
    continue
  end
  ResStat.Properties.VariableNames = {'Name', 'JacobiMode', 'RelError_until1stSing', ...
    'RelError20_80', 'RelError05_30', 'PercentSimSingFree', 'PercentTraj', ...
    'RealTimeRatio_Percent', 'Success'};
  save(fullfile(respath, sprintf('ResStat_%dT%dR_FG.mat', sum(EE_FG(1:3)), sum(EE_FG(4:6)))), 'ResStat');
  fprintf('Energiekonsistenz für %d PKM mit FG [%s] getestet. %d Erfolgreich\n', ...
    num_robots_tested, disp_array(EE_FG, '%1.0f'), num_robots_success);
  fprintf('Keine Energiekonsistenz bei folgenden %d Robotern:\n', length(robot_list_fail));
  disp(robot_list_fail(:));
  fprintf('Energiekonsistenz bei folgenden %d Robotern:\n', length(robot_list_succ));
  disp(robot_list_succ(:));
  fprintf('Ergebnistabelle für FG mit %d Einträgen:\n', size(ResStat,1));
  disp(ResStat);
  fprintf(['Erklärung: \nRelError20_80: maximaler Anteil der dissipierten ', ...
    'Energie am Gesamt-Leistungsfluss von 20%% bis 80%% der Simulation.\n']);
  fprintf(['PercentSimSingFree: Prozentualer Anteil der singularitätsfreien ', ...
    'Bewegung an der gesamten Simulationsdauer.\n']);
  fprintf(['PercentTraj: Prozentualer Anteil der erfolgreichen Simulation ', ...
    'an der geplanten Gesamtdauer der Simulation der freien Bewegung.\n']);
  fprintf(['RealTimeRatio_Percent: Echtzeitfaktor der Vorwärtsdynamik. Ein ', ...
    'Wert von 1(%%) bedeutet, 1s simulierte Zeit brauchen 100s Rechenzeit\n']);
  restabfile = fullfile(respath, sprintf('fdyn_energy_cons_test_DoF_%s.csv', ...
    char(48+EE_FG)));
  writetable(ResStat, restabfile, 'Delimiter', ';');
  %% Prüfe Erfolg. Fehler bei Nicht-Erfolg
  if sum(ResStat.Success) ~= length(ResStat.Success)
    warning('Nicht alle Vorwärts-Dynamik-Simulationen energetisch konsistent.');
    PKM_Success_Ratio = sum(ResStat.Success) / length(ResStat.Success);
    if all(EE_FG == [1 1 0 0 0 1])
      % Nur bei einer einzigen PKM ist der Fehler gering genug. TODO: Warum?
      warning('Aktuell keine verlässliche Prüfung der 2T1R-PKM. TODO: Fix.');
%       I_test = strcmp(ResStat.Name, 'P3RPP1G1P1A1');
%       if any(~ResStat.Success(I_test))
%         error('Eine vorher als funktionierend markierte 2T1R-PKM ist nicht konsistent. Fehler.');
%       end
    elseif all(EE_FG == [1 1 1 0 0 0])
      % Alle PKM müssen funktionieren
      error('Nicht alle PKM bei 3T0R energiekonsistent. Fehler.');
    elseif all(EE_FG == [1 1 1 0 0 1])
      % Nur eine PKM funktioniert aktuell nicht. TODO: Warum?
      I_test = ~strcmp(ResStat.Name, 'P4PRRRR6V1G2P1A1');
      if any(~ResStat.Success(I_test))
        error('Eine vorher als funktionierend markierte 3T1R-PKM ist nicht konsistent. Fehler.');
      end
    elseif all(EE_FG == [1 1 1 1 1 1])
      if PKM_Success_Ratio < 0.5
        error('Nicht mal die Hälfte der getesteten 3T3R-PKM ist energiekonsistent. Fehler?');
      end
    end
  end
end
