% Vergleiche die symbolische und numerische Implementierung der Dynamik
% Benutze unterschiedliche Implementierungen als Verbleich:
% * Dynamikparameter (inertial, parameterlinear, parameterminimal)
% * Eingabe mit Jacobi-Matrix aus unterschiedlichen Modellierungen
%   (Dadurch Validierung der Jacobi-Matrix-Implementierungen)
% 
% Ergebnis bei erfolgreichem Durchlauf:
% * Alle Dynamikmodellierungen sind konsistent

% TODO:
% * Falsches Ergebnis, wenn Transformation T_P_E gesetzt ist
% 
% Siehe auch: ParRob_mdltest_invdyn_energy_consistency.m (ähnlich)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-11
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

%% Benutzereingaben
usr_only_check_symvsnum = true;
usr_test_constr4 = true;
usr_debug = true;
usr_num_tests_per_dof = 5;
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
respath = fullfile(rob_path, 'examples_tests', 'results');
mkdirs(respath);
% Einstellungen für inverse Kinematik
s = struct( ...
       'n_min', 25, ... % Minimale Anzahl Iterationen
       'n_max', 1000, ... % Maximale Anzahl Iterationen
       'Phit_tol', 1e-12, ... % Toleranz für translatorischen Fehler
       'Phir_tol', 1e-12); % Toleranz für rotatorischen Fehler

%% Alle Roboter durchgehen (mit allen Dynamik-Modi)
for i_FG = 1:size(EEFG_Ges,1) % 2T1R, 3T0R, 3T1R, 3T3R
EE_FG = EEFG_Ges(i_FG,:);
fprintf('Untersuche PKM mit FG [%s]\n', char(48+EE_FG));
[PNames_Kin, PNames_Akt] = parroblib_filter_robots(sum(EE_FG), EE_FG, EE_FG_Mask, 6);
if isempty(PNames_Kin)
  fprintf('Keine Roboter mit FG [%s] in Datenbank\n', disp_array(EE_FG, '%1.0f'));
  continue
end
% Stelle Liste von PKM zusammen, die sich in ihrer Kinematik ohne
% Betrachtung des Plattform-Koppelgelenks unterscheiden
PNames_noP = cell(size(PNames_Kin));
for i = 1:length(PNames_Kin)
  PNames_noP{i} = PNames_Kin{i}(1:end-2);
end
[~,III] = unique(PNames_noP);
for DynParMode = 2:4
  fprintf('Untersuche Dynamik-Parameter Nr. %d\n', DynParMode);
  % 2 = Inertialparameter (kein Regressor)
  % 3 = Inertialparameter-Regressor
  % 4 = Minimalparameter-Regressor
  ResStat = table();
  num_robots_tested = 0;
  robot_list_succ = {}; % Liste erfolgreicher PKM
  robot_list_part = {}; % Liste teilweise erfolgreich
  robot_list_fail = {}; % Liste fehlgeschlagener

  % III = find(strcmp(PNames_Kin, 'P3RRR1G1P1')); % Debug; P6RRPRRR14V3G1P4
  for ii = III(:)' % 1:length(PNames_Kin)
    % Suche erste gültige Aktuierung in der Datenbank (ist für Dynamik
    % egal; es wird nur die Plattform-Dynamik betrachtet ohne Projektion in
    % die Antriebskoordinaten).
    IIa = find(contains(PNames_Akt, PNames_Kin{ii}),1,'first');
    PName = PNames_Akt{IIa};
    fprintf('Untersuche PKM %s\n', PName);
    paramfile_robot = fullfile(tmpdir_params, sprintf('%s_params.mat', PName));

    %% Roboter Initialisieren
    RP = parroblib_create_robot_class(PName, 1, 0.3);
    % Initialisierung der Funktionen: Kompilierte Funktionen nehmen
    files_missing = RP.fill_fcn_handles(false, false);
    % Prüfe, ob die Dynamik-Implementierung in symbolischer Form vorliegt
    if length(files_missing) >= 6 && usr_only_check_symvsnum
      fprintf('Keine symbolisch generierten Dynamik-Funktionen verfügbar. Kein Test Sym vs Num möglich.\n');
      continue
    end
    if any(contains(files_missing, 'para_pf_mdp')) && any(DynParMode == [3,4])
      fprintf('Regressor-Form soll getestet werden, aber Funktion nicht vorhanden\n');
      continue
    end
    if any(contains(files_missing, 'Jinv'))
      fprintf('Inverse Jacobi-Matrix ist nicht in symbolischer Form vorhanden\n');
      % Ist nicht so schlimm. Eigentlich geht es ja um die Dynamik.
    end
    % Initialisierung der Funktionen: Kompilierte Funktionen nehmen
    % (nur, wenn auch die symbolischen Funktionen da sind. Sonst wird
    % unnötig kompiliert)
    RP.fill_fcn_handles(true, true);
    %% Kinematikparameter durch Optimierung erzeugen (oder gespeichert laden)
    Set = cds_settings_defaults(struct('DoF', EE_FG));
    Set.task.Ts = 1e-2;
    Set.task.Tv = 1e-1;
    Set.task.profile = 1; % Zeitverlauf mit Geschwindigkeit
    Set.task.maxangle = 5*pi/180; % damit wird schneller eine Lösung gefunden (reicht für Dynamik-Test)
    % Set.general.debug_calc = true;
    Traj_W = cds_gen_traj(EE_FG, 1, Set.task);
    % Reduziere Punkte (geht dann schneller, aber auch schlechtere KinPar.
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
      % Prüfe die Lösbarkeit der IK
      [q_test,Phi]=RP.invkin_ser(Traj_0.X(1,:)', q0);
      if all(abs(Phi)<1e-6) && ~any(isnan(Phi))
        fprintf('IK erfolgreich mit abgespeicherten Parametern gelöst\n');
        params_success = true; % Parameter für erfolgreiche IK geladen.
      else
        warning('IK mit abgespeicherten Parametern nicht lösbar.');
        if usr_debug
          % Erzeuge ein Bild zur Diagnose der Ursache der falschen Kinematik
          q_test(isnan(q_test)) = q0(isnan(q_test));
          change_current_figure(10);clf;hold all;
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
      Set.optimization.objective = 'valid_act';
      % TODO: Folgendes sollte aktiviert werden, funktioniert aber noch nicht.
      Set.optimization.ee_rotation = false; % darf beliebige Werte einnehmen ....
      Set.optimization.ee_translation = false; % ... muss für Dynamik egal sein
      Set.optimization.ee_translation_only_serial = false; % damit obige Funktion wirkt
      Set.optimization.movebase = false;
      Set.optimization.base_size = false;
      Set.optimization.platform_size = false;
      % TODO: Deckenmontage funktioniert noch nicht richtig.
      Set.structures.mounting_parallel = 'floor';
%       if all(EE_FG==[1 1 1 0 0 0]) || all(EE_FG==[1 1 1 1 1 1])
%         % Deckenmontage funktioniert für diese FG schon.
%         Set.structures.mounting_parallel = {'ceiling'};
%       else
%         Set.structures.mounting_parallel = 'floor';
%       end
      Set.structures.use_parallel_rankdef = 6; % Rangdefizit ist egal
      Set.structures.whitelist = {PName}; % nur diese PKM untersuchen
      Set.structures.nopassiveprismatic = false; % Für Dynamik-Test egal 
      Set.structures.maxnumprismatic = 6; % Für Dynamik-Test egal wie viele Schubgelenke
      Set.general.noprogressfigure = true;
      Set.general.verbosity = 3;
      Set.general.nosummary = true;
      Traj = Traj_W;
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

    %% Parameter für Dynamik-Test
    RP.DynPar.mode = DynParMode;
    for il = 1:RP.NLEG, RP.Leg(il).DynPar.mode = DynParMode; end
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
    if DynParMode == 3 % Inertialparameter-Vektor
      dpv = RP.DynPar.ipv_n1s;
    else % Minimalparameter-Vektor
      dpv = RP.DynPar.mpv_n1s;
    end
    %% Dynamik-Test Symbolisch gegen numerisch
    n = 10; % Anzahl der Test-Konfigurationen
    % Nehme Eckpunkte der Trajektorie aus der Maßsynthese und füge
    % zufällige Zahlen hinzu
    X_test0 = repmat(Traj_0.XE,ceil(n/size(Traj_0.XE,1)),1);
    X_test0 = X_test0(1:n,:);
    XD_test0 = rand(n,6); XDD_test0 = rand(n,6);
    % Rechne in Plattform-Koordinaten um, füge Zufallszahlen dort hinzu und
    % wandle wieder in EE-Koordinaten zurück. Dadurch wird die Drehung der
    % PKM nach unten in der Maßsynthese ausgeglichen. Sonst Probleme mit
    % 2T1R (da weitere Euler-Winkel ungleich Null sind)
    [XP_test, XDP_test, XDDP_test] = RP.xE2xP_traj(X_test0, XD_test0, XDD_test0);
    % Entferne nicht betrachtete/bewegliche Koordinaten (im Plattform-KS
    % definiert)
    XP_test = XP_test + 1e-1*rand(n,6); XP_test(:,~RP.I_EE) = 0;
    XDP_test(:,~RP.I_EE) = 0;           XDDP_test(:,~RP.I_EE) = 0;
    % Zurück nach EE-KS umrechnen
    [XE_test, XDE_test, XDDE_test] = RP.xP2xE_traj(XP_test, XDP_test, XDDP_test);
    % Numerik-Fehler durch Umrechnung wieder entfernen
    XE_test(abs(XE_test)<1e-12)=0; XDE_test(abs(XDE_test)<1e-12)=0;
    XDDE_test(abs(XDDE_test)<1e-12)=0;
    Q_test = NaN(n, RP.NJ); QD_test = Q_test; QDD_test = Q_test; QDD_test_noacc = Q_test;
    
    Information_matrix = [];
    % Dynamik berechnen (mit den Zufallswerten)
    n_succ = 0;
    n_fail = 0;
    for i = 1:n
      % IK für Testkonfiguration berechnen
      [q,Phi] = RP.invkin_ser(XE_test(i,:)', q0, s);
      if any(abs(Phi) > 1e-8) || any(isnan(Phi)), continue; end % IK für diese Pose nicht erfolgreich
      Q_test(i,:) = q;
      [~, Jinv] = RP.jacobi_qa_x(q, XE_test(i,:)');
      QD_test(i,:) = Jinv*XDE_test(i,RP.I_EE)';
      % Teste symbolischen Aufruf der Jacobi-Matrix gegen numerischen
      Jinv_num_qa_x = Jinv(RP.I_qa,:);
      Jinv_sym_qa_x = RP.jacobi_qa_x(q, XE_test(i,:)');
      test_Jinv = Jinv_sym_qa_x - Jinv_num_qa_x;
      if max(abs(test_Jinv(:))) > 1e-10
        change_current_figure(11);clf;hold all;
        xlabel('x in m');ylabel('y in m');zlabel('z in m'); view(3);
        axis auto
        hold on;grid on;
        s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], ...
          'ks_platform', RP.NLEG+(1:2), 'straight', 0);
        RP.plot(q, XE_test(i,:)', s_plot);
        error('Inverse Jacobi-Matrix stimmt nicht. Max Fehler %1.1e.', max(abs(test_Jinv(:))));
      end

      [~, JinvD] = RP.jacobiD_qa_x(q, QD_test(i,:)', XE_test(i,:)', XDE_test(i,:)');
      QDD_test(i,:) = JinvD*XDE_test(i,RP.I_EE)' + Jinv*XDDE_test(i,RP.I_EE)';
      
      % Parameter zufällig setzen
      RP.update_gravity(rand(3,1));
      
      % Für Testkonfigurationen mit Null-Beschleunigung: Euler-Zeitableitung 
      % verursacht auch Beschleunigung über die Kopplung der Koordinaten
      % Effektiv ist es nur Null-Beschleunigung der Euler-Winkel. Es gibt
      % eine Winkelbeschleunigung. Siehe euljacD
      QDD_test_noacc(i,:) = JinvD*XDE_test(i,RP.I_EE)' + Jinv*zeros(sum(RP.I_EE),1);
      
      % Berechne Dynamik-Terme auf zwei Arten:
      % numerische Berechnung (Do Thanh, Matlab)
      Mx2 = RP.inertia2_platform(Q_test(i,:)' , XE_test(i,:)');
      Cx2 = RP.coriolisvec2_platform(Q_test(i,:)', QD_test(i,:)', XE_test(i,:)', XDE_test(i,:)');
      Gx2 = RP.gravload2_platform(Q_test(i,:)', XE_test(i,:)');
      Fx2 = RP.invdyn2_platform(Q_test(i,:)', QD_test(i,:)', QDD_test(i,:)', XE_test(i,:)', XDE_test(i,:)', XDDE_test(i,:)');
      FGx2 = RP.invdyn2_platform(Q_test(i,:)', 0*QD_test(i,:)', 0*QDD_test(i,:)', XE_test(i,:)', 0*XDE_test(i,:)', 0*XDDE_test(i,:)');
      FGCx2 = RP.invdyn2_platform(Q_test(i,:)', QD_test(i,:)', QDD_test_noacc(i,:)', XE_test(i,:)', XDE_test(i,:)', 0*XDDE_test(i,:)');
      % symbolische Berechnung (Abdelatif, Maple)
      Mx1 = RP.inertia_platform(Q_test(i,:)', XP_test(i,:)');
      Cx1 = RP.coriolisvec_platform(Q_test(i,:)', XP_test(i,:)', XDP_test(i,:)');
      Gx1 = RP.gravload_platform(Q_test(i,:)', XP_test(i,:)');
      Fx1 = RP.invdyn_platform(Q_test(i,:)', XP_test(i,:)', XDP_test(i,:)', XDDP_test(i,:)');
      FGx1 = RP.invdyn_platform(Q_test(i,:)', XP_test(i,:)', 0*XDP_test(i,:)', 0*XDDP_test(i,:)');
      FGCx1 = RP.invdyn_platform(Q_test(i,:)', XP_test(i,:)', XDP_test(i,:)', 0*XDDP_test(i,:)');
      test_M = Mx2 - Mx1;
      test_C = Cx2 - Cx1;
      test_G = Gx2 - Gx1;
      test_GC1 = FGCx1 - (Cx1+Gx1); % Teste Summe aus Gravitation und Coriolis
      test_F = Fx2 - Fx1;
      test_FG = FGx2 - FGx1;
      test_FGC = FGCx2 - FGCx1; % Summe Grav.+Cor. aus zwei Implementierungen
      test_FC = (FGCx2-FGx2) - (FGCx1-FGx1); % Nur Vergleich Coriolis aus kompletter Dynamik
      test_F_sum = (Mx2*XDDE_test(i,RP.I_EE)'+Cx2+Gx2) - Fx2;
      fail = false;
      if any(abs(test_M(:)) > 1e-6)
        warning('Massenmatrix stimmt nicht. Max Fehler %1.1e.', max(abs(test_M(:))));
        fail = true;
      end
      if any(abs(test_C(:)) > 1e-6)
        warning('Coriolis-Terme stimmen nicht. Max Fehler %1.1e.', max(abs(test_C(:))));
        fail = true;
      end
      if any(abs(test_G(:)) > 1e-6)
        warning('Gravitations-Terme stimmen nicht. Max Fehler %1.1e.', max(abs(test_G(:))));
        fail = true;
      end
      if any(abs(test_GC1(:)) > 1e-6)
        warning('Summe aus Grav. und Coriolis nach sym. Methode stimmt nicht. Max Fehler %1.1e.', max(abs(test_GC1(:))));
        fail = true;
      end
      if any(abs(test_F(:)) > 1e-6) || any(abs(test_F_sum(:)) > 1e-6)
        warning('Inversdynamik-Terme (gesamt) stimmen nicht. Max Fehler %1.1e.', max(abs(test_F(:))));
        fail = true;
      end
      if any(abs(test_FG(:)) > 1e-6) || any(abs(test_FG(:)) > 1e-6)
        warning('Gravitations-Terme (einzelne Komponenten aus Summenfunktion) stimmen nicht. Max Fehler %1.1e.', max(abs(test_FG(:))));
        fail = true;
      end
      if any(abs(test_FGC(:)) > 1e-6) || any(abs(test_FGC(:)) > 1e-6)
        warning('Gravitations- und Coriolis-Terme (einzelne Komponenten aus Summenfunktion) stimmen nicht. Max Fehler %1.1e.', max(abs(test_FGC(:))));
        fail = true;
      end
      if any(abs(test_FC(:)) > 1e-6) || any(abs(test_FC(:)) > 1e-6)
        warning('Coriolis-Terme (einzelne Komponenten aus Summenfunktion) stimmen nicht. Max Fehler %1.1e.', max(abs(test_FC(:))));
        fail = true;
      end
      % Prüfe Aufruf mit gegebener Jacobi-Matrix (sollte schneller sein)
      Mx2_Jinput = RP.inertia2_platform(Q_test(i,:)' , XE_test(i,:)', Jinv);
      Cx2_Jinput = RP.coriolisvec2_platform(Q_test(i,:)', QD_test(i,:)', XE_test(i,:)', XDE_test(i,:)', Jinv, JinvD);
      Gx2_Jinput = RP.gravload2_platform(Q_test(i,:)', XE_test(i,:)', Jinv);
      Fx2_Jinput = RP.invdyn2_platform(Q_test(i,:)', QD_test(i,:)', QDD_test(i,:)', XE_test(i,:)', XDE_test(i,:)', XDDE_test(i,:)', Jinv);
      FGx2_Jinput = RP.invdyn2_platform(Q_test(i,:)', 0*QD_test(i,:)', 0*QDD_test(i,:)', XE_test(i,:)', 0*XDE_test(i,:)', 0*XDDE_test(i,:)', Jinv);
      FGCx2_Jinput = RP.invdyn2_platform(Q_test(i,:)', QD_test(i,:)', QDD_test_noacc(i,:)', XE_test(i,:)', XDE_test(i,:)', 0*XDDE_test(i,:)', Jinv);
      test_Mx2Jin = Mx2_Jinput - Mx1;
      test_Cx2Jin = Cx2_Jinput - Cx1;
      test_Gx2Jin = Gx2_Jinput - Gx1;
      test_Fx2Jin = Fx2_Jinput - Fx1;
      testFGx2Jin = FGx2_Jinput - Gx2_Jinput;
      testFCx2Jin = (FGCx2_Jinput-FGx2_Jinput) - Cx2_Jinput;
      if any(abs(test_Mx2Jin(:)) > 1e-6)
        warning('Massenmatrix mit Jacobi-Matrix-Eingabe stimmt nicht. Max Fehler %1.1e.', max(abs(test_Mx2Jin(:))));
        fail = true;
      end
      if any(abs(test_Cx2Jin(:)) > 1e-6)
        warning('Coriolis-Terme mit Jacobi-Matrix-Eingabe stimmen nicht. Max Fehler %1.1e.', max(abs(test_Cx2Jin(:))));
        fail = true;
      end
      if any(abs(test_Gx2Jin(:)) > 1e-6)
        warning('Gravitations-Terme mit Jacobi-Matrix-Eingabe stimmen nicht. Max Fehler %1.1e.', max(abs(test_Gx2Jin(:))));
        fail = true;
      end
      if any(abs(test_Fx2Jin(:)) > 1e-6)
        warning('Inversdynamik-Terme mit Jacobi-Matrix-Eingabe stimmen nicht. Max Fehler %1.1e.', max(abs(test_Fx2Jin(:))));
        fail = true;
      end
      if any(abs(testFCx2Jin(:)) > 1e-6)
        warning('Inversdynamik-Terme mit Jacobi-Matrix-Eingabe stimmt nicht gegen Coriolis-Kräfte. Max Fehler %1.1e.', max(abs(testFCx2Jin(:))));
        fail = true;
      end
      if any(abs(testFGx2Jin(:)) > 1e-6)
        warning('Inversdynamik-Terme mit Jacobi-Matrix-Eingabe stimmt nicht gegen Gravitation. Max Fehler %1.1e.', max(abs(testFGx2Jin(:))));
        fail = true;
      end
      % Teste, welche Jacobi-Zeitableitung voraussichtlich richtig ist
      if usr_test_constr4
        % Neue Berechnung der Jacobi-Matrix
        G4_q = RP.constr4grad_q(Q_test(i,:)');
        G4_x = RP.constr4grad_x(XE_test(i,:)');
        GD4_q = RP.constr4gradD_q(Q_test(i,:)', QD_test(i,:)');
        GD4_x = RP.constr4gradD_x(XE_test(i,:)', XDE_test(i,:)');
        Jinv_4 = -G4_q\G4_x;
        JinvD_4 = G4_q\GD4_q/G4_q*G4_x - G4_q\GD4_x;
        % Prüfe Jacobi-Matrix
        test_JD = JinvD - JinvD_4;
        if max(abs(test_JD(:))) > 1e-4
          % warning('Modell 4 für die Jacobi-Matrix stimmt nicht gegen Modell 1');
        end  
        % Dynamik damit berechnen
        Mx2_J4input = RP.inertia2_platform(Q_test(i,:)' , XE_test(i,:)', Jinv_4);
        Cx2_J4input = RP.coriolisvec2_platform(Q_test(i,:)', QD_test(i,:)', XE_test(i,:)', XDE_test(i,:)', Jinv_4, JinvD_4);
        Gx2_J4input = RP.gravload2_platform(Q_test(i,:)', XE_test(i,:)', Jinv_4);
        % Testen
        test_Mx2J4in = Mx2_J4input - Mx1;
        test_Cx2J4in = Cx2_J4input - Cx1;
        test_Gx2J4in = Gx2_J4input - Gx1;
        if any(abs(test_Mx2J4in(:)) > 1e-6)
          warning('Massenmatrix mit Jacobi-Matrix-Eingabe (Var. 4) stimmt nicht. Max Fehler %1.1e.', max(abs(test_M(:))));
          fail = true;
        end
        if any(abs(test_Cx2J4in(:)) > 1e-6)
          warning('Coriolis-Terme mit Jacobi-Matrix-Eingabe (Var. 4)  stimmen nicht. Max Fehler %1.1e.', max(abs(test_C(:))));
          fail = true;
        end
        if any(abs(test_Gx2J4in(:)) > 1e-6)
          warning('Gravitations-Terme mit Jacobi-Matrix-Eingabe (Var. 4)  stimmen nicht. Max Fehler %1.1e.', max(abs(test_G(:))));
          fail = true;
        end
      end
      
      % Prüfe Regressor-Form
      if DynParMode == 3 || DynParMode == 4
        [Gx2,Gx2_reg] = RP.gravload2_platform(Q_test(i,:)', XE_test(i,:)');
        test_Greg = Gx2_reg*dpv - Gx2;
        if any(abs(test_Greg(:)) > 1e-6)
          error('Gravitationskraft-Regressor stimmt nicht. Max Fehler %1.1e.', max(abs(test_Greg(:))));
        end
        [M_full,M_full_reg] = RP.inertia2_platform_full(Q_test(i,:)', XE_test(i,:)');
        [Mx2,Mx2_reg] = RP.inertia2_platform(Q_test(i,:)' , XE_test(i,:)');
        Mx_reg_vec = Mx2_reg*dpv;
        test_Mreg = reshape(Mx_reg_vec, sum(RP.I_EE), sum(RP.I_EE)) - Mx2;
        if any(abs(test_Mreg(:)) > 1e-6)
          error('Massenmatrix-Regressor stimmt nicht. Max Fehler %1.1e.', max(abs(test_Mreg(:))));
        end
        M_full_regtest = zeros(size(M_full));
        for jj = 1:length(dpv)
          M_full_regtest = M_full_regtest + M_full_reg(:,:,jj) .* dpv(jj);
        end
        test_Mreg_full = M_full_regtest - M_full;
        if any(abs(test_Mreg_full(:)) > 1e-6)
          error('Massenmatrix-Regressor (vollständige Jacobi vor Projektion) stimmt nicht. Max Fehler %1.1e.', max(abs(test_Mreg_full(:))));
        end
        [Cx2,Cx2_reg] = RP.coriolisvec2_platform(Q_test(i,:)', QD_test(i,:)', XE_test(i,:)', XDE_test(i,:)') ;
        test_Creg = Cx2_reg*dpv - Cx2;
        if any(abs(test_Creg(:)) > 1e-6)
          error('Coriolis-Kraft-Regressor stimmt nicht. Max Fehler %1.1e.', max(abs(test_Creg(:))));
        end
        [Fx2,Fx2_reg] = RP.invdyn2_platform(Q_test(i,:)', QD_test(i,:)', QDD_test(i,:)', XE_test(i,:)', XDE_test(i,:)', XDDE_test(i,:)');
        test_Freg = Fx2_reg*dpv - Fx2;
        if any(abs(test_Freg(:)) > 1e-6)
          error('Inversdynamik-Kraft-Regressor stimmt nicht. Max Fehler %1.1e.', max(abs(test_Freg(:))));
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
    fprintf('%s: %d/%d Kombinationen getestet (%d i.O., %d n.i.O) (bei restlichen %d IK falsch).\n', ...
      PName, n_succ+n_fail, n, n_succ, n_fail, n-(n_succ+n_fail));
    num_robots_tested = num_robots_tested + 1;
    if n_succ == n % alle erfolgreich
      robot_list_succ = [robot_list_succ(:)', {PName}];
    elseif n_succ > 0 % min. einer erfolgreich
      robot_list_part = [robot_list_part(:)', {PName}];
    else
      robot_list_fail = [robot_list_fail(:)', {PName}];
    end
    if DynParMode == 3 || DynParMode == 4
      % Prüfe Rang der Informationsmatrix
      fprintf(['%s: Die Informationsmatrix der Inversen Dynamik hat Rang %d. \nBei ', ...
        '%d Dynamikparametern der PKM gibt es also %d zu viel. \nSymbolisch ', ...
        'wurden %d Parameter bestimmt\n'], PName, rank(Information_matrix), length(dpv), ...
        length(dpv)-rank(Information_matrix),length(dpv));
    end
    ResStat = [ResStat; {PName, n_succ, n_fail, n}]; %#ok<AGROW>
    if num_robots_tested >= usr_num_tests_per_dof
      break; % Testen aller Roboter dauert sehr lange. Ergebnis für wenige reicht aus.
    end
  end
  fprintf('Dynamik in symbolischer und numerischer Form für %d/%d PKM mit FG [%s] getestet. %d komplett und %d teilweise erfolgreich\n', ...
    num_robots_tested, length(PNames_Kin), disp_array(EE_FG, '%1.0f'), length(robot_list_succ), length(robot_list_part));
  if ~isempty(robot_list_fail)
    disp('Keine Übereinstimmung bei folgenden Robotern:');
    disp(robot_list_fail(:));
  end
  if ~isempty(robot_list_part(:))
    disp('Teilweise Übereinstimmung bei folgenden Robotern:');
    disp(robot_list_part(:));
  end
  if ~isempty(robot_list_succ)
    disp('Übereinstimmung bei folgenden Robotern:');
    disp(robot_list_succ(:));
  end
  if isempty(ResStat), continue; end
  ResStat.Properties.VariableNames = {'Name', 'AnzahlErfolg', 'AnzahlFehlschlag', 'AnzahlTests'};
  restabfile = fullfile(respath, sprintf('ParRob_invdyn_test_DoF_%s_DynParMode%d.csv', char(48+EE_FG), DynParMode));
  writetable(ResStat, restabfile, 'Delimiter', ';');
  %% Prüfe Erfolg. Fehler bei Nicht-Erfolg
  if sum(ResStat.AnzahlFehlschlag) > 0
    error('Die Dynamik nach zwei Implementierungen stimmt nicht überein. Fehler.');
  end
end % DynParMode
end % EEFG
