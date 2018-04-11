% Vergleiche die Integration von Winkelbeschleunigungen mit
% unterschiedlichen Methoden und Einstellungen
% 
% * kontinuierliche Integration
% * Integration von Quaternionen
% * Integration mit RPY-Winkeln
% * zeitdiskrete Integration mit Achse/Winkel-Umrechnung der Winkelgeschw.
% * Winkelgeschwindigkeit im Welt- oder mitgedrehtem Körper-Koordinatensystem
% 
% Quellen:
% [ZupanSaj2011] Integrating rotation from angular velocity (2011)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-09
% (c) Institut für Regelungstechnik, Universität Hannover

clear
clc
close all

TB_path = fileparts(which('robotics_toolbox_path_init.m'));
% in Ordner der Simulink-Modelle wechseln, damit mex-Datei des Modells dort liegt.
cd(fullfile(TB_path, 'examples_tests', 'rotation_integration'));

% Ergebnisordner
res_path = fullfile(TB_path, 'examples_tests', 'results');

%% Init
mex_script_dependencies(mfilename('fullpath'), false);

%% Einstellungen
T_end = 100;
Tolerance_vector = [1e-6 1e-10];
T_Step_vector = [1e-3 1e-4];

useFixedStep=true;

if (useFixedStep)
  feature_vector = T_Step_vector;
else
  feature_vector = Tolerance_vector; %#ok<UNRCH>
end

files = cell(2,length(feature_vector));
names = cell(2,length(feature_vector));
sl_modelnames = {...
  %   'rotation_integration_R_discrete_test', ...
  'rotation_integration_R_continous_test', ...
  'rotation_integration_rotvec_continous_test', ...
  'rotation_integration_quat_continous_test', ...
  'rotation_integration_rpyD_test'};
%% Modelle mit unterschiedlichen Einstellungen starten
for i = 1:length(feature_vector)

  % Trajektorie für RPY-Winkel generieren
  % Siehe [ZupanSaj2011] Kap. 5
  f = [10 10 10]';
  A = [0 1 -0.5]';
  rpy_offsets = [0 0 0]';
  rpy_phases = [0 0 0]';
  
  rpy_t0=A.*sin(rpy_phases)+rpy_offsets;
  omega0_t0 = rpyD2omega(rpy_t0, 2*pi*f.*A.*cos(rpy_phases));
  R_t0 = rpy2r(rpy_t0);
  [theta, k] = r2angvec(R_t0);
  rotvec_t0 = k*theta;
  quat_t0 = r2quat(R_t0);

  %% Simulink Modelle starten
  for j_mdl = 1:length(sl_modelnames)
    %% Configure Model
    sl_Modellname = sl_modelnames{j_mdl};
    load_system(sl_Modellname)
    configSet = getActiveConfigSet(sl_Modellname);
    if (useFixedStep)
      set_param(configSet, 'Solver', 'ode4','FixedStep',sprintf('%e',feature_vector(i)));
      %set_param(configSet, 'FixedStep', sprintf('%1.1e', T_step_vector(i)));
    else
      set_param(configSet, 'Solver', 'ode45','RelTol',sprintf('%e',feature_vector(i))); %#ok<UNRCH>
    end
    %% Start Simulation
    t1 = tic;
    simOut = sim(sl_Modellname, 'StopTime', sprintf('%f', T_end), ...
      'SimulationMode', 'normal'); % normal
    sl = get_simulink_outputs(simOut, sl_Modellname);
    fprintf('Simulink-Modell berechnet. Rechenzeit: %1.1fs\n', toc(t1));
    
    %% Nachbearbeitung
    sl.rpy_int = NaN(length(sl.t), 3);
    for jj = 1:length(sl.t)
      sl.rpy_int(jj,:) = r2rpy_mex(sl.R_int(:,:,jj));
    end
    
    t_input = sl.t;

    rpy_matrix =   [sl.alpha sl.beta sl.gamma];
    rpyD_matrix =  [sl.alphaD sl.betaD sl.gammaD];
    rpyDD_matrix = [sl.alphaDD sl.betaDD sl.gammaDD];

    omega_matrix = NaN(size(rpy_matrix));
    omegaD_matrix = NaN(size(rpy_matrix));
    % Winkelgeschwindigkeit und Beschleunigung generieren
    for jj = 1:size(rpy_matrix,1)
      omega_matrix(jj,:) = rpyD2omega(rpy_matrix(jj,:)', rpyD_matrix(jj,:)');
      omegaD_matrix(jj,:) = rpyDD2omegaD(rpy_matrix(jj,:)', rpyD_matrix(jj,:)', rpyDD_matrix(jj,:)');
    end
    
    %% Eingabedaten auch speichern
    sl.rpy_soll = rpy_matrix;
    sl.omega_soll = omega_matrix;
    sl.omegaD_soll = omegaD_matrix;
    sl.t_soll = t_input; % kann bei variabler Schrittweite anders sein
    sl.R_soll = NaN(3,3,size(rpy_matrix,1));
    for jj = 1:size(rpy_matrix,1)
      sl.R_soll(:,:,jj) = rpy2r(rpy_matrix(jj,:)');
    end
    
    %% Speichern
    files{j_mdl, i} = fullfile(res_path, sprintf('rotation_integration_test_mdl%d_step%d.mat', j_mdl, i));
    save(files{j_mdl, i}, 'sl');
    
    names{j_mdl, i} = sprintf('%s/%1.1e', sl_Modellname, feature_vector(i));
  end
end

%% Plot
xyz = {'x', 'y', 'z'};
figure(1);clf;set(1, 'Name', 'omegaD', 'NumberTitle', 'off');
figure(2);clf;set(2, 'Name', 'omega', 'NumberTitle', 'off');
figure(3);clf;set(3, 'Name', 'RPY', 'NumberTitle', 'off');
figure(4);clf;set(4, 'Name', 'omega_konsistenz', 'NumberTitle', 'off');
figure(5);clf;set(5, 'Name', 'omega_soll_ist', 'NumberTitle', 'off');
figure(6);clf;set(6, 'Name', 'rpy_soll_ist', 'NumberTitle', 'off');
for i = 1:length(feature_vector)
  for j_mdl = 1:length(sl_modelnames)
    load(files{j_mdl, i});
    % Winkelbeschleunigung
    figure(1);
    for k = 1:3
      subplot(3,2,sprc2no(3,2,k,1));hold all;
      plot(sl.t_soll, sl.omegaD_soll(:,k));
      ylabel(sprintf('omegaD %s', xyz{k}));
      grid on;
    end
    if i == length(feature_vector) && j_mdl == length(sl_modelnames)
      legend(names(:), 'interpreter', 'none');
    end
    linkxaxes
    
    % Winkelgeschwindigkeit
    figure(2);
    for k = 1:3
      subplot(3,2,sprc2no(3,2,k,1));hold all;
      plot(sl.t_soll, sl.omega_soll(:,k));
      ylabel(sprintf('omega soll %s', xyz{k}));
      grid on;
      
      subplot(3,2,sprc2no(3,2,k,2));hold all;
      plot(sl.t, sl.omega_int(:,k));
      ylabel(sprintf('omega int %s', xyz{k}));
      grid on;
    end
    if i == length(feature_vector) && j_mdl == length(sl_modelnames)
      legend(names(:), 'interpreter', 'none');
    end
    linkxaxes
    
    % Orientierung
    figure(3);
    for k = 1:3
      subplot(3,2,sprc2no(3,2,k,1));hold all;
      plot(sl.t_soll, sl.rpy_soll(:,k));
      ylabel(sprintf('RPY soll %d %s', k, xyz{k}));
      grid on;
      
      subplot(3,2,sprc2no(3,2,k,2));hold all;
      plot(sl.t, sl.rpy_int(:,k));
      ylabel(sprintf('RPY int %d %s', k, xyz{k}));
      grid on;
    end
    if i == length(feature_vector) && j_mdl == length(sl_modelnames)
      legend(names(:), 'interpreter', 'none');
    end
    linkxaxes
    
    % Konsistenz von Beschleunigung und Geschwindigkeit
    figure(4);
    for k = 1:3
      subplot(3,3,sprc2no(3,3,1,k));hold all;
      plot(sl.t_soll, sl.omegaD_soll(:,k));
      ylabel(sprintf('omegaD %s soll', xyz{k}));
      grid on;
      
      subplot(3,3,sprc2no(3,3,2,k));hold all;
      plot(sl.t_soll, sl.omega_soll(:,k));
      ylabel(sprintf('omega %s soll', xyz{k}));
      grid on;
      
      subplot(3,3,sprc2no(3,3,3,k));hold all;
      plot(sl.t, sl.omega_int(:,k));
      ylabel(sprintf('omega %s int', xyz{k}));
      grid on;
    end
    if i == length(feature_vector) && j_mdl == length(sl_modelnames)
      legend(names(:), 'interpreter', 'none');
    end
    linkxaxes
    
    figure(5);
    for k = 1:3
      if j_mdl==1 && i==1
        subplot(1,3,sprc2no(1,3,1,k));hold all;
        plot(sl.t_soll, sl.omega_soll(:,k));
      end
      
      subplot(1,3,sprc2no(1,3,1,k));hold all;
      plot(sl.t, sl.omega_int(:,k));
      ylabel(sprintf('omega %s', xyz{k}));
      grid on;
    end
    if i == length(feature_vector) && j_mdl == length(sl_modelnames)
      legend(['omega_soll'; names(:)], 'interpreter', 'none');
    end
    linkxaxes
    
    figure(6);
    for k = 1:3
      if j_mdl==1 && i==1
        subplot(1,3,sprc2no(1,3,1,k));hold all;
        plot(sl.t_soll, sl.rpy_soll(:,k));
      end
      
      subplot(1,3,sprc2no(1,3,1,k));hold all;
      plot(sl.t, sl.rpy_int(:,k));
      ylabel(sprintf('rpy %s', xyz{k}));
      grid on;
    end
    if i == length(feature_vector) && j_mdl == length(sl_modelnames)
      legend(['rpy_soll'; names(:)], 'interpreter', 'none');
    end
    linkxaxes
    
  end
end
dockall

%% Teste Zweiten Satz an Simulink-Modellen für die Integration
% Diese Modelle dienen dazu, mit der Matlab-Funktion angvel_int_sl direkt
% aufgerufen zu werden (im Gegensatz zu den Testmodellen für die
% Bibliotheksblöcke von oben)

sl_modelnames_worldframe = {...
  'angvel_int_quat', ...
  'angvel_int_rpy', ...
  'angvel_int_rotmat'};
sl_modelnames_bodyframe = {...
  'angvel_body_int_rotmat', ...
  'angvel_body_int_quat'};
legendentries = {sl_modelnames_worldframe{:}, sl_modelnames_bodyframe{:}}; %#ok<CCAT>
% Trajektorie für Winkelgeschwindigkeiten generieren
t_ges = (0:1e-3:20)';
f1 = [1;3;5];
f2 = [7;11;9];
omega_W_ges = (sin(2*pi*f1*t_ges') .* sin(2*pi*f2*t_ges'))';
R_t0 = rpy2r(rand(3,1));
Ts = 1e-3;
R_int_erg = {};
% Berechne Modelle mit Eingabe der Winkelgeschw. in Welt-KS
for i = 1:length(sl_modelnames_worldframe)
  sl_Modellname = sl_modelnames_worldframe{i};
  options=simset('FixedStep',sprintf('%e',Ts), 'Solver', 'ode4', ...
    'ReturnWorkspaceOutputs','on', ... % Ausgabe als Signalstruktur
    'SrcWorkspace', 'current'); % Variablennamen des Funktions-Workspace benutzen
  simOut = sim(sl_Modellname, t_ges(end)-t_ges(1),options, [t_ges-t_ges(1), omega_W_ges]);
  sl = get_simulink_outputs(simOut, sl_Modellname);
  R_int_erg{i} = sl.R_W_B;
end

% Rechne Winkelgeschw. in Körper-KS um.
% Nehme Ergebnisse des Quaternion-Modells als wahren Wert für die
% Rotationsmatrix
R_int1 = R_int_erg{1};
omega_B_ges = NaN(size(omega_W_ges));
for i = 1:size(omega_W_ges,1)
  R_W_B_i = R_int1(:,:,i);
  omega_B_ges(i,:) = R_W_B_i' * omega_W_ges(i,:)';
end

% Berechne Modelle mit Eingabe der Winkelgeschw. in bewegtem Körper-KS
for i = 1:length(sl_modelnames_bodyframe)
  sl_Modellname = sl_modelnames_bodyframe{i};
  options=simset('FixedStep',sprintf('%e',Ts), 'Solver', 'ode4', ...
    'ReturnWorkspaceOutputs','on', ... % Ausgabe als Signalstruktur
    'SrcWorkspace', 'current'); % Variablennamen des Funktions-Workspace benutzen
  simOut = sim(sl_Modellname, t_ges(end)-t_ges(1),options, [t_ges-t_ges(1), omega_B_ges]);
  sl = get_simulink_outputs(simOut, sl_Modellname);
  R_int_erg{length(sl_modelnames_worldframe)+i} = sl.R_W_B;
end

% Vergleiche alle Ergebnisse
figure(1);clf;
for i = 1:3
  subplot(3,1,i);hold on;
  plot(t_ges, omega_W_ges(:,i));
end
figure(2);clf;
for i = 1:3
  for j = 1:3
    subplot(3,3,sprc2no(3,3,i,j));
    hold on;
    for kk = 1:5
      R_int_erg_kk = R_int_erg{kk};
      plot(t_ges, squeeze(R_int_erg_kk(i,j,:)));
    end
    grid on;
  end
end
legend(legendentries);
linkxaxes