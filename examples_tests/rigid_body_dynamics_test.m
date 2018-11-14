% Teste Starrkörperdynamikmodelle (Vorwärtsdynamik eines Starrkörpers im
% freien Raum)
% 
% Modelle:
% * rigid_body_fdyn_rotmat_test_nocom
%     Vorwärtsdynamik des Starrkörpers ohne Betrachtung der
%     Schwerpunktskoordinaten. Da der Drallsatz um den Schwerpunkt gebildet
%     wird ist der Schwerpunkt auch egal (das Körper-KS wird nicht
%     verwendet).
% * rigid_body_fdyn_rotmat_test_com
%     Die gleiche Vorwärtsdynamik, es wird nur außerdem die Bewegung des
%     Körper-KS berechnet (und wieder herausgerechnet).
% * rigid_body_fdyn_rotmat_test_eulxyz
%     Die Vorwärtsdynamik wurde mit Lagrange aufgestellt (aus der
%     Maple-Toolbox). Für die Basis-Orientierung wurden XYZ-Eulerwinkel
%     verwendet.

% Ergebnis:
% Vorwärtsdynamik numerisch mit Impuls- und Drallsatz ergibt gleiche Bewegung und
% Energiesumme wie symbolisch mit Lagrange berechnete
% Bei einem passiven System ohne externe Kräfte (Variable `A_Fext` = 0)
% bleibt die Gesamtenergie konstant. Bei vorhandenen externen Kräften ist
% die Energie nicht konstant.

% Standardeinstellungen:
% * Zufälliger Anfangszustand (Transl. und rot. Position, Geschwindigkeit)
% * Gravitation
% * keine externen Kräfte

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
% (c) Institut für Regelungstechnik, Universität Hannover

clc
clear
close all

this_repo_path = fullfile(fileparts(which('robotics_toolbox_path_init.m')));
addpath(fullfile(this_repo_path, 'dynamics', 'rigidbody_fdyn'));

%% Einstellungen
% Masse
m = 1.5;

% Trägheitstensor um den Schwerpunkt im Körper-KS (B)
I_B_C = inertiavector2matrix([4, 8, 15,-1,-2,-0.5]);
if any(eig(I_B_C) < 0)
  error('Gewählter Trägheitstensor ist nicht positiv definit');
end

% beliebige Schwerpunktskoordinaten (im Körper-KS B)
r_B_B_C = [.16, .23, .42]';

% Parameter für symbolische Funktionen
KinPar_mdh = struct('a', [], 'd', [], 'alpha', [], ...
  'q_offset', [], 'b', [], 'beta', [], 'v', []);
DynPar1 = struct('m', m, 'r_S', r_B_B_C', 'I_S', inertiamatrix2vector(I_B_C));

% Gravitation
g_W = [0;0;-9.81];

% Beliebige Anfangsposition und -geschwindigkeit der _Basis_
r0 = [7;9;11];
rD0 = [-6;-2;8];

% beliebige Anfangs-Winkelgeschwindigkeit
omega0_t0 = [7;8;9];

% beliebige Anfangsorientierung
R_W_B_t0 = rotx(pi/3)*roty(pi/6)*rotz(pi/4);

t_End = 5;

% externe Kräfte (Welt-KS): Beliebige Werte für Amplitude, Frequenz, Phase
% einer harmonischen Schwingung für jede Komponente
t_Fext = (0:1e-3:t_End)'; % Zeitbasis
A_Fext = [5 10 37 1 4 6 ]*0; % Amplituden der Kraft- und Momentkomponenten
f_Fext = [2 4 3 5 6 1]; % Frequenz
phi_Fext = [0 30 60 120 180 210]*pi/180;
y_Fext = repmat(A_Fext, length(t_Fext), 1) .* ... % Amplitude
         sin(repmat(t_Fext, 1, 6).*2.*pi.*repmat(f_Fext, length(t_Fext), 1) + ... % Frequenz
         repmat(phi_Fext, length(t_Fext), 1)); % Phase
simin_F_ext = struct('time', t_Fext, ...
    'signals', struct('values', y_Fext, 'dimensions', 6), ...
    'name', 'tau_ext');

labelStrings = {'x', 'y', 'z', 'abs'};
%% Start
ErgStruct = cell(1,2);
Namen = {'RNEA-MitCOM', 'LagrangeEulerXYZ'}; % 'RNEA-OhneCOM', 
for i_Modus = 1:2
  if i_Modus == 3
    % Modell wird nicht mehr betrachtet
    sl_Modellname = 'rigid_body_fdyn_rotmat_test_nocom';
  elseif i_Modus == 1
    sl_Modellname = 'rigid_body_fdyn_rotmat_test_com';
  elseif i_Modus == 2
    NJ = 0;
    phi_base_t0 = r2eulxyz(R_W_B_t0);
    T_basevel_t0 = eulxyzjac(phi_base_t0);
    phiD_base_t0 = T_basevel_t0\omega0_t0;
    sl_Modellname = 'rigid_body_fdyn_rotmat_test_eulxyz';
  end
  load_system(sl_Modellname)
  configSet = getActiveConfigSet(sl_Modellname);
  set_param(configSet, 'Solver', 'ode4', 'FixedStep',sprintf('%e',1e-4));
  simOut = sim(sl_Modellname, 'StopTime', sprintf('%d', t_End), ...
    'SimulationMode', 'normal'); % normal
  sl = get_simulink_outputs(simOut, sl_Modellname);
  fprintf('Modell %s berechnet.\n', sl_Modellname);
  %% Nachbearbeitung
  % Beträge berechnen
  sl.r_W_C(:,4) = (sl.r_W_C(:,1).^2 + sl.r_W_C(:,2).^2 + sl.r_W_C(:,3).^2).^0.5;
  sl.rD_W_C(:,4) = (sl.rD_W_C(:,1).^2 + sl.rD_W_C(:,2).^2 + sl.rD_W_C(:,3).^2).^0.5;
  sl.rDD_W_C(:,4) = (sl.rDD_W_C(:,1).^2 + sl.rDD_W_C(:,2).^2 + sl.rDD_W_C(:,3).^2).^0.5;
  
  sl.r_W_B(:,4) = (sl.r_W_B(:,1).^2 + sl.r_W_B(:,2).^2 + sl.r_W_B(:,3).^2).^0.5;
  sl.rD_W_B(:,4) = (sl.rD_W_B(:,1).^2 + sl.rD_W_B(:,2).^2 + sl.rD_W_B(:,3).^2).^0.5;
  sl.rDD_W_B(:,4) = (sl.rDD_W_B(:,1).^2 + sl.rDD_W_B(:,2).^2 + sl.rDD_W_B(:,3).^2).^0.5;
  %% Abspeichern
  ErgStruct{i_Modus} = sl;
end
Ip = 1:10:length(sl.t);

%% Ergebnisse plotten
for i_Modus = 1:length(ErgStruct)
  sl = ErgStruct{i_Modus};
  %% Plot: Energie
  figure(1);
  if i_Modus==1, clf; end
  E_ges = sl.E_pot_transl + sl.E_kin_rot + sl.E_kin_transl;
  subplot(2,2,1);hold on;
  plot(sl.t(Ip), [sl.E_pot_transl(Ip)]);
  title('Energiebetrachtung (numerische Berechnung)');
  ylabel('pot,transl');grid on;
  subplot(2,2,2);hold on;
  plot(sl.t(Ip), [sl.E_kin_rot(Ip)]);
  ylabel('kin,rot');grid on;
  subplot(2,2,3);hold on;
  plot(sl.t(Ip), [sl.E_kin_transl(Ip)]);
  ylabel('kin,transl');grid on;
  subplot(2,2,4);hold on;
  plot(sl.t(Ip), E_ges(Ip)-E_ges(1));
  % plot(sl.t(Ip), sl.E_pot_transl+sl.E_kin_transl);
  ylabel('ges (abzgl. Startenergie)');grid on;
  if i_Modus==length(ErgStruct),    legend(Namen); end
  set(1, 'Name', 'E', 'NumberTitle', 'off');
  linkxaxes
  %% Energie (symbolisch berechnet)
  figure(10);
  if i_Modus==1, clf; end
  if isfield(sl, 'E_pot_sym')
    E_ges_sym = sl.E_pot_sym + sl.E_kin_sym;
  else
    E_ges_sym = NaN(length(sl.t), 1);
  end
  subplot(2,2,1);hold on;
  title('Energiebetrachtung (symbolische Berechnung)');
  if isfield(sl, 'E_pot_sym')
    plot(sl.t(Ip), [sl.E_pot_sym(Ip)]);
  else
    plot(sl.t(Ip), NaN(length(Ip), 1));
  end
  ylabel('pot,sym');grid on;
  subplot(2,2,2);hold on;
  if isfield(sl, 'E_pot_sym')
    plot(sl.t(Ip), [sl.E_kin_sym(Ip)]);
  else
    plot(sl.t(Ip), NaN(length(Ip), 1));
  end
  ylabel('kin,sym');grid on;

  subplot(2,2,4);hold on;
  plot(sl.t(Ip), E_ges_sym(Ip)-E_ges_sym(1));
  % plot(sl.t(Ip), sl.E_pot_transl+sl.E_kin_transl);
  ylabel('ges,sym (abzgl. Startenergie)');grid on;
  if i_Modus==length(ErgStruct),    legend(Namen); end
  set(10, 'Name', 'E_sym', 'NumberTitle', 'off');
  linkxaxes
  %% Plot: Translatorische Position (des Schwerpunktes)
  figure(5);
  if i_Modus==1, clf; end
  set(5, 'Name', 'r_C', 'NumberTitle', 'off');
  for i = 1:4
    subplot(2,2,i);hold on;grid on;
    plot(sl.t(Ip), sl.r_W_C(Ip,i));
    ylabel(sprintf('transl. Pos. %s (Schwerp.)', labelStrings{i}));
  end
  if i_Modus==length(ErgStruct),    legend(Namen); linkxaxes; end
  
  %% Plot: Translatorische Geschwindigkeit (des Schwerpunktes)
  figure(6);
  if i_Modus==1, clf; end
  set(6, 'Name', 'v_C', 'NumberTitle', 'off');
  for i = 1:4
    subplot(2,2,i);hold on;grid on;
    plot(sl.t(Ip), sl.rD_W_C(Ip,i));
    ylabel(sprintf('transl. Geschw. %s (Schwerp.)', labelStrings{i}));
  end
  if i_Modus==length(ErgStruct),    legend(Namen); linkxaxes; end
  
  %% Plot: Translatorische beschleunigung (des Schwerpunktes)
  figure(7);
  if i_Modus==1, clf; end
  set(7, 'Name', 'a_C', 'NumberTitle', 'off');
  for i = 1:4
    subplot(2,2,i);hold on;grid on;
    plot(sl.t(Ip), sl.rDD_W_C(Ip,i));
    ylabel(sprintf('transl. Beschl. %s (Schwerp.)', labelStrings{i}));
  end
  if i_Modus==length(ErgStruct),    legend(Namen); linkxaxes; end
  
  
  %% Plot: Translatorische Position (der Basis)
  figure(11);
  if i_Modus==1, clf; end
  set(11, 'Name', 'r_B', 'NumberTitle', 'off');
  for i = 1:4
    subplot(2,2,i);hold on;grid on;
    plot(sl.t(Ip), sl.r_W_B(Ip,i));
    ylabel(sprintf('transl. Pos. %s (Basis)', labelStrings{i}));
  end
  if i_Modus==length(ErgStruct),    legend(Namen); linkxaxes; end
  
  %% Plot: Translatorische Geschwindigkeit (der Basis)
  figure(12);
  if i_Modus==1, clf; end
  set(12, 'Name', 'v_B', 'NumberTitle', 'off');
  for i = 1:4
    subplot(2,2,i);hold on;grid on;
    plot(sl.t(Ip), sl.rD_W_B(Ip,i));
    ylabel(sprintf('transl. Geschw. %s (Basis)', labelStrings{i}));
  end
  if i_Modus==length(ErgStruct),    legend(Namen); linkxaxes; end
  
  %% Plot: Translatorische beschleunigung (der Basis)
  figure(13);
  if i_Modus==1, clf; end
  set(13, 'Name', 'a_B', 'NumberTitle', 'off');
  for i = 1:4
    subplot(2,2,i);hold on;grid on;
    plot(sl.t(Ip), sl.rDD_W_B(Ip,i));
    ylabel(sprintf('transl. Beschl. %s (Basis)', labelStrings{i}));
  end
  if i_Modus==length(ErgStruct),    legend(Namen); linkxaxes; end
  
  %% Plot: Winkelgeschwindigkeit
  figure(2);
  if i_Modus==1, clf; end
  set(2, 'Name', 'omega', 'NumberTitle', 'off');
  for i = 1:3
    subplot(3,1,i);hold on;grid on;
    plot(sl.t(Ip), sl.omega_W_B(Ip,i));
    ylabel(sprintf('Winkelgeschw. %s', labelStrings{i}));
  end
  if i_Modus==length(ErgStruct),    legend(Namen); linkxaxes; end
  
  %% Plot: Winkelbeschleunigung
  figure(3);
  if i_Modus==1, clf; end
  set(3, 'Name', 'omegaD', 'NumberTitle', 'off');
  for i = 1:3
    subplot(3,1,i);hold on;grid on;
    plot(sl.t(Ip), sl.omegaD_W_B(Ip,i));
    ylabel(sprintf('Winkelbeschl. %s', labelStrings{i}));
  end
  if i_Modus==length(ErgStruct),    legend(Namen); linkxaxes; end
  
  %% Plot: Rotationsmatrix
  figure(4); 
  if i_Modus==1, clf; end
  set(4, 'Name', 'RotMat', 'NumberTitle', 'off');
  for i = 1:3
    for j = 1:3
      subplot(3,3,sprc2no(3,3,i,j));hold on;grid on;
      plot(sl.t(Ip), squeeze(sl.R_W_B(i,j,Ip)));
    ylabel(sprintf('Rotationsmatrix Eintrag R_{%d,%d}', i, j));
    end
  end
  if i_Modus==length(ErgStruct),    legend(Namen); linkxaxes; end
  
  %% Test auf Energiekonsistenz
  Delta_E = max(E_ges) - min(E_ges);
  if Delta_E > 1e-5
    warning('Nicht energiekonsistent. Fehler: %e', Delta_E);
  end
end
dockall

return
%% Debug: Lagrange Euler-XYZ
% Nachrechnen des ersten Zeitschrittes

% Initialisierung für symbolisch generierte Funktionen
a_mdh = zeros(0,1);
d_mdh = zeros(0,1);
b_mdh = zeros(0,1);
beta_mdh = zeros(0,1);
alpha_mdh = zeros(0,1);
q_offset_mdh = zeros(0,1);

m = DynPar1.m;
r_S = DynPar1.r_S;
I_S = DynPar1.I_S;


phi_base = phi_base_t0;
xD_base = [rD0; phiD_base_t0];
g_world = g_W;
T_basevel = rpy2jac(phi_base);

% Symbolisch berechnete Ausdrücke
Mq =rigidbody_inertia_floatb_eulxyz_slag_vp1(zeros(0,1), phi_base, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh, m, r_S, I_S);
tauc = rigidbody_coriolisvec_floatb_eulxyz_slag_vp1(zeros(0,1), zeros(0,1), phi_base, xD_base, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh, m, r_S, I_S);
taug_base = rigidbody_gravload_base_floatb_eulxyz_slag_vp1(zeros(0,1), phi_base, g_world, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh, m, r_S);
tau_ext = [eye(3), zeros(3,3); zeros(3,3), T_basevel'] * y_Fext(1,1:6)';

% Nachrrechnen der Beschleunigungen im ersten Zeitschritt: Vergleich mit
% den Plots möglich
xDD0 = Mq \ (-tauc - taug_base + tau_ext);
phiDD0 = xDD0(4:6);
omegaD_t0 = eulxyzDD2omegaD(phi_base_t0', phiD_base_t0', phiDD0')';
fprintf('omegaD(t=0) = [%s]\n', disp_array(omegaD_t0', '%1.3f'));
aB_t0 = xDD0(1:3);
fprintf('aB(t=0) = [%s]\n', disp_array(aB_t0', '%1.3f'));

% Kartesische Beschleunigung des Schwerpunktes
r_W_B_C = R_W_B_t0 * r_B_B_C;
aC_t0 = aB_t0 + cross(omegaD_t0, r_W_B_C) + cross(omega0_t0, cross(omega0_t0, r_W_B_C));
fprintf('aC(t=0) = [%s]\n', disp_array(aC_t0', '%1.3f'));

% Probe des Gravitationsmomentes der symbolischen Berechnung
taug_base_test = -[eye(3), zeros(3,3); zeros(3,3), T_basevel']* [m*g_world; cross(r_W_B_C, m*g_world)];
fprintf('Fehler Gravitationsmoment: %e\n', max(taug_base-taug_base_test));
%% Probe
% Probe der Massenmatrix (selbst berechnet)

R_W_B = eulxyz2r(phi_base(1), phi_base(2), phi_base(3));
Mq_test_tt = m * eye(3);
Mq_test_tr = (-m*skew(r_S)*T_basevel)
Mq_test_rr = NaN(3,3);
Mq_test_rt = NaN(3,3);
Mq_test = [Mq_test_tt, Mq_test_tr; Mq_test_rt, Mq_test_rr];
Mq_test - Mq
