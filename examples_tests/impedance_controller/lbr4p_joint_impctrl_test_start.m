% Beispiel für Gelenk-Impedanzregler am Beispiel des KUKA LBR4+ Roboters
% 
% Inhalt:
% * Definition der Parameter für den LBR4+
% * Einrichtung und Parametrierung des Simulink-Modells
% * Simulation des Reglers mit Beispiel-Trajektorie und externer Kraft
% * Auswertung
% 
% Eigenschaften des Modells:
% * Zeitdiskreter Regler
% * Rauschen auf Sensorgrößen simulierbar
% * Keine Reibung (auch wenn Parameter schon implementiert sind)
% * Externe Kartesische Kraft (konstant im Basis-KS)
% 
% Ergebnis:
% * Roboter folgt Trajektorie im Gelenkraum
% * Durch externe Kraft wird der End-Effektor abgedrängt
% 
% Quelle:
% [VorndammeSchToeHad2016] Soft Robotics for the Hydraulic
% Atlas Arms: Joint Impedance Control with Collision Detection and
% Disturbance Compensation; Fig. 12
% (dort mit Atlas Arm v5 gezeigt)
% IRT-Materialien (Moritz Schappler): drc_paper/Atlas_IROS_16/simulations/
% atlas_joint_impctrl_sqrt_damping_test_extforce_comp.m

% Jonathan Vorndamme, Moritz Schappler, IRT, 2015-2016
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-01
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc

%% Vorbereitung
% Laden der Kinematikparameter aus der Modellsammlung für serielle Roboter
if isempty(which('serroblib_path_init.m'))
  warning('Repo mit Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end
SName='S7RRRRRRR1';
RName='S7RRRRRRR1_LWR4P';
% Instanz der Roboterklasse erstellen
RS = serroblib_create_robot_class(SName, RName);
% Debug: Modell neu mit HybrDyn generieren
% serroblib_generate_mapleinput({SName})
% serroblib_generate_code({SName}, true, true)

%% Parameter
% Anzahl Gelenke
NJ = 7;
NQJ = 7;

% Trajektorie: PTP zwischen zwei Posen
QE = [[  0,   0, 0,  0, 0,  0,  0]; ...
      [-45, -60, 0, 60, 0, -60, 0]] * pi/180;
[w_Q,w_QD,w_QDD,w_t] = traj_trapez2_multipoint(QE, 1, 1e-1, 1e-2, 1e-3, 0.25);

q_t0 = w_Q(1,:)';
qD_t0 = zeros(1,NJ)';

% Eingabe der Trajektorie über Hilfs-Struktur
qsollDaten.q = NaN(length(w_t), NJ);
qsollDaten.qD = NaN(length(w_t), NJ);
qsollDaten.qDD = NaN(length(w_t), NJ);
for i = 1:NJ
  qsollDaten.q(:,i) = w_Q(:,i);
  qsollDaten.qD(:,i) = w_QD(:,i);
  qsollDaten.qDD(:,i) = w_QDD(:,i);
end
qsollDaten.t = w_t; % Zeitbasis ist für alle Achsen gleich

%% Externe Kraft
% die externe Kraft wirkt in x0 Richtung 0.5s nach dem Ende der
% Trajektorie, für eine Sekunde
t_Fext = [0;w_t(end)+0.5;w_t(end)+1.5];
F_ext =  [[0;50;0], zeros(3,5)];

%% Simulink-Eingangsdaten
simin_q_soll = struct('time', qsollDaten.t, ...
    'signals', struct('values', qsollDaten.q, 'dimensions', NJ), ...
    'name', 'q_soll');

simin_qD_soll = struct('time', qsollDaten.t, ...
    'signals', struct('values', qsollDaten.qD, 'dimensions', NJ), ...
    'name', 'qD_soll');
  
simin_qDD_soll = struct('time', qsollDaten.t, ...
    'signals', struct('values', qsollDaten.qDD, 'dimensions', NJ), ...
    'name', 'qDD_soll');
  
simin_F0_ext = struct('time', t_Fext, ...
    'signals', struct('values', F_ext, 'dimensions', 6), ...
    'name', 'F0_ext');
  
g_base = [0;0;-9.81];


%% Modell-Parameter
noise = true;
% Dynamik-Parameter LBR (Quelle: DLR)
MPV_rob = [3.551454e-02, 0.000000e+00, 0.000000e+00, 1.285577e+00, -1.200143e-02, 1.997236e-02, 1.742543e-02, 1.330067e+00, -1.958075e-02, 3.470009e+00, 1.375856e-03, 2.281265e-02, -4.589467e-03, -1.001737e-02, 3.206131e-03, -1.143068e-02, -1.879885e-02, 4.075196e-01, 9.632204e-04, -4.441543e-03, 1.090635e-02, 4.263394e-01, -1.091096e-03, -1.374771e+00, 2.366000e-02, -1.365504e-03, -1.127019e-02, -3.428656e-04, 1.201199e-02, -9.829229e-03, 4.121137e-02, -1.786527e-03, -3.617277e-03, -2.520204e-03, -1.065936e-03, 1.101171e-02, 6.636787e-03, 3.635223e-02, -1.712267e-03, 6.564818e-04, -4.540437e-05, 5.329330e-04, 4.425318e-04, 5.916056e-03, -7.546328e-03]';

% Kinematik-Parameter
pkin = RS.pkin; % Aus Modelldatenbank

% Zusatz-Masse am End-Effektor, damit die letzte Achse nicht so stark
% schwingt (ist numerisch stabiler, da das Verhältnis der Eigenwerte
% angeglichen wird)
rSges = zeros(NJ+1,3);
mges = zeros(NJ+1,1);
Icges =  zeros(NJ+1,6);
% Masse 1kg mit Trägheitstensor wie Kugel, 100mm vom EE-Flansch entfernt.
mZ = 1;
rZ = [0;0;0.10];
IZges = zeros(1,6);
IZges(1:3) = 2/5 * mZ(1) * (60e-3)^2; % Kugel Radius 60mm
mges(end) = mZ;
rSges(end,:) = rZ;
Icges(end,:) = IZges;
[mrSges, Ifges] = ...
  inertial_parameters_convert_par1_par2(rSges, Icges, mges);
% Dynamik-Parameter der Zusatzmasse als Parametervektor
MPV_add = S7RRRRRRR1_convert_par2_MPV_fixb(pkin, mges, mrSges, Ifges);

% perfektes Robotermodell im Regler
MPV_plant=MPV_rob + MPV_add; % MPV for simulated plant
MPV_model=MPV_rob + MPV_add;% MPV for calculating the gravity model
MPV_damp =MPV_rob + MPV_add; % MPV for calculating the inertia matrix (has to be positive definite)
% set noise levels for simulated measurements (from VorndammeSchToeHad2016)
q_noise     = 3.4e-4;
qD_noise    = 5.5e-2;
tau_noise   = 7.3e-2;
F_ext_noise = 2.3;
g_noise     = 2e-4;
% set noise seeds for measurement noises
q_noise_seed     = 0;
qD_noise_seed    = 0;
tau_noise_seed   = 0;
F_ext_noise_seed = 0;
g_noise_seed     = 0;
% set noise switches for measurement noises (1=on, 0=off)
if noise
  q_noise_switch     = 1;
  qD_noise_switch    = 1;
  tau_noise_switch   = 1;
  F_ext_noise_switch = 1;
  g_noise_switch     = 1;
else
  q_noise_switch     = 0;
  qD_noise_switch    = 0;
  tau_noise_switch   = 0;
  F_ext_noise_switch = 0;
  g_noise_switch     = 0;
end
% Reibungsparameter der Strecke: Auf Null setzen
d   = 0.2*ones(NJ,1);
muC = 8*ones(NJ,1);
d = d*0;
muC = muC*0;

% controller parameters and gains
% Zur Bedeutung der Parameter, siehe Maske des Regler-Blocks im Modell und
% siehe [VorndammeSchToeHad2016]
K_d     = 300*eye(NJ);
D       = 0.7*eye(NJ); % modale Dämpfung 0.7 -> schnelles Einschwingen
K_O     = 10*eye(NJ); % Zeitkonstante für Beobachter: So langsam, dass man das Einschwingen gut erkennen kann
T       = 0.0005; % Abtastzeit des Reglers: 2kHz (etwas höher als in Realität, damit Simulation stabil bleibt)
T1      = 0;
K_tau_g = 1;
K_tau_c = 1;
K_tau_K = 1;
K_tau_D = 1;
K_tau_a = 1;
K_tau_f = 0;
K_tau_e = 0;
K_tau_o = 0;
K_ext_c = 0;
K_obs_c = 0;
K_fri_o = 0;
K_ext_o = 0;
K_qD_d  = 1;
K_qDD_d = 1;
% Reibungsparameter des Reglers: Auf Null setzen
tau_c   = muC - 4*ones(NJ,1);
B       = d - 0.1*ones(NJ,1);
qD_th   = 0.05*ones(NJ,1);
tau_th  = 0.02*ones(NJ,1);

tau_c = 0*tau_c;
B = 0*B;
qD_th = 0*qD_th;
tau_th = 0*tau_th;

% Filter für Ein- und Ausgänge des Reglers: Vorerst keine
% (Tiefpass-)Filterung
T1_filter_input = 0;
T1_filter_output = 0;

%% Configure Model
sl_Modellname = 'lbr4p_joint_impctrl_test';
load_system(sl_Modellname)
configSet = getActiveConfigSet(sl_Modellname);
set_param(configSet, 'Solver', 'ode4');
set_param(configSet, 'FixedStep', '1e-4');

%% Define Inputs
simin_tau_ext = struct('time', 0, ...
    'signals', struct('values', zeros(1,NJ), 'dimensions', NJ), ...
    'name', 'tau_ext');
simin_tau_m = struct('time', 0, ...
    'signals', struct('values', zeros(1,NJ), 'dimensions', NJ), ...
    'name', 'tau_m');

%% Start Simulation
t1 = tic;
simOut = sim(sl_Modellname, 'StopTime', sprintf('%1.1f', t_Fext(end)+1.1));
sl = get_simulink_outputs(simOut, sl_Modellname);
fprintf('Simulink-Modell berechnet. Rechenzeit: %1.1fs\n', toc(t1));

%% Plot
figure(1);clf;
set(1, 'Name', 'q', 'Numbertitle', 'off');
for i = 1:NJ
  subplot(NJ,1,i);hold on;
  plot(sl.t, sl.q(:,i));
  plot([simin_q_soll.time;sl.t(end)], [simin_q_soll.signals.values(:,i);simin_q_soll.signals.values(end,i)], '--');
  ylabel(sprintf('q_%d', i));grid on;
  legend({'sim.', 'des.'});
  if i == 1, title('Verlauf Gelenkwinkel (Ist vs Soll)'); end
end
grid on
linkxaxes

figure(2);clf;
set(2, 'Name', 'qD', 'Numbertitle', 'off');
for i = 1:NJ
  subplot(NJ,1,i);hold on;
  plot(sl.t, sl.qD(:,i));
  plot([simin_qD_soll.time;sl.t(end)], [simin_qD_soll.signals.values(:,i);simin_qD_soll.signals.values(end,i)], '--');
  ylabel(sprintf('qD_%d', i));grid on;
  legend({'sim.', 'des.'});
  if i == 1, title('Verlauf Gelenkgeschwindigkeit (Ist vs Soll)'); end
end
grid on
linkxaxes

figure(3);clf;
set(3, 'Name', 'qDD', 'Numbertitle', 'off');
for i = 1:NJ
  subplot(NJ,1,i);hold on;
  plot(sl.t, sl.qDD(:,i));
  plot([simin_qDD_soll.time;sl.t(end)], [simin_qDD_soll.signals.values(:,i);simin_qDD_soll.signals.values(end,i)], '--');
  ylabel(sprintf('qDD_%d', i));grid on;
  legend({'sim.', 'des.'});
  if i == 1, title('Verlauf Gelenkbeschleunigung (Ist vs Soll)'); end
end
grid on
linkxaxes

figure(4);clf;
set(4, 'Name', 'E', 'Numbertitle', 'off');
plot(sl.t, sl.E);
legend({'T', 'U', 'Ges'});
grid on
linkxaxes
title('Verlauf Energie');
ylabel('Energie [J]');

figure(5);clf;
set(5, 'Name', 'tau', 'Numbertitle', 'off');
for i = 1:NJ
  subplot(NJ,1,i);hold on;
  plot(sl.t, sl.tau_m(:,i));
  plot(sl.t, sl.tau_ext(:,i), '--');
  plot(sl.t, sl.tau_obs(:,i));
  ylabel(sprintf('\\tau_%d', i));grid on;
  legend({'motor', 'extern', 'observer'});
  if i == 1, title('Verlauf Gelenkmomente'); end
end
linkxaxes
grid on

figure(6);clf;
set(6, 'Name', 'xE', 'Numbertitle', 'off');
xE_names = {'x', 'y', 'z', 'phi_x', 'phi_y', 'phi_z'};
for i = 1:2
  for j = 1:3
    subplot(3,2,sprc2no(3,2,j,i));hold on;
    plot(sl.t, sl.x_E(:,3*(i-1)+j));
    ylabel(xE_names{3*(i-1)+j});grid on;
  end
end
linkxaxes
grid on

%% Bewegung visualisieren
s_anim = struct( 'gif_name', []);
s_plot = struct( 'ks', [1:RS.NJ, RS.NJ+2], 'mode', 2);
figure(10);clf;
set(10, 'Name', 'Visualisierung', 'Numbertitle', 'off');
hold on;
grid on;
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
view(3);
title('Simulationsergebnisse LBR4+');
II = 1:round(length(sl.t)/30):length(sl.t);
RS.anim( sl.q(II,:), s_anim, s_plot);

%% Formatierung
dockall