% Teste das Hunt-Crossley/LuGre Kontaktmodell anhand eines 1DOF-Beispiels:
% Ein Quader steht auf einer ebenen Fläche und wird geschoben
% 
% Es wird eine Parameterstudie durchgeführt: Jeder der LuGre-Parameter wird
% mit unterschiedlichen Werten simuliert und die Ergebnisse verglichen und
% abgespeichert.
% Um eine einzelne Simulation durchzuführen, müssen die Schleifen über die
% Parameter (i_EV) oder die Werte entsprechend gekürzt werden
% 
% Liste der Einstellungen Variable: i_EV
%  1: Variation der Implementierung der LuGre-Kontakte (zeitdiskret,
%  kontinuierlich)
%  2: 
%  ...
%  9: Variation mehrerer LuGre-Parameter, versuche gute Kombinationen für
%     vorgegebene unterschiedliche Steifigkeiten sigma0 zu finden
% 10: Standfläche
% 11: Masse
% 12: Sgn-Näherung der LuGre-Modellierung anpassen (Verschliff der
%     Signum-Funktion)
% 13: Schrittweiten und Solver anpassen (ode1,ode4,ode45)
% 14: Dämpfung des Hunt-Crossley-Modells (Normalkraft)
% 15: Steifigkeit des Hunt-Crossley-Modells (Normalkraft) 
% 16: Unterschiedliche Vorzeichen der Kraft (zur Überprüfung der korrekten
%     Vorzeichen in der Berechnung)
% 
% Vorgehensweise ist ähnlich zu [AstromWit2008], Figure 2
% 
% Ergebnisse der Einstellungen:
%  1: Verhalten qualitativ gleich, nur geringe Abweichungen
%  9: Plausibles Verhalten für Einstellungen 1 und 2, nicht konsistent für
%     3
% 11: Größere Masse verursacht geringere Verschiebung des Körpers
% 12: Plausibles Verhalten, für starke Annäherung an sgn-Funktion wird die
%     Simulation instabil und kinematisch inkonsistent.
%     Bei zeitdiskreter Implementierung ist die Grenze ca. bei kv=500, bei
%     zeitkontinuierlicher ca. bei kv=2000
% 14: Einschwingen beim ersten Aufsetzen ist unterschiedlich stark gedämpft
%     Kein Unterschied bei Schwingungen während des Schiebens
% 15: Höhere Steifigkeit HC verursacht geringeres Einsinken und Kippen. Eigenschaften
%     des LuGre-Modells ungefähr gleich
% 16: Kraft des Kontaktmodells und Bewegung des Körpers sind genau
%     vorzeichenverkehrt (wie physikalisch plausibel).
%     Der Schlupf hat immer das gleiche Vorzeichen
% 
% 
% Siehe auch: rigid_body_dynamics_test.m, LuGre_tangential_friction_model_func.m

% Quellen:
% [AstromWit2008] K.J. Astrom and C. Canudas-de-Wit: Revisiting the LuGre
% friction model 2008)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-11
% (c) Institut für Regelungstechnik, Universität Hannover

clear
clc
%% Init
tb_path = fileparts(which('robotics_toolbox_path_init.m'));
addpath(fullfile(tb_path, 'examples_tests', 'dynamics', 'rigidbody_fdyn'));
addpath(fullfile(tb_path, 'examples_tests', 'contact_models'));
%% Benutzereingaben
usr_BilderSpeichern = 0;

% Parameter 
CONTACT_DISCR=Simulink.Variant('CONTACTMODEL_SWITCH==1');
CONTACT_CONTI=Simulink.Variant('CONTACTMODEL_SWITCH==2');
CONTACTMODEL_SWITCH = 2; % 1=zeitdiskret, 2=kontinuierlich
LuGre_kv_arctan = 500; % Inf=sgn(v), 100=arctan(100*v). Gute Einstellung: 500 zeitdiskret, 2000 zeitkont.
ModalDamping = 1; % Wenn ungleich NaN wird die Einstellregel nach [AstromWit2008] genommen
solver = 'ode4';
usr_EV_Auswahl = 16;
%% Schleife über alle Einstellungsvariationen
% Namen der Einstellungen
EVN = {'Implement', 'LG_sigma0', 'LG_sigma1', 'LG_sigma2', ... % 1-4
  'LG_muC', 'LG_muS', 'LG_vs', 'LG_alpha', 'LG_Kombi', ... % 5-9
  'SupPoly', 'Masse', 'LG_arctan_kv', 'solvertype', ...  % 10-13
  'HC_alpha', 'HC_k', 'f_sgn'}; % 14, 15
% Gehe über alle zu untersuchenden Parameter
for i_EV = usr_EV_Auswahl

if i_EV == 11
  % Es soll nur die Masse geändert werden, nicht darauf noch die
  % LuGre-Parameter so dass sie zur Masse passen
  ModalDamping = NaN;
end
fprintf('Einstellungsvariation %d (%s)\n', i_EV, EVN{i_EV});

% Kontaktmodell
% Bodeneigenschaften
R = 50e-3;
h=2*(1-0.334^2)/69e9;
h=h*5e4;
alpha=5000;
HC_params = [R,h,alpha]';
HC_params0 = HC_params;
% [HaddadinKriAlb2015XX]
LuGre_params = [1000, 100, 0.1, 0.8, 0.9, 20, 1]';
LuGre_params(2) = 300; % [AstromWit2008] vor Gl. (8)
LuGre_params(3) = 40; % [AstromWit2008] Gl. (7)
if ~isnan(ModalDamping)
  LuGre_params(2) = NaN; % Einstellung nach [AstromWit2008]
end

% [AstromWit2008], Figure 2: Werte sind so hoch, dass die Simulation sofort
% instabil wird
% LuGre_params = [1.47*1e6, 2.42*1e3, 0, 2.94/9.81, 5.88/9.81, 0.001, 1]';


% Externe Kraft
% Keine Kraft
% simin_F_ext = struct('time', 0, ...
%     'signals', struct('values', zeros(1,6), 'dimensions', 6), ...
%     'name', 'tau_ext');

% Kraft rampenförmig aufbauen: Rampe startet bei T1, endet bei T2 (letzter
% Wert wird gehalten
T1 = 1;
T2 = 5;
f = [1e1; 0; 0];
t_rampe1 = (T1:1e-3:T2)';
F_rampe1 = ((t_rampe1-T1)/(T2-T1)) * [f;zeros(3,1)]';
% 5s halten, dann wieder mit gleicher Rampe abbauen
T3 = 10;
T4 = 14;
t_rampe2 = (T3:1e-3:T4)';
F_rampe2 = (1-(t_rampe2-T3)/(T4-T3)) * [f;zeros(3,1)]';

simin_F_ext = struct('time', [0;t_rampe1;t_rampe2;20], ...
    'signals', struct('values', [zeros(1,6);F_rampe1;F_rampe2;zeros(1,6)], 'dimensions', 6), ...
    'name', 'F_ext');

% figure();
% plot(simin_F_ext.time, simin_F_ext.signals.values(:,1));
  
%% Modell ausführen
sl_Modellname = 'rigid_body_fdyn_HC_LuGre';
t_End = 20;
Ts = 1e-4;
erg_pfad = fullfile(tb_path, 'examples_tests', 'results');

ErgStruct = {};
ergdat_pfad = {};

for i_Modus = 1:6 % für jede Einstellungsvariation 6 Möglichkeiten vorsehen

  %% Einstellungen des Modells
  % Modell des Quaders: Werte des Atlas-Fußes
  l1 = 0.227; % Tiefe (x-Koord)
  l2 = 0.134; % Breite (y-Koord)
  l3 = 0.05; % Höhe (z-Koord)
%   rho = 7850; % Stahl
%   m = rho*l1*l2*l3;
  m = 2.41;
  rho = m / l1*l2*l3;
  if i_EV == 10 % Änderung der Standfläche, Masse gleich lassen
    if i_Modus == 1
      % Standfläche so lassen
    elseif i_Modus == 2  
      l1 = sqrt(2)*0.227;
      l2 = sqrt(2)*0.134;
    elseif i_Modus == 3
      l1 = 2*0.227;
      l2 = 2*0.134;
    else
      break
    end
    Namen = {'A=1.0*A0', 'A=2.0*A0', 'A=4.0*A0'};
  end
  if i_EV == 11 % Änderung der Masse, alles andere gleich lassen.
    if i_Modus == 1
      % Masse so lassen
    elseif i_Modus == 2 % Größere Masse 
      % rho = 7850*1.5;
      % m = rho*l1*l2*l3;
      m = 1.5*2.41;
      % Masse so lassen
    elseif i_Modus == 3
      % rho = 7850*2;
      % m = rho*l1*l2*l3;
      m = 2*2.41;
    else
      break
    end
    Namen = {'m=1.0*m0', 'm=1.5*m0', 'm=2.0*m0'};
  end
  
  r_B_B_S = zeros(3,1); % Schwerpunkt in der Mitte
  % Trägheitstensor: TM FS S. 44
  I_B_S = diag( [1/12*m*(l2^2+l3^2); 1/12*m*(l1^2+l3^2); 1/12*m*(l1^2+l2^2)] );
  % Ein Flächenmittelpunkt, an dem Kontakt stattfindet
  % r_B_C_ges = [0;0;-l3/2];
  % 8 Eckpunkte, an denen Kontakt stattfindet
  nC = 8;
  r_B_C_ges = allcomb([l1/2;-l1/2], [l2/2;-l2/2], [l3/2;-l3/2])';

  % Anfangswerte
  r_t0 = [0;0;l1/2]; % steht genau auf dem Boden
  rD_t0 = zeros(3,1);
  phi_base_t0 = zeros(3,1);
  phiD_base_t0 = zeros(3,1);

  % Gravitation
  g_W = [0;0;-9.81];

  % Parameter für symbolische Funktionen
  KinPar_mdh = struct('a', [], 'd', [], 'alpha', [], ...
    'q_offset', [], 'b', [], 'beta', [], 'v', []);
  DynPar1 = struct('m', m, 'r_S', r_B_B_S', 'I_S', inertiamatrix2vector(I_B_S));
  NJ = 0;
  

  if i_EV == 1
    Namen = {'diskret', 'konti'};
    if i_Modus == 1
      CONTACTMODEL_SWITCH = 1; % Diskret
    elseif i_Modus == 2
      CONTACTMODEL_SWITCH = 2; % Kontinuierlich
    else
      break;
    end
  elseif i_EV == 2
    % Variiere sigma0
    if i_Modus == 1
      LuGre_params(1) = 500;
    elseif i_Modus == 2
      LuGre_params(1) = 1000;
    elseif i_Modus == 3
      LuGre_params(1) = 2000;
    else
      break
    end
    Namen = {'sigma0=500', 'sigma0=1000', 'sigma0=2000'};
  elseif i_EV == 3
    % Variiere sigma1
    if i_Modus == 1
      LuGre_params(2) = 50;
    elseif i_Modus == 2
      LuGre_params(2) = 100;
    elseif i_Modus == 3
      LuGre_params(2) = 250;
    else
      break
    end
    Namen = {'sigma1=50', 'sigma1=100', 'sigma1=250'};
  elseif i_EV == 4
    % Variiere sigma2
    if i_Modus == 1
      LuGre_params(3) = 0.1;
    elseif i_Modus == 2
      LuGre_params(3) = 1;
    elseif i_Modus == 3
      LuGre_params(3) = 10;
    end
    Namen = {'sigma1=50', 'sigma1=100', 'sigma1=250'};
  elseif i_EV == 5
    % Variiere muC
    if i_Modus == 1
      LuGre_params(4) = 0.4;
    elseif i_Modus == 2
      LuGre_params(4) = 0.8;
    elseif i_Modus == 3
      LuGre_params(4) = 1.2;
    else
      break
    end
    Namen = {'muC=0.4', 'muC=0.8', 'muC=1.2'};
  elseif i_EV == 6
    % Variiere muS
    if i_Modus == 1
      LuGre_params(5) = 0.5;
    elseif i_Modus == 2
      LuGre_params(5) = 0.9;
    elseif i_Modus == 3
      LuGre_params(5) = 1.3;
    else
      break
    end
    Namen = {'muS=0.5', 'muS=0.9', 'muS=1.3'};
  elseif i_EV == 7
    % Variiere vs
    if i_Modus == 1
      LuGre_params(6) = 1;
    elseif i_Modus == 2
      LuGre_params(6) = 10;
    elseif i_Modus == 3
      LuGre_params(6) = 20;
    else
      break
    end
    Namen = {'LG_vs=1', 'LG_vs=10', 'LG_vs=20'};
  elseif i_EV == 8
    % Variiere LG_alpha
    if i_Modus == 1
      LuGre_params(7) = 0.5;
    elseif i_Modus == 2
      LuGre_params(7) = 1;
    elseif i_Modus == 3
      LuGre_params(7) = 1.3;
    else
      break
    end
    Namen = {'LG_alpha=0.5', 'LG_alpha=1', 'LG_alpha=1.3'};
  elseif i_EV == 9
    if CONTACTMODEL_SWITCH == 1
      LuGre_kv_arctan = 500; % beste Näherung bei zeitdiskret
    else
      LuGre_kv_arctan = 2000; % beste Näherung bei zeitkontinuierlich
    end
    % Kombination von Parametern
    % Da die Parameter sich gegenseitig beeinflussen, werden sie gemeinsam
    % verändert
    LuGre_params(2) = NaN; % wird später festgelegt (Dämpfungsmaß)
    if i_Modus == 1
      % Standard
      LuGre_params(1) = 1000;
      LuGre_params(3) = 40;
    elseif i_Modus == 2
      LuGre_params(1) = 2000; % höhere Steifigkeit
      LuGre_params(3) = 60;
    elseif i_Modus == 3
      LuGre_params(1) = 3000;
      LuGre_params(3) = 80;
    else
      break
    end
    Namen = {'s0Faktor1', 's0Faktor2', 's0Faktor3'};
  elseif i_EV == 12
    % Kombination von Parametern
    % Da die Parameter sich gegenseitig beeinflussen, werden sie gemeinsam
    % verändert
    if i_Modus == 1
      % sgn-Funktion für LuGre-sgn
      LuGre_kv_arctan = Inf;
    elseif i_Modus == 2
      % sgn durch arctan angenähert, nur minimal verschliffen
      LuGre_kv_arctan = 10000;
    elseif i_Modus == 3
      LuGre_kv_arctan = 2000;
    elseif i_Modus == 4
      LuGre_kv_arctan = 500;
    elseif i_Modus == 5
      LuGre_kv_arctan = 100;
    elseif i_Modus == 6
      % sgn stark verschliffen
      LuGre_kv_arctan = 10;
    else
      break
    end
    Namen = {'sgn', 'arctan10000', 'arctan2000', 'arctan500', 'arctan100', 'arctan10'};
%     figure();hold on;
%     v_test = linspace(-1,1,5000);
%     plot(v_test, sign(v_test));
%     plot(v_test, 2/pi*atan(   10*v_test));
%     plot(v_test, 2/pi*atan(  100*v_test));
%     plot(v_test, 2/pi*atan(  500*v_test));
%     plot(v_test, 2/pi*atan( 2000*v_test));
%     plot(v_test, 2/pi*atan(10000*v_test));
  elseif i_EV == 14
    if i_Modus ==     1
      HC_params(3) = HC_params0(3) * 0.1;
    elseif i_Modus == 2
      HC_params(3) = HC_params0(3) * 0.4;
    elseif i_Modus == 3
      HC_params(3) = HC_params0(3); % Standardeinstellung
    elseif i_Modus == 4
      HC_params(3) = HC_params0(3) * 2.0;
    elseif i_Modus == 5
      HC_params(3) = HC_params0(3) * 5.0;
    else
      break;
    end
    Namen={'HCalpha=500','HCalpha=2000','HCalpha=5000','HCalpha=10000','HCalpha=20000'};
  elseif i_EV == 15 % Ändere HC-Nachgiebigkeit
    if i_Modus ==     1
      HC_params(2) = HC_params0(2) * 0.2;
    elseif i_Modus == 2
      HC_params(2) = HC_params0(2) * 0.5;
    elseif i_Modus == 3
      HC_params(2) = HC_params0(2) * 1.0; % Standardeinstellung
    elseif i_Modus == 4
      HC_params(2) = HC_params0(2) * 2.0;
    elseif i_Modus == 5
      HC_params(2) = HC_params0(2) * 5.0;
    else
      break;
    end
    Namen={'HC_k=500%','HC_k=200%','HC_k=100%','HC_k=50%','HC_k=20%'};
    % Passe die Dämpfung an die geänderte Steifigkeit an (funktioniert noch
    % nicht richtig, Dämpfung bei höherer Steifigkeit wird geringer
    % HC_params(3) = HC_params0(3) * (HC_params(2)/HC_params0(2))^(0.25);
  elseif i_EV == 16
    if i_Modus == 1
      % externe Kraft so lassen
    elseif i_Modus == 2
      % Vorzeichen umdrehen
      simin_F_ext.signals.values = -simin_F_ext.signals.values;
    else
      break;
    end
    Namen={'f>0', 'f<0'};
  elseif i_EV > 16
    error('Nicht definiert');
  end
  
  
  %% Parameter prüfen
  FC = LuGre_params(4);
  Fs = LuGre_params(5);
  sigma0=LuGre_params(1);
  sigma1=LuGre_params(2);
  sigma2=LuGre_params(3);
  
  % Vorgabe der modalen Dämpfung des Reibmodells
  if ~isnan(ModalDamping)
    % [AstromWit2008], Gl. (8)
    zeta=ModalDamping;
    sigma1_ideal = 2*zeta*sqrt(sigma0*m) - sigma2;
    fprintf('Ändere LuGre-Parameter sigma1 für zeta=%1.1f: %1.1f -> %1.1f\n', zeta, LuGre_params(2), sigma1_ideal);
    LuGre_params(2) = sigma1_ideal;
    if sigma1_ideal < 0
      warning('Negative Mikrodämpfung sigma1 berechnet!');
    end
  end
  
  
  % Prüfe [AstromWit2008], Gl. (8)
  zeta_max = sigma2/(2*sqrt(sigma0*m)) * (FC/(Fs-FC)+1);
  if ~isnan(ModalDamping) && zeta > zeta_max
    warning('Passivität nach [AstromWit2008], Gl. (8) verletzt. zeta=%1.1f>%1.1f', zeta, zeta_max);
    % Erhöhe sigma2 so, dass Grenzwert für zeta nicht mehr überschritten wird
    sigma2_alt = sigma2;
    sigma2 = sigma2_alt * zeta_max/zeta*1.1; % 10% obendrauf 
    fprintf('Ändere LuGre-Parameter sigma2: %1.1f -> %1.1f\n',sigma2_alt, sigma2);
  end
%   return
  % Prüfe [AstromWit2008], Gl. (7)
  sigma2_max = sigma1*(Fs-FC)/FC;
  if sigma2 <= sigma2_max
    warning('Passivität nach [AstromWit2008], Gl. (7) verletzt. sigma2=%1.1f < %1.1f', sigma2, sigma2_max);
    % Erhöhe sigma1 so, dass Maximalwert für sigma2 nicht mehr überschritten wird
    sigma1_alt = sigma1;
    sigma1 = sigma1_alt * (Fs-FC)/FC * 1.1; % 10% obendrauf
    fprintf('Ändere LuGre-Parameter sigma1: %1.1f -> %1.1f\n',sigma1_alt, sigma1);
  end

  %% Simulation starten
  load_system(sl_Modellname)
  configSet = getActiveConfigSet(sl_Modellname);
  set_param(configSet, 'Solver', solver, 'FixedStep',sprintf('%e',Ts));
  if i_EV == 13
    if i_Modus == 1
      Ts = 2e-4;
      % tmp = configSet.getComponent('Solver');
      % tmp.m
      % Mit diesen Einstellungen ist die Anzahl der Schritte schon doppelt
      % so hoch wie beim fixedStep
      set_param(configSet, 'SolverType', 'Variable-step', 'Solver', 'ode45', 'RelTol', '1e-4', 'AbsTol', '1e-4', ...
        'MinStep', '1e-6', 'MaxStep', '1e-4');
    elseif i_Modus == 2
      Ts = 1e-4;
      set_param(configSet, 'SolverType', 'Fixed-step', 'Solver', 'ode4', 'FixedStep',sprintf('%e',Ts));
    elseif i_Modus == 3
      Ts = 2e-4;
      set_param(configSet, 'SolverType', 'Fixed-step', 'Solver', 'ode4', 'FixedStep',sprintf('%e',Ts));
    elseif i_Modus == 4
      Ts = 1e-4;
      set_param(configSet, 'SolverType', 'Fixed-step', 'Solver', 'ode1', 'FixedStep',sprintf('%e',Ts));
    else
      break;
    end
    Namen = {'ode45', 'ode4/1e-4', 'ode4/2e-4', 'ode1/1e-4'};
  end
  
  simOut = sim(sl_Modellname, 'StopTime', sprintf('%d', t_End), ...
    'SimulationMode', 'normal'); % normal
  [sl, sl_logsout] = get_simulink_outputs(simOut, sl_Modellname);
  
  %% Nachbearbeitung
  % Beträge berechnen
  sl.r_W_B(:,4) = (sl.r_W_B(:,1).^2 + sl.r_W_B(:,2).^2 + sl.r_W_B(:,3).^2).^0.5;
  sl.rD_W_B(:,4) = (sl.rD_W_B(:,1).^2 + sl.rD_W_B(:,2).^2 + sl.rD_W_B(:,3).^2).^0.5;
  sl.rDD_W_B(:,4) = (sl.rDD_W_B(:,1).^2 + sl.rDD_W_B(:,2).^2 + sl.rDD_W_B(:,3).^2).^0.5;
  
  % Speichern
  ErgStruct{i_Modus} = sl; %#ok<SAGROW>
  ergdat_pfad{i_Modus} = fullfile(erg_pfad, ...
    sprintf('HuntCrossley_LuGre_1DOF_EV%d_res%d_CM%d_LGM%d.mat', ...
    i_EV, i_Modus, CONTACTMODEL_SWITCH, LuGre_kv_arctan)); %#ok<SAGROW>
  save(ergdat_pfad{i_Modus}, '-v7.3', 'sl', 'sl_logsout'); % v7.3 gegen Fehler bei logsout-Speicherung
  fprintf('EV%d (%s): Modus %d berechnet (%s).\n', i_EV, EVN{i_EV}, i_Modus, Namen{i_Modus});
  
  %% Debug
  if i_EV == 13
    % Auswertung der variablen Schrittweite
    figure();
    plot(diff(sl.t));
    title('Differenz der Zeitschritte');
  end 
end

% continue
%% Ergebnisse
labelStrings = {'x', 'y', 'z', 'abs'};
KraftKompNamen = {'fx', 'fy', 'fz', 'mx', 'my', 'mz'};
nModus = length(ErgStruct);
for i_Modus = 1:nModus
  tmp = load(ergdat_pfad{i_Modus});
  sl = tmp.sl;
  sl_logsout = tmp.sl_logsout;
  % Ip = 1:10:length(sl.t);
  Ip=1:length(sl.t);
  %% Energie (symbolisch berechnet)
  figure(10);
  if i_Modus==1, clf; end
  set(10, 'Name', 'E', 'NumberTitle', 'off');
  E_ges = sl.E_pot_sym + sl.E_kin_sym + sl.E_contact_z;
  subplot(2,2,1);hold on;
  title('Energiebetrachtung (symbolische Berechnung)');
  plot(sl.t(Ip), sl.E_pot_sym(Ip));
  ylabel('pot,sym');grid on;
  subplot(2,2,2);hold on;
  plot(sl.t(Ip), sl.E_kin_sym(Ip));
  ylabel('kin,sym');grid on;
  subplot(2,2,3);hold on;
  plot(sl.t(Ip), sl.E_contact_z(Ip));
  ylabel('contact (z)');grid on;
  subplot(2,2,4);hold on;
  plot(sl.t(Ip), E_ges(Ip));
  ylabel('Gesamt');grid on;
  if i_Modus==nModus,    legend(Namen); linkxaxes; end  
  
  %% Plot: Translatorische Position (der Basis)
  figure(11);
  if i_Modus==1, clf; end
  set(11, 'Name', 'r_B', 'NumberTitle', 'off');
  for i = 1:4
    subplot(2,2,i);hold on;grid on;
    plot(sl.t(Ip), sl.r_W_B(Ip,i));
    ylabel(sprintf('transl. Pos. %s (Basis)', labelStrings{i}));
    if i_Modus==nModus && i == 3
      plot(sl.t([1 end]), l3/2*[1;1], '--');
    end
  end
  if i_Modus==nModus,    legend(Namen); linkxaxes; end  
  
  %% Plot: Translatorische Geschwindigkeit (der Basis)
  figure(12);
  if i_Modus==1, clf; end
  set(12, 'Name', 'v_B', 'NumberTitle', 'off');
  for i = 1:4
    subplot(2,2,i);hold on;grid on;
    plot(sl.t(Ip), sl.rD_W_B(Ip,i));
    ylabel(sprintf('transl. Geschw. %s (Basis)', labelStrings{i}));
  end
  if i_Modus==nModus,    legend(Namen); linkxaxes; end
  
  %% Plot: Translatorische beschleunigung (der Basis)
  figure(13);
  if i_Modus==1, clf; end
  set(13, 'Name', 'a_B', 'NumberTitle', 'off');
  for i = 1:4
    subplot(2,2,i);hold on;grid on;
    plot(sl.t(Ip), sl.rDD_W_B(Ip,i));
    ylabel(sprintf('transl. Beschl. %s (Basis)', labelStrings{i}));
  end
  if i_Modus==nModus,    legend(Namen); linkxaxes; end
  
  
  %% Plot: Externe Kraft (Kontaktmodell)
  figure(21);
  if i_Modus==1, clf; end
  set(21, 'Name', 'F_contact', 'NumberTitle', 'off');
  for i = 1:6
    subplot(3,2,sprc2no(3,2,i-3*(i>3),1+(i>3)) );hold on;grid on;
    plot(sl.t(Ip), sl.F_ext_contact(Ip,i));
    ylabel(sprintf('%s', KraftKompNamen{i}));
  end
  if i_Modus==nModus,    legend(Namen); linkxaxes; end

  %% Plot: Externe Kraft (Schieben)
  figure(22);
  if i_Modus==1, clf; end
  set(22, 'Name', 'F_push', 'NumberTitle', 'off');
  for i = 1:6
    subplot(3,2,sprc2no(3,2,i-3*(i>3),1+(i>3)) );hold on;grid on;
    plot(sl.t(Ip), sl.F_ext_push(Ip,i));
    ylabel(sprintf('%s', KraftKompNamen{i}));
  end
  if i_Modus==nModus,    legend(Namen); linkxaxes; end
  
  %% Plot: Winkelgeschwindigkeit
  figure(2);
  if i_Modus==1, clf; axhdl=NaN(3,1); end
  set(2, 'Name', 'omega', 'NumberTitle', 'off');
  for i = 1:3
    axhdl(i)=subplot(3,1,i);hold on;grid on;
    plot(sl.t(Ip), sl.omega_W_B(Ip,i));
    ylabel(sprintf('Winkelgeschw. %s', labelStrings{i}));
  end
  if i_Modus==nModus
    legend(Namen); 
    linkxaxes; 
    remove_inner_labels(axhdl,1); 
  end
  
  %% Plot: Winkelbeschleunigung
  figure(3);
  if i_Modus==1, clf; end
  set(3, 'Name', 'omegaD', 'NumberTitle', 'off');
  for i = 1:3
    subplot(3,1,i);hold on;grid on;
    plot(sl.t(Ip), sl.omegaD_W_B(Ip,i));
    ylabel(sprintf('Winkelbeschl. %s', labelStrings{i}));
  end
  if i_Modus==nModus,    legend(Namen); linkxaxes; end
  
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
  if i_Modus==nModus,    legend(Namen); linkxaxes; end
  
  %% Plot: RPY-Winkel
  figure(5); 
  if i_Modus==1, clf; end
  set(5, 'Name', 'RPY', 'NumberTitle', 'off');
  for i = 1:3
    subplot(3,1,i);hold on;grid on;
    plot(sl.t(Ip), 180/pi*squeeze(sl.phi_base(Ip, i)));
    ylabel(sprintf('RPY %s [deg]', char(119+i)));
  end
  if i_Modus==nModus,    legend(Namen); linkxaxes; end
  
  %% Plot: Positionsverlauf der Kontaktpunkte
  figure(50); 
  if i_Modus==1, clf; axhdl=NaN(nC,3); end
  set(50, 'Name', 'r_W_C_ges', 'NumberTitle', 'off');
  for i = 1:nC
    r_W_W_Ci_ges = NaN(length(sl.t),3);
    for jt = 1:length(sl.t)
      r_W_W_Ci_ges(jt,:) = sl.r_W_B(jt,1:3)' + sl.R_W_B(:,:,jt)*r_B_C_ges(:,i);
    end
    for j = 1:3
      axhdl(i,j)=subplot(nC,3,sprc2no(nC,3,i,j));hold on;grid on;
      plot(sl.t(Ip), r_W_W_Ci_ges(Ip,j));
      ylabel(sprintf('r_{W,C%d,%s}', i, char(119+j)));
    end
  end
  if i_Modus==nModus
    legend(Namen); 
    remove_inner_labels(axhdl,1); 
    subplot_expand(50, nC, 3, axhdl);
  end
  linkxaxes
  %% Plot: Geschwindigkeitsverlauf der Kontaktpunkte
  figure(53); 
  if i_Modus==1, clf; axhdl=NaN(nC,3); end
  set(53, 'Name', 'rD_W_C_ges', 'NumberTitle', 'off');
  for i = 1:nC
    rD_W_W_Ci_ges = squeeze(sl_logsout.rD_W_W_C_ges(:,i,:))';
    for j = 1:3
      axhdl(i,j)=subplot(nC,3,sprc2no(nC,3,i,j));hold on;grid on;
      plot(sl.t(Ip), rD_W_W_Ci_ges(Ip,j));
      ylabel(sprintf('rD_{W,C%d,%s}', i, char(119+j)));
    end
  end
  if i_Modus==nModus
    legend(Namen); 
    remove_inner_labels(axhdl,1); 
    subplot_expand(53, nC, 3, axhdl);
  end
  linkxaxes
  %% Plot: LuGre-Schlupf der Kontaktpunkte
  figure(51); 
  if i_Modus==1, clf; axhdl=NaN(nC,2); end
  set(51, 'Name', 'LuGre_s_ges', 'NumberTitle', 'off');
  for i = 1:nC
    for j = 1:2
      axhdl(i,j)=subplot(nC,2,sprc2no(nC,2,i,j));hold on;grid on;
      plot(sl.t(Ip), sl_logsout.LuGre_s(Ip,2*i-1+j-1));
      ylabel(sprintf('s_{C%d,%s}', i, char(119+j)));
    end
  end
  if i_Modus==nModus
    legend(Namen); 
    remove_inner_labels(axhdl,1); 
    subplot_expand(51, nC, 2, axhdl);
  end
  
  %% Plot: LuGre-Schlupf-Änderung der Kontaktpunkte
  figure(52); 
  if i_Modus==1, clf; axhdl=NaN(nC,2); end
  set(52, 'Name', 'LuGre_sD_ges', 'NumberTitle', 'off');
  for i = 1:nC
    for j = 1:2
      axhdl(i,j)=subplot(nC,2,sprc2no(nC,2,i,j));hold on;grid on;
      plot(sl.t(Ip), sl_logsout.LuGre_sD(Ip,2*i-1+j-1));
      ylabel(sprintf('sD_{C%d,%s}', i, char(119+j)));
    end
  end
  if i_Modus==nModus
    legend(Namen); 
    remove_inner_labels(axhdl,1); 
    subplot_expand(52, nC, 2, axhdl);
  end
  
  %% Plot: Kräfte an den Kontaktpunkten
  figure(60); 
  if i_Modus==1, clf; end
  set(60, 'Name', 'F_ext_C_ges', 'NumberTitle', 'off');
  for i = 1:nC
    for j = 1:3
      subplot(nC,3,sprc2no(nC,3,i,j));hold on;grid on;
      plot( sl.t(Ip), squeeze(sl_logsout.F_ext_C_ges(j,i,Ip)) );
      ylabel(sprintf('f_{ext,C%d,%s} [N]', i, char(119+j)));
    end
  end
  if i_Modus==nModus,    legend(Namen); linkxaxes; end
  %% Plot: Komponenten der LuGre-Trangentialkraft
  for i = 1:nC
    figure(70+i); 
    if i_Modus==1, clf; end
    set(70+i, 'Name', sprintf('f_ext_C%d_Komp',i), 'NumberTitle', 'off');
    for j = 1:2
      % Kraft aus Term sigma0
      f_sigma0 = LuGre_params(1) * sl_logsout.LuGre_s(:,2*i-1+j-1)  .* ...
        squeeze(sl_logsout.F_ext_C_ges(3,i,:));
      % Kraft aus Term sigma1
      f_sigma1 = LuGre_params(2) * sl_logsout.LuGre_sD(:,2*i-1+j-1) .* ...
        squeeze(sl_logsout.F_ext_C_ges(3,i,:));
      % Kraft aus Term sigma2
      f_sigma2 = LuGre_params(3) * squeeze(sl_logsout.rD_W_W_C_ges(j,i,:)) .* ...
        squeeze(sl_logsout.F_ext_C_ges(3,i,:));
      
      subplot(3,2,sprc2no(3,2,1,j));hold on;grid on;
      plot(sl.t(Ip), f_sigma0(Ip) );
      ylabel(sprintf('f_{sigma0,%s}',char(119+j)));
      subplot(3,2,sprc2no(3,2,2,j));hold on;grid on;
      plot(sl.t(Ip), f_sigma1(Ip) );
      ylabel(sprintf('f_{sigma1,%s}',char(119+j)));
      subplot(3,2,sprc2no(3,2,3,j));hold on;grid on;
      plot(sl.t(Ip), f_sigma2(Ip) );
      ylabel(sprintf('f_{sigma2,%s}',char(119+j)));
    end
  end
  if i_Modus==nModus,    legend(Namen); linkxaxes; end

  %% Plot: Kinematische Konsistenz (Translatorisch)
  figure(80);
  if i_Modus==1, clf; end
  set(80, 'Name', 'KinTranslKonsist', 'NumberTitle', 'off');
    
  v_aint = cumtrapz(sl.t, sl.rDD_W_B(:,1:3)) + repmat(sl.rD_W_B(1,1:3),length(sl.t),1);
  r_vint = cumtrapz(sl.t, sl.rD_W_B(:,1:3)) +  repmat(sl.r_W_B(1,1:3), length(sl.t),1);

  a_vdiff = [zeros(1,3);diff(sl.rD_W_B(:,1:3))./diff(repmat(sl.t,1,3))];
  v_rdiff = [zeros(1,3); diff(sl.r_W_B(:,1:3))./diff(repmat(sl.t,1,3))];


  for j = 1:3 % xyz-Komponenten
    % Position
    subplot(3,3,sprc2no(3,3,j,1));hold on;grid on;
    plot(sl.t(Ip), sl.r_W_B(Ip,j), '-');set(gca,'ColorOrderIndex',get(gca, 'ColorOrderIndex')-1);
    plot(sl.t(Ip), r_vint(Ip,j), ':');
    if j == 1 && i_Modus==1
      title('Pos.');
    end
    if j==3 && i_Modus==1,    legend({'r_{Sim}', 'r_{v,Int}'}); end
    
    % Geschwindigkeit
    subplot(3,3,sprc2no(3,3,j,2));hold on;grid on;
    plot(sl.t(Ip), sl.rD_W_B(Ip,j), '-');set(gca,'ColorOrderIndex',get(gca, 'ColorOrderIndex')-1);
    plot(sl.t(Ip), v_rdiff(Ip,j), '--');set(gca,'ColorOrderIndex',get(gca, 'ColorOrderIndex')-1);
    plot(sl.t(Ip), v_aint(Ip,j), ':');
    if j == 1 && i_Modus==1
      title('Geschw.');
    end
    if j==3 && i_Modus==1,    legend({'v_{Sim}', 'v_{r,Diff}', 'v_{a,Int}'}); end
    
    % Beschleunigung
    subplot(3,3,sprc2no(3,3,j,3));hold on;grid on;
    plot(sl.t(Ip), sl.rDD_W_B(Ip,j), '-');set(gca,'ColorOrderIndex',get(gca, 'ColorOrderIndex')-1);
    plot(sl.t(Ip), a_vdiff(Ip,j), '--');
    if j == 1 && i_Modus==1
      title('Beschl.');
    end
    if j==3 && i_Modus==1,    legend({'a_{Sim}', 'a_{v,Diff}'}); end
  end
  if i_Modus==nModus,    linkxaxes; end
end

dockall
%% Alle Bilder speichern
if usr_BilderSpeichern
  save_figures(erg_pfad, {'fig', 'png'}, ...
    sprintf('HC_LuGre_example_EV%d_%s_CM%d_LGM%d_', i_EV, EVN{i_EV}, ...
    CONTACTMODEL_SWITCH, LuGre_kv_arctan));
end

end
return
%% Debug: Wiedergabe in Gazebo
% Benötigt Humanoids Collision-Handling Repo
xq_W_B = [sl.r_W_B(:,1:3), NaN(length(sl.t),4)];
for i = 1:length(sl.t)
  xq_W_B(i,4:7) = r2quat(sl.R_W_B(:,:,i));
end
qJ_t0_gzb = zeros(31,1)'; %#ok<UNRCH>
base_pose_quat_t0 = [0 0 1000 1 0 0 0]'; % Roboter ist versteckt
obj1_pose_quat_t0 = [0 0 100 1 0 0 0]'; % Kugel ist versteckt
obj2_pose_quat_t0 = xq_W_B(1,:)';

simin_qJ_atlas = struct('time', 0, ...
    'signals', struct('values', zeros(1,30), 'dimensions', 30), ...
    'name', 'qJ');
  
simin_xq_atlas = struct('time', 0, ...
    'signals', struct('values', base_pose_quat_t0', 'dimensions', 7), ...
    'name', 'xq_atlas');
  
simin_xq_sphere = struct('time', 0, ...
    'signals', struct('values', obj1_pose_quat_t0' , 'dimensions', 7), ...
    'name', 'xq_sphere');


simin_xq_box = struct('time', sl.t, ...
    'signals', struct('values', xq_W_B, 'dimensions', 7), ...
    'name', 'xq_box');
  
ReplayStepSize = 5e-4;
replaymodellname = 'atlas5_wbody_sphere_box_replay';  
load_system(replaymodellname)
configSet = getActiveConfigSet(replaymodellname);
set_param(configSet, 'Solver', 'ode4');
set_param(configSet, 'FixedStep', sprintf('%e', ReplayStepSize));
  
sim(replaymodellname, 'StopTime', sprintf('%1.5e', sl.t(end)), ...
  'SimulationMode', 'normal');

%% Debug: Polstellen des Gesamtsystems
% Auswertung der Polstellen des linearisierten Einmassensystems mit
% LuGre-Kontaktmodell (Annahme: ext. Kraft positiv, Geschwindigkeit >=0,
% Reibkraft <=0)
% Linearisierung ist evtl nicht so zulässig (Ergebnisse entsprechen aber
% qualitativ der nichtlinearen Vorwärtsdynamiksimulation)
n = 1e4;
k = linspace(0.01,50,n);
EW_ges = NaN(n,3);
for ik = 1:n
sigma0 =1000; % LuGre_params(1);
sigma1 = 100;% LuGre_params(2);
sigma2 = 0.1*k(ik); % LuGre_params(3);
muS = 0.9; % LuGre_params(5);
g = -g_W(3);
% Stationärer Wert des Schlupfes (Arbeitspunkt)
s0 = sl_logsout.LuGre_s(end,3);
% muS/sigma0
% Arbeitspunkt: Keine Geschwindigkeit
xD0 = 0;

A = [[0 1 0]; [0, -sigma1*g*(1-sigma0/muS*s0)-sigma2*g, -sigma0*g]; ...
  [0, (1-sigma0/muS*s0), -sigma0/muS*xD0]];
EW_ges(ik,:)= eig(A);
end
figure();clf;
plot(real(EW_ges), imag(EW_ges), 'x-');
xlabel('Re');ylabel('Im');