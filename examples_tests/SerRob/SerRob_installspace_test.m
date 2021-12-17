% Teste die Einhaltung von Bauraumgrenzen in der Nullraumbewegung für
% serielle Roboter in verschiedenen Fällen
% 
% Akademisches Beispiel: Planarer Roboter mit 2T0R-Aufgabenredundanz
% bekommt einen vorgegebenen definierten Bauraum.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-09
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
close all

usr_create_anim = true; % zum Aktivieren der Video-Animationen (dauert etwas)
rob_path = fileparts(which('robotics_toolbox_path_init.m'));
resdir = fullfile(rob_path, 'examples_tests', 'results', 'InstallSpace');
mkdirs(resdir);
%% Initialisierung
SName='S3RRR1';
RS = serroblib_create_robot_class(SName);
% Kein Modell aus Datenbank laden, sondern Kinematikparameter vorgeben
pkin = zeros(length(RS.pkin),1);
pkin(strcmp(RS.pkin_names, 'a2')) = 0.4;
pkin(strcmp(RS.pkin_names, 'a3')) = 0.3;
RS.update_mdh(pkin);
RS.update_EE([0.2;0;0]);
RS.qlim = repmat([-pi, pi], RS.NQJ, 1);
RS.qDlim = repmat([-4*pi, 4*pi], RS.NQJ, 1); % 2rpm
RS.qDDlim = repmat([-100, 100], RS.NQJ, 1); % Entspricht 1.5 rpm in 100ms
% Debug: Funktionen neu generieren und kompilieren (bei Änderung)
serroblib_update_template_functions({SName});
% serroblib_create_template_functions({SName}, false);
% err = false;
% err = err | matlabfcn2mex({'S3RRR1_invkin_eulangresidual'});
% err = err | matlabfcn2mex({'S3RRR1_invkin_traj'});
% assert(~err, 'Kompilieren nicht erfolgreich');
RS.fill_fcn_handles(true); % true: kompilierte Funktionen nehmen (schneller)

%% Definition der Kollisionskörper für Bauraumprüfung
% Reine Prüfung von Punkten in den Gelenken reicht aus für Plausibilität
collbodies = struct( ...
        'link', [], ... % nx1 uint8, Nummer des zugehörigen Segments (0=Basis)
        'type', [], ... % nx1 uint8, Art des Ersatzkörpers
        'params', []); % Parameter des jeweiligen Ersatzkörpers
a = [RS.MDH.a;0];
for i = 1:RS.NJ
  % Punkte im Gelenk
  collbodies.link = [collbodies.link; uint8([i,i])];
  collbodies.type = [collbodies.type; uint8(9)]; % Punkt
  collbodies.params = [collbodies.params; NaN(1,10)]; % keine Parameter
end
% Bauraum-Begrenzungsobjekt definieren
b_Quader = 1.0; % Seitenlänge des Quaders
t_Quader = 0.4; % Tiefe des Quaders
h_Quader = 0.4; % Höhe des Quaders (planarer Roboter, nur zur Visualisierung)
p_Quader = [[-b_Quader/10, -t_Quader*1/4, -h_Quader/2], ... % Aufpunkt (unten links);
  [b_Quader,0,0], [0,t_Quader,0], ... % Zwei Kantenvektoren
  h_Quader]; % Länge des letzten Kantenvektors
collbodies.link = [collbodies.link; [0,0]]; % Der Basis zugerechnet
collbodies.type = [collbodies.type; 10]; % Quader im Basis-KS
collbodies.params = [collbodies.params; p_Quader];

% Liste der Baraum-Kollisionsprüfungen definieren. Teste alle
% Robotersegmente (1:3) gegen den Bauraum (4).
collchecks = uint8([(1:3)', 4*ones(3,1)]);
% Eintragen in Roboter-Klasse
RS.collbodies_instspc = collbodies;
RS.collchecks_instspc = collchecks;

% Roboter mit Objekten zeichnen
q0 = pi/180*[30;-60;30];
s_plot = struct( 'ks', [1:RS.NJ, RS.NJ+2], 'straight', 1, 'mode', [1 6]);
figure(1);clf;set(1,'Name','Startpose','NumberTitle','off');
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
RS.plot(q0, s_plot);
view([0, 90]); % 2D-Ansicht von oben
title('Startpose mit Bauraum-Kollisionsmodell des Roboters');


%% Trajektorie definieren
x0 = RS.t2x(RS.fkineEE(q0));
k=1; XL = x0';
% Nach -y fahren, bis über den Rand des Bauraums hinaus
% Gleichzeitig in -x fahren, damit die Strecklage nicht so stark entsteht
k=k+1; XL(k,:) = XL(k-1,:) + [ -0.15,-0.3,0, 0,0,0];
% In Richtung Basis fahren (-x)
k=k+1; XL(k,:) = XL(k-1,:) + [ -0.35,0,0, 0,0,0];

% Trajektorie einzeichnen
plot3(XL(:,1), XL(:,2), XL(:,3), 'c-');

%% Verfahrbewegung in Positions-IK mit verschiedenen Einstellungen
x1 = XL(2,:)'; % am weitesten entfernter Eckpunkt
s_basic = struct('maxrelstep', 0.001, 'retry_limit', 0, 'wn', zeros(RS.idx_ik_length.wnpos,1));
% IK mit 3T3R (ohne Bauraum-Betrachtung)
s_3T3R = s_basic;
s_3T3R.I_EE = logical([1 1 0 0 0 1]);
[q_3T3R, Phi, ~, Stats_3T3R] = RS.invkin2(RS.x2tr(x1), q0, s_3T3R);
figure(2);clf;set(2,'Name','Zielpose_3T3R','NumberTitle','off');
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
trplot(RS.x2t(x0), 'frame', 'S', 'rgb', 'length', 0.2);
RS.plot( q_3T3R, s_plot );
view([0, 90]); % 2D-Ansicht von oben
title('Zielpose des Roboters (3T3R)');
assert(all(abs(Phi)<1e-8), 'IK mit 3T3R nicht lösbar');

% Verfahrbewegung mit Aufgabenredundanz, ohne Bauraumeinhaltung
s_3T2R = s_basic;
s_3T2R.I_EE = logical([1 1 0 0 0 0]);
[q_3T2R, Phi, ~, Stats_3T2R] = RS.invkin2(RS.x2tr(x1), q0, s_3T2R);
figure(3);clf;set(3,'Name','Zielpose_3T2R','NumberTitle','off');
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
trplot(RS.x2t(x0), 'frame', 'S', 'rgb', 'length', 0.2);
RS.plot( q_3T2R, s_plot );
view([0, 90]); % 2D-Ansicht von oben
title('Zielpose des Roboters (3T2R)');
assert(all(abs(Phi)<1e-8), 'IK mit 3T2R (ohne Nebenbedingungen) nicht lösbar');

% Verfahrbewegung mit Bauraumeinhaltung (aufgabenredundant).
s_instspc = s_basic;
s_instspc.I_EE = logical([1 1 0 0 0 0]);
s_instspc.wn(RS.idx_ikpos_wn.instspc_hyp) = 1; % Bauraumeinhaltung aktiv
[q1_InstSpc, Phi_InstSpc, Tcstack1_InstSpc, Stats_InstSpc] = RS.invkin2(RS.x2tr(x1), q0, s_instspc);
figure(4);clf;set(4,'Name','Zielpose_BauraumOpt','NumberTitle','off');
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
RS.plot( q1_InstSpc, s_plot );
view([0, 90]); % 2D-Ansicht von oben
title('Zielpose des Roboters mit Bauraumeinhaltung');
assert(all(abs(Phi_InstSpc)<1e-8), 'IK mit Bauraumeinhaltung im Nullraum nicht lösbar');

%% Animation der PTP-Bewegungen mit und ohne Bauraumeinhaltung
if usr_create_anim
for k = 1:3
  if k == 1
    Q_t_plot = Stats_3T3R.Q(1:1+Stats_3T3R.iter,:);
    filesuffix = 'no_installspace_3T3R';
    plottitle = 'Inverse Kinematics (3T3R) without InstallSpace Optimization';
  elseif k == 2
    Q_t_plot = Stats_3T2R.Q(1:1+Stats_3T2R.iter,:);
    filesuffix = 'no_installspace_3T2R';
    plottitle = 'Inverse Kinematics (3T2R) without InstallSpace Optimization';
  elseif k == 3
    Q_t_plot = Stats_InstSpc.Q(1:1+Stats_InstSpc.iter,:);
    filesuffix = 'with_installspace';
    plottitle = 'Inverse Kinematics with InstallSpace Optimization';
  end
  t = (1:size(Q_t_plot,1))';
  maxduration_animation = 5; % Dauer des mp4-Videos in Sekunden
  t_Vid = (0:1/30*(t(end)/maxduration_animation):t(end))';
  I_anim = knnsearch( t , t_Vid );
  I_anim = [I_anim; repmat(length(t),15,1)]; %#ok<AGROW> % 15 Standbilder (0.5s) mit letztem Wert
  
  anim_filename = fullfile(resdir, sprintf('SerRob_Nullspace_InstallSpace_Test_PTP_%s', filesuffix));
  s_anim = struct( 'mp4_name', [anim_filename,'.mp4'] );
  s_plot = struct( 'ks', [1, RS.NJ+2], 'straight', 1, 'mode', [1 6]);
  figure(9);clf;
  set(9, 'name', sprintf('Anim'), ...
    'color','w', 'NumberTitle', 'off', 'units','normalized',...
    'outerposition',[0 0 1 1]); % Vollbild, damit Video größer wird
  trplot(RS.x2t(x1), 'frame', 'D', 'rgb', 'length', 0.2);
  hold on; grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
  view([0, 90]); % 2D-Ansicht von oben
  title(plottitle);
  RS.anim( Q_t_plot(I_anim,:), [], s_anim, s_plot);
end
end
%% Trajektorien-IK mit Aufgabenredundanz
% Definieren Trajektorie. Muss relativ geringe Geschwindigkeit haben, damit
% Nullraumbewegung zur Bauraumeinhaltung noch gut funktioniert.
[X,XD,XDD,T] = traj_trapez2_multipoint(XL, 0.1, 1e-1, 1e-2, 1e-3, 0);

% Mit 3T3R (keine Aufgabenredundanz)
s_traj_3T3R = struct('I_EE', logical([1 1 0 0 0 1]));
[Q_3T3R, QD_3T3R, QDD_3T3R, PHI, JP_3T3R, Stats_3T3R] = RS.invkin2_traj(X,XD,XDD,T,q0,s_traj_3T3R);
assert(all(abs(PHI(:))<1e-8), 'Trajektorie mit 3T3R nicht erfolgreich berechnet');
[coll_3T3R, dist_3T3R] = check_collisionset_simplegeom(RS.collbodies_instspc, ...
  RS.collchecks_instspc, JP_3T3R, struct('collsearch', false));
[X_3T3R, XD_3T3R, XDD_3T3R] = RS.fkineEE2_traj(Q_3T3R, QD_3T3R, QDD_3T3R);

% Mit 3T2R (Aufgabenredundanz, aber ohne Optimierung)
s_traj_3T2R = struct('I_EE', logical([1 1 0 0 0 0]), 'wn', zeros(RS.idx_ik_length.wntraj,1));
[Q_3T2R, QD_3T2R, QDD_3T2R, PHI, JP_3T2R, Stats_3T2R] = RS.invkin2_traj(X,XD,XDD,T,q0,s_traj_3T2R);
assert(all(abs(PHI(:))<1e-8), 'Trajektorie mit 3T2R nicht erfolgreich berechnet');
[coll_3T2R, dist_3T2R] = check_collisionset_simplegeom(RS.collbodies_instspc, ...
  RS.collchecks_instspc, JP_3T2R, struct('collsearch', false));
[X_3T2R, XD_3T2R, XDD_3T2R] = RS.fkineEE2_traj(Q_3T2R, QD_3T2R, QDD_3T2R);

% Mit 3T2R (Aufgabenredundanz, Bauraumeinhaltung mit P-Regler)
s_traj_InstSpcP = s_traj_3T2R;
s_traj_InstSpcP.wn(RS.idx_iktraj_wnP.instspc_hyp) = 1e-5; % P-Verstärkung Bauraumeinhaltung
s_traj_InstSpcP.wn(RS.idx_iktraj_wnP.qDlim_par) = 0.5; % Zusätzliche Dämpfung gegen Schwingungen
[Q_ISP, QD_ISP, QDD_ISP, PHI, JP_ISP, Stats_ISP] = RS.invkin2_traj(X,XD,XDD,T,q0,s_traj_InstSpcP);
assert(all(abs(PHI(:))<1e-8), 'Trajektorie mit P-Bauraumeinhaltung nicht erfolgreich berechnet');
[coll_ISP, dist_ISP] = check_collisionset_simplegeom(RS.collbodies_instspc, ...
  RS.collchecks_instspc, JP_ISP, struct('collsearch', false));
[X_ISP, XD_ISP, XDD_ISP] = RS.fkineEE2_traj(Q_ISP, QD_ISP, QDD_ISP);

% Mit 3T2R (Aufgabenredundanz, Bauraumeinhaltung mit PD-Regler)
s_traj_InstSpcPD = s_traj_3T2R;
s_traj_InstSpcPD.wn(RS.idx_iktraj_wnP.instspc_hyp) = 1e-5; % P-Verstärkung Bauraumeinhaltung
s_traj_InstSpcPD.wn(RS.idx_iktraj_wnD.instspc_hyp) = 4e-6; % D-Verstärkung Bauraumeinhaltung
s_traj_InstSpcPD.wn(RS.idx_iktraj_wnP.qDlim_par) = 0.5; % Zusätzliche Dämpfung gegen Schwingungen
[Q_ISPD, QD_ISPD, QDD_ISPD, PHI, JP_ISPD, Stats_ISPD] = RS.invkin2_traj(X,XD,XDD,T,q0,s_traj_InstSpcPD);
[coll_ISPD, dist_ISPD] = check_collisionset_simplegeom(RS.collbodies_instspc, ...
  RS.collchecks_instspc, JP_ISPD, struct('collsearch', false));
[X_ISPD, XD_ISPD, XDD_ISPD] = RS.fkineEE2_traj(Q_ISPD, QD_ISPD, QDD_ISPD);
assert(all(abs(PHI(:))<1e-8), 'Trajektorie mit PD-Bauraumeinhaltung nicht erfolgreich berechnet');
assert(all(dist_ISPD(:)<0), 'Trajektorie mit PD-Bauraumeinhaltung hält den Bauraum nicht ein');

% Zeitverläufe plotten
Namen = {'3T3R', '3T2R', 'ISP', 'ISPD'};
for kk = 1:length(Namen)
  if kk == 1
    Q_kk = Q_3T3R; QD_kk = QD_3T3R; QDD_kk = QDD_3T3R; CD_kk = dist_3T3R;
    X_kk = X_3T3R; XD_kk = XD_3T3R; XDD_kk = XDD_3T3R;
    h_kk = Stats_3T3R.h; condJ = Stats_3T3R.condJ;
  elseif kk == 2
    Q_kk = Q_3T2R; QD_kk = QD_3T2R; QDD_kk = QDD_3T2R; CD_kk = dist_3T2R;
    X_kk = X_3T2R; XD_kk = XD_3T2R; XDD_kk = XDD_3T2R;
    h_kk = Stats_3T2R.h; condJ = Stats_3T2R.condJ;
  elseif kk == 3
    Q_kk = Q_ISP; QD_kk = QD_ISP; QDD_kk = QDD_ISP; CD_kk = dist_ISP;
    X_kk = X_ISP; XD_kk = XD_ISP; XDD_kk = XDD_ISP;
    h_kk = Stats_ISP.h; condJ = Stats_ISP.condJ;
  elseif kk == 4
    Q_kk = Q_ISPD; QD_kk = QD_ISPD; QDD_kk = QDD_ISPD; CD_kk = dist_ISPD;
    X_kk = X_ISPD; XD_kk = XD_ISPD; XDD_kk = XDD_ISPD;
    h_kk = Stats_ISPD.h; condJ = Stats_ISPD.condJ;
  end
  change_current_figure(20); if kk == 1, clf; end
  % EE-Winkel
  subplot(3,1+RS.NJ,sprc2no(3, 1+RS.NJ, 1, 1)); hold on;
  plot(T, X_kk(:,6)); ylabel('phi z'); grid on;
  % EE-Geschw.
  subplot(3,1+RS.NJ,sprc2no(3, 1+RS.NJ, 2, 1)); hold on;
  plot(T, XD_kk(:,6)); ylabel('phiD z'); grid on;
  % EE-Beschl.
  subplot(3,1+RS.NJ,sprc2no(3, 1+RS.NJ, 3, 1)); hold on;
  plot(T, XDD_kk(:,6)); ylabel('phiDD z'); grid on;
  for i = 1:RS.NJ
    % Gelenkposition
    subplot(3,1+RS.NJ,sprc2no(3, 1+RS.NJ, 1, 1+i)); hold on;
    plot(T, Q_kk(:,i));
    ylabel(sprintf('q %d', i)); grid on;
    % Gelenkgeschwindigkeit
    subplot(3,1+RS.NJ,sprc2no(3, 1+RS.NJ, 2, 1+i)); hold on;
    plot(T, QD_kk(:,i));
    ylabel(sprintf('qD %d', i)); grid on;
    % Gelenkbeschleunigung
    subplot(3,1+RS.NJ,sprc2no(3, 1+RS.NJ, 3, 1+i)); hold on;
    plot(T, QDD_kk(:,i));
    ylabel(sprintf('qDD %d', i)); grid on;
  end
  if kk == length(Namen)
    sgtitle('Roboterbewegung');
    legend(Namen);
    linkxaxes
  end
  saveas(20, fullfile(resdir, 'SerRob_InstallSpace_Test_Traj_Joint.fig'));

  change_current_figure(21); if kk == 1, clf; end
  subplot(2,2,1); hold on;
  plot(T, max(CD_kk,[],2)); % positive Werte sind schlecht (im Gegensatz zur Kollision)
  if kk == length(Namen)
    ylabel('Abstand Bauraum-Prüfkörper (>0 ist außerhalb)'); grid on;
  end
  subplot(2,2,2); hold on;
  plot(T, h_kk(:,1+RS.idx_iktraj_hn.instspc_hyp));
  if kk == length(Namen)
    ylabel('Zielfunktion'); grid on;
  end
  subplot(2,2,3); hold on;
  plot(T, condJ(:,1));
  if kk == length(Namen)
    ylabel('Konditionszahl'); grid on;
  end
  if kk == length(Namen)
    sgtitle('Bauraumprüfung');
    legend(Namen);
    xlim([0,T(end)]);
    linkxaxes
  end
  saveas(21, fullfile(resdir, 'SerRob_InstallSpace_Test_PTP_Criterion.fig'));
end

%% Animation der Trajektorien-Bewegungen mit und ohne Bauraumeinhaltung
if usr_create_anim
for k = 1:length(Namen)
  if k == 1
    Q_t_plot = Q_3T3R;
    filesuffix = 'no_installspace_3T3R';
    plottitle = 'Trajectory Inverse Kinematics (3T3R) without InstallSpace Optimization';
  elseif k == 2
    Q_t_plot = Q_3T2R;
    filesuffix = 'no_installspace_3T2R';
    plottitle = 'Trajectory Inverse Kinematics (3T2R) without InstallSpace Optimization';
  elseif k == 3
    Q_t_plot = Q_ISP;
    filesuffix = 'with_installspace_P';
    plottitle = 'Trajectory Inverse Kinematics with P-Controller InstallSpace Optimization';
  elseif k == 4
    Q_t_plot = Q_ISPD;
    filesuffix = 'with_installspace_PD';
    plottitle = 'Trajectory Inverse Kinematics with PD-Controller InstallSpace Optimization';
  end
  t = (1:size(Q_t_plot,1))';
  maxduration_animation = 5; % Dauer des mp4-Videos in Sekunden
  t_Vid = (0:1/30*(t(end)/maxduration_animation):t(end))';
  I_anim = knnsearch( t , t_Vid );

  anim_filename = fullfile(resdir, sprintf('Nullspace_InstallSpace_Test_Traj_%s', filesuffix));
  s_anim = struct( 'mp4_name', [anim_filename,'.mp4'] );
  s_plot = struct( 'ks', [1, RS.NJ+2], 'straight', 1, 'mode', [1 6]);
  figure(9);clf;
  set(9, 'name', sprintf('Anim'), ...
    'color','w', 'NumberTitle', 'off', 'units','normalized',...
    'outerposition',[0 0 1 1]); % Vollbild, damit Video größer wird
  plot3(X(:,1), X(:,2), X(:,3), 'b-');
  hold on; grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
  view([0, 90]); % 2D-Ansicht von oben
  title(plottitle);
  RS.anim( Q_t_plot(I_anim,:), [], s_anim, s_plot);
end
end
%% Probiere Nullraumbewegung auf der Stelle
% Starte in einer Pose im Bauraum, aber nah genug an Grenze dran. Dann
% Null-Trajektorie vorgeben und Ausschwingbewegung simulieren.
% Dadurch gutes Tuning der Einstellparameter möglich.
x1 = XL(2,:)'+[0;0.18;0;0;0;0]; % Abgeleitet aus Bild vom Anfang
[q1, Phi] = RS.invkin2(RS.x2tr(x1), q_3T2R);
[~,~,Tc] = RS.fkine(q1);
[coll_1, dist_1] = check_collisionset_simplegeom(RS.collbodies_instspc, ...
  RS.collchecks_instspc, Tc(:,4)', struct('collsearch', false));
assert(all(dist_1<0), 'In Startpose bereits Arbeitsraum verletzt');

% Dummy-Trajektorie
T_end = 5;
T_dummy = (0:1e-3:T_end)';
X_dummy = repmat(x1', length(T_dummy), 1);
XD_dummy = zeros(size(X_dummy,1),6); XDD_dummy = XD_dummy;

% Neue Einstellungen testen. Nehme sehr kleine Zahlenwerte. Nur so wird
% verhindert, dass die Knick-Stelle beim Minimum zu weit überschritten wird
s_traj_sm = struct('wn', zeros(RS.idx_ik_length.wntraj,1), 'I_EE', logical([1 1 0 0 0 0]));
s_traj_sm.wn(RS.idx_iktraj_wnP.instspc_hyp) = 1e-5; % P-Verstärkung Bauraumeinhaltung
s_traj_sm.wn(RS.idx_iktraj_wnD.instspc_hyp) = 0.2*s_traj_sm.wn(RS.idx_iktraj_wnP.instspc_hyp); % D-Verstärkung Bauraumeinhaltung
s_traj_sm.wn(RS.idx_iktraj_wnP.qDlim_par) = 0.4; % Zusätzliche Dämpfung gegen Schwingungen
RS.fill_fcn_handles(true); % Debuggen der Funktion S3RRR1_invkin_traj
[Q_sm, QD_sm, QDD_sm, PHI, JP_sm, Stats_sm] = RS.invkin2_traj(X_dummy, ...
  XD_dummy,XDD_dummy,T_dummy,q1,s_traj_sm);
[coll_sm, dist_sm] = check_collisionset_simplegeom(RS.collbodies_instspc, ...
  RS.collchecks_instspc, JP_sm, struct('collsearch', false));
[X_sm, XD_sm, XDD_sm] = RS.fkineEE2_traj(Q_sm, QD_sm, QDD_sm);

% Ergebnis zeichnen
figure(600);clf;set(600,'Name','Zeitverlauf_OhneTraj','NumberTitle','off');
subplot(3,3,1);
plot(T_dummy, Q_sm);
ylabel('q'); grid on;
subplot(3,3,2);
plot(T_dummy, QD_sm);
ylabel('qD'); grid on;
subplot(3,3,3);
plot(T_dummy, QDD_sm);
ylabel('qDD'); grid on;
subplot(3,3,4); hold on;
plot(T_dummy, X_sm(:,6));
plot(T_dummy(1), X_sm(1,6), 'gs');
plot(T_dummy(end), X_sm(end,6), 'rx');
ylabel('phi z'); grid on;
subplot(3,3,5);
plot(T_dummy, XD_sm(:,6));
ylabel('phiD z'); grid on;
subplot(3,3,6);
plot(T_dummy, XDD_sm(:,6));
ylabel('phiDD z'); grid on;
subplot(3,3,7);
% plot(T_dummy, Stats_sm.h(:,1+5));
% ylabel('h5 = cond'); grid on;
plot(T_dummy, max(dist_sm,[],2));
ylabel('Bauraumverletzung (Distanz)'); grid on;
subplot(3,3,8); hold on;
plot(T_dummy, Stats_sm.h(:,1+RS.idx_iktraj_hn.instspc_hyp));
plot(T_dummy(1), Stats_sm.h(1,1+RS.idx_iktraj_hn.instspc_hyp), 'gs');
plot(T_dummy(end), Stats_sm.h(end,1+RS.idx_iktraj_hn.instspc_hyp), 'rx');
ylabel('h7 = installspace'); grid on;
linkxaxes
subplot(3,3,9); hold on;
plot(X_sm(:,6), Stats_sm.h(:,1+RS.idx_iktraj_hn.instspc_hyp));
plot(X_sm(1,6), Stats_sm.h(1,1+RS.idx_iktraj_hn.instspc_hyp), 'gs');
plot(X_sm(end,6), Stats_sm.h(end,1+RS.idx_iktraj_hn.instspc_hyp), 'rx');
xlabel('phi z');
ylabel('h7 = installspace'); grid on;

figure(8);clf;set(8,'Name','Zielpose_OhneTraj','NumberTitle','off');
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
RS.plot( Q_sm(end,:)', s_plot );
view([0, 90]); % 2D-Ansicht von oben
title('Zielpose des Roboters (stationäre Nullraumbewegung)');

assert(all(abs(PHI(:))<1e-8), 'Nullraumbewegung auf der Stelle mit PD-Bauraumeinhaltung nicht erfolgreich berechnet');
assert(all(dist_sm(:)<0), 'Nullraumbewegung auf der Stelle mit PD-Bauraumeinhaltung hält den Bauraum nicht ein');

if usr_create_anim
t_Vid = (0:1/30*(T_dummy(end)/maxduration_animation):T_dummy(end))';
I_anim = knnsearch( T_dummy , t_Vid );
anim_filename = fullfile(resdir, sprintf('Nullspace_InstallSpace_Test_SelfMotion'));
s_anim = struct( 'mp4_name', [anim_filename,'.mp4'] );
s_plot = struct( 'ks', [1, RS.NJ+2], 'straight', 1, 'mode', [1 6]);
figure(9);clf;
set(9, 'name', sprintf('Anim'), ...
  'color','w', 'NumberTitle', 'off', 'units','normalized',...
  'outerposition',[0 0 1 1]); % Vollbild, damit Video größer wird
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
view([0, 90]); % 2D-Ansicht von oben
title('Self Motion with Installation Space Optimization');
RS.anim( Q_sm(I_anim,:), [], s_anim, s_plot);
end
%% Debuggen der Zielfunktion
return
isexcdist_test = linspace(-1, 1, 1e4)';
installspace_thresh = 0.1;
h_test = NaN(length(isexcdist_test),1)';
for i = 1:length(isexcdist_test)
  h_test(i) = invkin_optimcrit_limits2(isexcdist_test(i), ...
    [-100.0, installspace_thresh], [-90, -installspace_thresh]);
end
figure(1);clf;
plot(isexcdist_test, h_test);
xlim([-0.2, 0])
xlabel('Bauraumeinhaltung; negativ ist drin, positiv ist draußen)');
ylabel('Zielkriterium');
