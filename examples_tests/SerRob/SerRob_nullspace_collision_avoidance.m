% Teste die Kollisionsvermeidung in der Nullraumbewegung für serielle
% (Industrie-)Roboter in verschiedenen Fällen
% 
% Akademisches Beispiel: z-Achse zeigt seitlich aus Roboter-Flansch heraus
% und ist die Symmetrieachse. Drehung um diese Achse wird für
% Kollisionsvermeidung genutzt (Aufgabenredundanz).

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

usr_create_anim = false; % zum Aktivieren der Video-Animationen (dauert etwas)
rob_path = fileparts(which('robotics_toolbox_path_init.m'));
resdir = fullfile(rob_path, 'examples_tests', 'results', 'CollAvoidance');
mkdirs(resdir);
%% Initialisierung
SName='S6RRRRRR10V2';
RName='S6RRRRRR10V2_KUKA1';
RS = serroblib_create_robot_class(SName, RName);
RS.update_EE([0;0;-0.05], [pi/2;0;-pi/2]); % z-Achse aus Zeichenebene, x-Achse aus EE heraus
RS.qDDlim = repmat([-100, 100], RS.NJ, 1);
% Debug: Funktionen neu generieren und kompilieren (bei Änderung)
% serroblib_create_template_functions({SName}, false);
% matlabfcn2mex({'S6RRRRRR10V2_invkin_eulangresidual', 'S6RRRRRR10V2_invkin_traj'});
RS.fill_fcn_handles(true); % kompilierte Funktionen nehmen (schneller)
%% Definition der Kollisionskörper
collbodies = struct( ...
        'link', [], ... % nx1 uint8, Nummer des zugehörigen Segments (0=Basis)
        'type', [], ... % nx1 uint8, Art des Ersatzkörpers
        'params', []); % Parameter des jeweiligen Ersatzkörpers
a = [RS.MDH.a;0];
for i = 1:RS.NJ
  % Schräge Verbindung mit Kapseln in Matlab-Klasse berechnen
  collbodies.link = [collbodies.link; uint8([i,i-1])];
  collbodies.type = [collbodies.type; uint8(6)];
  collbodies.params = [collbodies.params; [50e-3,NaN(1,9)]];
  % Alternative für Kollisionskörper:
  % Gewinkelte Verbindung entlang der DH-Parameter (nach Standard-Notation)
  % Kollisionskörper hier berechnen (siehe SerRob/plot). Mit Zylinder unter
  % Angabe von Anfangs- und Endpunkt im Körper-KS
  % Diese Definition funktioniert nicht mit der Kollisionsvermeidung (lässt
  % sich nicht nur mit der Gelenkposition beschreiben, sondern benötigt
  % alle Rotationsmatrizen der Körper des Roboters).
  % Für d-Parameter:
%   if RS.MDH.d(i) ~= 0
%     collbodies.link = [collbodies.link; uint8(i)];
%     collbodies.type = [collbodies.type; uint8(3)]; % Kapsel
%     collbodies.params = [collbodies.params; [[0,0,0], ...
%       [0,0,-RS.MDH.d(i)],70e-3,NaN(1,3)]];
%   end
%   % Für a-Parameter:
%   if a(i+1) ~= 0
%     collbodies.link = [collbodies.link; uint8(i)];
%     collbodies.type = [collbodies.type; uint8(3)]; % Kapsel
%     collbodies.params = [collbodies.params; [[0,0,0], ...
%       [a(i+1),0,0],70e-3,NaN(1,3)]];
%   end
end
% Arbeitsraum-Kollisionsobjekt definieren
collbodies.link = [collbodies.link; [0,0]]; % Der Basis zugerechnet
collbodies.type = [collbodies.type; 15]; % Kugel
collbodies.params = [collbodies.params; [01.0, 0, 1.6, 0.1, NaN(1,6)]]; % Position und Radius

% Liste der Kollisionsprüfungen definieren. Teste alle Robotersegmente
% gegen den Arbeitsraum-Kollisionskörper. Keine Kollisionsprüfung innerhalb
% des Roboters.
collchecks = uint8([(1:6)', 7*ones(6,1)]);
% Eintragen in Roboter-Klasse
RS.collbodies = collbodies;
RS.collchecks = collchecks;

% Roboter mit Objekten zeichnen
q0 = RS.qref+[0;-25;-25;0;-50;0]*pi/180;
s_plot = struct( 'ks', [1:RS.NJ, RS.NJ+2], 'straight', 0, 'mode', 5, 'only_bodies', true);
figure(1);clf;set(1,'Name','Startpose','NumberTitle','off');
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
view(3);
RS.plot(q0, s_plot);
title('Startpose mit Kollisionsmodell des Roboters');

%% Verfahrbewegung in Positions-IK mit verschiedenen Einstellungen
s_basic = struct('maxrelstep', 0.001, 'retry_limit', 0, 'wn', zeros(4,1));
x0 = RS.t2x(RS.fkineEE(q0));
x1 = x0 + [0.2; 0; 0.5; zeros(3,1)]; % bewege den End-Effektor nach oben rechts

% IK mit 3T3R (ohne Kollisions-Betrachtung)
s_3T3R = s_basic;
s_3T3R.I_EE = logical([1 1 1 1 1 1]);
[q_3T3R, Phi, ~, Stats_3T3R] = RS.invkin2(RS.x2tr(x1), q0, s_3T3R);
assert(all(abs(Phi)<1e-8), 'IK mit 3T3R nicht lösbar');

figure(2);clf;set(2,'Name','Zielpose_3T3R','NumberTitle','off');
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
trplot(RS.x2t(x0), 'frame', 'S', 'rgb', 'length', 0.2);
RS.plot( q_3T3R, s_plot );
title('Zielpose des Roboters (3T3R)');

% Verfahrbewegung mit Aufgabenredundanz, ohne Kollisionsbetrachtung
s_3T2R = s_basic;
s_3T2R.I_EE = logical([1 1 1 1 1 0]);
[q_3T2R, Phi, ~, Stats_3T2R] = RS.invkin2(RS.x2tr(x1), q0, s_3T2R);
assert(all(abs(Phi)<1e-8), 'IK mit 3T2R (ohne Nebenbedingungen) nicht lösbar');

figure(3);clf;set(3,'Name','Zielpose_3T2R','NumberTitle','off');
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
trplot(RS.x2t(x0), 'frame', 'S', 'rgb', 'length', 0.2);
RS.plot( q_3T2R, s_plot );
title('Zielpose des Roboters (3T2R)');

% Verfahrbewegung mit Kollisionsvermeidung (aufgabenredundant). Kollisionen
% in Zwischenphasen erlauben.
s_collav = s_basic;
s_collav.I_EE = logical([1 1 1 1 1 0]);
s_collav.wn(4) = 1; % Kollisionsvermeidung aktiv
[q1_cav, Phi_cav, Tcstack1_cav, Stats_CollAvoid] = RS.invkin2(RS.x2tr(x1), q0, s_collav);
assert(all(abs(Phi_cav)<1e-8), 'IK mit Kollisionsvermeidung im Nullraum nicht lösbar');

figure(4);clf;set(4,'Name','Zielpose_KollVermFinal','NumberTitle','off');
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
RS.plot( q1_cav, s_plot );
title('Zielpose des Roboters mit finaler Kollisionsvermeidung');

% Verfahrbewegung mit Kollisionsvermeidung (aufgabenredundant). Kollisionen
% in Zwischenphasen verbieten.
s_collstop = s_basic;
s_collstop.scale_coll = 0.5;
s_collstop.I_EE = logical([1 1 1 1 1 0]);
s_collstop.wn(4) = 1; % Kollisionsvermeidung aktiv
[q1_cst, Phi_cst, Tcstack1_cst, Stats_CollStop] = RS.invkin2(RS.x2tr(x1), q0, s_collstop);
assert(all(abs(Phi_cst)<1e-8), 'IK mit absoluter Kollisionsvermeidung nicht lösbar');

figure(5);clf;set(5,'Name','Zielpose_KollVermStreng','NumberTitle','off');
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
trplot(RS.x2t(x0), 'frame', 'S', 'rgb', 'length', 0.2);
trplot(RS.x2t(x1), 'frame', 'D', 'rgb', 'length', 0.3);
RS.plot( q1_cst, s_plot );
title('Zielpose des Roboters mit strikter Kollisionsvermeidung');

% Kollisionsprüfung für Endposen beider Verfahren
[colldet,colldist] = check_collisionset_simplegeom(RS.collbodies, RS.collchecks, ...
  [Tcstack1_cst(:,4)'; Tcstack1_cav(:,4)'], struct('collsearch', true));
assert(~any(colldet(:)), 'Trotz Kollisionsvermeidungsstrategie gibt es in Endpose eine Kollision');
% Prüfe Kollisionsprüfung in Zwischenschritten. Erneute Berechnung
% notwendig, da in IK die Kollision noch hyperbolisch gewichtet wird. 
% Schwellwert zwischen Warnbereich und Eindringung bei Kollision unbekannt.
JP_all_cst = NaN(1+Stats_CollStop.iter, 3*RS.NL); % Menge aller Gelenkpositionen
for i = 1:1+Stats_CollStop.iter
  [~,~,Tc_stack_i] = RS.fkine(Stats_CollStop.Q(i,:)');
  JP_all_cst(i,:) = Tc_stack_i(:,4)';
end
[colldet_cst, colldist_cst] = check_collisionset_simplegeom(RS.collbodies, ...
  RS.collchecks, JP_all_cst, struct('collsearch', true));
assert(all(~colldet_cst(:)), ['Trotz absoluter Kollisionsvermeidung ', ...
  'gibt es Kollisionen in Zwischenschritten']);

%% Animation der PTP-Bewegungen mit und ohne Kollisionsvermeidung
if usr_create_anim
for k = 1:4
  if k == 1
    Q_t_plot = Stats_3T3R.Q(1:1+Stats_3T3R.iter,:);
    filesuffix = 'no_collavoidance_3T3R';
    plottitle = 'Inverse Kinematics (3T3R) without Collision Avoidance';
  elseif k == 2
    Q_t_plot = Stats_3T2R.Q(1:1+Stats_3T2R.iter,:);
    filesuffix = 'no_collavoidance_3T2R';
    plottitle = 'Inverse Kinematics (3T2R) without Collision Avoidance';
  elseif k == 3
    Q_t_plot = Stats_CollAvoid.Q(1:1+Stats_CollAvoid.iter,:);
    filesuffix = 'with_collavoidance_final';
    plottitle = 'Inverse Kinematics with Final Collision Avoidance';
  elseif k == 4
    Q_t_plot = Stats_CollStop.Q(1:1+Stats_CollStop.iter,:);
    filesuffix = 'with_collavoidance_strict';
    plottitle = 'Inverse Kinematics with Strict Collision Avoidance';
  end
  t = (1:size(Q_t_plot,1))';
  maxduration_animation = 5; % Dauer des mp4-Videos in Sekunden
  t_Vid = (0:1/30*(t(end)/maxduration_animation):t(end))';
  I_anim = knnsearch( t , t_Vid );
  I_anim = [I_anim; repmat(length(t),15,1)]; %#ok<AGROW> % 15 Standbilder (0.5s) mit letztem Wert
  
  anim_filename = fullfile(resdir, sprintf('Nullspace_Collision_Test_PTP_%s', filesuffix));
  s_anim = struct( 'mp4_name', [anim_filename,'.mp4'] );
  s_plot = struct( 'ks', [1, RS.NJ+2], 'straight', 0, 'mode', 5, 'only_bodies', true);
  figure(9);clf;
  set(9, 'name', sprintf('Anim'), ...
    'color','w', 'NumberTitle', 'off', 'units','normalized',...
    'outerposition',[0 0 1 1]); % Vollbild, damit Video größer wird
  trplot(RS.x2t(x1), 'frame', 'D', 'rgb', 'length', 0.2);
  hold on; grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
  view([0, 0]); % 2D-Ansicht
  title(plottitle);
  RS.anim( Q_t_plot(I_anim,:), [], s_anim, s_plot);
end
end
%% Trajektorien-IK mit Aufgabenredundanz
% Definieren Trajektorie. Muss relativ geringe Geschwindigkeit haben, damit
% Nullraumbewegung zur Kollisionsvermeidung noch gut funktioniert.
[X,XD,XDD,T] = traj_trapez2_multipoint([x0';x1'], 0.2, 1e-1, 1e-2, 1e-3, 0);

% Mit 3T3R (keine Aufgabenredundanz)
s_traj_3T3R = struct('I_EE', logical([1 1 1 1 1 1]));
[Q_3T3R, QD_3T3R, QDD_3T3R, PHI, J_3T3R, Stats_3T3R] = RS.invkin2_traj(X,XD,XDD,T,q0,s_traj_3T3R);
assert(all(abs(PHI(:))<1e-8), 'Trajektorie mit 3T3R nicht erfolgreich berechnet');
[coll_3T3R, dist_3T3R] = check_collisionset_simplegeom(RS.collbodies, ...
  RS.collchecks, J_3T3R, struct('collsearch', false));

% Mit 3T2R (Aufgabenredundanz, aber ohne Optimierung)
s_traj_3T2R = struct('I_EE', logical([1 1 1 1 1 0]));
[Q_3T2R, QD_3T2R, QDD_3T2R, PHI, J_3T2R, Stats_3T2R] = RS.invkin2_traj(X,XD,XDD,T,q0,s_traj_3T2R);
assert(all(abs(PHI(:))<1e-8), 'Trajektorie mit 3T2R nicht erfolgreich berechnet');
[coll_3T2R, dist_3T2R] = check_collisionset_simplegeom(RS.collbodies, ...
  RS.collchecks, J_3T2R, struct('collsearch', false));

% Mit 3T2R (Aufgabenredundanz, Kollisionsvermeidung mit P-Regler)
s_traj_KollP = struct('wn', zeros(10,1), 'I_EE', logical([1 1 1 1 1 0]));
s_traj_KollP.wn(9) = 1; % P-Verstärkung Kollisionsvermeidung
s_traj_KollP.wn(3) = 0.7; % Zusätzliche Dämpfung gegen Schwingungen
[Q_CAP, QD_CAP, QDD_CAP, PHI, JP_CAP, Stats_CAP] = RS.invkin2_traj(X,XD,XDD,T,q0,s_traj_KollP);
assert(all(abs(PHI(:))<1e-8), 'Trajektorie mit P-Kollisionsvermeidung nicht erfolgreich berechnet');
[coll_CAP, dist_CAP] = check_collisionset_simplegeom(RS.collbodies, ...
  RS.collchecks, JP_CAP, struct('collsearch', false));

% Mit 3T2R (Aufgabenredundanz, Kollisionsvermeidung mit PD-Regler)
s_traj_KollPD = struct('wn', zeros(10,1), 'I_EE', logical([1 1 1 1 1 0]));
s_traj_KollPD.wn(9) = 1; % P-Verstärkung Kollisionsvermeidung
s_traj_KollPD.wn(10) = 0.1; % D-Verstärkung Kollisionsvermeidung
s_traj_KollPD.wn(3) = 0.7; % Zusätzliche Dämpfung gegen Schwingungen
[Q_CAPD, QD_CAPD, QDD_CAPD, PHI, JP_CAPD, Stats_CAPD] = RS.invkin2_traj(X,XD,XDD,T,q0,s_traj_KollPD);
assert(all(abs(PHI(:))<1e-8), 'Trajektorie mit PD-Kollisionsvermeidung nicht erfolgreich berechnet');
[coll_CAPD, dist_CAPD] = check_collisionset_simplegeom(RS.collbodies, ...
  RS.collchecks, JP_CAPD, struct('collsearch', false));

% Zeitverläufe plotten
Namen = {'3T3R', '3T2R', 'CAP', 'CAPD'};
for kk = 1:4
  if kk == 1
    Q_kk = Q_3T3R; QD_kk = QD_3T3R; QDD_kk = QDD_3T3R; CD_kk = dist_3T3R;
    h_kk = Stats_3T3R.h;
  elseif kk == 2
    Q_kk = Q_3T2R; QD_kk = QD_3T2R; QDD_kk = QDD_3T2R; CD_kk = dist_3T2R;
    h_kk = Stats_3T2R.h;
  elseif kk == 3
    Q_kk = Q_CAP; QD_kk = QD_CAP; QDD_kk = QDD_CAP; CD_kk = dist_CAP;
    h_kk = Stats_CAP.h;
  elseif kk == 4
    Q_kk = Q_CAPD; QD_kk = QD_CAPD; QDD_kk = QDD_CAPD; CD_kk = dist_CAPD;
    h_kk = Stats_CAPD.h;
  end
  change_current_figure(20); if kk == 1, clf; end
  for i = 1:RS.NJ
    % Gelenkposition
    subplot(3,RS.NJ,sprc2no(3, RS.NJ, 1, i));
    hold on;
    plot(T, Q_kk(:,i));
    ylabel(sprintf('q %d', i)); grid on;
    % Gelenkgeschwindigkeit
    subplot(3,RS.NJ,sprc2no(3, RS.NJ, 2, i));
    hold on;
    plot(T, QD_kk(:,i));
    ylabel(sprintf('qD %d', i)); grid on;
    % Gelenkbeschleunigung
    subplot(3,RS.NJ,sprc2no(3, RS.NJ, 3, i));
    hold on;
    plot(T, QDD_kk(:,i));
    ylabel(sprintf('qDD %d', i)); grid on;
  end
  if kk == 4
    sgtitle('Gelenkbewegung');
    legend(Namen);
  end
  change_current_figure(21); if kk == 1, clf; end
  subplot(2,1,1); hold on;
  plot(T, min(CD_kk,[],2));
  if kk == 4
    ylabel('Abstand Kollisionskörper (<0 ist Koll.)'); grid on;
  end
  subplot(2,1,2); hold on;
  plot(T, h_kk(:,1+6));
  if kk == 4
    ylabel('Zielfunktion'); grid on;
  end
  if kk == 4
    sgtitle('Kollisionsprüfung');
    legend(Namen);
    xlim([0,T(end)]);
  end
end


%% Animation der Trajektorien-Bewegungen mit und ohne Kollisionsvermeidung
if usr_create_anim
for k = 1:4
  if k == 1
    Q_t_plot = Q_3T3R;
    filesuffix = 'no_collavoidance_3T3R';
    plottitle = 'Trajectory Inverse Kinematics (3T3R) without Collision Avoidance';
  elseif k == 2
    Q_t_plot = Q_3T2R;
    filesuffix = 'no_collavoidance_3T2R';
    plottitle = 'Trajectory Inverse Kinematics (3T2R) without Collision Avoidance';
  elseif k == 3
    Q_t_plot = Q_CAP;
    filesuffix = 'with_collavoidance_P';
    plottitle = 'Trajectory Inverse Kinematics with P-Controller Collision Avoidance';
  elseif k == 4
    Q_t_plot = Q_CAPD;
    filesuffix = 'with_collavoidance_PD';
    plottitle = 'Trajectory Inverse Kinematics with PD-Controller Collision Avoidance';
  end
  t = (1:size(Q_t_plot,1))';
  maxduration_animation = 5; % Dauer des mp4-Videos in Sekunden
  t_Vid = (0:1/30*(t(end)/maxduration_animation):t(end))';
  I_anim = knnsearch( t , t_Vid );

  anim_filename = fullfile(resdir, sprintf('Nullspace_Collision_Test_Traj_%s', filesuffix));
  s_anim = struct( 'mp4_name', [anim_filename,'.mp4'] );
  s_plot = struct( 'ks', [1, RS.NJ+2], 'straight', 0, 'mode', 5, 'only_bodies', true);
  figure(9);clf;
  set(9, 'name', sprintf('Anim'), ...
    'color','w', 'NumberTitle', 'off', 'units','normalized',...
    'outerposition',[0 0 1 1]); % Vollbild, damit Video größer wird
  plot3(X(:,1), X(:,2), X(:,3), 'b-');
  hold on; grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
  view([0, 0]); % 2D-Ansicht
  title(plottitle);
  RS.anim( Q_t_plot(I_anim,:), [], s_anim, s_plot);
end
end

%% Debuggen der Zielfunktion
return
colldepth_test = linspace(-1, 0.3, 1e4)';
maxcolldepths = 0.3;
h_test = NaN(length(colldepth_test),1)';
for i = 1:length(colldepth_test)
  h_test(i) = invkin_optimcrit_limits2(colldepth_test(i), [-100*maxcolldepths, maxcolldepths], [-80*maxcolldepths, -2*maxcolldepths]);
end
figure(1);clf;
plot(colldepth_test, h_test);
xlim([-0.9, 0.25])
xlabel('Eindringtiefe (positiv ist Kollision, negativ ist Abstand)');
ylabel('Zielkriterium');
