% Teste die Kollisionsvermeidung in der Nullraumbewegung für PKM
% 


% Moritz Schappler, moritz.schappler@imes.uni-hannoveRP.de, 2021-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

usr_create_anim = false; % zum Aktivieren der Video-Animationen (dauert etwas)
usr_test_class = false;
rob_path = fileparts(which('robotics_toolbox_path_init.m'));
resdir = fullfile(rob_path, 'examples_tests', 'results', 'CollAvoidance');
mkdirs(resdir);

%% Initialisierung
if isempty(which('parroblib_path_init.m'))
  warning('Repo mit parallelen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbaRP.');
  return
end
RP = parroblib_create_robot_class('P6RRRRRR10V3G6P4A1', [0.2; 0.1], [0.2; 0.1]);
RP.fill_fcn_handles(true, true); % keine mex-Funktionen, einfache Rechnung

pkin = NaN(length(RP.Leg(1).pkin_names),1);
pkin(strcmp(RP.Leg(1).pkin_names, 'd1')) = 0;
pkin(strcmp(RP.Leg(1).pkin_names, 'a2')) = 0.3;
pkin(strcmp(RP.Leg(1).pkin_names, 'd2')) = 0;
pkin(strcmp(RP.Leg(1).pkin_names, 'alpha2')) = 0;
pkin(strcmp(RP.Leg(1).pkin_names, 'a3')) = 0; % für Kardan-Gelenk
pkin(strcmp(RP.Leg(1).pkin_names, 'd3')) = 0;
pkin(strcmp(RP.Leg(1).pkin_names, 'alpha3')) = pi/2;
pkin(strcmp(RP.Leg(1).pkin_names, 'a4')) = 0.4;
pkin(strcmp(RP.Leg(1).pkin_names, 'alpha4')) = 0;
pkin(strcmp(RP.Leg(1).pkin_names, 'd4')) = 0;
pkin(strcmp(RP.Leg(1).pkin_names, 'd6')) = 0;
pkin(strcmp(RP.Leg(1).pkin_names, 'a6')) = 0;
for i = 1:RP.NLEG
  RP.Leg(i).update_mdh(pkin);
end

% Markiere Kardan- und Kugelgelenk (zum Plotten)
for i = 1:RP.NLEG
  RP.Leg(i).DesPar.joint_type(2:3) = 2; % Kardan
  RP.Leg(i).DesPar.joint_type(4:6) = 3; % Kugel
end

% Debug
% serroblib_create_template_functions({RP.Leg(1).mdlname},false,false);
% matlabfcn2mex({[RP.Leg(1).mdlname,'_invkin_eulangresidual']});
% parroblib_create_template_functions({RP.mdlname},false,false);
% matlabfcn2mex({[RP.mdlname(1:12),'_invkin3']});
% matlabfcn2mex({[RP.mdlname(1:12),'_invkin']});
%% Grenzen für die Gelenkpositionen setzen
% Dadurch wird die Schrittweite bei der inversen Kinematik begrenzt (auf 5%
% der Spannbreite der Gelenkgrenzen) und die Konfiguration klappt nicht um.
for i = 1:RP.NLEG
  % Begrenze die Winkel der Kugel- und Kardangelenke auf +/- 360°
  RP.Leg(i).qlim = repmat([-2*pi, 2*pi], RP.Leg(i).NQJ, 1);
end
qlim_pkm = cat(1, RP.Leg.qlim);

%% Startpose bestimmen
% Mittelstellung im Arbeitsraum
X = [ [0.15;0.05;0.3]; [10;-10;5]*pi/180 ];
q0 = qlim_pkm(:,1)+rand(36,1).*(qlim_pkm(:,2)-qlim_pkm(:,1));
q0(1:6:end) = 60*pi/180; % Erstes Gelenk sollte nach außen zeigen
q0(2:6:end) = -120*pi/180; % damit Außenstellung gelingt
q0(3:6:end) = 0;
q0(4:6:end) = 0;
q0(5:6:end) = 0;
q0(6:6:end) = 0;
[q, Phis, Tc_stack, Stats] = RP.invkin_ser(X, q0);
JPE(i,:) = Tc_stack(:,4); % Vierte Spalte ist Koordinatenursprung der Körper-KS
% Zwangsbedingungen in Startpose testen
Phi1=RP.constr1(q, X);
Phit1=RP.constr1_trans(q, X);
Phir1=RP.constr1_rot(q, X);


%% Definition der Kollisionskörper
collbodies_empty = struct( ...
        'link', [], ... % nx1 uint8, Nummer des zugehörigen Segments (0=Basis)
        'type', [], ... % nx1 uint8, Art des Ersatzkörpers
        'params', []); % Parameter des jeweiligen Ersatzkörpers
% Kollisionskörper der Beinketten eintragen
for j = 1:RP.NLEG
  collbodies_j = collbodies_empty;
  a = [RP.Leg(j).MDH.a;0];
  for i = 1:RP.Leg(j).NJ
    % Prüfe, ob es überhaupt eine Verbindung gibt
    if all(RP.Leg(j).MDH.d(i)==0 & RP.Leg(j).MDH.a(i)==0)
      continue
    end
    % Schräge Verbindung mit Kapseln in Matlab-Klasse berechnen
    collbodies_j.link = [collbodies_j.link; uint8([i,i-1])];
    collbodies_j.type = [collbodies_j.type; uint8(6)];
    collbodies_j.params = [collbodies_j.params; [10e-3,NaN(1,9)]];
  end
  RP.Leg(j).collbodies = collbodies_j;
end
% Kollisionskörper der Plattform eintragen
% Indizes der jeweiligen vorherigen benachbarten Beinkette
I1 = (1:RP.NLEG)'; I2 = [RP.NLEG, 1:RP.NLEG-1]';
% Variable für Kollisionsobjekte vorbereiten:
collbodies = collbodies_empty;
% Kollisionsobjekte für das Gestell
% Sternförmige Basis mit Kollisionskörpern: Nummer für PKM-Basis=0; Setze
% den Vorgänger auf die jeweiligen Basiskörper der einzelnen Beinketten.
% Kapseln verbinden die Koppelgelenke.
collbodies.link = [collbodies.link; ...
  uint8([zeros(RP.NLEG,1), RP.I1L_LEG-(I1-1)])];
collbodies.type = [collbodies.type; repmat(uint8(6),RP.NLEG,1)];
collbodies.params = [collbodies.params; ...
  [repmat(10e-3, RP.NLEG, 1), NaN(RP.NLEG, 9)]];
% Ringförmige Basis; verbindet die Basis der Beinketten mit der jeweils
% vorherigen
collbodies.link = [collbodies.link; ...
  uint8([RP.I1L_LEG(I1)-(I1-1), RP.I1L_LEG(I2)-(I2-1)])];
collbodies.type = [collbodies.type; repmat(uint8(6),RP.NLEG,1)];
collbodies.params = [collbodies.params; ...
  [repmat(10e-3, RP.NLEG, 1), NaN(RP.NLEG, 9)]];
I_cb_base = 1:size(collbodies.type,1); % Indizes der Basis-Koll.-körper
% Kollisionsobjekte für die Plattform. Kapseln für jeden virtuellen
% Körper der Plattform-Koppelgelenke (auf Plattform-Seite). Kapsel als
% Verbindung zum jeweils vorherigen Koppelgelenk. Erzeugt Ring an der
% Plattform
collbodies.link = [collbodies.link; ...
  uint8([RP.I2L_LEG(I1)-(I1-1)-1, RP.I2L_LEG(I2)-(I2-1)-1])];
collbodies.type = [collbodies.type; repmat(uint8(6),RP.NLEG,1)];
collbodies.params = [collbodies.params; ...
  [repmat(10e-3, RP.NLEG, 1), NaN(RP.NLEG, 9)]];
I_cb_platform = I_cb_base(end)+1:size(collbodies.type,1); % Ind. der Platf.-Koll.-körper
% Arbeitsraum-Kollisionsobjekt definieren
collbodies.link = [collbodies.link; [0,0]]; % Der Basis zugerechnet
collbodies.type = [collbodies.type; 15]; % Kugel
collbodies.params = [collbodies.params; [-0.40, 0.25, 0.10, 0.1, NaN(1,6)]]; % Position und Radius
I_cb_obj = I_cb_platform(end)+1:size(collbodies.type,1); % Index des Arb.-Raum-Objekts
% Eintragen in Roboter-Klasse
RP.collbodies_nonleg = collbodies;
% Aktualisiere die Gesamt-Variable
RP.update_collbodies();
I_cb_legs = I_cb_obj(end)+1:size(RP.collbodies.type,1);
assert(all(RP.collbodies.link(:) < (1+7*6)), 'Segment-Nummer der PKM-Kollisionskörper stimmt nicht')

% Liste der Kollisionsprüfungen definieren.
% Teste alle Robotersegmente gegen den Arbeitsraum-Kollisionskörpers.
collchecks = uint8(zeros(0,2));
for i = I_cb_obj
  for j = I_cb_legs
    collchecks = [collchecks; [j, i]]; %#ok<AGROW>
  end
end
% Teste alle Beinketten gegeneinander (nur jeweils das zweite Segment.
% Dieses neigt zu Kollisionen, da die Ketten zur Plattform gehen.
% RP.collbodies.link(I_cb_legs,:)
for i = I_cb_legs
  for j = I_cb_legs
    if i >= j, continue; end % nicht sich selbst und nicht doppelt testen
    if abs(int16(RP.collbodies.link(i,1)) - int16(RP.collbodies.link(j,1))) <= 2
      continue % die Kollisionskörper liegen auf dem selben Bein. Für diesen Roboter nicht sinnvoll.
    end
    collchecks = [collchecks; [j, i]]; %#ok<AGROW>
  end
end
% Eintragen in Roboter-Klasse
RP.collchecks = collchecks;

% Roboter mit Objekten zeichnen
s_plot = struct( 'ks_legs', [], 'straight', 0, 'mode', 5, 'only_bodies', true);
figure(1);clf;set(1,'Name','Startpose','NumberTitle','off');
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
view(3);
RP.plot(q, X, s_plot);
title('Startpose mit Kollisionsmodell des Roboters');

%% Verfahrbewegung in Positions-IK mit verschiedenen Einstellungen
s_basic = struct('maxrelstep', 0.001, 'retry_limit', 0, 'wn', zeros(4,1));
s_basic.wn(4) = 1; % Optimiere PKM-Konditionszahl
q0 = q;
x0 = X;
x1 = x0 + [-0.4; 0; 0; zeros(3,1)]; % bewege den End-Effektor nach unten (in die Kollision)

% IK mit 3T3R (ohne Kollisions-Betrachtung)
t1 = tic();
RP.update_EE_FG(logical([1 1 1 1 1 1]), logical([1 1 1 1 1 1]));
s_3T3R = s_basic;
[q_3T3R, Phi, ~, Stats_3T3R] = RP.invkin2(x1, q0, s_3T3R);
assert(all(abs(Phi)<1e-8), 'IK mit 3T3R nicht lösbar');
fprintf(['Positions-IK für 3T3R berechnet. ', ...
  'Dauer: %1.1fs. Schritte: %d (für alle Beinketten)\n'], toc(t1), sum(Stats_3T3R.iter));
% Statistik nachverarbeiten: Letzten Wert halten statt NaN für jede
% Beinkette
for i = 1:RP.NLEG
  Ii = RP.I1J_LEG(i):RP.I2J_LEG(i);
  Stats_3T3R.Q(Stats_3T3R.iter(i)+2:end,Ii) = repmat(...
    Stats_3T3R.Q(Stats_3T3R.iter(i)+1,Ii), size(Stats_3T3R.Q,1)-Stats_3T3R.iter(i)-1,1);
end
figure(2);clf;set(2,'Name','Zielpose_3T3R','NumberTitle','off');
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
trplot(RP.x2t(x0), 'frame', 'S', 'rgb', 'length', 0.2);
RP.plot( q_3T3R, x1, s_plot );
title('Zielpose des Roboters (3T3R)');

% Verfahrbewegung mit Aufgabenredundanz, ohne Kollisionsbetrachtung
t1 = tic();
s_3T2R = s_basic;
RP.update_EE_FG(logical([1 1 1 1 1 1]), logical([1 1 1 1 1 0]));
[q_3T2R, Phi, ~, Stats_3T2R] = RP.invkin4(x1, q0, s_3T2R);
assert(all(abs(Phi)<1e-8), 'IK mit 3T2R (ohne Nebenbedingungen) nicht lösbar');
if usr_test_class % Prüfe, ob das Ergebnis mit Klassen-Implementierung gleich ist
  [q_3T2R2, Phi2, ~, Stats_3T2R2] = RP.invkin3(x1, q0, s_3T2R);
  delta_q_ct = normalizeAngle(q_3T2R2-q_3T2R, 0);
  assert(all(abs(delta_q_ct) < 1e-6), 'Klassen-Implementierung ungleich (3T2R)');
end
fprintf(['Positions-IK für 3T2R berechnet. ', ...
  'Dauer: %1.1fs. Schritte: %d\n'], toc(t1), Stats_3T2R.iter);

figure(3);clf;set(3,'Name','Zielpose_3T2R','NumberTitle','off');
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
trplot(RP.x2t(x0), 'frame', 'S', 'rgb', 'length', 0.2);
RP.plot( q_3T2R, x1, s_plot );
title('Zielpose des Roboters (3T2R)');

% Verfahrbewegung mit Kollisionsvermeidung (aufgabenredundant). Kollisionen
% in Zwischenphasen erlauben.
t1 = tic();
s_collav = s_basic;
RP.update_EE_FG(logical([1 1 1 1 1 1]), logical([1 1 1 1 1 0]));
s_collav.wn(5) = 1; % Kollisionsvermeidung aktiv
[q1_cav, Phi_cav, Tcstack1_cav, Stats_CollAvoid] = RP.invkin4(x1, q0, s_collav);
assert(all(abs(Phi_cav)<1e-8), 'IK mit Kollisionsvermeidung im Nullraum nicht lösbar');
if usr_test_class % Prüfe, ob das Ergebnis mit Klassen-Implementierung gleich ist
  [q1_cav2, Phi_cav2, Tcstack1_cav2, Stats_CollAvoid2] = RP.invkin3(x1, q0, s_collav);
  delta_q_ct = normalizeAngle(q1_cav2-q1_cav, 0);
  assert(all(abs(delta_q_ct) < 1e-6), 'Klassen-Implementierung ungleich (finale Kollisionsvermeidung)');
end
fprintf(['Positions-IK für finale Kollisionsvermeidung berechnet. ', ...
  'Dauer: %1.1fs. Schritte: %d\n'], toc(t1), Stats_CollAvoid.iter);

figure(4);clf;set(4,'Name','Zielpose_KollVermFinal','NumberTitle','off');
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
RP.plot( q1_cav, x1, s_plot );
title('Zielpose des Roboters mit finaler Kollisionsvermeidung');

% Verfahrbewegung mit Kollisionsvermeidung (aufgabenredundant). Kollisionen
% in Zwischenphasen verbieten.
t1 = tic();
s_collstop = s_basic;
s_collstop.scale_coll = 0.5;
RP.update_EE_FG(logical([1 1 1 1 1 1]), logical([1 1 1 1 1 0]));
s_collstop.wn(5) = 1; % Kollisionsvermeidung aktiv
[q1_cst, Phi_cst, Tcstack1_cst, Stats_CollStop] = RP.invkin4(x1, q0, s_collstop);
assert(all(abs(Phi_cst)<1e-8), 'IK mit absoluter Kollisionsvermeidung nicht lösbar');
if usr_test_class % Prüfe, ob das Ergebnis mit Klassen-Implementierung gleich ist
  [q1_cst2, Phi_cst2, Tcstack1_cst2, Stats_CollStop2] = RP.invkin3(x1, q0, s_collstop);
  delta_q_ct = normalizeAngle(q1_cst2-q1_cst, 0);
  assert(all(abs(delta_q_ct) < 1e-6), 'Klassen-Implementierung ungleich (strenge Kollisionsvermeidung)');
end
fprintf(['Positions-IK für strikte Kollisionsvermeidung berechnet. ', ...
  'Dauer: %1.1fs. Schritte: %d\n'], toc(t1), Stats_CollStop.iter);

figure(5);clf;set(5,'Name','Zielpose_KollVermStreng','NumberTitle','off');
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
trplot(RP.x2t(x0), 'frame', 'S', 'rgb', 'length', 0.2);
trplot(RP.x2t(x1), 'frame', 'D', 'rgb', 'length', 0.3);
RP.plot( q1_cst, x1, s_plot );
title('Zielpose des Roboters mit strikter Kollisionsvermeidung');

% Kollisionsprüfung für Endposen beider Verfahren
[colldet,colldist] = check_collisionset_simplegeom_mex(RP.collbodies, RP.collchecks, ...
  [Tcstack1_cst(:,4)'; Tcstack1_cav(:,4)'], struct('collsearch', false));
for i = 1:2
  if any(colldet(i,:))
    cc_det = RP.collchecks(colldet(i,:),:);
    cb_det = RP.collbodies.link(cc_det(:),:);
    fprintf('Methode %d: Kollisionen der Körper: [%s]\n', i, disp_array(unique(cb_det(:)'),'%d'));
  end
end
assert(~any(colldet(:)), 'Trotz Kollisionsvermeidungsstrategie gibt es in Endpose eine Kollision');
% Prüfe Kollisionsprüfung in Zwischenschritten. Erneute Berechnung
% notwendig, da in IK die Kollision noch hyperbolisch gewichtet wird. 
% Schwellwert zwischen Warnbereich und Eindringung bei Kollision unbekannt.
JP_all_cst = NaN(1+Stats_CollStop.iter, size(Tcstack1_cst,1)); % Menge aller Gelenkpositionen
for i = 1:1+Stats_CollStop.iter
  [~,JP_i] = RP.fkine_coll(Stats_CollStop.Q(i,:)');
  JP_all_cst(i,:) = JP_i;
end
[colldet_cst, colldist_cst] = check_collisionset_simplegeom_mex(RP.collbodies, ...
  RP.collchecks, JP_all_cst, struct('collsearch', true));
assert(all(~colldet_cst(:)), ['Trotz absoluter Kollisionsvermeidung ', ...
  'gibt es Kollisionen in Zwischenschritten']);
test_Q_cst_cav = Stats_CollStop.Q - Stats_CollAvoid.Q;
if ~any(abs(test_Q_cst_cav(:)) > 1e-6)
  warning(['Ergebnisse für strenge und nur finale Kollisionsvermeidung ', ...
    'sind gleich. Voraussichtlich nie Kollisionen in Zwischenschritten.']);
end
t1 = tic();
Namen = {'3T3R', '3T2R', 'FinalAvoid', 'StrictAvoid'};
for kk = 1:4
  if kk == 1
    Q_kk = Stats_3T3R.Q;
    h_kk = NaN(size(Q_kk,1),6);
  elseif kk == 2
    Q_kk = Stats_3T2R.Q;
    h_kk = NaN(size(Q_kk,1),6);
  elseif kk == 3
    Q_kk = Stats_CollAvoid.Q;
    h_kk = Stats_CollAvoid.h;
  elseif kk == 4
    Q_kk = Stats_CollStop.Q;
    h_kk = Stats_CollStop.h;
  end
  X1_kk = RP.fkineEE_traj(Q_kk);
  JP_all_kk = NaN(size(Q_kk,1), size(Tcstack1_cav,1)); % Menge aller Gelenkpositionen
  for i = 1:size(Q_kk,1)
    [~,JP_i] = RP.fkine_coll(Q_kk(i,:)');
    JP_all_kk(i,:) = JP_i;
  end
  [colldet_kk, colldist_kk] = check_collisionset_simplegeom_mex(RP.collbodies, ...
    RP.collchecks, JP_all_kk, struct('collsearch', false));
  
  change_current_figure(20);
  if kk == 1
    set(20, 'Name', 'PosIK_Q', 'NumberTitle', 'off');
    clf;
  end
  ii = 0;
  for i = 1:RP.NLEG
    for j = 1:RP.Leg(i).NJ
      ii = ii + 1;
      subplot(RP.NLEG,RP.Leg(1).NJ,sprc2no(RP.NLEG, RP.Leg(1).NJ, i, j));
      hold on; grid on;
      stairs(Q_kk(:,ii));
      if j == 1, ylabel(sprintf('BK %d', i)); end
      if i == 1, title(sprintf('q %d', j)); end
    end
  end
  if kk == 4
    sgtitle('Gelenkkoordinaten');
    legend(Namen);
  end
  linkxaxes
  change_current_figure(21);
  if kk == 1
    set(21, 'Name', 'PosIK_Coll', 'NumberTitle', 'off');
    clf;
  end
  subplot(2,1,1); hold on;
  plot(min(colldist_kk,[],2));
  if kk == 4
    ylabel('Abstand Kollisionskörper (<0 ist Koll.)'); grid on;
  end
  subplot(2,1,2); hold on;
  plot(h_kk(:,1+5));
  if kk == 4
    ylabel('Zielfunktion'); grid on;
  end
  if kk == 4
    sgtitle('Kollisionsprüfung');
    legend(Namen);
  end
  linkxaxes
  change_current_figure(22);
  if kk == 1
    set(22, 'Name', 'PosIK_X', 'NumberTitle', 'off');
    clf;
  end
  for i = 1:6
    subplot(2,3,i); hold on;
    plot(X1_kk(:,i));
    if kk == 4
      ylabel(sprintf('x %d', i)); grid on;
    end
  end
  if kk == 4
    sgtitle('Plattform-Koordinaten (Beinkette 1)');
    legend(Namen);
  end
  linkxaxes
end
fprintf('Debug-Bilder generiert. Dauer: %1.1fs\n', toc(t1));
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
  X_t_plot = RP.fkineEE2_traj(Q_t_plot);
  t = (1:size(Q_t_plot,1))';
  maxduration_animation = 5; % Dauer des mp4-Videos in Sekunden
  t_Vid = (0:1/30*(t(end)/maxduration_animation):t(end))';
  I_anim = knnsearch( t , t_Vid );
  I_anim = [I_anim; repmat(length(t),15,1)]; %#ok<AGROW> % 15 Standbilder (0.5s) mit letztem Wert
  
  anim_filename = fullfile(resdir, sprintf('ParRob_Nullspace_Collision_Test_PTP_%s', filesuffix));
  s_anim = struct( 'mp4_name', [anim_filename,'.mp4'] );
  s_plot = struct( 'ks_legs', [], 'straight', 0, 'mode', 5, 'only_bodies', true);
  figure(9);clf;
  set(9, 'name', sprintf('Anim'), ...
    'color','w', 'NumberTitle', 'off', 'units','normalized',...
    'outerposition',[0 0 1 1]); % Vollbild, damit Video größer wird
  trplot(RP.x2t(x1), 'frame', 'D', 'rgb', 'length', 0.2);
  hold on; grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
  view(3); % 3D-Ansicht
  title(plottitle);
  RP.anim( Q_t_plot(I_anim,:), X_t_plot(I_anim,:), s_anim, s_plot);
end
end
