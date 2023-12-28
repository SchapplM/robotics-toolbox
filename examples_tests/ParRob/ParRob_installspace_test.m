% Teste die Einhaltung von Bauraumgrenzen in der Nullraumbewegung für PKM
% 
% Siehe auch: SerRob_installspace_test.m

% Moritz Schappler, moritz.schappler@imes.uni-hannoveRP.de, 2021-09
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
% close all

usr_save_figures = false;
usr_create_anim = false; % zum Aktivieren der Video-Animationen (dauert etwas)
usr_use_mex = true; % Zum Debuggen ohne Kompilieren
usr_test_class = true;
usr_debug_ik = true; % Für redundante Debug-Berechnungen in IK-Funktionen
rob_path = fileparts(which('robotics_toolbox_path_init.m'));
resdir = fullfile(rob_path, 'examples_tests', 'results', 'InstallSpace');
mkdirs(resdir);

%% Initialisierung
if isempty(which('parroblib_path_init.m'))
  warning('Repo mit parallelen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbaRP.');
  return
end
RP = parroblib_create_robot_class('P3RRR1G1P1A1', '', 0.75, 0.25);
parroblib_update_template_functions({'P3RRR1G1P1A1'});
RP.fill_fcn_handles(usr_use_mex, true);
pkin_gen = zeros(length(RP.Leg(1).pkin_names),1);
% Nachbearbeitung einiger Kinematikparameter
pkin_gen(strcmp(RP.Leg(1).pkin_names,'a2')) = 0.5;
pkin_gen(strcmp(RP.Leg(1).pkin_names,'a3')) = 0.5;
for i = 1:RP.NLEG
  RP.Leg(i).update_mdh(pkin_gen);
end

% Debug:
% serroblib_create_template_functions({RP.Leg(1).mdlname},false,false);
% matlabfcn2mex({[RP.Leg(1).mdlname,'_invkin_eulangresidual']});
% parroblib_create_template_functions({RP.mdlname},false,false);
% matlabfcn2mex({[RP.mdlname(1:end-6),'_invkin3']});
% matlabfcn2mex({[RP.mdlname(1:end-6),'_invkin']});
% matlabfcn2mex({[RP.mdlname(1:end-6),'_invkin_traj']});

%% Grenzen für die Gelenkkoordinaten setzen
% Dadurch wird die Schrittweite bei der inversen Kinematik begrenzt (auf 5%
% der Spannbreite der Gelenkgrenzen) und die Konfiguration klappt nicht um.
for i = 1:RP.NLEG
  % Begrenze die Winkel der Kugel- und Kardangelenke auf +/- 360°
  RP.Leg(i).qlim = repmat([-2*pi, 2*pi], RP.Leg(i).NQJ, 1);
  % Begrenze Dynamik
  RP.Leg(i).qDlim = repmat([-4*pi, 4*pi], RP.Leg(i).NQJ, 1); % 2rpm
  RP.Leg(i).qDDlim = repmat([-100, 100], RP.Leg(i).NQJ, 1); % Entspricht 1.5 rpm in 100ms
end
qlim_pkm = cat(1, RP.Leg.qlim);

%% Startpose bestimmen
% Mittelstellung im Arbeitsraum
X = [ [0.1;0.1;0]; [0;0;5]*pi/180 ];
% q0 = qlim_pkm(:,1)+rand(RP.NJ,1).*(qlim_pkm(:,2)-qlim_pkm(:,1));
q0 = [2.5;2.4;0.1;NaN(6,1)]; % manuell aus erster Lösung
[q, Phis, Tc_stack_Start, Stats] = RP.invkin_ser(X, q0);
% Zwangsbedingungen in Startpose testen
Phi1=RP.constr1(q, X);
Phit1=RP.constr1_trans(q, X);
Phir1=RP.constr1_rot(q, X);


%% Definition der Kollisionskörper für Bauraumprüfung
collbodies_empty = struct( ...
        'link', [], ... % nx1 uint16, Nummer des zugehörigen Segments (0=Basis)
        'type', [], ... % nx1 uint8, Art des Ersatzkörpers
        'params', []); % Parameter des jeweiligen Ersatzkörpers
% Bauraumprüfungs-Kollisionskörper der Beinketten eintragen
for j = 1:RP.NLEG
  collbodies_instspc_j = collbodies_empty;
  a = [RP.Leg(j).MDH.a;0];
  for i = 1:RP.Leg(j).NJ
    % Punkte im Gelenk
    collbodies_instspc_j.link = [collbodies_instspc_j.link; uint16([i,i])];
    collbodies_instspc_j.type = [collbodies_instspc_j.type; uint8(9)]; % Punkt
    collbodies_instspc_j.params = [collbodies_instspc_j.params; NaN(1,10)]; % keine Parameter
  end
  RP.Leg(j).collbodies_instspc = collbodies_instspc_j;
end

% Bauraum-Begrenzungsobjekt definieren

% Variable für Kollisionsobjekte vorbereiten:
collbodies_instspc = collbodies_empty;
p_Zylinder = [[0,0,-0.3], [0,0,+0.3], 0.900, NaN(1,3)]; % im Ursprung, Radium 900mm
collbodies_instspc.link = [collbodies_instspc.link; [0,0]]; % Der Basis zugerechnet
collbodies_instspc.type = [collbodies_instspc.type; 12]; % Zylinder im Basis-KS
collbodies_instspc.params = [collbodies_instspc.params; p_Zylinder];

I_cb_obj = 1:size(collbodies_instspc.type,1); % Index des Arb.-Raum-Objekts
% Eintragen in Roboter-Klasse
RP.collbodies_instspc_nonleg = collbodies_instspc;
% Aktualisiere die Gesamt-Variable
RP.update_collbodies();
I_cb_legs = I_cb_obj(end)+1:size(RP.collbodies_instspc.type,1);
assert(all(RP.collbodies_instspc.link(:) < (1+7*6)), 'Segment-Nummer der PKM-Kollisionskörper stimmt nicht')

% Liste der Kollisionsprüfungen definieren.
% Teste alle Robotersegmente gegen den Arbeitsraum-Kollisionskörpers.
collchecks_instspc = uint8(zeros(0,2));
for i = I_cb_obj
  for j = I_cb_legs
    collchecks_instspc = [collchecks_instspc; [j, i]]; %#ok<AGROW>
  end
end
% Eintragen in Roboter-Klasse
RP.collchecks_instspc = collchecks_instspc;

% Roboter mit Objekten zeichnen
s_plot = struct( 'ks_legs', [], 'straight', 1, 'mode', [1 6], 'only_bodies', true);
fhdl=change_current_figure(1);clf;set(fhdl,'Name','Startpose','NumberTitle','off');
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
view(3);
RP.plot(q, X, s_plot);
title('Startpose mit Kollisionsmodell des Roboters');
view([0, 90])

%% Verfahrbewegung in Positions-IK mit verschiedenen Einstellungen

s_basic = struct('maxrelstep', 0.001, 'maxrelstep_ns', 0.001, ...
  'retry_limit', 0, 'wn', zeros(RP.idx_ik_length.wnpos,1));
s_basic.wn(RP.idx_ikpos_wn.jac_cond) = 1; % Optimiere PKM-Konditionszahl
s_basic.debug = usr_debug_ik;
q0 = q;
x0 = X;
x1 = x0 + [0.3; -0.1; 0; zeros(3,1)]; % bewege den End-Effektor nach unten rechts (außerhalb des Bauraums)

% IK mit 3T3R (ohne Kollisions-Betrachtung). Benutze 
t1 = tic();
RP.update_EE_FG(logical([1 1 0 0 0 1]), logical([1 1 0 0 0 1]));
s_3T3R = s_basic;
[q_3T3R, Phi, Tcstack1_3T3R, Stats_PosIK_3T3R] = RP.invkin4(x1, q0, s_3T3R);
assert(all(abs(Phi)<1e-8), 'IK mit 3T3R nicht lösbar');
fprintf(['Positions-IK für 3T3R berechnet. ', ...
  'Dauer: %1.1fs. Schritte: %d (für alle Beinketten)\n'], toc(t1), sum(Stats_PosIK_3T3R.iter));
% Statistik nachverarbeiten: Letzten Wert halten statt NaN für jede
% Beinkette (nur bei Nutzung von invkin2)
% for i = 1:RP.NLEG
%   Ii = RP.I1J_LEG(i):RP.I2J_LEG(i);
%   Stats_3T3R.Q(Stats_3T3R.iter(i)+2:end,Ii) = repmat(...
%     Stats_3T3R.Q(Stats_3T3R.iter(i)+1,Ii), size(Stats_3T3R.Q,1)-Stats_3T3R.iter(i)-1,1);
% end
fhdl=change_current_figure(2);clf;set(fhdl,'Name','Zielpose_3T3R','NumberTitle','off');
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
trplot(RP.x2t(x0), 'frame', 'S', 'rgb', 'length', 0.2);
RP.plot( q_3T3R, x1, s_plot ); view([0, 90]);
title('Zielpose des Roboters (3T3R)');


% Verfahrbewegung mit Aufgabenredundanz, ohne Kollisionsbetrachtung
t1 = tic();
s_3T2R = s_basic;
% s_3T2R = rmfield(s_3T2R, 'maxrelstep');
RP.update_EE_FG(logical([1 1 0 0 0 1]), logical([1 1 0 0 0 0]));
[q_3T2R, Phi, Tcstack1_3T2R, Stats_PosIK_3T2R] = RP.invkin4(x1, q0, s_3T2R);
assert(all(abs(Phi)<1e-8), 'IK mit 3T2R (ohne Nebenbedingungen) nicht lösbar');
if usr_test_class % Prüfe, ob das Ergebnis mit Klassen-Implementierung gleich ist
  [q_3T2R2, Phi2, ~, Stats_3T2R2] = RP.invkin3(x1, q0, s_3T2R);
  delta_q_ct = normalizeAngle(q_3T2R2-q_3T2R, 0);
  assert(all(abs(delta_q_ct) < 1e-3), 'Klassen-Implementierung ungleich (3T2R)');
end
fprintf(['Positions-IK für 3T2R berechnet. ', ...
  'Dauer: %1.1fs. Schritte: %d\n'], toc(t1), Stats_PosIK_3T2R.iter);

fhdl=change_current_figure(3);clf;set(fhdl,'Name','Zielpose_3T2R','NumberTitle','off');
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
trplot(RP.x2t(x0), 'frame', 'S', 'rgb', 'length', 0.2);
RP.plot( q_3T2R, x1, s_plot ); view([0, 90])
title('Zielpose des Roboters (3T2R)');
if usr_save_figures
  saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_InstallSpace_Test_Zielpose_3T2R.fig'));
end


% Verfahrbewegung mit Bauraumeinhaltung hyperbolisch (aufgabenredundant).
% Bauraumverletzung in Zwischenphasen erlauben.
t1 = tic();
s_instspc = s_basic;
RP.update_EE_FG(logical([1 1 0 0 0 1]), logical([1 1 0 0 0 0]));
s_instspc.wn(RP.idx_ikpos_wn.instspc_hyp) = 1; % Bauraumeinhaltung aktiv
[q1_instspc, Phi_instspc, Tcstack1_instspc, Stats_PosIK_InstSpc] = RP.invkin4(x1, q0, s_instspc);
assert(all(abs(Phi_instspc)<1e-8), 'IK mit Bauraumeinhaltung im Nullraum nicht lösbar');
if usr_test_class % Prüfe, ob das Ergebnis mit Klassen-Implementierung gleich ist
  [q1_instspc2, Phi_instspc2, Tcstack1_instspc2, Stats_InstSpc2] = RP.invkin3(x1, q0, s_instspc);
  delta_q_ct = normalizeAngle(q1_instspc2-q1_instspc, 0);
  assert(all(abs(delta_q_ct) < 1e-2), 'Klassen-Implementierung ungleich (Bauraumeinhaltung)');
end
fprintf(['Positions-IK für Bauraumeinhaltung (hyperbolisch) berechnet. ', ...
  'Dauer: %1.1fs. Schritte: %d\n'], toc(t1), Stats_PosIK_InstSpc.iter);

fhdl=change_current_figure(4);clf;set(fhdl,'Name','Zielpose_Bauraumopt','NumberTitle','off');
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
RP.plot( q1_instspc, x1, s_plot ); view([0, 90])
title('Zielpose des Roboters mit Bauraumeinhaltung');
if usr_save_figures
  saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_InstallSpace_Test_Zielpose_Bauraumopt.fig'));
end

% Verfahrbewegung mit Bauraumeinhaltung quadratisch (aufgabenredundant).
% Bauraumverletzung in Zwischenphasen erlauben.
t1 = tic();
s_instspcQ = s_basic;
RP.update_EE_FG(logical([1 1 0 0 0 1]), logical([1 1 0 0 0 0]));
s_instspcQ.wn(RP.idx_ikpos_wn.instspc_par) = 1; % Bauraumeinhaltung aktiv
[q1_instspcQ, Phi_instspcQ, Tcstack1_instspcQ, Stats_PosIK_InstSpcQ] = RP.invkin4(x1, q0, s_instspcQ);
assert(all(abs(Phi_instspcQ)<1e-8), 'IK mit Bauraumeinhaltung quadratisch im Nullraum nicht lösbar');
if usr_test_class % Prüfe, ob das Ergebnis mit Klassen-Implementierung gleich ist
  [q1_instspcQ2, Phi_instspcQ2, Tcstack1_instspcQ2, Stats_InstSpcQ2] = RP.invkin3(x1, q0, s_instspcQ);
  delta_q_ct = normalizeAngle(q1_instspcQ2-q1_instspcQ, 0);
  assert(all(abs(delta_q_ct) < 1e-2), 'Klassen-Implementierung ungleich (Bauraumeinhaltung)');
end
fprintf(['Positions-IK für Bauraumeinhaltung (quadratisch) berechnet. ', ...
  'Dauer: %1.1fs. Schritte: %d\n'], toc(t1), Stats_PosIK_InstSpcQ.iter);

fhdl=change_current_figure(5);clf;set(fhdl,'Name','Zielpose_BauraumoptQ','NumberTitle','off');
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
RP.plot( q1_instspc, x1, s_plot ); view([0, 90])
title('Zielpose des Roboters mit quadr. Bauraumeinhaltung');
if usr_save_figures
  saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_InstallSpace_Test_Zielpose_Bauraumopt_quadr.fig'));
end



% Bauraumprüfung für die einzelnen Posen
[colldet,colldist] = check_collisionset_simplegeom_mex(RP.collbodies_instspc, ...
  RP.collchecks_instspc, [Tc_stack_Start(:,4)'; Tcstack1_3T3R(:,4)'; Tcstack1_3T2R(:,4)'; ...
  Tcstack1_instspc(:,4)'], struct('collsearch', false));
assert(all(colldist(1,:)<=0), 'Die Startpose muss im Bauraum liegen');
assert(all(any(colldist(2:3,:)>0,2)), ['Die 3T2R- und 3T3R-Zielpose muss außerhalb ', ...
  'des Bauraums liegen (Beispiel wurde so manuell eingerichtet, dass das der Fall ist)']);
assert(all(colldist(4,:)<=0), ['Trotz Bauraumeinhaltungsstrategie gibt es ', ...
  'in Endpose eine Bauraumverletzung']);

%% Debug-Plots für Positions-IK
RP.update_EE_FG(logical([1 1 0 0 0 1]), logical([1 1 0 0 0 0]));
t1 = tic();
Namen = {'3T3R', '3T2R', 'InstallSpaceOpt_Hyp','InstallSpaceOpt_Quad'};
for kk = 1:length(Namen)
  if kk == 1,     Stats_kk = Stats_PosIK_3T3R;
  elseif kk == 2, Stats_kk = Stats_PosIK_3T2R;
  elseif kk == 3, Stats_kk = Stats_PosIK_InstSpc;
  elseif kk == 4, Stats_kk = Stats_PosIK_InstSpcQ;
  end
  Q_kk = Stats_kk.Q;
  X1_kk = RP.fkineEE_traj(Q_kk);
  % Detail-Ausgabe rekonstruieren
  for i = 1:Stats_kk.iter+1
    s_dummy = struct('wn', ones(RP.idx_ik_length.wnpos,1), 'K', zeros(RP.NJ,1), ...
      'Kn', zeros(RP.NJ,1), 'n_max', 1, 'retry_limit', 0);
    [q_i, Phi_i, ~, Stats_i] = RP.invkin4(X1_kk(i,:)', Q_kk(i,:)', s_dummy);
    assert(all(abs(normalizeAngle(q_i-Q_kk(i,:)')) < 1e-10), 'Neuberechnung der Opt.-Krit. fehlgeschlagen');
    Stats_kk.h(i,:) = Stats_i.h(Stats_i.iter+1,:);
    Stats_kk.instspc_mindst(i,:) = Stats_i.instspc_mindst(Stats_i.iter+1,:);
  end
  h_kk = Stats_kk.h;
  JP_all_kk = NaN(size(Q_kk,1), size(Tcstack1_instspc,1)); % Menge aller Gelenkpositionen
  for i = 1:size(Q_kk,1)
    [~,JP_i] = RP.fkine_coll(Q_kk(i,:)');
    JP_all_kk(i,:) = JP_i;
  end
  [colldet_kk, colldist_kk] = check_collisionset_simplegeom_mex(RP.collbodies_instspc, ...
  RP.collchecks_instspc, JP_all_kk, struct('collsearch', false));
  fhdl=change_current_figure(20);
  if kk == 1
    set(fhdl, 'Name', 'PosIK_Q', 'NumberTitle', 'off');
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
  if kk == length(Namen)
    sgtitle('Gelenkkoordinaten');
    legend(Namen, 'interpreter', 'none');
    linkxaxes
    if usr_save_figures
      saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_InstallSpace_Test_EinzelPose_DebugQ_InstSpc.fig'));
    end
  end
  fhdl=change_current_figure(21);
  if kk == 1
    set(fhdl, 'Name', 'PosIK_InstSpc', 'NumberTitle', 'off'); clf;
  end
  subplot(2,2,1); hold on;
  plot(max(colldist_kk,[],2)); % Maximum, da >0 der kritische Wert ist
  if kk == length(Namen)
    ylabel('Abstand Bauraumkörper (>0 ist Verletzung)'); grid on;
  end
  subplot(2,2,2); hold on;
  plot(h_kk(:,1+RP.idx_ikpos_hn.instspc_hyp));
  if kk == length(Namen)
    ylabel('Zielfunktion hyp.'); grid on;
    set(gca, 'yscale', 'log');
  end
  subplot(2,2,3); hold on;
  plot(h_kk(:,1+RP.idx_ikpos_hn.instspc_par));
  if kk == length(Namen)
    ylabel('Zielfunktion par.'); grid on;
  end
  if kk == length(Namen)
    sgtitle('Bauraumprüfung');
    legend(Namen, 'interpreter', 'none');
    linkxaxes
    if usr_save_figures
      saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_InstallSpace_Test_EinzelPose_Debug_InstSpc.fig'));
    end
  end
  fhdl=change_current_figure(22);
  if kk == 1
    set(fhdl, 'Name', 'PosIK_X', 'NumberTitle', 'off');
    clf;
  end
  for i = 1:6
    subplot(2,3,i); hold on;
    plot(X1_kk(:,i));
    if kk == length(Namen)
      ylabel(sprintf('x %d', i)); grid on;
    end
  end
  if kk == length(Namen)
    sgtitle('Plattform-Koordinaten (Beinkette 1)');
    legend(Namen, 'interpreter', 'none');
    linkxaxes
    if usr_save_figures
      saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_InstallSpace_Test_EinzelPose_Debug_X.fig'));
    end
  end
end
fprintf(['Debug-Bilder (Positions-IK) generiert. Dauer: %1.1fs. Gespeichert ', ...
  'nach %s\n'], toc(t1), resdir);

%% Animation der PTP-Bewegungen mit und ohne Bauraumeinhaltung
if usr_create_anim
for k = 1:4
  if k == 1
    Q_t_plot = Stats_PosIK_3T3R.Q(1:1+Stats_PosIK_3T3R.iter,:);
    filesuffix = 'no_InstallSpace_3T3R';
    plottitle = 'Inverse Kinematics (3T3R) without Installation Space Restriction';
  elseif k == 2
    Q_t_plot = Stats_PosIK_3T2R.Q(1:1+Stats_PosIK_3T2R.iter,:);
    filesuffix = 'no_InstallSpace_3T2R';
    plottitle = 'Inverse Kinematics (3T2R) without Installation Space Restriction';
  elseif k == 3
    Q_t_plot = Stats_PosIK_InstSpc.Q(1:1+Stats_PosIK_InstSpc.iter,:);
    filesuffix = 'with_InstallSpace_final_hyp';
    plottitle = 'Inverse Kinematics with Installation Space Restriction (Hyp)';
  elseif k == 4
    Q_t_plot = Stats_PosIK_InstSpcQ.Q(1:1+Stats_PosIK_InstSpc.iter,:);
    filesuffix = 'with_InstallSpace_final_par';
    plottitle = 'Inverse Kinematics with Installation Space Restriction (Par)';
  end
  X_t_plot = RP.fkineEE2_traj(Q_t_plot);
  t = (1:size(Q_t_plot,1))';
  maxduration_animation = 5; % Dauer des mp4-Videos in Sekunden
  t_Vid = (0:1/30*(t(end)/maxduration_animation):t(end))';
  I_anim = knnsearch( t , t_Vid );
  I_anim = [I_anim; repmat(length(t),15,1)]; %#ok<AGROW> % 15 Standbilder (0.5s) mit letztem Wert
  
  anim_filename = fullfile(resdir, sprintf('ParRob_Nullspace_InstallSpace_Test_PTP_%s', filesuffix));
  s_anim = struct( 'mp4_name', [anim_filename,'.mp4'] );
  s_plot = struct( 'ks_legs', [], 'straight', 1, 'mode', [1 6]);
  fhdl=change_current_figure(9);clf;
  set(fhdl, 'name', sprintf('Anim'), ...
    'color','w', 'NumberTitle', 'off', 'units','normalized',...
    'outerposition',[0 0 1 1]); % Vollbild, damit Video größer wird
  trplot(RP.x2t(x1), 'frame', 'D', 'rgb', 'length', 0.2);
  hold on; grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
  view([0, 90]); % 2D-Ansicht
  title(plottitle);
  RP.anim( Q_t_plot(I_anim,:), X_t_plot(I_anim,:), s_anim, s_plot);
end
end

%% Trajektorien-IK mit Aufgabenredundanz
% Definieren Trajektorie. Muss relativ geringe Geschwindigkeit haben, damit
% Nullraumbewegung zur Bauraumeinhaltung noch gut funktioniert.
[X,XD,XDD,T] = traj_trapez2_multipoint([x0';x1'], 0.2, 1e-1, 1e-2, 1e-3, 0);
fprintf(['Berechne die Trajektorien-IK für %d Bahnpunkte mit verschiedenen ', ...
  'Ansätzen\n'], size(X,1));
% Mit 3T3R (keine Aufgabenredundanz)
t1 = tic();
RP.update_EE_FG(logical([1 1 0 0 0 1]), logical([1 1 0 0 0 1]));
[Q_3T3R, QD_3T3R, QDD_3T3R, PHI, ~, ~, JP_3T3R, Stats_3T3R] = RP.invkin2_traj(X,XD,XDD,T,q0);
if usr_test_class
  [Q_3T3R_2, QD_3T3R_2, QDD_3T3R_2, ~, ~, ~, JP_3T3R_2, Stats_3T3R_2] = RP.invkin_traj(X,XD,XDD,T,q0);
  delta_q_ct = [normalizeAngle(Q_3T3R_2-Q_3T3R, 0);QDD_3T3R_2-QDD_3T3R];
  assert(all(abs(delta_q_ct(:)) < 1e-3), 'Klassen-Implementierung Traj. ungleich (3T3R)');
end
fprintf('Trajektorien-IK für 3T3R berechnet. Dauer: %1.1fs\n', toc(t1));
assert(all(abs(PHI(:))<1e-8), 'Trajektorie mit 3T3R nicht erfolgreich berechnet');
[coll_3T3R, dist_3T3R] = check_collisionset_simplegeom_mex(RP.collbodies_instspc, ...
  RP.collchecks_instspc, JP_3T3R, struct('collsearch', false));
assert(any(dist_3T3R(:)>0), ['In 3T3R-Traj. muss die Bauraumgrenze ver', ...
  'letzt werden. Wurde manuell so eingestellt.']);

% Mit 3T2R (Aufgabenredundanz)
t1 = tic();
RP.update_EE_FG(logical([1 1 0 0 0 1]), logical([1 1 0 0 0 0]));
s_traj_3T2R = struct('wn', zeros(RP.idx_ik_length.wntraj,1));
s_traj_3T2R.wn(RP.idx_iktraj_wnP.qDlim_par) = 0.7; % Zusätzliche Dämpfung gegen Schwingungen
RP.update_EE_FG(logical([1 1 0 0 0 1]), logical([1 1 0 0 0 0]));
[Q_3T2R, QD_3T2R, QDD_3T2R, PHI, ~, ~, JP_3T2R, Stats_3T2R] = RP.invkin2_traj(X,XD,XDD,T,q0,s_traj_3T2R);
if usr_test_class
  [Q_3T2R_2, QD_3T2R_2, QDD_3T2R_2, PHI_2, ~, ~, JP_3T2R_2, Stats_3T2R_2] = RP.invkin_traj(X,XD,XDD,T,q0,s_traj_3T2R);
  delta_q_ct = normalizeAngle(Q_3T2R_2-Q_3T2R, 0);
  delta_qDD_ct = QDD_3T2R_2-QDD_3T2R;
  assert(all(abs([delta_q_ct(:);delta_qDD_ct(:)]) < 1e-3), ...
    'Klassen-Implementierung Traj. ungleich (3T2R)');
end
fprintf('Trajektorien-IK für 3T2R berechnet. Dauer: %1.1fs\n', toc(t1));
assert(all(abs(PHI(:))<1e-8), 'Trajektorie mit 3T2R nicht erfolgreich berechnet');
[coll_3T2R, dist_3T2R] = check_collisionset_simplegeom_mex(RP.collbodies_instspc, ...
  RP.collchecks_instspc, JP_3T2R, struct('collsearch', false));
assert(any(dist_3T2R(:)>0), ['In 3T2R-Traj. muss die Bauraumgrenze ver', ...
  'letzt werden. Wurde manuell so eingestellt.']);

% Mit 3T2R (Aufgabenredundanz, Bauraumeinhaltung mit PD-Regler, hyperbolisch)
% parroblib_create_template_functions({RP.mdlname},false,false);
t1 = tic();
RP.update_EE_FG(logical([1 1 0 0 0 1]), logical([1 1 0 0 0 1]));
s_traj_IS1 = struct('wn', zeros(RP.idx_ik_length.wntraj,1));
s_traj_IS1.wn(RP.idx_iktraj_wnP.qDlim_par) = 0.7; % Zusätzliche Dämpfung gegen Schwingungen
s_traj_IS1.wn(RP.idx_iktraj_wnP.instspc_hyp) = 1e-4; % P-Verstärkung Bauraumeinhaltung (hyperbolisch)
s_traj_IS1.wn(RP.idx_iktraj_wnD.instspc_hyp) = 1e-5; % D-Verstärkung Bauraumeinhaltung (hyperbolisch)
RP.update_EE_FG(logical([1 1 0 0 0 1]), logical([1 1 0 0 0 0]));
[Q_InstSpc1, QD_InstSpc1, QDD_InstSpc1, PHI, ~, ~, JP_InstSpc1, Stats_InstSpc1] = RP.invkin2_traj(X,XD,XDD,T,q0,s_traj_IS1);
if usr_test_class && all(~isnan(PHI(:))) % nur, wenn Traj. i.O., sonst numerische Abweichungen
  [Q_InstSpc1_2, QD_InstSpc1_2, QDD_InstSpc1_2, PHI_2, ~, ~, JP_InstSpc1_2, Stats_InstSpc1_2] = RP.invkin_traj(X,XD,XDD,T,q0,s_traj_IS1);
  delta_q_ct = normalizeAngle(Q_InstSpc1_2-Q_InstSpc1, 0);
  delta_qDD_ct = QDD_InstSpc1_2-QDD_InstSpc1;
  assert(all(abs(delta_q_ct(:)) < 1e-3), 'Klassen-Implementierung Traj. ungleich (Bauraum Var. 1)');
end
fprintf('Trajektorien-IK mit hyperbolischer Bauraumeinhaltung berechnet. Dauer: %1.1fs\n', toc(t1));
assert(all(abs(PHI(:))<1e-8) && all(~isnan(PHI(:))), 'Trajektorie mit Bauraumeinhaltung (hyp.) nicht erfolgreich berechnet');
[coll_InstSpc1, dist_InstSpc1] = check_collisionset_simplegeom_mex(RP.collbodies_instspc, ...
  RP.collchecks_instspc, JP_InstSpc1, struct('collsearch', false));
assert(all(dist_InstSpc1(:)<0), ['Trotz Bauraumeinhaltung (hyp) ', ...
  'gibt es Bauraumverletzungen in Zwischenschritten']);

% Mit 3T2R (Aufgabenredundanz, Bauraumeinhaltung mit PD-Regler, quadratisch)
t1 = tic();
RP.update_EE_FG(logical([1 1 0 0 0 1]), logical([1 1 0 0 0 1]));
s_traj_IS2 = struct('wn', zeros(RP.idx_ik_length.wntraj,1));
s_traj_IS2.wn(RP.idx_iktraj_wnP.qDlim_par) = 0.7; % Zusätzliche Dämpfung gegen Schwingungen
s_traj_IS2.wn(RP.idx_iktraj_wnP.instspc_hyp) = 1; % P-Verstärkung Bauraumeinhaltung (quadr.)
s_traj_IS2.wn(RP.idx_iktraj_wnD.instspc_hyp) = 0.3; % D-Verstärkung Bauraumeinhaltung (quadr.)
RP.update_EE_FG(logical([1 1 0 0 0 1]), logical([1 1 0 0 0 0]));
[Q_InstSpc2, QD_InstSpc2, QDD_InstSpc2, PHI, ~, ~, JP_InstSpc2, Stats_InstSpc2] = RP.invkin2_traj(X,XD,XDD,T,q0,s_traj_IS2);
if usr_test_class && all(~isnan(PHI(:))) % nur, wenn Traj. i.O., sonst numerische Abweichungen
  [Q_InstSpc2_2, QD_InstSpc2_2, QDD_InstSpc2_2, PHI_2, ~, ~, JP_InstSpc12_2, Stats_InstSpc2_2] = RP.invkin_traj(X,XD,XDD,T,q0,s_traj_IS2);
  delta_q_ct = normalizeAngle(Q_InstSpc2_2-Q_InstSpc2, 0);
  delta_qDD_ct = QDD_InstSpc2_2-QDD_InstSpc2;
  assert(all(abs(delta_q_ct(:)) < 1e-3), 'Klassen-Implementierung Traj. ungleich (Bauraum Var. 2)');
end
fprintf('Trajektorien-IK mit quadratischer Bauraumeinhaltung berechnet. Dauer: %1.1fs\n', toc(t1));
assert(all(abs(PHI(:))<1e-8) && all(~isnan(PHI(:))), 'Trajektorie mit Bauraumeinhaltung (par.) nicht erfolgreich berechnet');
[coll_InstSpc2, dist_InstSpc2] = check_collisionset_simplegeom_mex(RP.collbodies_instspc, ...
  RP.collchecks_instspc, JP_InstSpc2, struct('collsearch', false));
assert(all(dist_InstSpc1(:)<0), ['Trotz Bauraumeinhaltung (par) ', ...
  'gibt es Bauraumverletzungen in Zwischenschritten']);

%% Debug-Plots für Trajektorien-IK
t1 = tic();
Namen = {'3T3R', '3T2R', 'InstallSpace_Hyp', 'InstallSpace_Par'};
for kk = 1:length(Namen)
  if kk == 1
    Q_kk = Q_3T3R; QD_kk = QD_3T3R; QDD_kk = QDD_3T3R; JP_all_kk = JP_3T3R;
    Stats_kk = Stats_3T3R;
  elseif kk == 2
    Q_kk = Q_3T2R; QD_kk = QD_3T2R; QDD_kk = QDD_3T2R; JP_all_kk = JP_3T2R;
    Stats_kk = Stats_3T2R;
  elseif kk == 3
    Q_kk = Q_InstSpc1; QD_kk = QD_InstSpc1; QDD_kk = QDD_InstSpc1; JP_all_kk = JP_InstSpc1;
    Stats_kk = Stats_InstSpc1;
  elseif kk == 4
    Q_kk = Q_InstSpc2; QD_kk = QD_InstSpc2; QDD_kk = QDD_InstSpc2; JP_all_kk = JP_InstSpc2;
    Stats_kk = Stats_InstSpc2;
  end
  [X1_kk, XD1_kk, XDD1_kk] = RP.fkineEE2_traj(Q_kk, QD_kk, QDD_kk);
  X1_kk(:,4:6) = denormalize_angle_traj(X1_kk(:,4:6));
  [colldet_kk, colldist_kk] = check_collisionset_simplegeom_mex(RP.collbodies_instspc, ...
    RP.collchecks_instspc, JP_all_kk, struct('collsearch', false));
  % Detail-Ausgabe rekonstruieren
  Stats_kk.instspc_mindst = NaN(size(Q_kk,1),2); % Feld manuell hinzufügen
  for i = 1:size(Q_kk,1)
    s_dummy = struct('wn', ones(RP.idx_ik_length.wnpos,1), 'K', zeros(RP.NJ,1), ...
      'Kn', zeros(RP.NJ,1), 'n_max', 1, 'retry_limit', 0);
    [q_i, Phi_i, ~, Stats_i] = RP.invkin4(X1_kk(i,:)', Q_kk(i,:)', s_dummy);
    assert(all(abs(normalizeAngle(q_i-Q_kk(i,:)')) < 1e-10), 'Neuberechnung der Opt.-Krit. fehlgeschlagen');
    Stats_kk.instspc_mindst(i,:) = Stats_i.instspc_mindst(Stats_i.iter+1,:);
    for f = fields(RP.idx_ikpos_hn)'
      Stats_kk.h(i,1+RP.idx_iktraj_hn.(f{1})) = Stats_i.h(Stats_i.iter+1,1+RP.idx_ikpos_hn.(f{1}));
    end
  end
  % Prüfe, ob das Nachrechnen erfolgreich war.
  test_dist = Stats_kk.instspc_mindst(:,1)-max(colldist_kk,[],2);
  assert(all(abs(test_dist) < 1e-10), 'Rekonstruktion der Abstände fehlgeschlagen');
  
  fhdl=change_current_figure(40);
  if kk == 1
    set(fhdl, 'Name', 'TrajIK_Q', 'NumberTitle', 'off'); clf;
  end
  ii = 0;
  for i = 1:RP.NLEG
    for j = 1:RP.Leg(i).NJ
      ii = ii + 1;
      subplot(RP.NLEG,RP.Leg(1).NJ,sprc2no(RP.NLEG, RP.Leg(1).NJ, i, j));
      hold on; grid on;
      plot(T, Q_kk(:,ii));
      if j == 1, ylabel(sprintf('BK %d', i)); end
      if i == 1, title(sprintf('q %d', j)); end
    end
  end
  if kk == length(Namen)
    sgtitle('Gelenkkoordinaten');
    legend(Namen, 'interpreter', 'none');
    linkxaxes
    if usr_save_figures
      saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_InstallSpace_Test_Traj_Debug_Q.fig'));
    end
  end
  fhdl=change_current_figure(41);
  if kk == 1
    set(fhdl, 'Name', 'TrajIK_QD', 'NumberTitle', 'off'); clf;
  end
  ii = 0;
  for i = 1:RP.NLEG
    for j = 1:RP.Leg(i).NJ
      ii = ii + 1;
      subplot(RP.NLEG,RP.Leg(1).NJ,sprc2no(RP.NLEG, RP.Leg(1).NJ, i, j));
      hold on; grid on;
      plot(T, QD_kk(:,ii));
      if j == 1, ylabel(sprintf('BK %d', i)); end
      if i == 1, title(sprintf('qD %d', j)); end
    end
  end
  if kk == length(Namen)
    sgtitle('Gelenkgeschwindigkeiten');
    legend(Namen, 'interpreter', 'none');
    linkxaxes
    if usr_save_figures
      saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_InstallSpace_Test_Traj_Debug_QD.fig'));
    end
  end
  fhdl=change_current_figure(42);
  if kk == 1
    set(fhdl, 'Name', 'TrajIK_QDD', 'NumberTitle', 'off'); clf;
  end
  ii = 0;
  for i = 1:RP.NLEG
    for j = 1:RP.Leg(i).NJ
      ii = ii + 1;
      subplot(RP.NLEG,RP.Leg(1).NJ,sprc2no(RP.NLEG, RP.Leg(1).NJ, i, j));
      hold on; grid on;
      plot(T, QDD_kk(:,ii));
      if j == 1, ylabel(sprintf('BK %d', i)); end
      if i == 1, title(sprintf('qDD %d', j)); end
    end
  end
  if kk == length(Namen)
    sgtitle('Gelenkbeschleunigungen');
    legend(Namen, 'interpreter', 'none');
    linkxaxes
    if usr_save_figures
      saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_InstallSpace_Test_Traj_Debug_QDD.fig'));
    end
  end
  fhdl=change_current_figure(43);
  if kk == 1
    set(fhdl, 'Name', 'TrajIK_InstSpc', 'NumberTitle', 'off');
    clf;
  end
  subplot(2,1,1); hold on;
  plot(T, max(colldist_kk,[],2)); % größter (positiver) Wert ist kritisch
  if kk == length(Namen)
    ylabel('Abstand Bauraumkörper (>0 ist Verl.)'); grid on;
  end
  subplot(2,2,3); hold on;
  plot(T, Stats_kk.h(:,1+RP.idx_iktraj_hn.instspc_hyp));
  if kk == length(Namen)
    ylabel('Zielfunktion (hyp)'); grid on;
    set(gca, 'yscale', 'log');
  end
  subplot(2,2,4); hold on;
  plot(T, Stats_kk.h(:,1+RP.idx_iktraj_hn.instspc_par));
  if kk == length(Namen)
    ylabel('Zielfunktion (par)'); grid on;
  end
  if kk == length(Namen)
    sgtitle('Kollisionsprüfung');
    legend(Namen, 'interpreter', 'none');
    linkxaxes
    if usr_save_figures
      saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_InstallSpace_Test_Traj_Debug_Bauraum.fig'));
    end
  end
  
  fhdl=change_current_figure(44);
  if kk == 1
    set(fhdl, 'Name', 'TrajIK_X', 'NumberTitle', 'off');
    clf;
  end
  for i = 1:6
    subplot(3,6,sprc2no(3,6,1,i)); hold on;
    plot(T, X1_kk(:,i));
    if kk == length(Namen)
      ylabel(sprintf('x %d', i)); grid on;
    end
    subplot(3,6,sprc2no(3,6,2,i)); hold on;
    plot(T, XD1_kk(:,i));
    if kk == length(Namen)
      ylabel(sprintf('xD %d', i)); grid on;
    end
    subplot(3,6,sprc2no(3,6,3,i)); hold on;
    plot(T, XDD1_kk(:,i));
    if kk == length(Namen)
      ylabel(sprintf('xDD %d', i)); grid on;
    end
  end
  if kk == length(Namen)
    sgtitle('Plattform-Koordinaten (Beinkette 1)');
    legend(Namen, 'interpreter', 'none');
    linkxaxes
    if usr_save_figures
      saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_InstallSpace_Test_Traj_Debug_X.fig'));
    end
  end
end
fprintf(['Debug-Bilder für Trajektorien-IK generiert. Dauer: %1.1fs. ', ...
  'Gespeichert nach %s\n'], toc(t1), resdir);

%% Animation der Trajektorien-Bewegungen mit und ohne Bauraumeinhaltung
if usr_create_anim
for k = 1:length(Namen)
  if k == 1
    Q_t_plot = Q_3T3R;
    filesuffix = 'no_InstallSpace_3T3R';
    plottitle = 'Inverse Kinematics (3T3R) without Install Space Constraint';
  elseif k == 2
    Q_t_plot = Q_3T2R;
    filesuffix = 'no_InstallSpace_3T2R';
    plottitle = 'Inverse Kinematics (3T2R) without Install Space Constraint';
  elseif k == 3
    Q_t_plot = Q_InstSpc1;
    filesuffix = 'with_InstallSpaceOpt_Hyp';
    plottitle = 'Inverse Kinematics with Install Space Constraint (Hyperbolic)';
  elseif k == 4
    Q_t_plot = Q_InstSpc2;
    filesuffix = 'with_InstallSpaceOpt_Par';
    plottitle = 'Inverse Kinematics with Install Space Constraint (Parabolic)';
  end
  X_t_plot = RP.fkineEE2_traj(Q_t_plot);
  maxduration_animation = 5; % Dauer des mp4-Videos in Sekunden
  t_Vid = (0:1/30*(T(end)/maxduration_animation):T(end))';
  I_anim = knnsearch( T , t_Vid );
  I_anim = [I_anim; repmat(length(T),15,1)]; %#ok<AGROW> % 15 Standbilder (0.5s) mit letztem Wert
  
  anim_filename = fullfile(resdir, sprintf('ParRob_Nullspace_InstallSpace_Test_Traj_%s', filesuffix));
  s_anim = struct( 'mp4_name', [anim_filename,'.mp4'] );
  s_plot = struct( 'ks_legs', [], 'straight', 1, 'mode', [1 6]);
  fhdl=change_current_figure(9);clf;
  set(fhdl, 'name', sprintf('Anim'), ...
    'color','w', 'NumberTitle', 'off', 'units','normalized',...
    'outerposition',[0 0 1 1]); % Vollbild, damit Video größer wird
  trplot(RP.x2t(x1), 'frame', 'D', 'rgb', 'length', 0.2);
  plot3(X_t_plot(:,1), X_t_plot(:,2), X_t_plot(:,3), 'c-');
  hold on; grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
  view([0 90]); % 2D-Ansicht
  title(plottitle);
  RP.anim( Q_t_plot(I_anim,:), X_t_plot(I_anim,:), s_anim, s_plot);
end
end
