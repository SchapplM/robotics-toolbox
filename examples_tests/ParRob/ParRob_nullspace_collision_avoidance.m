% Teste die Kollisionsvermeidung in der Nullraumbewegung für PKM
% TODO: Das Beispiel ist noch nicht besonders deutlich. In Traj.-IK werden
% die Kollisionen nicht vollständig vermieden.
% 
% Siehe auch: SerRob_nullspace_collision_avoidance.m

% Moritz Schappler, moritz.schappler@imes.uni-hannoveRP.de, 2021-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
close all

usr_create_anim = false; % zum Aktivieren der Video-Animationen (dauert etwas)
usr_test_class = true;
usr_create_plots = true;
rob_path = fileparts(which('robotics_toolbox_path_init.m'));
resdir = fullfile(rob_path, 'examples_tests', 'results', 'CollAvoidance');
mkdirs(resdir);

%% Initialisierung
if isempty(which('parroblib_path_init.m'))
  warning('Repo mit parallelen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbaRP.');
  return
end
RP = parroblib_create_robot_class('P6RRRRRR10V3G6P4A1', '', [0.2; 0.1], [0.2; 0.1]);
parroblib_update_template_functions({'P6RRRRRR10V3G6P4A1'});
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

% Debug:
% serroblib_create_template_functions({RP.Leg(1).mdlname},false,false);
% matlabfcn2mex({[RP.Leg(1).mdlname,'_invkin_eulangresidual']});
% parroblib_create_template_functions({RP.mdlname},false,false);
% mexerr = false;
% mexerr = mexerr | matlabfcn2mex({[RP.mdlname(1:12),'_invkin3']}, true);
% mexerr = mexerr | matlabfcn2mex({[RP.mdlname(1:12),'_invkin']}, true);
% mexerr = mexerr | matlabfcn2mex({[RP.mdlname(1:12),'_invkin_traj']}, true);
% if mexerr
%   error('Fehler beim Kompilieren');
% end

%% Grenzen für die Gelenkkoordinaten setzen
% Dadurch wird die Schrittweite bei der inversen Kinematik begrenzt (auf 5%
% der Spannbreite der Gelenkgrenzen) und die Konfiguration klappt nicht um.
for i = 1:RP.NLEG
  % Begrenze die Winkel der Kugel- und Kardangelenke auf +/- 360°
  RP.Leg(i).qlim = repmat([-2*pi, 2*pi], RP.Leg(i).NQJ, 1);
  % Begrenze Dynamik (Gleicher Zahlenwert auch für Schubgelenke)
  RP.Leg(i).qDlim = repmat([-4*pi, 4*pi], RP.Leg(i).NQJ, 1); % 2rpm
  RP.Leg(i).qDDlim = repmat([-100, 100], RP.Leg(i).NQJ, 1); % Entspricht 1.5 rpm in 100ms
end
qlim_pkm = cat(1, RP.Leg.qlim);

%% Startpose bestimmen
RP.update_EE_FG(logical([1 1 1 1 1 1]), logical([1 1 1 1 1 1]));
% Mittelstellung im Arbeitsraum
x0 = [ [0.15;0.05;0.3]; [10;-10;5]*pi/180 ];
q0_init = NaN(36,1);
q0_init(1:6:end) = 60*pi/180; % Erstes Gelenk sollte nach außen zeigen
q0_init(2:6:end) = -120*pi/180; % damit Außenstellung gelingt
q0_init(3:6:end) = 0;
q0_init(4:6:end) = 0.1; % Gegen rechnerische Singularität
q0_init(5:6:end) = -0.1;
q0_init(6:6:end) = 0.1;
[q0, Phis, Tc_start, Stats] = RP.invkin_ser(x0, q0_init, struct('retry_limit', 0));
% Zwangsbedingungen in Startpose testen
Phi1=RP.constr1(q0, x0);
Phit1=RP.constr1_trans(q0, x0);
Phir1=RP.constr1_rot(q0, x0);
assert(all(abs(Phis)<1e-6), 'IK in Startpose nicht erfolgreich');
%% Definition der Kollisionskörper
collbodies_empty = struct( ...
        'link', [], ... % nx1 uint16, Nummer des zugehörigen Segments (0=Basis)
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
    collbodies_j.link = [collbodies_j.link; uint16([i,i-1])];
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
  uint16([zeros(RP.NLEG,1), RP.I1L_LEG-(I1-1)])];
collbodies.type = [collbodies.type; repmat(uint8(6),RP.NLEG,1)];
collbodies.params = [collbodies.params; ...
  [repmat(10e-3, RP.NLEG, 1), NaN(RP.NLEG, 9)]];
% Ringförmige Basis; verbindet die Basis der Beinketten mit der jeweils
% vorherigen
collbodies.link = [collbodies.link; ...
  uint16([RP.I1L_LEG(I1)-(I1-1), RP.I1L_LEG(I2)-(I2-1)])];
collbodies.type = [collbodies.type; repmat(uint8(6),RP.NLEG,1)];
collbodies.params = [collbodies.params; ...
  [repmat(10e-3, RP.NLEG, 1), NaN(RP.NLEG, 9)]];
I_cb_base = 1:size(collbodies.type,1); % Indizes der Basis-Koll.-körper
% Kollisionsobjekte für die Plattform. Kapseln für jeden virtuellen
% Körper der Plattform-Koppelgelenke (auf Plattform-Seite). Kapsel als
% Verbindung zum jeweils vorherigen Koppelgelenk. Erzeugt Ring an der
% Plattform
collbodies.link = [collbodies.link; ...
  uint16([RP.I2L_LEG(I1)-(I1-1)-1, RP.I2L_LEG(I2)-(I2-1)-1])];
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
% Teste alle Beinketten gegeneinander (nur jeweils das zweite Segment).
% Dieses neigt zu Kollisionen, da die Ketten zur Plattform gehen.
for i = I_cb_legs
  for j = I_cb_legs
    if i >= j, continue; end % nicht sich selbst und nicht doppelt testen
%     fprintf('%d (Link %d) - %d (Link %d)', i, RP.collbodies.link(i,1), ...
%       RP.collbodies.link(j,1), j);
    % Für Vergleich Typumwandlung in Format mit Vorzeichen
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
if usr_create_plots
  fhdl=figure(1);clf;set(fhdl,'Name','Startpose','NumberTitle','off');
  hold on; grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
  view(3);
  RP.plot(q0, x0, s_plot);
  title('Startpose mit Kollisionsmodell des Roboters');
end
[colldet_start, colldist_start] = check_collisionset_simplegeom_mex(RP.collbodies, ...
  RP.collchecks, Tc_start(:,4)', struct('collsearch', true));
assert(all(~colldet_start(:)), ['Es sollte keine Kollision in der Startpose ', ...
  'vorliegen (manuell geprüft)']);
% Debug: RP.collchecks(any(colldet_start),:)

%% Verfahrbewegung in Positions-IK mit verschiedenen Einstellungen

s_basic = struct('maxrelstep', 0.001, 'maxrelstep_ns', 0.001, ...
  'retry_limit', 0, 'wn', zeros(RP.idx_ik_length.wnpos,1));
s_basic.wn(RP.idx_ikpos_wn.jac_cond) = 1; % Optimiere PKM-Konditionszahl
x1 = x0 + [-0.4; 0; 0; zeros(3,1)]; % bewege den End-Effektor nach unten (in die Kollision)
% if false
% IK mit 3T3R (ohne Kollisions-Betrachtung). Benutze 
t1 = tic();
RP.update_EE_FG(logical([1 1 1 1 1 1]), logical([1 1 1 1 1 1]));
s_3T3R = s_basic;
[q_3T3R, Phi, Tc_3T3R, Stats_PosIK_3T3R] = RP.invkin4(x1, q0, s_3T3R);
assert(all(abs(Phi)<1e-8), 'IK mit 3T3R nicht lösbar');
fprintf(['Positions-IK für 3T3R berechnet. ', ...
  'Dauer: %1.1fs. Schritte: %d (für alle Beinketten)\n'], toc(t1), sum(Stats_PosIK_3T3R.iter));
[colldet_3T3R, colldist_3T3R] = check_collisionset_simplegeom_mex(RP.collbodies, ...
  RP.collchecks, Tc_3T3R(:,4)', struct('collsearch', true));
assert(any(colldet_3T3R(:)), ['Es sollte eine Kollision am Ende der ', ...
  '3T3R-IK geben (manuell so eingestellt)']);
% Statistik nachverarbeiten: Letzten Wert halten statt NaN für jede
% Beinkette (nur bei Nutzung von invkin2)
% for i = 1:RP.NLEG
%   Ii = RP.I1J_LEG(i):RP.I2J_LEG(i);
%   Stats_3T3R.Q(Stats_3T3R.iter(i)+2:end,Ii) = repmat(...
%     Stats_3T3R.Q(Stats_3T3R.iter(i)+1,Ii), size(Stats_3T3R.Q,1)-Stats_3T3R.iter(i)-1,1);
% end
if usr_create_plots
  fhdl=figure(2);clf;set(fhdl,'Name','Zielpose_3T3R','NumberTitle','off');
  hold on; grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
  trplot(RP.x2t(x0), 'frame', 'S', 'rgb', 'length', 0.2);
  RP.plot( q_3T3R, x1, s_plot );
  title('Zielpose des Roboters (3T3R)');
end
% Verfahrbewegung mit Aufgabenredundanz, ohne Kollisionsbetrachtung
t1 = tic();
s_3T2R = s_basic;
% s_3T2R = rmfield(s_3T2R, 'maxrelstep');
RP.update_EE_FG(logical([1 1 1 1 1 1]), logical([1 1 1 1 1 0]));
[q_3T2R, Phi, Tc_3T2R, Stats_PosIK_3T2R] = RP.invkin4(x1, q0, s_3T2R);
assert(all(abs(Phi)<1e-8), 'IK mit 3T2R (ohne Nebenbedingungen) nicht lösbar');
if usr_test_class % Prüfe, ob das Ergebnis mit Klassen-Implementierung gleich ist
  [q_3T2R2, Phi2, ~, Stats_3T2R2] = RP.invkin3(x1, q0, s_3T2R);
  delta_q_ct = normalizeAngle(q_3T2R2-q_3T2R, 0);
  % Debug:
%   figure(100);clf;
%   for i = 1:RP.NJ
%     subplot(6,6,i); hold on;
%     plot(Stats_3T2R.Q(:,i));
%     plot(Stats_3T2R2.Q(:,i));
%     grid on;
%   end
%   linkxaxes
%   legend({'Stand-Alone', 'Klasse'});
  assert(all(abs(delta_q_ct) < 1e-3), 'Klassen-Implementierung ungleich (3T2R)');
end
[colldet_3T2R, colldist_3T2R] = check_collisionset_simplegeom_mex(RP.collbodies, ...
  RP.collchecks, Tc_3T2R(:,4)', struct('collsearch', true));
assert(any(colldet_3T2R(:)), ['Es sollte eine Kollision am Ende der ', ...
  '3T2R-IK geben (manuell so eingestellt)']);
fprintf(['Positions-IK für 3T2R berechnet. ', ...
  'Dauer: %1.1fs. Schritte: %d\n'], toc(t1), Stats_PosIK_3T2R.iter);
if usr_create_plots
  fhdl=figure(3);clf;set(fhdl,'Name','Zielpose_3T2R','NumberTitle','off');
  hold on; grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
  trplot(RP.x2t(x0), 'frame', 'S', 'rgb', 'length', 0.2);
  RP.plot( q_3T2R, x1, s_plot );
  title('Zielpose des Roboters (3T2R)');
  saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_Collision_Test_Zielpose_3T2R.fig'));
end
% Verfahrbewegung mit Kollisionsvermeidung (aufgabenredundant). Kollisionen
% in Zwischenphasen erlauben.
t1 = tic();
s_collav = s_basic;
RP.update_EE_FG(logical([1 1 1 1 1 1]), logical([1 1 1 1 1 0]));
s_collav.wn(RP.idx_ikpos_wn.coll_hyp) = 1; % Kollisionsvermeidung (hyperbolisch) aktiv
[q1_cav, Phi_cav, Tcstack1_cav, Stats_PosIK_CollAvoid] = RP.invkin4(x1, q0, s_collav);
assert(all(abs(Phi_cav)<1e-8), 'IK mit Kollisionsvermeidung im Nullraum nicht lösbar');
if usr_test_class % Prüfe, ob das Ergebnis mit Klassen-Implementierung gleich ist
  [q1_cav2, Phi_cav2, Tcstack1_cav2, Stats_CollAvoid2] = RP.invkin3(x1, q0, s_collav);
  delta_q_ct = normalizeAngle(q1_cav2-q1_cav, 0);
  assert(all(abs(delta_q_ct) < 1e-2), 'Klassen-Implementierung ungleich (finale Kollisionsvermeidung)');
end
fprintf(['Positions-IK für finale Kollisionsvermeidung (hyperbolisch) berechnet. ', ...
  'Dauer: %1.1fs. Schritte: %d\n'], toc(t1), Stats_PosIK_CollAvoid.iter);
if usr_create_plots
  fhdl=figure(4);clf;set(fhdl,'Name','Zielpose_KollVermFinal','NumberTitle','off');
  hold on; grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
  RP.plot( q1_cav, x1, s_plot );
  title('Zielpose des Roboters mit finaler Kollisionsvermeidung');
  saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_Collision_Test_Zielpose_KollVermFinal.fig'));
end
% Zweite Implementierung mit quadratischer Abstandsfunktion, die den
% Abstand global minimieren soll, statt nur die Kollision zu vermeiden
s_collav2 = s_basic;
RP.update_EE_FG(logical([1 1 1 1 1 1]), logical([1 1 1 1 1 0]));
s_collav2.wn(:) = 0; % Jacobi-Matrix-Optimierung ignorieren
s_collav2.wn(RP.idx_ikpos_wn.coll_par) = 1; % Kollisionsvermeidung (quadratisch) aktiv
[q1_cav2, Phi_cav2, Tcstack1_cav2, Stats_PosIK_CollAvoid2] = RP.invkin4(x1, q0, s_collav2);
assert(all(abs(Phi_cav)<1e-8), 'IK mit Kollisionsvermeidung im Nullraum nicht lösbar');
if usr_test_class % Prüfe, ob das Ergebnis mit Klassen-Implementierung gleich ist
  [q1_cav22, Phi_cav22, Tcstack1_cav22, Stats_CollAvoid22] = RP.invkin3(x1, q0, s_collav2);
  delta_q_ct = normalizeAngle(q1_cav22-q1_cav2, 0);
  assert(all(abs(delta_q_ct) < 1e-2), 'Klassen-Implementierung ungleich (finale Kollisionsvermeidung quadratisch)');
end
fprintf(['Positions-IK für finale Kollisionsvermeidung (quadratisch) berechnet. ', ...
  'Dauer: %1.1fs. Schritte: %d\n'], toc(t1), Stats_PosIK_CollAvoid2.iter);
if usr_create_plots
  fhdl=figure(5);clf;set(fhdl,'Name','Zielpose_KollVermFinalQuadr','NumberTitle','off');
  hold on; grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
  RP.plot( q1_cav, x1, s_plot );
  title('Zielpose des Roboters mit finaler Kollisionsvermeidung (quadr.)');
  saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_Collision_Test_Zielpose_KollVermFinalQuadr.fig'));
end
% Verfahrbewegung mit Kollisionsvermeidung (aufgabenredundant). Kollisionen
% in Zwischenphasen verbieten.
t1 = tic();
s_collstop = s_basic;
s_collstop.scale_coll = 0.5;
RP.update_EE_FG(logical([1 1 1 1 1 1]), logical([1 1 1 1 1 0]));
s_collstop.wn(RP.idx_ikpos_wn.coll_hyp) = 1; % Kollisionsvermeidung aktiv
[q1_cst, Phi_cst, Tcstack1_cst, Stats_PosIK_CollStop] = RP.invkin4(x1, q0, s_collstop);
assert(all(abs(Phi_cst)<1e-8), 'IK mit absoluter Kollisionsvermeidung nicht lösbar');
if usr_test_class % Prüfe, ob das Ergebnis mit Klassen-Implementierung gleich ist
  [q1_cst2, Phi_cst2, Tcstack1_cst2, Stats_CollStop2] = RP.invkin3(x1, q0, s_collstop);
  delta_q_ct = normalizeAngle(q1_cst2-q1_cst, 0);
  assert(all(abs(delta_q_ct) < 1e-2), 'Klassen-Implementierung ungleich (strenge Kollisionsvermeidung)');
end
Stats_PosIK_CollStop = Stats_CollStop2; % Debug. TODO: Weg, wenn tpl aktualisiert.
fprintf(['Positions-IK für strikte Kollisionsvermeidung berechnet. ', ...
  'Dauer: %1.1fs. Schritte: %d\n'], toc(t1), Stats_PosIK_CollStop.iter);
if usr_create_plots
  fhdl=figure(6);clf;set(fhdl,'Name','Zielpose_KollVermStreng','NumberTitle','off');
  hold on; grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
  trplot(RP.x2t(x0), 'frame', 'S', 'rgb', 'length', 0.2);
  trplot(RP.x2t(x1), 'frame', 'D', 'rgb', 'length', 0.3);
  RP.plot( q1_cst, x1, s_plot );
  title('Zielpose des Roboters mit strikter Kollisionsvermeidung');
  saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_Collision_Test_Zielpose_KollVermStreng.fig'));
end
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
JP_all_cst = NaN(1+Stats_PosIK_CollStop.iter, size(Tcstack1_cst,1)); % Menge aller Gelenkpositionen
for i = 1:1+Stats_PosIK_CollStop.iter
  [~,JP_i] = RP.fkine_coll(Stats_PosIK_CollStop.Q(i,:)');
  JP_all_cst(i,:) = JP_i;
end
[colldet_cst, colldist_cst] = check_collisionset_simplegeom_mex(RP.collbodies, ...
  RP.collchecks, JP_all_cst, struct('collsearch', true));
assert(all(~colldet_cst(:)), ['Trotz absoluter Kollisionsvermeidung ', ...
  'gibt es Kollisionen in Zwischenschritten']);
test_Q_cst_cav = Stats_PosIK_CollStop.Q - Stats_PosIK_CollAvoid.Q;
if ~any(abs(test_Q_cst_cav(:)) > 1e-6)
  warning(['Ergebnisse für strenge und nur finale Kollisionsvermeidung ', ...
    'sind gleich. Voraussichtlich nie Kollisionen in Zwischenschritten.']);
end

%% Debug-Plots für Positions-IK
t1 = tic();
Namen = {'3T3R', '3T2R', 'FinalAvoidHyp', 'FinalAvoidSquare', 'StrictAvoid'};
for kk = 1:length(Namen)
  if kk == 1
    Q_kk = Stats_PosIK_3T3R.Q;
    h_kk = NaN(size(Q_kk,1),1+9);
  elseif kk == 2
    Q_kk = Stats_PosIK_3T2R.Q;
    h_kk = NaN(size(Q_kk,1),1+9);
  elseif kk == 3
    Q_kk = Stats_PosIK_CollAvoid.Q;
    h_kk = Stats_PosIK_CollAvoid.h;
  elseif kk == 4
    Q_kk = Stats_PosIK_CollAvoid2.Q;
    h_kk = Stats_PosIK_CollAvoid2.h;
  elseif kk == 5
    Q_kk = Stats_PosIK_CollStop.Q;
    h_kk = Stats_PosIK_CollStop.h;
  end
  X1_kk = RP.fkineEE_traj(Q_kk);
  JP_all_kk = NaN(size(Q_kk,1), size(Tcstack1_cav,1)); % Menge aller Gelenkpositionen
  for i = 1:size(Q_kk,1)
    [~,JP_i] = RP.fkine_coll(Q_kk(i,:)');
    JP_all_kk(i,:) = JP_i;
  end
  [colldet_kk, colldist_kk] = check_collisionset_simplegeom_mex(RP.collbodies, ...
    RP.collchecks, JP_all_kk, struct('collsearch', false));
  
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
    legend(Namen);
  end
  linkxaxes
  fhdl=change_current_figure(21);
  if kk == 1
    set(fhdl, 'Name', 'PosIK_Coll', 'NumberTitle', 'off');
    clf;
  end
  subplot(3,1,1); hold on;
  plot(min(colldist_kk,[],2));
  if kk == length(Namen)
    ylabel('Abstand Kollisionskörper (<0 ist Koll.)'); grid on;
  end
  subplot(3,1,2); hold on;
  plot(h_kk(:,1+5));
  if kk == length(Namen)
    ylabel('Zielfunktion Hyp.'); grid on;
  end
  subplot(3,1,3); hold on;
  plot(h_kk(:,1+9));
  if kk == length(Namen)
    ylabel('Zielfunktion Quadr.'); grid on;
  end
  if kk == length(Namen)
    sgtitle('Kollisionsprüfung');
    legend(Namen);
    linkxaxes
    saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_Collision_Test_EinzelPose_Debug_Koll.fig'));
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
    legend(Namen);
    linkxaxes
    saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_Collision_Test_EinzelPose_Debug_X.fig'));
  end
end
fprintf(['Debug-Bilder (Positions-IK) generiert. Dauer: %1.1fs. Gespeichert ', ...
  'nach %s\n'], toc(t1), resdir);
%% Animation der PTP-Bewegungen mit und ohne Kollisionsvermeidung
if usr_create_anim
for k = 1:4
  if k == 1
    Q_t_plot = Stats_PosIK_3T3R.Q(1:1+Stats_PosIK_3T3R.iter,:);
    filesuffix = 'no_collavoidance_3T3R';
    plottitle = 'Inverse Kinematics (3T3R) without Collision Avoidance';
  elseif k == 2
    Q_t_plot = Stats_PosIK_3T2R.Q(1:1+Stats_PosIK_3T2R.iter,:);
    filesuffix = 'no_collavoidance_3T2R';
    plottitle = 'Inverse Kinematics (3T2R) without Collision Avoidance';
  elseif k == 3
    Q_t_plot = Stats_PosIK_CollAvoid.Q(1:1+Stats_PosIK_CollAvoid.iter,:);
    filesuffix = 'with_collavoidance_final';
    plottitle = 'Inverse Kinematics with Final Collision Avoidance';
  elseif k == 4
    Q_t_plot = Stats_PosIK_CollStop.Q(1:1+Stats_PosIK_CollStop.iter,:);
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
  fhdl=figure(9);clf;
  set(fhdl, 'name', sprintf('Anim'), ...
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

%% Trajektorien-IK mit Aufgabenredundanz
% Definieren Trajektorie. Muss relativ geringe Geschwindigkeit haben, damit
% Nullraumbewegung zur Kollisionsvermeidung noch gut funktioniert.
[X,XD,XDD,T] = traj_trapez2_multipoint([x0';x1'], 0.2, 1e-1, 1e-2, 1e-3, 0);

% Mit 3T3R (keine Aufgabenredundanz)
t1 = tic();
RP.update_EE_FG(logical([1 1 1 1 1 1]), logical([1 1 1 1 1 1]));
[Q_3T3R, QD_3T3R, QDD_3T3R, PHI, ~, ~, JP_3T3R, Stats_3T3R] = ...
  RP.invkin2_traj(X,XD,XDD,T,q0);
if usr_test_class
  [Q_3T3R_2, QD_3T3R_2, QDD_3T3R_2, PHI_2, ~, ~, JP_3T3R_2, Stats_3T3R_2] = ...
    RP.invkin_traj(X,XD,XDD,T,q0);
  delta_q_ct = normalizeAngle(Q_3T3R_2-Q_3T3R, 0);
  delta_qDD_ct = QDD_3T3R_2-QDD_3T3R;
  assert(all(abs([delta_q_ct(:);delta_qDD_ct(:)]) < 1e-3), ['Klassen-', ...
    'Implementierung Traj. ungleich (3T3R)']);
  assert(all(abs(PHI_2(:))<1e-8), ['Trajektorie mit 3T3R (Klasse) nicht ', ...
    'erfolgreich berechnet']);
end
fprintf('Trajektorien-IK für 3T3R berechnet. Dauer: %1.1fs\n', toc(t1));
assert(all(abs(PHI(:))<1e-8), 'Trajektorie mit 3T3R nicht erfolgreich berechnet');

% Mit 3T2R (Aufgabenredundanz)
t1 = tic();
RP.update_EE_FG(logical([1 1 1 1 1 1]), logical([1 1 1 1 1 0]));
% Singularitätsvermeidung aktivieren (bzgl. IK-Jacobi-Matrix)
s_traj_3T2R = struct('wn', zeros(RP.idx_ik_length.wntraj,1));
s_traj_3T2R.cond_thresh_ikjac = 1e3;
s_traj_3T2R.wn(RP.idx_iktraj_wnP.ikjac_cond) = 1; % P-Verstärkung Optimierung IK-Jacobi-Konditionszahl
s_traj_3T2R.wn(RP.idx_iktraj_wnD.ikjac_cond) = 0.1; % D-Verstärkung Optimierung IK-Jacobi-Konditionszahl
s_traj_3T2R.wn(RP.idx_iktraj_wnP.qDlim_par) = 0.7; % Zusätzliche Dämpfung gegen Schwingungen
RP.update_EE_FG(logical([1 1 1 1 1 1]), logical([1 1 1 1 1 0]));
[Q_3T2R, QD_3T2R, QDD_3T2R, PHI, ~, ~, JP_3T2R, Stats_3T2R] = ...
  RP.invkin2_traj(X,XD,XDD,T,q0,s_traj_3T2R);
if usr_test_class
  [Q_3T2R_2, QD_3T2R_2, QDD_3T2R_2, PHI_2, ~, ~, JP_3T2R_2, Stats_3T2R_2] = ...
    RP.invkin_traj(X,XD,XDD,T,q0,s_traj_3T2R);
  delta_q_ct = normalizeAngle(Q_3T2R_2-Q_3T2R, 0);
  delta_qDD_ct = QDD_3T2R_2-QDD_3T2R;
  assert(all(abs([delta_q_ct(:);delta_qDD_ct(:)]) < 1e-3), 'Klassen-Implementierung Traj. ungleich (3T2R)');
  assert(all(abs(PHI_2(:))<1e-8), 'Trajektorie mit 3T2R (Klasse) nicht erfolgreich berechnet');
end
fprintf('Trajektorien-IK für 3T2R berechnet. Dauer: %1.1fs\n', toc(t1));
assert(all(abs(PHI(:))<1e-8), 'Trajektorie mit 3T2R nicht erfolgreich berechnet');

% Mit 3T2R (Aufgabenredundanz, Kollisionsvermeidung mit PD-Regler, Variante 1)
t1 = tic();
s_traj_Koll1 = struct('wn', zeros(RP.idx_ik_length.wntraj,1));
s_traj_Koll1 = s_traj_3T2R;
s_traj_Koll1.wn(RP.idx_iktraj_wnP.qDlim_par) = 0.7; % Zusätzliche Dämpfung gegen Schwingungen
% Die Einstellung der Parameter ist abhängig von den Maximalwerten für qDD
s_traj_Koll1.wn(RP.idx_iktraj_wnP.coll_hyp) = 0.1; % P-Verstärkung Kollisionsvermeidung
s_traj_Koll1.wn(RP.idx_iktraj_wnD.coll_hyp) = 0.01; % D-Verstärkung Kollisionsvermeidung
RP.update_EE_FG(logical([1 1 1 1 1 1]), logical([1 1 1 1 1 0]));
[Q_Koll1, QD_Koll1, QDD_Koll1, PHI, ~, ~, JP_Koll1, Stats_Koll1] = ...
  RP.invkin2_traj(X,XD,XDD,T,q0,s_traj_Koll1);
assert(all(abs(PHI(:))<1e-8), 'Trajektorie mit Kollisionsvermeidung Var. 1 nicht erfolgreich berechnet');
if usr_test_class
  [Q_Koll1_2, QD_Koll1_2, QDD_Koll1_2, ~, ~, ~, JP_Koll1_2, Stats_Koll1_2] = ...
    RP.invkin_traj(X,XD,XDD,T,q0,s_traj_Koll1);
  delta_q_ct = normalizeAngle(Q_Koll1_2-Q_Koll1, 0);
  delta_qDD_ct = QDD_Koll1_2-QDD_Koll1;
  assert(all(abs(delta_q_ct(:)) < 1e-3), ['Klassen-Implementierung Traj. ', ...
    'ungleich (Kollisionsvermeidung Var. 1)']);
end
fprintf('Trajektorien-IK mit starker Kollisionsvermeidung berechnet. Dauer: %1.1fs\n', toc(t1));
[coll_Koll1, dist_Koll1] = check_collisionset_simplegeom_mex(RP.collbodies, ...
  RP.collchecks, JP_Koll1, struct('collsearch', false));
if any(coll_Koll1(:)) % TODO: Mit assert
  warning(['Trotz Kollisionsvermeidung (Var. 1) ', ...
  'gibt es Kollisionen in Zwischenschritten']);
end

% Mit 3T2R (Aufgabenredundanz, Kollisionsvermeidung mit PD-Regler, Variante 2)
% Benutze nur eine sehr schwache Auslegung des Reglers um zu zeigen, dass
% die Vermeidung dann nicht funktioniert.
t1 = tic();
s_traj_Koll2 = s_traj_3T2R;
s_traj_Koll2.wn(RP.idx_iktraj_wnP.coll_hyp) = 1e-4; % P-Verstärkung Kollisionsvermeidung
s_traj_Koll2.wn(RP.idx_iktraj_wnD.coll_hyp) = 1e-3; % D-Verstärkung Kollisionsvermeidung
RP.update_EE_FG(logical([1 1 1 1 1 1]), logical([1 1 1 1 1 0]));
[Q_Koll2, QD_Koll2, QDD_Koll2, PHI, ~, ~, JP_Koll2, Stats_Koll2] = ...
  RP.invkin2_traj(X,XD,XDD,T,q0,s_traj_Koll2);
assert(all(abs(PHI(:))<1e-8), ['Trajektorie mit Kollisionsvermeidung ', ...
  'Var. 2 nicht erfolgreich berechnet']);
if usr_test_class
  [Q_Koll2_2, QD_Koll2_2, QDD_Koll2_2, ~, ~, ~, JP_Koll2_2, Stats_Koll2_2] = ...
    RP.invkin_traj(X,XD,XDD,T,q0,s_traj_Koll2);
  delta_q_ct = normalizeAngle(Q_Koll2_2-Q_Koll2, 0);
  delta_qDD_ct_abs = QDD_Koll2_2-QDD_Koll2;
  delta_qDD_ct_rel = delta_qDD_ct_abs ./ QDD_Koll2;
  % Komplexerer Prüfterm, da beide Ergebnisse leicht divergieren
  I_qDD_iO = abs(delta_qDD_ct_abs) < 1e-3 | abs(delta_qDD_ct_rel)<5e-2;
  I_q_iO = abs(delta_q_ct) < 1e-2;
  assert(all(I_qDD_iO(:)|I_q_iO(:)), ['Klassen-Implementierung Traj. ', ...
    'ungleich (Kollisionsvermeidung Var. 2)']);
end
fprintf('Trajektorien-IK mit schwacher Kollisionsvermeidung berechnet. Dauer: %1.1fs\n', toc(t1));
[coll_Koll2, dist_Koll2] = check_collisionset_simplegeom_mex(RP.collbodies, ...
  RP.collchecks, JP_Koll2, struct('collsearch', false));
if any(coll_Koll2(:))
  % TODO: Beispiel so einstellen, dass es mit dieser IK-Einstellung
  % funktioniert. Dann assert
  warning('Trotz Kollisionsvermeidung (Var. 2) gibt es Kollisionen in Zwischenschritten');
end

% Mit 3T2R (Aufgabenredundanz, Kollisionsvermeidung mit PD-Regler, größerer
% Aktivitätsbereich der Kennzahl)
t1 = tic();
s_traj_Koll3 = s_traj_3T2R;
s_traj_Koll3.wn(RP.idx_iktraj_wnP.coll_hyp) = 0.8; % P-Verstärkung Kollisionsvermeidung
s_traj_Koll3.wn(RP.idx_iktraj_wnD.coll_hyp) = 0.2; % D-Verstärkung Kollisionsvermeidung
s_traj_Koll3.collbodies_thresh = 3; % 200% größere Kollisionskörper für Aktivierung
RP.update_EE_FG(logical([1 1 1 1 1 1]), logical([1 1 1 1 1 0]));
[Q_Koll3, QD_Koll3, QDD_Koll3, PHI, ~, ~, JP_Koll3, Stats_Koll3] = RP.invkin2_traj(X,XD,XDD,T,q0,s_traj_Koll3);
assert(all(abs(PHI(:))<1e-8), 'Trajektorie mit Kollisionsvermeidung Var. 3 nicht erfolgreich berechnet');
if usr_test_class
  [Q_Koll3_2, QD_Koll3_2, QDD_Koll3_2, PHI_2, ~, ~, JP_Koll3_2, Stats_Koll3_2] = RP.invkin_traj(X,XD,XDD,T,q0,s_traj_Koll3);
  delta_q_ct = normalizeAngle(Q_Koll3_2-Q_Koll3, 0);
  delta_qDD_ct_abs = QDD_Koll3_2-QDD_Koll3;
  delta_qDD_ct_rel = delta_qDD_ct_abs ./ QDD_Koll3;
  % Komplexerer Prüfterm, da beide Ergebnisse leicht divergieren
  I_qDD_iO = abs(delta_qDD_ct_abs) < 1e-3 | abs(delta_qDD_ct_rel)<5e-2;
  I_q_iO = abs(delta_q_ct) < 1e-2;
  if ~all(I_qDD_iO(:)|I_q_iO(:)) % TODO: Nochmal prüfen, ob korrigierbar
    warning(['Klassen-Implementierung Traj. ', ...
    'ungleich (Kollisionsvermeidung Var. 3)']);
  end
end
fprintf('Trajektorien-IK mit starker Kollisionsvermeidung und früher Aktivierung berechnet. Dauer: %1.1fs\n', toc(t1));
[coll_Koll3, dist_Koll3] = check_collisionset_simplegeom_mex(RP.collbodies, ...
  RP.collchecks, JP_Koll3, struct('collsearch', false));
if any(coll_Koll3(:)) % TODO: Mit assert
  warning(['Trotz Kollisionsvermeidung (Var. 3) ', ...
  'gibt es Kollisionen in Zwischenschritten']);
end

% Mit 3T2R (Aufgabenredundanz, Quadratische Kollisionsvermeidung mit PD-Regler)
t1 = tic();
s_traj_Koll4 = s_traj_3T2R;
s_traj_Koll4.wn(RP.idx_iktraj_wnP.coll_par) = 0.8; % P-Verstärkung Kollisionsvermeidung
s_traj_Koll4.wn(RP.idx_iktraj_wnD.coll_par) = 0.2; % D-Verstärkung Kollisionsvermeidung
RP.update_EE_FG(logical([1 1 1 1 1 1]), logical([1 1 1 1 1 0]));
[Q_Koll4, QD_Koll4, QDD_Koll4, PHI, ~, ~, JP_Koll4, Stats_Koll4] = RP.invkin2_traj(X,XD,XDD,T,q0,s_traj_Koll4);
% assert(all(abs(PHI(:))<1e-8), 'Trajektorie mit Kollisionsvermeidung Var. 4 nicht erfolgreich berechnet');
if ~all(abs(PHI(:))<1e-8)
  warning('Trajektorie mit Kollisionsvermeidung Var. 4 nicht erfolgreich berechnet');
end
if usr_test_class
  [Q_Koll4_2, QD_Koll4_2, QDD_Koll4_2, PHI_2, ~, ~, JP_Koll4_2, Stats_Koll4_2] = RP.invkin_traj(X,XD,XDD,T,q0,s_traj_Koll4);
  delta_q_ct = normalizeAngle(Q_Koll4_2-Q_Koll4, 0);
  delta_qDD_ct_abs = QDD_Koll4_2-QDD_Koll4;
  delta_qDD_ct_rel = delta_qDD_ct_abs ./ QDD_Koll4;
  % Komplexerer Prüfterm, da beide Ergebnisse leicht divergieren
  I_qDD_iO = abs(delta_qDD_ct_abs) < 1e-3 | abs(delta_qDD_ct_rel)<5e-2;
  I_q_iO = abs(delta_q_ct) < 1e-2;
  if ~all(I_qDD_iO(:)|I_q_iO(:)) % TODO: Nochmal prüfen, ob korrigierbar
    warning(['Klassen-Implementierung Traj. ', ...
    'ungleich (Kollisionsvermeidung Var. 4)']);
  end
end
fprintf('Trajektorien-IK mit quadratischer Kollisionsvermeidung berechnet. Dauer: %1.1fs\n', toc(t1));
[coll_Koll4, dist_Koll4] = check_collisionset_simplegeom_mex(RP.collbodies, ...
  RP.collchecks, JP_Koll4, struct('collsearch', false));
if any(coll_Koll4(:)) % TODO: Mit assert
  warning(['Trotz Kollisionsvermeidung (Var. 4) ', ...
  'gibt es Kollisionen in Zwischenschritten']);
end
%% Debug-Plots für Trajektorien-IK
% Definiere die vergrößerten Warn-Kollisionskörper nochmal neu. Muss
% konsistent zu den IK-Funktionen sein.
collbodies_ns = RP.collbodies;
collbodies_ns.params(collbodies_ns.type==6,1) = ... % Kapseln (Direktverbindung)
  1.5*collbodies_ns.params(collbodies_ns.type==6,1); % Faktor 1.5 Standard-Wert. In einer Variante auf 2 gesetzt
collbodies_ns.params(collbodies_ns.type==13,7) = ... % Kapseln (Basis-KS)
  1.5*collbodies_ns.params(collbodies_ns.type==13,7);
collbodies_ns.params(collbodies_ns.type==4|collbodies_ns.type==15,4) = ... % Kugeln
  1.5*collbodies_ns.params(collbodies_ns.type==4|collbodies_ns.type==15,4);
maxcolldepth = max([RP.collbodies.params(RP.collbodies.type==6,1);  ...
                      RP.collbodies.params(RP.collbodies.type==13,7); ...
                      RP.collbodies.params(RP.collbodies.type==4|collbodies_ns.type==15,4)]);
maxcolldepth_ns = max([collbodies_ns.params(collbodies_ns.type==6,1);  ...
                      collbodies_ns.params(collbodies_ns.type==13,7); ...
                      collbodies_ns.params(collbodies_ns.type==4|collbodies_ns.type==15,4)]);
collobjdist_thresh = 0.3*maxcolldepth_ns;

t1 = tic();
Namen = {'3T3R', '3T2R', 'CollAvoid1', 'CollAvoid2', 'CollAvoid3_wide', 'CollAvoid4_square'};
for kk = 1:length(Namen)
  if kk == 1
    Q_kk = Q_3T3R; QD_kk = QD_3T3R; QDD_kk = QDD_3T3R; JP_all_kk = JP_3T3R;
    h_kk = NaN(size(Q_kk,1),1+12);
  elseif kk == 2
    Q_kk = Q_3T2R; QD_kk = QD_3T2R; QDD_kk = QDD_3T2R; JP_all_kk = JP_3T2R;
    h_kk = NaN(size(Q_kk,1),1+12);
  elseif kk == 3
    Q_kk = Q_Koll1; QD_kk = QD_Koll1; QDD_kk = QDD_Koll1; JP_all_kk = JP_Koll1;
    h_kk = Stats_Koll1.h;
  elseif kk == 4
    Q_kk = Q_Koll2; QD_kk = QD_Koll2; QDD_kk = QDD_Koll2; JP_all_kk = JP_Koll2;
    h_kk = Stats_Koll2.h;
  elseif kk == 5
    Q_kk = Q_Koll3; QD_kk = QD_Koll3; QDD_kk = QDD_Koll3; JP_all_kk = JP_Koll3;
    h_kk = Stats_Koll3.h;
  elseif kk == 6
    Q_kk = Q_Koll4; QD_kk = QD_Koll4; QDD_kk = QDD_Koll4; JP_all_kk = JP_Koll4;
    h_kk = Stats_Koll4.h;
  end
  % Reduziere die Einträge, falls die Trajektorie nicht erfolgreich war.
  I_firstnan = min([find(any(isnan(Q_kk),2),1,'first'); size(Q_kk,1)]);
  T_kk = T(1:I_firstnan); Q_kk = Q_kk(1:I_firstnan,:);
  QD_kk = QD_kk(1:I_firstnan,:);  QDD_kk = QDD_kk(1:I_firstnan,:);
  JP_all_kk = JP_all_kk(1:I_firstnan,:); h_kk = h_kk(1:I_firstnan,:);
%   [colldet_kk, colldist_kk] = check_collisionset_simplegeom_mex(RP.collbodies, ...
%     RP.collchecks, JP_all_kk, struct('collsearch', false));
  [X1_kk, XD1_kk, XDD1_kk] = RP.fkineEE2_traj(Q_kk, QD_kk, QDD_kk);
  X1_kk(:,4:6) = denormalize_angle_traj(X1_kk(:,4:6));
  [colldet_kk, colldist_kk] = check_collisionset_simplegeom_mex(RP.collbodies, ...
    RP.collchecks, JP_all_kk, struct('collsearch', false));
  [colldet_ns_kk, colldist_ns_kk] = check_collisionset_simplegeom_mex(collbodies_ns, ...
    RP.collchecks, JP_all_kk, struct('collsearch', false));
  h_coll_post = zeros(size(h_kk,1),2);
  for i = 1:size(h_kk,1)
    if any(colldet_ns_kk(i,:))
      h_coll_post(i,1) = invkin_optimcrit_limits2(-min(colldist_kk(i,:)), ... % zurückgegebene Distanz ist zuerst negativ
        [-100*maxcolldepth, 0], [-80*maxcolldepth, -collobjdist_thresh]);
      h_coll_post(i,2) = invkin_optimcrit_limits1(-min(colldist_kk(i,:)), ...
        [-100*maxcolldepth, 0]);
      % Test:
%       invkin_optimcrit_limits2(-collobjdist_thresh, ... % zurückgegebene Distanz ist zuerst negativ
%         [-100*maxcolldepth, maxcolldepth], [-80*maxcolldepth, -collobjdist_thresh]);
    end
  end
  test_hcoll = h_coll_post - h_kk(:,1+[7 12]);
%   assert(all(abs(test_hcoll) < 1e-6), 'Nachträglich berechnete Kollisions-Kennzahl stimmt nicht');
  
  fhdl=change_current_figure(40);
  if kk == 1
    set(fhdl, 'Name', 'TrajIK_Q', 'NumberTitle', 'off');
    clf;
  end
  ii = 0;
  for i = 1:RP.NLEG
    for j = 1:RP.Leg(i).NJ
      ii = ii + 1;
      subplot(RP.NLEG,RP.Leg(1).NJ,sprc2no(RP.NLEG, RP.Leg(1).NJ, i, j));
      hold on; grid on;
      plot(T_kk, Q_kk(:,ii));
      if j == 1, ylabel(sprintf('BK %d', i)); end
      if i == 1, title(sprintf('q %d', j)); end
    end
  end
  if kk == length(Namen)
    sgtitle('Gelenkkoordinaten');
    legend(Namen, 'interpreter', 'none');
    linkxaxes
    saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_Collision_Test_Traj_Debug_Q.fig'));
  end
  fhdl=change_current_figure(41);
  if kk == 1
    set(fhdl, 'Name', 'TrajIK_QD', 'NumberTitle', 'off');
    clf;
  end
  ii = 0;
  for i = 1:RP.NLEG
    for j = 1:RP.Leg(i).NJ
      ii = ii + 1;
      subplot(RP.NLEG,RP.Leg(1).NJ,sprc2no(RP.NLEG, RP.Leg(1).NJ, i, j));
      hold on; grid on;
      plot(T_kk, QD_kk(:,ii));
      if j == 1, ylabel(sprintf('BK %d', i)); end
      if i == 1, title(sprintf('qD %d', j)); end
    end
  end
  if kk == length(Namen)
    sgtitle('Gelenkgeschwindigkeiten');
    legend(Namen, 'interpreter', 'none');
    linkxaxes
    saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_Collision_Test_Traj_Debug_QD.fig'));
  end
  fhdl=change_current_figure(42);
  if kk == 1
    set(fhdl, 'Name', 'TrajIK_QDD', 'NumberTitle', 'off');
    clf;
  end
  ii = 0;
  for i = 1:RP.NLEG
    for j = 1:RP.Leg(i).NJ
      ii = ii + 1;
      subplot(RP.NLEG,RP.Leg(1).NJ,sprc2no(RP.NLEG, RP.Leg(1).NJ, i, j));
      hold on; grid on;
      plot(T_kk, QDD_kk(:,ii));
      if j == 1, ylabel(sprintf('BK %d', i)); end
      if i == 1, title(sprintf('qDD %d', j)); end
    end
  end
  if kk == length(Namen)
    sgtitle('Gelenkbeschleunigungen');
    legend(Namen, 'interpreter', 'none');
    linkxaxes
    saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_Collision_Test_Traj_Debug_QDD.fig'));
  end
  fhdl=change_current_figure(43);
  if kk == 1
    set(fhdl, 'Name', 'TrajIK_Coll', 'NumberTitle', 'off');
    clf;
  end
  subplot(2,2,1); hold on;
  plot(T_kk, min(colldist_kk,[],2));
  if kk == length(Namen)
    ylabel('Abstand Kollisionskörper (<0 ist Koll.)'); grid on;
  end
  subplot(2,2,2); hold on;
  plot(T_kk, min(colldist_ns_kk,[],2));
  if kk == length(Namen)
    ylabel('Abstand vergrößerte Kollisionskörper (<0 ist Koll.)'); grid on;
  end
  subplot(2,2,3); hold on;
  plot(T_kk, h_kk(:,1+7));
  if kk == length(Namen)
    ylabel('Zielfunktion (online)'); grid on;
  end
  subplot(2,2,4); hold on;
  plot(T_kk, h_coll_post);
  if kk == length(Namen)
    ylabel('Zielfunktion (offline)'); grid on;
  end
  if kk == length(Namen)
    sgtitle('Kollisionsprüfung');
    legend(Namen, 'interpreter', 'none');
    linkxaxes
    saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_Collision_Test_Traj_Debug_Koll.fig'));
  end
  
  fhdl=change_current_figure(44);
  if kk == 1
    set(fhdl, 'Name', 'TrajIK_X', 'NumberTitle', 'off');
    clf;
  end
  for i = 1:6
    subplot(3,6,sprc2no(3,6,1,i)); hold on;
    plot(T_kk, X1_kk(:,i));
    if kk == length(Namen)
      ylabel(sprintf('x %d', i)); grid on;
    end
    subplot(3,6,sprc2no(3,6,2,i)); hold on;
    plot(T_kk, XD1_kk(:,i));
    if kk == length(Namen)
      ylabel(sprintf('xD %d', i)); grid on;
    end
    subplot(3,6,sprc2no(3,6,3,i)); hold on;
    plot(T_kk, XDD1_kk(:,i));
    if kk == length(Namen)
      ylabel(sprintf('xDD %d', i)); grid on;
    end
  end
  if kk == length(Namen)
    sgtitle('Plattform-Koordinaten (Beinkette 1)');
    legend(Namen, 'interpreter', 'none');
    linkxaxes
    saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_Collision_Test_Traj_Debug_X.fig'));
  end
  fprintf('Debug-Plots für Trajektorie %d/%d gezeichnet.\n', kk, length(Namen));

  fhdl=change_current_figure(45);
  if kk == 1
    set(fhdl, 'Name', 'TrajIK_Cond', 'NumberTitle', 'off');
    clf;
  end
  subplot(2,1,1); hold on;
  plot(T_kk, h_kk(:,1+5));
  if kk == length(Namen)
    ylabel('Konditionszahl IK-Jacobi'); grid on;
  end
  subplot(2,1,2); hold on;
  plot(T_kk, h_kk(:,1+6));
  if kk == length(Namen)
    ylabel('Konditionszahl PKM-Jacobi'); grid on;
  end
  if kk == length(Namen)
    sgtitle('Konditionszahlen (Singularitätsvermeidung)');
    legend(Namen, 'interpreter', 'none');
    linkxaxes
    saveas(fhdl, fullfile(resdir, 'ParRob_Nullspace_Collision_Test_Traj_Debug_Cond.fig'));
  end
end
fprintf(['Debug-Bilder für Trajektorien-IK generiert. Dauer: %1.1fs. ', ...
  'Gespeichert nach %s\n'], toc(t1), resdir);

%% Animation der Trajektorien-Bewegungen mit und ohne Kollisionsvermeidung
if usr_create_anim
for k = 1:length(Namen)
  if k == 1
    Q_t_plot = Q_3T3R;
    filesuffix = 'no_collavoidance_3T3R';
    plottitle = 'Inverse Kinematics (3T3R) without Collision Avoidance';
  elseif k == 2
    Q_t_plot = Q_3T2R;
    filesuffix = 'no_collavoidance_3T2R';
    plottitle = 'Inverse Kinematics (3T2R) without Collision Avoidance';
  elseif k == 3
    Q_t_plot = Q_Koll1;
    filesuffix = 'with_collavoidance_strong';
    plottitle = 'Inverse Kinematics with Strong Collision Avoidance';
  elseif k == 4
    Q_t_plot = Q_Koll2;
    filesuffix = 'with_collavoidance_weak';
    plottitle = 'Inverse Kinematics with Weak Collision Avoidance';
  elseif k == 5
    Q_t_plot = Q_Koll3;
    filesuffix = 'with_collavoidance_strong_early';
    plottitle = 'Inverse Kinematics with Strong and Early Collision Avoidance';
  elseif k == 6
    Q_t_plot = Q_Koll4;
    filesuffix = 'with_collavoidance_strong_square';
    plottitle = 'Inverse Kinematics with Strong and Quadratic Collision Avoidance';
  end
  X_t_plot = RP.fkineEE2_traj(Q_t_plot);
  maxduration_animation = 5; % Dauer des mp4-Videos in Sekunden
  t_Vid = (0:1/30*(T(end)/maxduration_animation):T(end))';
  I_anim = knnsearch( T , t_Vid );
  I_anim = [I_anim; repmat(length(T),15,1)]; %#ok<AGROW> % 15 Standbilder (0.5s) mit letztem Wert
  
  anim_filename = fullfile(resdir, sprintf('ParRob_Nullspace_Collision_Test_Traj_%s', filesuffix));
  s_anim = struct( 'mp4_name', [anim_filename,'.mp4'] );
  s_plot = struct( 'ks_legs', [], 'straight', 0, 'mode', 5, 'only_bodies', true);
  fhdl=figure(9);clf;
  set(fhdl, 'name', sprintf('Anim'), ...
    'color','w', 'NumberTitle', 'off', 'units','normalized',...
    'outerposition',[0 0 1 1]); % Vollbild, damit Video größer wird
  trplot(RP.x2t(x1), 'frame', 'D', 'rgb', 'length', 0.2);
  plot3(X_t_plot(:,1), X_t_plot(:,2), X_t_plot(:,3), 'c-');
  hold on; grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
  view(3); % 3D-Ansicht
  title(plottitle);
  RP.anim( Q_t_plot(I_anim,:), X_t_plot(I_anim,:), s_anim, s_plot);
end
end
