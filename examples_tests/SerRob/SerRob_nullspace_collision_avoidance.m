% Teste die Kollisionsvermeidung in der Nullraumbewegung für serielle
% (Industrie-)Roboter in verschiedenen Fällen
% 
% Akademisches Beispiel: z-Achse zeigt seitlich aus Roboter-Flansch heraus
% und ist die Symmetrieachse. Drehung um diese Achse wird für
% Kollisionsvermeidung genutzt.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

rob_path = fileparts(which('robotics_toolbox_path_init.m'));
resdir = fullfile(rob_path, 'examples_tests', 'results');
%% Initialisierung
SName='S6RRRRRR10V2';
RName='S6RRRRRR10V2_KUKA1';
RS = serroblib_create_robot_class(SName, RName);
RS.update_EE([0;0;-0.05], [pi/2;0;-pi/2]); % z-Achse aus Zeichenebene, x-Achse aus EE heraus

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

% Kollisionsprüfung für verschiedene Endposen
[colldet,colldist] = check_collisionset_simplegeom(RS.collbodies, RS.collchecks, ...
  [Tcstack1_cst(:,4)'; Tcstack1_cav(:,4)'], struct('collsearch', true));
assert(~any(colldet(:)), 'Trotz Kollisionsvermeidungsstrategie gibt es in Endpose eine Kollision');
% Prüfe Kollisionsprüfung in Zwischenschritten
assert(all(Stats_CollStop.h(1:1+Stats_CollStop.iter,5)<=0), ...
  'Trotz absoluter Kollisionsvermeidung gibt es Kollisionen in Zwischenschritten');

%% Animation der Bewegungen mit und ohne Kollisionsvermeidung
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
  
  anim_filename = fullfile(resdir, sprintf('Nullspace_Collision_Test_%s', filesuffix));
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
%% Trajektorien-IK mit Aufgabenredundanz
% TODO