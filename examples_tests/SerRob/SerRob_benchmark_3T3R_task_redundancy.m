% Benchmark-Test für Aufgabenredundanz (3T2R) bei Industrieroboter (3T3R)
% Testet und vergleicht die Funktion SerRob/invkin2 für verschiedene Einst.
% 
% Ablauf:
% * Roboter definieren
% * Beispiel-Trajektorie definieren
% * IK für Eckpunkte der Trajektorie mit verschiedenen Einstellungen
% * IK für ganze Trajektorie mit verschiedenen Einstellungen
% * Bilder zeichnen
% 
% Getestete Methoden für IK der Trajektorie:
% Nullraum-Optimierung (NRO), Geschwindigkeitsgrenzen (GG)
% * 1: 3T2R, NRO, GG
% * 2: 3T2R, NRO
% * 3: 3T2R, GG (ohne Grenze für Beschleunigung)
% * 4: 3T2R, GG
% * 5: 3T2R, GG, NRO für Konditionszahl
% * 6: 3T2R, NRO, GG. Jacobi-Zeitableitung mit Differenzenquotient
% * 7: 3T3R: Benutze symmetrischen FG aus globaler Optimierung für Eckpunkte
% 
% Ergebnis:
% * Mit Aufgabenredundanz (M.1) wird die Zielfunktion (Einhaltung der Gelenk-
%   Grenzen, Position und Geschwindigkeit) am besten erfüllt.
% * Ohne NRO auf Geschwindigkeitsebene kommt es durch GG zu Oszillationen
%   (M3)
% * Differenzenquotient stellt fast keinen Unterschied dar (M4 vs M1)
% 
% Erzeugt Bilder: Nummerierung mit 100er-Stelle für Nr. des Roboters
% * 0: Eckpunkte der Trajektorie
% * 1: Zielfunktion für Einzelpunkte
% * 2: Bild des Roboters in Startpose
% * 20: Gesamt-Zielfunktion (Position)
% * 21: Gesamt-Zielfunktion (Geschwindigkeit)
% * 23: Gelenkwinkelverlauf
% * 41-44: Prüfung der Konsistenz von Position/Geschwindigkeit/Beschl. (q)
% * 51-54: Konsistenz von x/xD/xDD
% * 60,61: Untersuchung zu unterschiedlichen Abtastzeiten der Trajektorie
%   Es sollte bei jeder Abtastzeit das gleiche Ergebnis herauskommen.

% Siehe auch:
% * ParRob_benchmark_3T3R_task_redundancy.m
% * [1] serrob_traj.m; github.com/SchapplM/robsynth-paper_mdpirobotics2019

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

if isempty(which('serroblib_path_init.m'))
  warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end

%% Definitionen, Benutzereingaben
% Benutzereingaben zur Ablaufsteuerung
short_traj = true; % Trajektorie stark abkürzen, um prinzipielle Funktionalität zu zeigen. TODO: Noch singulär mit ganzer Länge.
eckpunkte_berechnen = true; % Einzelpunkt-IK für alle Eckpunkte berechnen
debug_plot = false;
use_mex_functions = true;

% Definitionen
rob_path = fileparts(which('robotics_toolbox_path_init.m'));
respath = fullfile(rob_path, 'examples_tests', 'results', 'SerRob_3T3R_task_red_benchmark');
mkdirs(respath);
I_EE_3T3R = logical([1 1 1 1 1 1]);
I_EE_3T2R = logical([1 1 1 1 1 0]);

% Formatierung der Linien der verschiedenen Methoden
format_mlines = { 'r', 'v', '-', 8; ...
                  'g', 'd', '-', 5; ...
                  'b', 's', '--', 7; ...
                  'k', 'x', '--', 9; ...
                  'm', 'o', ':', 6; ...
                  'c', '^', '-', 3; ...
                  'r', '+', ':', 6};

% Endeffektor-Transformation ungleich Null festlegen, um zu prüfen, ob die
% Implementierung davon richtig ist
r_W_E = [0.1;0.2;0.3];
phi_W_E = [20; 40; 50]*pi/180;

Robots = {{'S6RRRRRR10V2', 'S6RRRRRR10V2_FANUC6'}};
%% Alle Robotermodelle durchgehen
for robnr = 1
  Robot_Data = Robots{robnr};
  SName = Robot_Data{1};
  RName = Robot_Data{2};
  
  %% Funktionen ggf neu generieren
  % Kann händisch aktiviert werden, wenn Änderungen durchgeführt werden:
%   serroblib_create_template_functions({SName}, false);
  
  %% Klasse für seriellen Roboter erstellen
  % Instanz der Roboterklasse erstellen
  RS = serroblib_create_robot_class(SName, RName);
  RS.fill_fcn_handles(use_mex_functions, true);
  % matlabfcn2mex({'S6RRRRRR10V2_invkin_traj','S6RRRRRR10V2_invkin_eulangresidual'});
  
  % Grenzen festlegen (für Zusatz-Optimierung)
  RS.qlim = repmat([-pi, pi], RS.NQJ, 1);
  RS.qDDlim = repmat([-100, 100], RS.NQJ, 1); % Entspricht 1.5 rpm in 100ms
  RS.update_EE(r_W_E, phi_W_E, []);

  fprintf('Starte Untersuchung für %s\n', RS.descr);
  
  %% Startpose bestimmen
  s = struct('n_max', 1000, 'Phit_tol', 1e-12, 'Phir_tol', 1e-12, ...
    'reci', true, 'scale_lim', 0); % keine Vorgabe von K oder Kn (Standard-Werte)
  % Werte aus [1]. TODO: Anpassen, falls andere Roboter getestet werden.
  x0Ref = NaN(6,1);
  T_part = transl([1.15; 0.2; -0.2;]);
  x0Ref(1:3) = T_part(1:3,4) + [-0.200;0;0.400];
  x0Ref(4:6) = [pi;0;0];
  X0 = x0Ref;
  q0 = zeros(6,1);
  q0(2) = pi/2;
  q0_ik_fix = q0 + [0;25;-35;0;15;0]*pi/180;
  [qs, Phi] = RS.invkin2(RS.x2tr(X0), q0_ik_fix, s);
  if any(abs(Phi) > 1e-8)
    error('Inverse Kinematik konnte in Startpose nicht berechnet werden');
  end
  s_ep = struct( ...
    'n_min', 1000, 'n_max', 2500, 'Phit_tol', 1e-7, 'Phir_tol', 1e-7, ...
    'reci', true, 'wn', [1;0]);% keine Vorgabe von K oder Kn (Standard-Werte)
  %% Initialisierung Teil 2
  % Roboter auf 3T2R einstellen
  RS.I_EE_Task = I_EE_3T2R;

  %% Eckpunkte für Beispiel-Trajektorie bestimmen und IK prüfen
  % Würfel-Trajektorie (Kantenlänge 300mm)
  traj_d=0.15; traj_h=0.15;
  X1 = X0+[-traj_d/2,traj_d/2,traj_h/2,0,0,0]'; % Start so, dass Würfel genau mittig ist
  k=1; XL = X1';
  k=k+1; XL(k,:) = XL(k-1,:) + [ traj_d,0,0, 0,0,0];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,-traj_d,0  0,0, pi/4];
  k=k+1; XL(k,:) = XL(k-1,:) + [-traj_d,0,0, 0,0,-pi/4];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,0,-traj_h, pi/4,0, 0];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,0, traj_h, -pi/4,0,0];
  if ~short_traj
  k=k+1; XL(k,:) = XL(k-1,:) + [0,traj_d,0,  0,pi/4,0];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,0,-traj_h, 0,-pi/4,0];
  k=k+1; XL(k,:) = XL(k-1,:) + [ traj_d,0,0, pi/6,-pi/6,0];
  k=k+1; XL(k,:) = XL(k-1,:) + [ 0,0,traj_h, pi/6,-pi/6,0];
  k=k+1; XL(k,:) = XL(k-1,:) + [ 0,0,-traj_h, -pi/6,pi/3,0];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,-traj_d,0  -pi/6,pi/6,0];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,0, traj_h, -pi/4,pi/4,-pi/3];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,0,-traj_h, pi/2,-pi/6,-pi/3];
  k=k+1; XL(k,:) = XL(k-1,:) + [-traj_d,0,0, pi/12,-pi/6,pi/2];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,traj_d,0,  -pi/4,0,-pi/6];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,0, traj_h, -pi/12,pi/12,pi/3];
  end
  % Debug: Zeichne Eckpunkte
  if debug_plot
    change_current_figure(100*robnr+0);clf; hold all; view(3); %#ok<UNRCH>
    plot3(1e3*X0(1), 1e3*X0(2), 1e3*X0(3), 'g^');
    plot3(1e3*X1(1), 1e3*X1(2), 1e3*X1(3), 'mv');
    plot3(1e3*XL(:,1), 1e3*XL(:,2), 1e3*XL(:,3), 'r-');
    for jj = 1:size(XL,1)
      text(1e3*XL(jj,1), 1e3*XL(jj,2), 1e3*XL(jj,3)+3*jj, sprintf('%d', jj));
    end
    xlabel('x in mm');ylabel('y in mm');zlabel('z in mm');
  end
  % Berechne IK zu den einzelnen Eckpunkten. Wenn das nicht geht, bringt die
  % Trajektorie sowieso nichts. Benutze die IK mit Aufgabenredundanz 
  if eckpunkte_berechnen
    % Teil 1
    h1 = NaN(size(XL,1),1);
    h2 = NaN(size(XL,1),1);
    nsteps_angle = 180;
    h1_go = NaN(size(XL,1),nsteps_angle);
    h2_go = NaN(size(XL,1),nsteps_angle);
    t0 = tic();
    for i = 1:size(XL,1)
      t1 = tic();
      % Berechnung mit aus Vorlagendatei generierter Funktion
      [q_i, Phi_i] = RS.invkin2(RS.x2tr(XL(i,:)'), qs, s_ep);
      fprintf('Eckpunkt %d/%d berechnet. Dauer %1.1fs (tpl-Funktion). Bis hier %1.1fs.\n', ...
        i, size(XL,1), toc(t1), toc(t0));
      if max(abs(Phi_i)) > 1e-6
        error('Eckpunkt %d geht nicht', i);
      end
      h1(i) = invkin_optimcrit_limits1(q_i, RS.qlim);
      h2(i) = invkin_optimcrit_limits2(q_i, RS.qlim);
      if any(isinf(h2(i)))
        warning('Grenzverletzung bei Eckpunkt %d (kann an Einstellungen liegen)', i);
      end
      % Bestimme best- und schlechtmöglichstes IK-Ergebnis (ohne Aufgabenredundenz)
      t3 = tic();
      RS.I_EE_Task = I_EE_3T3R;
      s_ep_3T3R = s_ep; % Neue Konfiguration für Einzelpunkt-IK mit 3T3R
      s_ep_3T3R.I_EE = I_EE_3T3R; % keine Aufgabenredundanz
      s_ep_3T3R.scale_lim = 0; % Grenzen ignorieren. Gibt eh nur eine Lösung
      qs_bruteforce = qs;
      for j = 1:nsteps_angle % in 2-Grad-Schritten durchgehen
        x = XL(i,:)';
        x(6) = 360/nsteps_angle*(j-1)*pi/180; % EE-Drehung vorgeben
        [q_i_kls, Phi_i_kls] = RS.invkin2(RS.x2tr(x), qs_bruteforce, s_ep_3T3R);
        qs_bruteforce = q_i_kls;
        if max(abs(Phi_i_kls)) > 1e-6
          error('Eckpunkt %d geht nicht', i);
        end
        % Berechne Zielkriterien
        h1_go(i,j) = invkin_optimcrit_limits1(q_i_kls, RS.qlim);
        h2_go(i,j) = invkin_optimcrit_limits2(q_i_kls, RS.qlim);
        if any(isinf(h2_go(i)))
          warning('Grenzverletzung (kls) bei Eckpunkt %d (kann an Einstellungen liegen)', i);
        end
        % Speichere den besten Punkt für die 3T3R-IK ab
        [~,j_best]=min(h1_go(i,:));
        XL(i,6) = 360/nsteps_angle*(j_best-1)*pi/180;
      end
      RS.I_EE_Task = I_EE_3T2R; % Aufgabenredundanz zuruecksetzen
      fprintf('Eckpunkt %d/%d berechnet. Dauer %1.1fs (Brute-Force 3T3R).\n', ...
        i, size(XL,1), toc(t3));
    end
  end
  %% Plot: Zielfunktion für die Eckpunkte mit verschiedenen IK-Einstellungen
  Index_p = 1:size(XL,1);
  change_current_figure(100*robnr+1);clf;
  set(100*robnr+1, 'Name', sprintf('Rob%d_Zielf_Einzelpunkte', robnr), 'NumberTitle', 'off');
  sgtitle('Zielfunktionen für Eckpunkte');
  subplot(2,2,1); hold on;
  plot(Index_p, h1_go, 'c:');
  plot(Index_p, min(h1_go,[],2), 'r--^');
  plot(Index_p, h1, 'b-v');
  ylim([0.8*min(h1_go(:)), 1.5*max(h1(:))]);

  title('Zielfunktion 1 (nicht opt.)');
  ylabel('Zielfkt 1'); grid on;
  subplot(2,2,3); hold on;
  plot(Index_p, log10(h1_go), 'c:');
  plot(Index_p, log10(min(h1_go,[],2)), 'r--^');
  plot(Index_p, log10(h1), 'b-v');
  ylabel('Log.-Zielfkt 1 (n.o.)'); grid on;
  xlabel('Eckpunkt lfd Nr');
  ylim([log10(0.8*min(h1_go(:))), log10(1.5*max(h1(:)))]);
  
  subplot(2,2,2); hold on;
  plot(Index_p, h2_go, 'c:');
  plot(Index_p, min(h2_go,[],2), 'r--^');
  plot(Index_p, h2, 'b-v');
  title('Optimierungs-Zielfunktion 2');
  ylabel('Zielfkt 2'); grid on;
  ylim([0.9*min(h2_go(:)), 1.5*max(h2(:))]);
  
  subplot(2,2,4); hold on;
  hdl1=plot(Index_p, log10(h2_go), 'c:');
  hdl2=plot(Index_p, log10(min(h2_go,[], 2)), 'r--^');
  hdl3=plot(Index_p, log10(h2), 'b-v');
  ylabel('Log.-Zielfkt 2'); grid on;
  xlabel('Eckpunkt lfd Nr');
  ylim([log10(0.9*min(h2_go(:))), log10(1.5*max(h2(:)))]);
  legend([hdl1(1), hdl2, hdl3], {'Ohne AR, 2°-Schritte.', 'Ohne AR, bester', 'Mit Aufgabenredundenz'});
  saveas(100*robnr+1, fullfile(respath,sprintf( ...
    'Rob%d_%s_EinzelTraj_Zielf.fig',robnr, RS.mdlname)));

  %% Zeitverlauf der Trajektorie generieren
  [X_t,XD_t,XDD_t,t,IL] = traj_trapez2_multipoint(XL, 3, 0.05, 0.01, 2e-3, 1e-2);
  if short_traj  % Debug: Trajektorie reduzieren
    n = 200;
  else
    n = length(t); %#ok<UNRCH>
  end
  II = 1:n;
  t = t(II);
  X_t = X_t(II,:);
  XD_t = XD_t(II,:);
  XDD_t = XDD_t(II,:);
  
  %% Inverse Kinematik zum Startpunkt der Trajektorie
  % Inverse Kinematik berechnen;  Lösung der IK von oben als Startwert
  t0 = tic();
  s_start = s;
  % Toleranz maximal stark setzen, damit es keinen Sprung im Verlauf gibt
  % (durch die vielen Nullraumiterationen ist die Aufgabentoleranz später
  % sowieso ungefähr Null.
  s_start.Phit_tol = 1e-12;
  s_start.Phir_tol = 1e-12;
  s_start.normalize = false;
  s_start.maxrelstep = 0.02; % sehr kleine Schritte
  s_start.n_min = 1000; % Nullraum muss auf jeden Fall konvergiert sein
  s_start.n_max = 5000;
  s_start.wn = [1;0];
  % Berechne IK mit 3T2R
  RS.I_EE_Task = I_EE_3T2R;
  warning on
  % Berechne Ersten Punkt der Trajektorie mit Aufgabenredundanz.
  % Dadurch bestmögliche Startkonfiguration
  [q1, Psi_num1] = RS.invkin2(RS.x2tr(X_t(1,:)'), qs, s_start);
  if any(abs(Psi_num1) > 1e-4)
    warning('IK konvergiert nicht für Startpunkt der Trajektorie');
  end
  q1norm = (q1-RS.qlim(:,1)) ./ (RS.qlim(:,2) - RS.qlim(:,1));
  if any(q1norm > 1) || any(q1norm < 0) % Winkel mit +- 2*pi berücksichtigen
    warning('Anfangs-Konfiguration für Trajektorie verletzt bereits die Grenzen');
  end

  %% Roboter in Startpose plotten
  change_current_figure(100*robnr+2);clf;
  set(100*robnr+2, 'Name', sprintf('Rob%d_Skizze', robnr), 'NumberTitle', 'off');
  title(sprintf('Rob %d - %s', robnr, RS.mdlname));
  hold on;grid on;
  xlabel('x in m');ylabel('y in m');zlabel('z in m');
  view(3);
  s_plot = struct( 'straight', 0);
  RS.plot( qs, s_plot );
  plot3(X_t(:,1), X_t(:,2), X_t(:,3));
  
  %% IK für Trajektorie berechnen
  RS.fill_fcn_handles(false);
  Namen_Methoden = cell(1,7);
  Q_t_all = NaN(length(t), RS.NJ, length(Namen_Methoden)); QD_t_all = Q_t_all; QDD_t_all = Q_t_all;
  H1_all = NaN(length(t), length(Namen_Methoden)); H2_all = H1_all;
  H1D_all = H1_all; H2D_all = H1_all; Hcond_all = H1_all;
  XE_all = NaN(length(t), 6, length(Namen_Methoden)); XDE_all = XE_all; XDDE_all = XE_all;
  % Allgemeine Einstellungen für Trajektorie
  s_Traj = struct('n_min', 50, 'n_max', 1500, 'Phit_tol', 1e-7, 'Phir_tol', 1e-7, ...
      'I_EE', I_EE_3T2R, 'reci', true, ...
      'wn', [0;1;20;0;0], ... % Hohe Gewichtung der Geschw.-Nebenbedingung, damit Überschreitungen gar nicht erst auftreten
      'normalize', false);
  % Ziehe 2 Prozent der Spannweite von den Geschw.- und Beschl.-Grenzen ab.
  % Dadurch wird die Grenze durch numerische Fehler hoffentlich nicht über-
  % schritten
  qDlim_red = RS.qDlim + repmat([0.02,-0.02],RS.NJ,1).*(RS.qDlim(:,2)-RS.qDlim(:,1));
  qDDlim_red = RS.qDDlim + repmat([0.01,-0.01],RS.NJ,1).*(RS.qDDlim(:,2)-RS.qDDlim(:,1));
  s_Traj.qDlim = qDlim_red;
  s_Traj.qDDlim = qDDlim_red;
  for kk = 1:length(Namen_Methoden)
    s_kk = s_Traj;
    switch kk
      case 1
        name_method='3T2R-IK (mit Opt.)';
        RS.I_EE_Task = I_EE_3T2R;
        s_kk.I_EE = I_EE_3T2R;
      case 2
        name_method='3T2R-IK ohne qD lim.';
        RS.I_EE_Task = I_EE_3T2R;
        s_kk.I_EE = I_EE_3T2R;
        s_kk.qDlim = RS.qDlim*NaN; % Dadurch Grenzen nicht aktiv
        s_kk.wn(3:4) = 0; % Zielfunktionen basierend auf qDlim deaktivieren
      case 3
        name_method='3T2R-IK ohne qDD lim.';
        RS.I_EE_Task = I_EE_3T2R;
        s_kk.I_EE = I_EE_3T2R;
        s_kk.qDDlim = RS.qDlim*NaN; % Dadurch Grenzen nicht aktiv
      case 4
        name_method='3T2R-IK ohne Opt.';
        RS.I_EE_Task = I_EE_3T2R;
        s_kk.I_EE = I_EE_3T2R;
        s_kk.wn = [0;0;0;0;0]; % nur Begrenzung der Geschwindigkeit. der Nullraumprojektor bleibt Null
      case 5
        name_method='3T2R-IK mit cond.-Opt.';
        RS.I_EE_Task = I_EE_3T2R;
        s_kk.I_EE = I_EE_3T2R;
        s_kk.wn(5) = 1; % Auch Konditionszahl verbessern
      case 6
        name_method='3T2R-IK simplify acc';
        RS.I_EE_Task = I_EE_3T2R;
        s_kk.I_EE = I_EE_3T2R;
        s_kk.simplify_acc = true;
      case 7
        name_method='3T3R-IK';
        RS.I_EE_Task = I_EE_3T3R;
        s_kk.I_EE = I_EE_3T3R;
        s_kk.wn(:) = 0; % Keine zusätzliche Optimierung möglich
      otherwise
        error('Fall %d noch nicht definiert', kk);
    end
    % Positions-IK zum Startpunkt der Trajektorie mit genau den gleichen
    % Optimierungs-Nebenbedingungen wie in der Trajektorie. Dadurch keine
    % Nullraumbewegung am Anfang (Start in lokalem Optimum)
    s_pik_kk = struct('I_EE', s_kk.I_EE);
    s_pik_kk.wn = s_kk.wn([1 2 5]); % Positions-Grenzen und Kondition
    [qs_kk, Phi_s, ~, Stats_s] = RS.invkin2(RS.x2tr(XL(i,:)'), qs, s_pik_kk);
    if any(abs(Phi_s)>1e-6)
      error(['Zusätzliche Nullraumbewegung am Beginn der Trajektorie ', ...
        'fehlgeschlagen']);
    end
    
    Namen_Methoden{kk} = name_method;
    t1=tic();
    [Q_t_kk, QD_t_kk, QDD_t_kk, Phi_t_kk] = RS.invkin2_traj( ...
      X_t,XD_t,XDD_t,t,qs_kk,s_kk);
    fprintf('Inverse Kinematik mit Methode %d (%s) berechnet. Dauer: %1.1fs\n', ...
      kk, name_method, toc(t1));
    Q_t_all(:,:,kk) = Q_t_kk;
    QD_t_all(:,:,kk) = QD_t_kk;
    QDD_t_all(:,:,kk) = QDD_t_kk;
    % Zielfunktionen für Position
    for jj = 1:length(t)
      H1_all(jj,kk) = invkin_optimcrit_limits1(Q_t_kk(jj,:)', RS.qlim);
      H2_all(jj,kk) = invkin_optimcrit_limits2(Q_t_kk(jj,:)', RS.qlim);
    end
    % Zielfunktionen für Geschwindigkeit
    for jj = 1:length(t)
      H1D_all(jj,kk) = invkin_optimcrit_limits1(QD_t_kk(jj,:)', RS.qDlim);
      H2D_all(jj,kk) = invkin_optimcrit_limits2(QD_t_kk(jj,:)', RS.qDlim);
    end
    % Zielfunktion für Konditionszahl
    for jj = 1:length(t)
      if any(isnan(Q_t_kk(jj,:)'))
        warning('Trajektorie nur bis Zeitschritt %d/%d berechnet', jj, length(t));
        break;
      end
      Hcond_all(jj,kk) = cond(RS.jacobig(Q_t_kk(jj,:)'));
    end
    % Berechne Ist-EE-Traj.
    I_traj = ~any(isnan(Q_t_kk),2); % Bei Fehlern wird NaN ausgegeben
    [X_ist, XD_ist, XDD_ist] = RS.fkineEE_traj(Q_t_kk(I_traj,:), QD_t_kk(I_traj,:), QDD_t_kk(I_traj,:));
    XE_all(I_traj,:,kk) = X_ist;
    XDE_all(I_traj,:,kk) = XD_ist;
    XDDE_all(I_traj,:,kk) = XDD_ist;
  end
  %% Konsistenz von Position, Geschwindigkeit und Beschleunigung testen
  for kk = 1:length(Namen_Methoden)
    Q = Q_t_all(:,:,kk);
    QD = QD_t_all(:,:,kk);
    QDD = QDD_t_all(:,:,kk);
    X = XE_all(:,:,kk);
    XD = XDE_all(:,:,kk);
    XDD = XDDE_all(:,:,kk);
    % Integriere die Beschleunigung und Geschwindigkeit
    QD_int = cumtrapz(t, QDD) + repmat(QD(1,:),n,1);
    % Integriere die Geschwindigkeit zur Position
    Q_int = cumtrapz(t, QD) + repmat(Q(1,:),n,1);
    % Vergleiche die Verläufe graphisch
    change_current_figure(100*robnr+40+kk);clf;
    set(100*robnr+40+kk, 'Name', sprintf('Rob%d_Kons_q_M%d', robnr, kk), 'NumberTitle', 'off');
    sgtitle(sprintf('Konsistenz q-int(qD), qD-int(qDD). M%d (%s)', kk, Namen_Methoden{kk}));
    for i = 1:RS.NJ
      % Gelenkposition
      subplot(3,RS.NJ,sprc2no(3,RS.NJ, 1, i));
      hold on;
      hdl1=plot(t, Q(:,i));
      hdl2=plot(t, Q_int(:,i), '--');
      plot(t([1,end]), repmat(RS.qlim(i,:),2,1), 'r-');
      ylabel(sprintf('q %d', i)); grid on;
      if i == RS.NJ, legend([hdl1;hdl2],{'direkt', 'integral'}); end
      ylim([-0.1, +0.1]+minmax2([Q(:,i);Q_int(:,i)]')); % Gelenkgrenzen nicht für Plot-Grenzen berücksichtigen
      % Gelenkgeschwindigkeit
      subplot(3,RS.NJ,sprc2no(3,RS.NJ, 2, i));
      hold on;
      hdl1=plot(t, QD(:,i));
      hdl2=plot(t, QD_int(:,i), '--');
      plot(t([1,end]), repmat(RS.qDlim(i,:),2,1), 'r-');
      ylabel(sprintf('qD %d', i)); grid on;
      if i == RS.NJ, legend([hdl1;hdl2],{'direkt', 'integral'}); end
      ylim([-0.1, +0.1]+minmax2([QD(:,i);QD_int(:,i)]'));
      % Gelenkbeschleunigung
      subplot(3,RS.NJ,sprc2no(3,RS.NJ, 3, i));
      hold on;
      plot(t, QDD(:,i));
      plot(t([1,end]), repmat(RS.qDDlim(i,:),2,1), 'r-');
      ylabel(sprintf('qDD %d', i)); grid on;
    end
    linkxaxes
    saveas(100*robnr+40+kk, fullfile(respath,sprintf('Rob%d_M%d_Konsistenz_q', robnr, kk)));
    % Integriere die Endeffektor Beschleunigung und Geschwindigkeit
    XD_int = cumtrapz(t, XDD) + repmat(XD(1,:),n,1);
    % Integriere die Geschwindigkeit zur Position
    X_int = cumtrapz(t, XD) + repmat(X(1,:),n,1);
    % Vergleiche die Verläufe graphisch
    change_current_figure(100*robnr+50+kk);clf;
    set(100*robnr+50+kk, 'Name', sprintf('Rob%d_Kons_x_M%d', robnr, kk), 'NumberTitle', 'off');
    sgtitle(sprintf('Konsistenz x-int(xD), xD-int(xDD). M%d (%s)', kk, Namen_Methoden{kk}));
    for i = 1:6
      % EE-Position
      subplot(3,6,sprc2no(3,6, 1, i));
      hold on;
      plot(t, X(:,i), 'b-');
      plot(t, X_int(:,i), 'r--');
      plot(t, X_t(:,i), 'g--');
      ylabel(sprintf('x %d', i)); grid on;
      if i == 6, legend({'direkt', 'integral', 'Soll'}); end
      % EE-Geschwindigkeit
      subplot(3,6,sprc2no(3,6, 2, i));
      hold on;
      plot(t, XD(:,i), 'b-');
      plot(t, XD_int(:,i), 'r--');
      plot(t, XD_t(:,i), 'g--');
      ylabel(sprintf('xD %d', i)); grid on;
      if i == RS.NJ, legend({'direkt', 'integral', 'Soll'}); end
      % EE-Beschleunigung
      subplot(3,6,sprc2no(3,6, 3, i));
      hold on;
      plot(t, XDD(:,i), 'b-');
      plot(t, XDD_t(:,i), 'g--');
      ylabel(sprintf('xDD %d', i)); grid on;
    end
    linkxaxes
    saveas(100*robnr+50+kk, fullfile(respath,sprintf('Rob%d_M%d_Konsistenz_x', robnr, kk)));
  end
  
  %% Trajektorie: Bild für Gesamt-Zielfunktionen
  change_current_figure(100*robnr+20); clf;
  set(100*robnr+20, 'Name', sprintf('Rob%d_Zielf_q', robnr), 'NumberTitle', 'off');
  sgtitle('Zielfunktionen (Position)');
  subplot(2,2,1); hold on;
  hdl={};
  hdl{1}=plot(t, H1_all);
  title('Zielfunktion 1 (nicht opt.)');
  ylabel('Zielfkt 1'); grid on;
  subplot(2,2,3); hold on;
  hdl{2}=plot(t, log10(H1_all));
  ylabel('Log.-Zielfkt 1 (n.o.)'); grid on;
  subplot(2,2,2); hold on;
  hdl{3}=plot(t, H2_all);
  title('Optimierungs-Zielfunktion 2');
  ylabel('Zielfkt 2'); grid on;
  subplot(2,2,4); hold on;
  hdl{4}=plot(t, log10(H2_all));
  ylabel('Log.-Zielfkt 2'); grid on;
  linkxaxes
  % Linien nachträglich neu formatieren (bessere Lesbarkeit)
  for k = 1:4, leghdl=line_format_publication(hdl{k}, format_mlines); end
  legend(leghdl, Namen_Methoden);
  saveas(100*robnr+20, fullfile(respath,sprintf('Rob%d_Zielf_Pos', robnr)));
  
  change_current_figure(100*robnr+21); clf;
  set(100*robnr+21, 'Name', sprintf('Rob%d_Zielf_qD', robnr), 'NumberTitle', 'off');
  sgtitle('Zielfunktionen (Geschwindigkeit)');
  subplot(2,2,1); hold on;
  hdl={};
  hdl{1}=plot(t, H1D_all);
  title('Zielfunktion 1 (nicht opt.)');
  ylabel('Zielfkt 1'); grid on;
  subplot(2,2,3); hold on;
  hdl{2}=plot(t, log10(H1D_all));
  ylabel('Log.-Zielfkt 1 (n.o.)'); grid on;
  subplot(2,2,2); hold on;
  hdl{3}=plot(t, H2D_all);
  title('Optimierungs-Zielfunktion 2');
  ylabel('Zielfkt 2'); grid on;
  subplot(2,2,4); hold on;
  hdl{4}=plot(t, log10(H2D_all));
  ylabel('Log.-Zielfkt 2'); grid on;
  linkxaxes
  % Linien nachträglich neu formatieren (bessere Lesbarkeit)
  for k = 1:4, leghdl=line_format_publication(hdl{k}, format_mlines); end
  legend(leghdl, Namen_Methoden);
  saveas(100*robnr+21, fullfile(respath,sprintf('Rob%d_Zielf_Geschw', robnr)));
  
  change_current_figure(100*robnr+22); clf;
  set(100*robnr+22, 'Name', sprintf('Rob%d_Zielf_cond', robnr), 'NumberTitle', 'off');
  hdl=plot(t, Hcond_all);
  leghdl=line_format_publication(hdl, format_mlines);
  legend(leghdl, Namen_Methoden);
  title('Konditionszahl (mögliche Opt. Zielf.)');
  ylabel('Konditionszahl'); grid on;
  saveas(100*robnr+22, fullfile(respath,sprintf('Rob%d_Zielf_cond', robnr)));
  %% Trajektorie: Vergleich Gelenkverlauf nach verschiedenen Methoden
  change_current_figure(100*robnr+23); clf;
  set(100*robnr+23, 'Name', sprintf('Rob%d_Gelenke', robnr), 'NumberTitle', 'off');
  sgtitle('Gelenkverläufe');
  hdl = NaN(3,RS.NJ,length(Namen_Methoden));
  for kk = 1:length(Namen_Methoden)
    Q = Q_t_all(:,:,kk);
    QD = QD_t_all(:,:,kk);
    QDD = QDD_t_all(:,:,kk);
    for i = 1:RS.NJ
      % Gelenkposition
      subplot(3,RS.NJ,sprc2no(3,RS.NJ, 1, i));
      hold on;
      hdl(1,i,kk)=plot(t, Q(:,i));
      if kk == 1, plot(t([1,end]), repmat(RS.qlim(i,:),2,1), 'r-'); end
      if kk == 1, ylabel(sprintf('q %d', i)); grid on; end
      % Gelenkgeschwindigkeit
      subplot(3,RS.NJ,sprc2no(3,RS.NJ, 2, i));
      hold on;
      hdl(2,i,kk)=plot(t, QD(:,i));
      if kk == 1, plot(t([1,end]), repmat(RS.qDlim(i,:),2,1), 'r-'); end
      if kk == 1, ylabel(sprintf('qD %d', i)); grid on; end
      % Gelenkbeschleunigung
      subplot(3,RS.NJ,sprc2no(3,RS.NJ, 3, i));
      hold on;
      hdl(3,i,kk)=plot(t, QDD(:,i));
      if kk == 1, ylabel(sprintf('qDD %d', i)); grid on; end
    end
  end
  linkxaxes
    
  saveas(100*robnr+23, fullfile(respath,sprintf('Rob%d_Zielf', robnr)));
  % Linien nachträglich neu formatieren (bessere Lesbarkeit)
  for k = 1:RS.NJ
    for j = 1:3
      leghdl=line_format_publication(hdl(j,k,:), format_mlines);
    end
  end
  legend(leghdl, Namen_Methoden);
  
  %% Prüfe, ob die Rechnung Unabhängigkeit von der Abtastzeit der Trajektorie
  dt_array = [5e-4, 1e-3, 2e-3];
  change_current_figure(100*robnr+60); clf;
  set(100*robnr+60, 'Name', sprintf('Rob%d_Abtastzeit_q', robnr), 'NumberTitle', 'off');
  sgtitle('Gelenkverläufe nach Abtastzeit');
  change_current_figure(100*robnr+61); clf;
  set(100*robnr+61, 'Name', sprintf('Rob%d_Abtastzeit_x', robnr), 'NumberTitle', 'off');
  sgtitle('EE-Verläufe nach Abtastzeit');
  hdlq = NaN(3,RS.NJ,length(dt_array));
  hdlx = NaN(3,RS.NJ,length(dt_array));
  for ii = 1:length(dt_array)
    dt = dt_array(ii);
    [X_ii,XD_ii,XDD_ii,t_ii] = traj_trapez2_multipoint(XL, 3, 0.05, 0.01, dt, 1e-2);
    s_kk = s_Traj;
    RS.I_EE_Task = I_EE_3T2R;
    s_kk.I_EE = I_EE_3T2R;
    t1=tic();
    [Q_t_ii, QD_t_ii, QDD_t_ii, Phi_t_ii] = RS.invkin2_traj(X_ii,XD_ii,XDD_ii,t_ii,qs,s_kk);
    fprintf('Trajektorien-IK für Abtastzeit %1.1fms berechnet (%d Bahnpunkte). Dauer: %1.1fs\n', ...
      1e3*dt, length(t_ii), toc(t1));
    change_current_figure(100*robnr+60);
    for i = 1:RS.NJ
      % Gelenkposition
      subplot(3,RS.NJ,sprc2no(3,RS.NJ, 1, i));
      hold on;
      hdlq(1,i,ii)=plot(t_ii, Q_t_ii(:,i));
      if ii == 1, plot(t_ii([1,end]), repmat(RS.qlim(i,:),2,1), 'r-'); end
      if ii == 1, ylabel(sprintf('q %d', i)); grid on; end
      % Gelenkgeschwindigkeit
      subplot(3,RS.NJ,sprc2no(3,RS.NJ, 2, i));
      hold on;
      hdlq(2,i,ii)=plot(t_ii, QD_t_ii(:,i));
      if ii == 1, plot(t_ii([1,end]), repmat(RS.qDlim(i,:),2,1), 'r-'); end
      if ii == 1, ylabel(sprintf('qD %d', i)); grid on; end
      % Gelenkbeschleunigung
      subplot(3,RS.NJ,sprc2no(3,RS.NJ, 3, i));
      hold on;
      hdlq(3,i,ii)=plot(t_ii, QDD_t_ii(:,i));
      if ii == 1, ylabel(sprintf('qDD %d', i)); grid on; end
    end
    linkxaxes
    change_current_figure(100*robnr+61);
    for i = 1:6
      % Position
      subplot(3,6,sprc2no(3,6, 1, i));
      hold on;
      hdlx(1,i,ii)=plot(t_ii, X_ii(:,i));
      ylabel(sprintf('x %d', i)); grid on;
      % Geschwindigkeit
      subplot(3,6,sprc2no(3,6, 2, i));
      hold on;
      hdlx(2,i,ii)=plot(t_ii, XD_ii(:,i));
      ylabel(sprintf('xD %d', i)); grid on;
      % Beschleunigung
      subplot(3,6,sprc2no(3,6, 3, i));
      hold on;
      hdlx(3,i,ii)=plot(t_ii, XDD_ii(:,i));
      ylabel(sprintf('xDD %d', i)); grid on;
    end
    linkxaxes
  end
  for k = 1:RS.NJ
    for j = 1:3
      legdhlq=line_format_publication(hdlq(j,k,:), format_mlines);
      legdhlx=line_format_publication(hdlx(j,k,:), format_mlines);
    end
  end
  lstrs={};
  for ii = 1:length(dt_array), lstrs{ii}=sprintf('dt=%1.1fms',1e3*dt_array(ii)); end
  legend(squeeze(hdlq(j,k,:)), lstrs);
  legend(squeeze(hdlx(j,k,:)), lstrs);
  linkxaxes
  saveas(100*robnr+60, fullfile(respath,sprintf('Rob%d_dt_Vergl_q', robnr)));
  saveas(100*robnr+61, fullfile(respath,sprintf('Rob%d_dt_Vergl_x', robnr)));
  %% Debug: Vergleich mit und ohne simplify_acc
  if debug_plot
  change_current_figure(100*robnr+70); clf; %#ok<UNRCH>
  set(100*robnr+70, 'Name', sprintf('Rob%d_simplify_acc', robnr), 'NumberTitle', 'off');
  sgtitle('Vergleich Ergebnisse mit Option simplify acc');
  hdl = NaN(3,RS.NJ,length(Namen_Methoden));
  iihdl = 0;
  for kk = [1 4]
    iihdl = iihdl + 1;
    Q = Q_t_all(:,:,kk);
    QD = QD_t_all(:,:,kk);
    QDD = QDD_t_all(:,:,kk);
    for i = 1:RS.NJ
      % Gelenkposition
      subplot(3,RS.NJ,sprc2no(3,RS.NJ, 1, i));
      hold on;
      hdl(1,i,iihdl)=plot(t, Q(:,i));
%       if iihdl == 1, plot(t([1,end]), repmat(RS.qlim(i,:),2,1), 'r-'); end
      if iihdl == 1, ylabel(sprintf('q %d', i)); grid on; end
      % Gelenkgeschwindigkeit
      subplot(3,RS.NJ,sprc2no(3,RS.NJ, 2, i));
      hold on;
      hdl(2,i,iihdl)=plot(t, QD(:,i));
%       if iihdl == 1, plot(t([1,end]), repmat(RS.qDlim(i,:),2,1), 'r-'); end
      if iihdl == 1, ylabel(sprintf('qD %d', i)); grid on; end
      % Gelenkbeschleunigung
      subplot(3,RS.NJ,sprc2no(3,RS.NJ, 3, i));
      hold on;
      hdl(3,i,iihdl)=plot(t, QDD(:,i));
      if iihdl == 1, ylabel(sprintf('qDD %d', i)); grid on; end
    end
  end
  linkxaxes
  % Linien nachträglich neu formatieren (bessere Lesbarkeit)
  for i = 1:3
    for j = 1:RS.NJ
      leghdl=line_format_publication(squeeze(hdl(i,j,1:2)), format_mlines([1 4],:)); end
  end
  legend(leghdl, Namen_Methoden([1 4]));
    
  end
  %% Ergebniss speichern
  save(fullfile(respath,sprintf('Rob%d_%s_result.mat',robnr,RS.mdlname)));
end
