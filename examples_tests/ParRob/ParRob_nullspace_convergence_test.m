% Prüfe die Konvergenz der Nullraumbewegung der IK für parallele Roboter.
% Vergleich von Trajektorien-IK und Einzelpunkt-IK.
% 
% Ablauf:
% * Anfahrt einer Pose im Arbeitsraum mit vorgegebener Orientierung
%   (auch der z-Drehung)
% * Nullraumbewegung nach gewählten Zielkriterien mit Einzelpunkt-IK
%   (Annahme von Aufgabenredundanz um die zE-Achse)
% * Nullraumbewegung zum gleichen Kriterium mit Trajektorien-IK
% Erwartetes Ergebnis:
% * Beide Verfahren konvergieren zum gleichen Optimum. Es findet eine
%   Nullraumbewegung auf der zulässigen Lösung statt (Aufgabenredundanz)
% 
% Der Ablauf wird für verschiedene Punkte im Arbeitsraum und Orientierungen
% der PKM-Plattform durchgeführt. Bei vermehrten Abweichungen wird ein
% Fehler aufgeworfen.
% 
% Optional: Plotten der Ergebnisse und Erzeugen eines Videos. Einstellung
% mit "usr_..."-Variablen möglich.
% Falls viele Bilder erzeugt werden, läuft das Skript zu lange. In diesem
% Fall sollten manuell die Testfälle ausgewählt werden (Schleife k/l).
% 
% Siehe auch: SerRob_nullspace_convergence_test.m
% Quelle: Schappler, M. and Ortmaier, T.: Singularity Avoidance of
% Task-Redundant Robots in Pointing Tasks: On Nullspace Projection and
% Cardan Angles as Orientation Coordinates, ICINCO 2021.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-03
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc
% Benutzereinstellungen. Standardmäßig als reines Testskript laufen lassen
% und nur bei übermäßig vielen Fehlern abbrechen (am Ende).
use_mex_functions = true; % Mex-Funktionen benutzen (wesentlich schneller)
usr_recreate_mex = true; % Mex-Funktionen neu erzeugen (und kompilieren)
usr_plot_debug = false; % Debug-Plots über den Verlauf der Berechnungen
usr_plot_objfun = false; % Verlauf der Zielfunktionen (über red. Koord.) zeichnen
usr_plot_robot = false; % Skizze des Roboters zeichnen
usr_raise_on_err = false; % Bei Fehler Durchlauf abbrechen
usr_plot_on_err = false; % Nur Bilder im Fehlerfall zeichnen (zum Debuggen)
usr_save_figures = true; % Um die Bilder als Datei zu speichern
usr_save_anim = false; % zur Erzeugung eines Videos
%#ok<*UNRCH>
% Sonstige Initialisierung:
respath = fullfile(fileparts(which('robotics_toolbox_path_init.m')), ...
  'examples_tests', 'results', 'ParRob_nullspace_convergence_test');
mkdirs(respath);
if isempty(which('serroblib_path_init.m'))
  warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Skript nicht ausführbar.');
  return
end
if isempty(which('parroblib_path_init.m'))
  warning('Repo mit parallelen Robotermodellen ist nicht im Pfad. Skript nicht ausführbar.');
  return
end
% Namen der Optimierungskriterien der IK (für Diagramme)
hnames = {'qlim quadrat.', 'qlim hyperb.', 'cond IK-Jac.', 'cond PKM-Jac.', ...
  'xlim quadrat.', 'xlim hyperb.'};
% Zuordnung zwischen Nebenbedingungen der Einzelpunkt- und Traj.-IK
% Reihenfolge: quadratischer Grenzabstand, hyperbolischer Grenzabstand,
% Konditionszahl IK-Jacobi, Konditionszahl PKM-Jacobi, quadr. EE-Grenzen, hyperb. EE-Grenzen
I_wn_traj = [1 2 5 6 15 17]; % Siehe P3RRR1_invkin_traj (Eingabe wn)
I_stats_traj = [1 2 5 6 9 10]; % Siehe P3RRR1_invkin_traj (Ausgabe Stats.h)
I_wn_ep = [1 2 3 4 7 8]; % Siehe P3RRR1_invkin3 (Eingabe wn, Ausgabe h)
%% Alle Robotermodelle durchgehen
for robnr = 1:5
  %% Initialisierung
  if robnr == 1 % 3RRR
    RP = parroblib_create_robot_class('P3RRR1G1P1A1', 1.0, 0.2);
    pkin_gen = zeros(length(RP.Leg(1).pkin_names),1);
    % Nachbearbeitung einiger Kinematikparameter
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'a2')) = 0.6;
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'a3')) = 0.6;
    for i = 1:RP.NLEG
      RP.Leg(i).update_mdh(pkin_gen);
    end
  elseif robnr == 2 % 6UPS
    RP = parroblib_create_robot_class('P6RRPRRR14V3G1P4A1', 1.0, 0.2);
  elseif robnr == 3 % 6PUS
    RP = parroblib_create_robot_class('P6PRRRRR6V4G8P1A1', [0.8;0.3;pi/3], 0.2);
    pkin = zeros(length(RP.Leg(1).pkin),1); % Namen, siehe RP.Leg(1).pkin_names
    pkin(strcmp(RP.Leg(1).pkin_names,'a2')) = 0.2;
    pkin(strcmp(RP.Leg(1).pkin_names,'theta1')) = -pi/2; % So drehen, dass a2 nach oben zeigt
    pkin(strcmp(RP.Leg(1).pkin_names,'a4')) = 0.4;
    pkin(strcmp(RP.Leg(1).pkin_names,'alpha2')) = pi/2;
    for i = 1:RP.NLEG
      RP.Leg(i).update_mdh(pkin);
    end
  elseif robnr == 4 % 6RUS (Hexa)
    RP = parroblib_create_robot_class('P6RRRRRR10V6G6P4A1', [0.2; 0.1], [0.2; 0.1]);
    pkin = NaN(length(RP.Leg(1).pkin_names),1);
    pkin(strcmp(RP.Leg(1).pkin_names, 'd1')) = 0;
    pkin(strcmp(RP.Leg(1).pkin_names, 'a2')) = 0.4;
    pkin(strcmp(RP.Leg(1).pkin_names, 'd2')) = 0;
    pkin(strcmp(RP.Leg(1).pkin_names, 'alpha2')) = 0;
    pkin(strcmp(RP.Leg(1).pkin_names, 'a4')) = 0.5;
    pkin(strcmp(RP.Leg(1).pkin_names, 'alpha4')) = 0;
    pkin(strcmp(RP.Leg(1).pkin_names, 'd4')) = 0;
    for i = 1:RP.NLEG
      RP.Leg(i).update_mdh(pkin);
    end
  elseif robnr == 5 % 3T1R-PKM mit Drehgelenken
    % Parameter aus Maßsynthese
    RP = parroblib_create_robot_class('P4RRRRR5V1G1P1A1', 0.6399, 0.2316);
    pkin_gen = [-0.4235   -0.4619   -0.5137         0   -0.4207    0.1396         0]';
    for i = 1:RP.NLEG
      RP.Leg(i).update_mdh(pkin_gen);
    end
  end
  % Entwurfsparameter eintragen (für Plot)
  for i = 1:RP.NLEG
    if robnr == 2
      RP.Leg(i).DesPar.joint_type(RP.I_qa((RP.I1J_LEG(i):RP.I2J_LEG(i)))) = 5;
    elseif robnr == 3
      % Setze Schubgelenke als Linearachsen mit Führungsschiene
      RP.Leg(i).DesPar.joint_type(RP.I_qa((RP.I1J_LEG(i):RP.I2J_LEG(i)))) = 4;
    end
    % Setze Segmente als Hohlzylinder mit Durchmesser 50mm
    RP.Leg(i).DesPar.seg_par=repmat([5e-3,50e-3],RP.Leg(i).NL,1);
  end
  RP.DesPar.platform_par(end) = 5e-3;
  if usr_recreate_mex
    serroblib_create_template_functions({RP.Leg(1).mdlname}, false, false);
    parroblib_create_template_functions({RP.mdlname(1:end-2)}, false, false);
    mexerr = false;
    mexerr = mexerr | matlabfcn2mex({[RP.mdlname(1:end-6), '_invkin']});
    mexerr = mexerr | matlabfcn2mex({[RP.mdlname(1:end-6), '_invkin3']});
    mexerr = mexerr | matlabfcn2mex({[RP.mdlname(1:end-6), '_invkin_traj']});
    if mexerr
      error('Fehler beim Kompilieren');
    end
  end
  RP.fill_fcn_handles(use_mex_functions,true);
  % Definition der Freiheitsgrade (vollständig und reduziert)
  I_EE_full = RP.I_EE;
  if all(RP.I_EE == [1 1 1 1 1 1])
    I_EE_red = logical([1 1 1 1 1 0]);
  elseif  all(RP.I_EE == [1 1 1 0 0 1])
    I_EE_red = logical([1 1 1 0 0 0]);
  elseif  all(RP.I_EE == [1 1 0 0 0 1])
    I_EE_red = logical([1 1 0 0 0 0]);
  else
    error('EE-FG des Roboters nicht vorgesehen');
  end
  I_EE_full_str = sprintf('%dT%dR', sum(I_EE_full(1:3)), sum(I_EE_full(4:6)));
  I_EE_red_str = sprintf('%dT%dR', sum(I_EE_red(1:3)), sum(I_EE_red(4:6)));
  for i = 1:RP.NLEG
    % Begrenze die Winkel der Kugel- und Kardangelenke auf +/- 360°
    RP.Leg(i).qlim = repmat([-2*pi, 2*pi], RP.Leg(i).NQJ, 1);
    % Begrenze die Länge der Schubgelenke
    if robnr == 2
      qpris_minmax = [0.6, 1.5];
    elseif robnr == 3 % Sonst überschneiden sich die Schienen
      qpris_minmax = [0.1, 0.8];
    else
      qpris_minmax = [0.1, 1.5];
    end
    RP.Leg(i).qlim(RP.Leg(i).MDH.sigma==1,:) = ...
      repmat(qpris_minmax, sum(RP.Leg(i).MDH.sigma==1), 1);
    % Setze Geschwindigkeit und Beschleunigung auf moderate Werte, damit
    % die Schrittweite in der Traj.-IK nicht zu groß wird und die Klein-
    % winkelnäherung nicht verletzt wird.
    RP.Leg(i).qDlim = repmat(2*[-5*pi, 5*pi], RP.Leg(i).NQJ, 1); % 2*pi sind 720deg/s
    RP.Leg(i).qDlim(RP.Leg(i).MDH.sigma==1,:) = ...
      repmat([-5, 5], sum(RP.Leg(i).MDH.sigma==1), 1); % 5m/s
    
    RP.Leg(i).qDDlim = repmat([-50, 50], RP.Leg(i).NQJ, 1); % 100 sind 10g auf 1m Länge
    RP.Leg(i).qDDlim(RP.Leg(i).MDH.sigma==1,:) = ...
      repmat([-20, 20], sum(RP.Leg(i).MDH.sigma==1), 1); % 50m/s² sind ca. 5g
  end
  qlim   = cat(1, RP.Leg.qlim);
  qDlim  = cat(1, RP.Leg.qDlim);
  qDDlim = cat(1, RP.Leg.qDDlim);
  RS.xDlim = [repmat([-2, 2], 3, 1);repmat([-pi, pi], 3, 1)];
  xlim_abs = NaN(6,2); % Absolute Grenzen für EE-Koordinate (in Funktion relativ)
  xlim_abs(6,:) = [-45, 45]*pi/180;
  %% Eckpunkte bestimmen, die angefahren werden
  % Orientierung nicht exakt senkrecht, da das oft singulär ist.
  X0 = [ [0.05;0.03;0.6]; [2;-3;0]*pi/180 ];
  if all(I_EE_full == [1 1 0 0 0 1]), X0(3:5) = 0; end
  % Eckpunkte definieren, die einen Würfel ergeben
  traj_d=0.15; traj_h=0.15;
  X1 = X0+[-traj_d/2,traj_d/2,traj_h/2,0,0,0]'; % Start so, dass Würfel genau mittig ist
  k=1; XL = X1';
  k=k+1; XL(k,:) = XL(k-1,:) + [ traj_d,0,0, 0,0,0];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,-traj_d,0  0,0, pi/4];
  k=k+1; XL(k,:) = XL(k-1,:) + [-traj_d,0,0, 0,0,-pi/4];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,0,-traj_h, pi/4,0, 0];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,0, traj_h, -pi/4,0,0];
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
  % Reduziere alle Schwenkwinkel gegenüber der Vorlage (funktioniert eher)
  XL(:,4:6) = 0.1*XL(:,4:6);
  % Entferne die Drehwinkel. Werden später sowieso überschrieben
  XL(:,6) = 0;
  if ~all(RP.I_EE == [1 1 1 1 1 1])
    XL(:, ~RP.I_EE) = 0; XL = uniquetol(XL, 1e-10,'ByRows',true);
  end
  % Ergebnis-Tabellen vorbereiten (siehe Schleifen unten)
  ResStat = array2table(NaN(size(XL,1)*9*6,9));
  ResStat.Properties.VariableNames = {'PktNr', 'OriNr', 'IK_Fall', ...
    'h_start', 'h_err_abs', 'h_err_rel', 'h_step_traj', 'h_step_ep', 'Error'};
  ResStat_emptyrow = ResStat(1,:);
  ResStat_Start  = array2table(NaN(size(XL,1)*9,4));
  ResStat_Start.Properties.VariableNames = {'PktNr', 'OriNr', 'Erfolg', 'Grenzen'};
  fprintf('Starte Untersuchung für Rob. %d (%s)\n', robnr, RP.mdlname);
  %% Inverse Kinematik berechnen
  q0 = -0.5+rand(RP.NJ,1);
  if robnr == 2
    q0(RP.I_qa) = 1.0; % damit bei 6UPS das Schubgelenk positiv bleibt
  elseif robnr == 3
    q0(RP.I_qa) = 0.2; % damit bei 6PUS das Schubgelenke nicht vorne übersteht
  end
  q0_ik_fix = q0;
  q0_ik_fix(RP.I1J_LEG(2):end) = NaN; % damit Übernahe Ergebnis Beinkette 1
  % Einstellungen für Einzelpunkt-IK.
  s_ep = struct( 'wn', zeros(7,1), ... % Platzhalter einsetzen
    'n_max', 5000, 'Phit_tol', 1e-12, 'Phir_tol', 1e-12); % , 'finish_in_limits', true
  % Einstellungen für Dummy-Berechnung ohne Änderung der Gelenkwinkel.
  s_ep_dummy = s_ep;
  s_ep_dummy.retry_limit = 0;
  s_ep_dummy.wn = ones(8,1); % hierdurch werden die Kriterien berechnet
  s_ep_dummy.K = zeros(RP.NJ,1); % hierdurch keine Bewegung und damit ...
  s_ep_dummy.Kn = zeros(RP.NJ,1); % ... sofortiger Abbruch
  ii_restab = 0; ii_restab_start = 0;
  for k = 1:size(XL,1) % Schleife über alle Eckpunkte
    x_k = XL(k,:)';
    % Absolute Verteilung der Zielfunktion feststellen (Diskretisierung)
    x_test_ges = [];
    h_ges = [];
    if usr_plot_objfun
      nn = 90;
      x_test_ges = repmat(x_k',nn,1);
      x_test_ges(:,6) = linspace(-pi,pi,nn);
      h_ges = NaN(nn,8); % Konsistent mit ParRob/invkin4
      q_jj = q0_ik_fix; % nehme immer den Wert von davor als Startwert. Dann weniger Konfigurationswechsel
      t0 = tic();
      for jj = 1:nn
        RP.update_EE_FG(I_EE_full,I_EE_full); % Roboter auf 3T3R einstellen
        x_jj = x_test_ges(jj,:)';
        t1=tic();
        for iktrynum = 1:20 % mehrfache Versuche der IK für plausibles Ergebnis
          s_ep_jj = s_ep; % Einstellungen so lassen (Grenzverletzung erlauben)
          [q_jj, Phi,~,Stats] = RP.invkin2(x_jj, q_jj, s_ep_jj);
          % Korrigiere die Konfiguration bei Robotern mit Schubantrieben.
          % Hier ist ein Umklappen der Konfiguration möglich und unerwünscht.
          if robnr == 2 && any(q_jj(RP.I_qa)<0)
            if iktrynum < 20
              % Ändere die Anfangswerte und versuche erneut
              ILeg_invalid = find(q_jj(RP.I_qa)<0);
              for iii = ILeg_invalid(:)'
                q_jj(RP.I1J_LEG(iii):RP.I2J_LEG(iii)) = -0.5+rand(RP.Leg(iii).NJ,1);
                if robnr == 3
                  q_jj(1) = -0.8 + rand(1); % sehr niedrigen Wert für untere Konfiguration
                end
              end
              continue
            end
            if robnr == 2
              error('Die Schubgelenke sind in einer ungünstigen Konfiguration. Darf nicht sein.');
            else
              % Bei 6PUS sind die Schubgelenke teilweise unplausibel weit
              % vorne (liegt an Plattform-Drehung)
              continue
            end
            change_current_figure(999);clf;
            set(999,'Name','Rob_Debug','NumberTitle','off');
            hold on; grid on; view(3);
            xlabel('x in m');ylabel('y in m');zlabel('z in m');
            s_plot = struct( 'straight', 0, 'mode', 4);
            RP.plot( q_jj, x_jj, s_plot );
          end
          break;
        end
        if jj == 1
          fprintf(['Rastere den Nullraum mit %d IK-Aufrufen. Dauer ', ...
            'für ersten Aufruf: %1.1fms. Gesamt geschätzt: %1.1fs\n'], ...
            nn, toc(t1)*1e3, (nn-1)*toc(t1));
        end
        if any(abs(Phi) > 1e-8)
          continue
        end
        RP.update_EE_FG(I_EE_full,I_EE_red); % Roboter auf 3T2R einstellen
        % IK benutzen, um Zielfunktionswerte zu bestimmen (ohne Neuberechnung)
        % Grenzen setzen. xlim ist relativ definiert zur Soll-Pose aus x_l
        s_ep_dummy.xlim = xlim_abs - [zeros(5,2);repmat(x_jj(6),1,2)];
        [q_dummy, Phi,~,Stats_dummy] = RP.invkin4(x_jj, q_jj, s_ep_dummy);
        if any(abs(q_jj - q_dummy) > 1e-8)
          error('IK-Ergebnis hat sich bei Test verändert');
        end
        h_ges(jj,:) = Stats_dummy.h(1+Stats_dummy.iter,2:end);
      end
      fprintf(['Berechnung von %d Raster-Konfigurationen in %1.1fs ', ...
        'abgeschlossen. %d/%d erfolgreich\n'], nn, toc(t0), sum(~isnan(h_ges(:,1))), nn);
      % Verdoppele die Daten aufgrund der Periodizität
      x_test_ges = [x_test_ges(:,1:5),x_test_ges(:,6)-2*pi; x_test_ges;
                    x_test_ges(:,1:5),x_test_ges(:,6)+2*pi]; %#ok<AGROW>
      h_ges = repmat(h_ges,3,1);
      % Debug: Verteilung der Zielfunktionen über die redundante Koordinate
      % plotten. Wird jetzt weiter unten gemacht.
%       figure(10);clf;
%       optimcritaxhdl = NaN(3,1);
%       hnames = {'qlim quadrat.', 'qlim hyperb.', 'cond IK-Jac.', 'cond PKM-Jac.'};
%       for i = 1:4
%         optimcritaxhdl(i) = subplot(2,2,i); hold on;
%         plot(x_test_ges(:,6)*180/pi, h_ges(:,i));
%         ylabel(sprintf('h %d (%s)', i, hnames{i})); grid on;
%         xlabel('x6 in deg');
%       end
%       sgtitle(sprintf('Zielfunktionen Punkt %d', k));
%       linkxaxes
    end
    x6_l_range = [linspace(0,pi,5),linspace(0,-pi,5)];
    x6_l_range = unique(x6_l_range, 'stable');
    num_ik_qs_successfull = 0;
    for l = 1:length(x6_l_range) % Schleife über Anfangswerte der IK
      x_l = x_k;
      x_l(6) = x6_l_range(l);
      % Grenzen setzen. xlim ist relativ definiert zur Soll-Pose aus x_l
      xlim_l = xlim_abs - [zeros(5,2);repmat(x_l(6),1,2)];
      % Inverse Kinematik zum Startpunkt
      RP.update_EE_FG(I_EE_full,I_EE_full); % Roboter auf 3T3R einstellen
      s_ep_start = s_ep;
      s_ep_start.scale_lim = 0.5; % Grenzen nicht verlassen (lieber neu versuchen)
      [qs, Phi_ep_s, ~, Stats_ep_s] = RP.invkin2(x_l, q0_ik_fix, s_ep_start);
      if any(abs(Phi_ep_s) > 1e-8)
        % Mit Einhaltung der Grenzen keine Lösung gefunden. Dann eben mit
        % Verletzung der Grenzen. Führt auch zu Abbruch, ist aber für
        % Statistik wichtig, ob es rechnerisch überhaupt funktioniert.
        s_ep_start = s_ep;
        [qs, Phi_ep_s, ~, Stats_ep_s] = RP.invkin2(x_l, q0_ik_fix, s_ep_start);
      end
      xs = RP.fkineEE_traj(qs')';
      ii_restab_start = ii_restab_start + 1;
      ResStat_Start.PktNr(ii_restab_start) = k;
      ResStat_Start.OriNr(ii_restab_start) = l;
      if any(abs(Phi_ep_s) > 1e-8)
        ResStat_Start.Erfolg(ii_restab_start) = 0;
        warning('Inverse Kinematik konnte in Startpose für Punkt %d / Ori. %d nicht berechnet werden', k, l);
        continue % ist nicht immer möglich (Erzwungene Einhaltung der Grenzen)
      end
      ResStat_Start.Erfolg(ii_restab_start) = 1;
      if any(qs(RP.I_qa&RP.MDH.sigma==1)<0)
        error('Die Schubgelenke sind umgeklappt. Darf nicht sein.');
      end
        qs_norm = (qs - qlim(:,1)) ./ (qlim(:,2)-qlim(:,1));
      if any(qs<qlim(:,1)) || any(qs>qlim(:,2))
        % Dieser Fall sollte nicht mehr auftreten, da meistens scale_lim benutzt wird
        ResStat_Start.Grenzen(ii_restab_start) = 0;
        warning('Startpose für Punkt %d / Ori. %d verletzt Grenzen', k, l);
        continue; % Die Grenzen müssen beim Start eingehalten werden (sonst destabiliseren die Korrekturterme die IK)
        change_current_figure(999);clf;
        set(999,'Name','Rob_Debug','NumberTitle','off');
        title(sprintf('Startpose Punkt %d / Ori. %d', k, l));
        hold on; grid on; view(3);
        xlabel('x in m');ylabel('y in m');zlabel('z in m');
        s_plot = struct( 'straight', 0, 'mode', 4);
        RP.plot( qs, xs, s_plot );
      end
      ResStat_Start.Grenzen(ii_restab_start) = 1;
      num_ik_qs_successfull = num_ik_qs_successfull + 1;
      % IK benutzen, um Zielfunktionswerte zu bestimmen (ohne Neuberechnung)
      RP.update_EE_FG(I_EE_full,I_EE_red); % Roboter dafür auf 3T3R einstellen
      s_ep_dummy.xlim = xlim_l; % aktualisieren, damit Kriterien zu xlim richtig berechnet werden
      [q_dummy, Phi_dummy,~,Stats_dummy] = RP.invkin4(x_l, qs, s_ep_dummy);
      if any(abs(qs - q_dummy) > 1e-8)
        error('IK-Ergebnis hat sich bei Test verändert');
      end
      hs = Stats_dummy.h(Stats_dummy.iter,2:end)';
      % Füge Datenpunkt zu Gesamt-Rasterung hinzu
      x_test_ges = [x_test_ges; xs']; %#ok<AGROW>
      h_ges = [h_ges; hs']; %#ok<AGROW>
      [~,Isort] = sort(x_test_ges(:,6));
      x_test_ges = x_test_ges(Isort,:);
      h_ges = h_ges(Isort,:);
      %% Verschiedene IK-Zielfunktionen durchgehen
      for ii = [2 4 6 8 9 10] % Schleife über verschiedene Zielkriterien
        filename_pre = sprintf('Rob%d_Fall%d_%s_%s', robnr, ii, RP.mdlname);
        s_ep_ii = s_ep;
        if usr_save_anim % sehr feinschrittige Bewegungen (für flüssige Animation)
          s_ep_ii.maxrelstep = 0.005;
          s_ep_ii.n_max = s_ep_ii.n_max * 2;
        end
        s_ep_ii.xlim = xlim_l;
        s_ep_ii.retry_limit = 0; %nur ein Versuch. Sonst zufällig neue Gelenkwinkel.
        % Grenzen dürfen auch in Zwischenschritten nicht überschritten
        % werden.
        s_ep_ii.scale_lim = 0.9;
        % Kriterien zusammenstellen
        wn_traj = zeros(19,1);
        % Dämpfung der Geschwindigkeit immer einbauen. Bei symmetrischen
        % Grenzen entspricht das dem Standard-Dämpfungsterm aus der
        % Literatur
        wn_traj(3) = 0.5;
        % Zusätzlich Dämpfung bei Überschreitung des Grenzbereichs zu den
        % Positions-Grenzen. Dadurch weniger Überschreitungen der Grenze.
        wn_traj(2) = 1;
        wn_traj(8) = 1;
        optimcrit_limits_hyp_deact = 0.9;
        switch ii
          case 1 % P-Regler Quadratische Grenzfunktion
            wn_traj(1) = 1; % konvergiert schlecht ohne D-Anteil
          case 2 % PD-Regler Quadratische Grenzfunktion
            wn_traj(1) = 1; % P
            wn_traj(7) = 1; % D
          case 3 % P-Regler Hyperbolische Grenzfunktion
            wn_traj(2) = 1; % konvergiert schlecht ohne D-Anteil
            % Damit das Zielkriterium optimiert werden kann, muss es im
            % kompletten Gelenkbereich aktiv sein (nicht nur nahe Grenzen).
            optimcrit_limits_hyp_deact = NaN;
          case 4 % PD-Regler Hyperbolische Grenzfunktion
            wn_traj(2) = 50; % ist mit Verstärkung 1 sehr langsam (aber in richtige Richtung)
            wn_traj(8) = 20;
            optimcrit_limits_hyp_deact = NaN;
          case 5 % P-Regler IK-Jacobi-Konditionszahl
            wn_traj(5) = 1; % funktioniert ordentlich
          case 6 % PD-Regler IK-Jacobi-Konditionszahl
            wn_traj(5) = 1;
            wn_traj(9) = 0.2; % höhere D-Verstärkung kann Schwingung verursachen
          case 7 % P-Regler PKM-Jacobi-Konditionszahl
            wn_traj(6) = 1; % funktioniert manchmal ordentlich, aber ohne D-Anteil unzuverlässig
          case 8 % PD-Regler PKM-Jacobi-Konditionszahl
            wn_traj(3) = 0.8;
            wn_traj(6) = 1;
            % Höhere Werte für D-Verstärkung führen zu (Dauer-)Schwingungen
            wn_traj(10) = 0.1;
          case 9 % PD-Regler EE-Grenze für phi_z (quadratisch gewichtet)
            wn_traj(15) = 1;
            wn_traj(16) = 0.5;
            wn_traj(19) = 0.1; % zusätzliche Dämpfung
          case 10 % PD-Regler EE-Grenze für phi_z (quadratisch+hyperb. gewichtet)
            wn_traj(15) = 1; % P quadr.
            wn_traj(16) = 0.5; % D quadr.
            wn_traj(17) = 0.5; % P hyperb.
            wn_traj(18) = 0.05; % D hyperb. -> höhere Terme werden instabil
            wn_traj(19) = 0.1; % zusätzliche Dämpfung
        end
        % Roboter auf 3T2R einstellen
        RP.update_EE_FG(I_EE_full,I_EE_red);

        % IK mit Einzelpunkt-Verfahren berechnen
        s_ep_ii.wn(I_wn_ep) = wn_traj(I_wn_traj);
        % Zur Konsistenz zwischen Trajektorien- und Einzelpunkt-IK. Da
        % scale_lim gesetzt ist, wäre das eigentlich nicht notwendig.
        s_ep_ii.optimcrit_limits_hyp_deact = optimcrit_limits_hyp_deact;
%         s_ep_ii.wn(s_ep_ii.wn~=0) = 1; % Geht nicht. Sonst später kein Vergleich möglich!
        t1 = tic();
        [q_ep_ii, Phi,~,Stats_ep] = RP.invkin4(x_l, qs, s_ep_ii);
        t_ep = toc(t1);
        assert(all(abs(Stats_ep.PHI(1,RP.I_constr_red))<1e-9), 'Residuum im ersten Schritt muss Null sein');
        % IK mit Trajektorien-Verfahren berechnen. Setze virtuelle
        % Trajektorie, die keine Soll-Vorgaben hat. Dadurch entsteht eine
        % reine Nullraumbewegung.
        n = 2000; dt = 5e-3; % Länge der Trajektorie
        Traj_t = (0:dt:(n-1)*dt)';
        assert(length(Traj_t)==n, 'Zeit-Basis der virtuellen Trajektorie ist falsch');
        Traj_X = repmat(x_l', n, 1);
        Traj_XD = zeros(n,6); Traj_XDD = Traj_XD;
        s_traj_ii = struct('wn', wn_traj);
        s_traj_ii.enforce_qlim = false; % Keine strikte Einhaltung der Gelenkgrenzen
        s_traj_ii.enforce_qDlim = false;
        s_traj_ii.xlim = xlim_l;
        s_traj_ii.optimcrit_limits_hyp_deact = optimcrit_limits_hyp_deact;
        % Nullraumprojektion erst in Antriebskoordinaten, dann in Gesamt- 
        % koordinaten (nur Trajektorien-IK)
        s_traj_ii.thresh_ns_qa = 1; % full joint projector
        t1 = tic();
        [Q_ii, QD_ii, QDD_ii, Phi_ii,~,~,~,Stats_traj] = RP.invkin2_traj(Traj_X, Traj_XD, Traj_XDD, Traj_t, qs, s_traj_ii);
        t_traj = toc(t1);
        % Berechne Trajektorie nochmal mit anderer Methode. Dient dazu,
        % diese Implementierung gleich mit zu validieren.
        s_traj_ii2 = s_traj_ii;
        s_traj_ii2.thresh_ns_qa = inf; % actuated joint projector
        [Q_ii2, QD_ii2, QDD_ii2, Phi_ii2,~,~,~,Stats_traj2] = RP.invkin2_traj(Traj_X, Traj_XD, Traj_XDD, Traj_t, qs, s_traj_ii2);
        % Kürze die Trajektorie, falls Bewegung vorzeitig abgeklungen ist
        I_noacc = all(abs(QDD_ii)<1e-8,2);
        if all(I_noacc)
          if ~all(RP.I_EE == [1 1 1 0 0 1])
            error('Fehler in Parametrierung der Trajektorien-Funktion. Alle Beschleunigungen Null.');
          else
            warning('3T1R-PKM funktionieren aktuell noch nicht gut. Alle Beschleunigungen Null.');
            continue
          end
        end
        I_finishacc = find(I_noacc==0,1,'last');
        fprintf(['Pkt. %d/ Ori. %d/ Fall %d. IK berechnet: %d Schritte Ein', ...
          'zelpunkt-IK (%1.1fs; %1.1fms); %d Schritte Traj.-IK (%1.1fs; %1.1fms)\n'], k, l, ...
          ii, Stats_ep.iter, t_ep, 1e3*t_ep/Stats_ep.iter, I_finishacc, t_traj, 1e3*t_traj/I_finishacc);
        Q_ii = Q_ii(1:I_finishacc,:);
        QD_ii = QD_ii(1:I_finishacc,:);
        QDD_ii = QDD_ii(1:I_finishacc,:);
        Phi_ii = Phi_ii(1:I_finishacc,:);
        Traj_t = Traj_t(1:I_finishacc);
        Stats_traj.h = Stats_traj.h(1:I_finishacc,:);
        Q_ii2 = Q_ii2(1:I_finishacc,:);
        QD_ii2 = QD_ii2(1:I_finishacc,:);
        QDD_ii2 = QDD_ii2(1:I_finishacc,:);
        Phi_ii2 = Phi_ii2(1:I_finishacc,:);
        Stats_traj2.h = Stats_traj2.h(1:I_finishacc,:);
        q_traj_ii = Q_ii(end,:)'; % Ergebnis der Trajektorien-IK
        q_traj_ii2 = Q_ii2(end,:)'; % Ergebnis der Trajektorien-IK
        % Traj-Ergebnis kürzen (wenn Ende mit Toleranz erreicht ist)
        I_finalvalue = all(abs(repmat(q_traj_ii',I_finishacc,1)-Q_ii) < 1e-10,2) & ...
                       abs(repmat(Stats_traj.h(end,1),I_finishacc,1)-Stats_traj.h(:,1)) < 1e-10;
        I_finish = find(I_finalvalue==0,1,'last');
        Traj_t = Traj_t(1:I_finish,:);
        Q_ii = Q_ii(1:I_finish,:);
        QD_ii = QD_ii(1:I_finish,:);
        QDD_ii = QDD_ii(1:I_finish,:);
        Phi_ii = Phi_ii(1:I_finish,:);
        Stats_traj.h = Stats_traj.h(1:I_finish,:);
        Q_ii2 = Q_ii2(1:I_finish,:);
        QD_ii2 = QD_ii2(1:I_finish,:);
        QDD_ii2 = QDD_ii2(1:I_finish,:);
        Phi_ii2 = Phi_ii2(1:I_finish,:);
        Stats_traj2.h = Stats_traj2.h(1:I_finish,:);
        % Speichere EE-Pose resultierend aus Gelenkwinkeln aus dem
        % Konvergenzverlauf.
        % Bei Traj.-IK keine Normalisierung der Euler-Winkel vornehmen
        [X_ii, XD_ii] = RP.fkineEE2_traj(Q_ii, QD_ii);
        X_ii(:,6) = denormalize_angle_traj(X_ii(:,6), XD_ii(:,6), Traj_t);
        [X_ii2, XD_ii2] = RP.fkineEE2_traj(Q_ii2, QD_ii2);
        X_ii2(:,6) = denormalize_angle_traj(X_ii2(:,6), XD_ii2(:,6), Traj_t);
        % Benutze bei Einzelpunkt-IK nur i.O.-Posen (keine Posen, bei denen
        % die Ketten nicht geschlossen sind).
        Stats_ep.X = RP.fkineEE2_traj(Stats_ep.Q);
        I_invalid = any(abs(Stats_ep.PHI(:,RP.I_constr_red))>1e-3,2);
        Stats_ep.X(I_invalid,:) = NaN;

        % TODO: Um 2pi verschoben, so dass am nächsten an Startwert
        % (aktuell springt das Ergebnis im Bild, falls es über +/- pi geht.
        x_traj_ii = RP.fkineEE_traj(q_traj_ii')';
        x_traj_ii2 = RP.fkineEE_traj(q_traj_ii2')';
        x_ep_ii = RP.fkineEE_traj(q_ep_ii')';
        
        % Summe der positionsbezogenen Leistungsmerkmale der Traj. am Ende.
        % Schließe geschwindigkeitsbezogene Merkmale aus Vergleich aus.
        % Dadurch ist der Vergleich von Positions- und Traj.-IK möglich.
        h_traj_ii = Stats_traj.h(I_finish,:);
        h_traj_ii_sum = sum(Stats_traj.h(I_finish,1+I_stats_traj)*s_ep_ii.wn(I_wn_ep));
        % Leistungsmerkmale der Positions-IK am Ende
        h_ep_ii = Stats_ep.h(1+Stats_ep.iter,:);
        % Bilde Summe der Merkmale neu (eigentlich nur notwendig, wenn Ge-
        % wichtungen zwischen Trajektorie und Einzelpunkt unterschiedlich
        % sind aber hier trotzdem die Ergebnisse verglichen werden sollen.
        h_ep_ii_sum = sum(Stats_ep.h(1+Stats_ep.iter,1+I_wn_ep)'.*wn_traj(I_wn_traj));
        % Ergebnis prüfen
        reserr_q = q_traj_ii - q_ep_ii;
        reserr_h_sum = h_traj_ii_sum - h_ep_ii_sum;
        I_wnact = s_ep_ii.wn ~= 0; % Indizes der aktiven Nebenbedingungen
        hs_ii = Stats_traj.h(1,1); % Startwert der Nebenbedingungen (bei Traj.)
        % Die Gelenkwinkel sind bei beiden Methoden beim Start nicht
        % identisch, da in Traj.-IK noch ein Korrekturschritt durchgeführt
        % wird. Die Optimierungskriterien sind daher auch nicht identisch.
%         if ~isinf(Stats_ep.h(1,1)) && ~isinf(hs_ii)
%           assert(abs(hs_ii - Stats_ep.h(1,1)) < 1e-10, ['Startwert des ', ...
%             'Kriteriums bei Einzelpunkt und Trajektorie ungleich.']);
%         end
        step_h_traj = h_traj_ii_sum - hs_ii; % Verbesserung der Nebenbedingungen bei Traj.
        step_h_ep = h_ep_ii_sum - hs_ii; % Verbesserung der Nebenbedingungen bei Einzelpunkt
        % Bestimme relativen Fehler (bezogen auf die Methode mit dem
        % besseren Schritt; Schritte sind negativ, da Minimierung)
        reserr_h_rel = reserr_h_sum/min(step_h_traj,step_h_ep);
        fprintf(['Punkt %d/ Orientierung %d (x6=%1.1f deg). Ergebnis aus Traj.-IK ist um ', ...
          '%1.1e schlechter als Einzelpunkt-IK (%1.5f vs %1.5f). Verbesserung ', ...
          'gegen Start: %1.3f bzw. %1.3f\n'], ...
          k, l, 180/pi*x_l(6), h_traj_ii_sum - h_ep_ii(1), ...
          h_traj_ii_sum, Stats_ep.h(Stats_ep.iter,1), -step_h_traj, -step_h_ep);
        % Speichere Ergebnis in Tabelle
        ii_restab = ii_restab + 1;
        ResStat(ii_restab,:) = ResStat_emptyrow;
        ResStat.PktNr(ii_restab) = k;
        ResStat.OriNr(ii_restab) = l;
        ResStat.IK_Fall(ii_restab) = ii;
        ResStat.h_start(ii_restab) = hs_ii;
        ResStat.h_err_abs(ii_restab) = reserr_h_sum;
        ResStat.h_err_rel(ii_restab) = reserr_h_rel(1);
        ResStat.h_step_traj(ii_restab) = step_h_traj;
        ResStat.h_step_ep(ii_restab) = step_h_ep;        
        ResStat.Error(ii_restab) = 0;
        
        % Prüfe hier, ob ein Fehler vorliegt und breche dann ab, nachdem
        % die Bilder gezeichnet wurden.
        raise_error_h = abs(reserr_h_rel)>5e-2 && reserr_h_sum >1e-6;
        if isnan(raise_error_h)
          warning('Ungültiges Ergebnis');
          ResStat.Error(ii_restab) = 10;
          raise_error_h = true;
        end
        % Prüfung auf NaN: Deaktivieren. Trajektorie kann auch instabil
        % werden.
%         if ~all(isinf([h_ep_ii_sum;h_traj_ii_sum])) && ... % inf-inf=nan. Wäre kein Fehler
%            ~all(isinf([hs_ii;h_traj_ii_sum])) && ...
%             (isnan(reserr_h_sum) || isnan(step_h_traj)) % NaN deutet auf Fehler hin
%           error('Ein Wert wird NaN. Darf nicht passieren.');
%         end
        if any(abs(reserr_h_sum) > 1e-3)
          warning('Zielfunktion weicht absolut bei beiden Methoden ab. Aber kein Fehler, da eventuell auch Verzweigung der Lösung.');
        end
        if raise_error_h
          warning('Zielfunktion weicht bei beiden Methoden zu stark voneinander ab (relativer Fehler %1.1f%%)', reserr_h_rel*100);
        end
        if ~any(ii == [1 3 5 7]) && step_h_traj >= 1e-6
          % In Traj.-IK Schwingungen, wenn kein PD-Regler benutzt wird.
          warning('Nebenbedingungen haben sich bei Traj.-IK verschlechtert');
          if ResStat.Error(ii_restab)==0, ResStat.Error(ii_restab) = 1; end
          raise_error_h = true;
        end
        if step_h_ep >= 1e-6
          warning('Nebenbedingungen haben sich bei Einzelpunkt.-IK verschlechtert');
          if ResStat.Error(ii_restab)<10
            ResStat.Error(ii_restab) = ResStat.Error(ii_restab) + 2;
          end
          raise_error_h = true;
        end
        % Normiere die Größen
        Q_ii_norm = (Q_ii - repmat(qlim(:,1)',size(Q_ii,1),1)) ./ ...
                    repmat(qlim(:,2)'-qlim(:,1)',size(Q_ii,1),1);
        QD_ii_norm = (QD_ii - repmat(qDlim(:,1)',size(QD_ii,1),1)) ./ ...
                    repmat(qDlim(:,2)'-qDlim(:,1)',size(QD_ii,1),1);
        QDD_ii_norm = (QDD_ii - repmat(qDDlim(:,1)',size(QDD_ii,1),1)) ./ ...
                    repmat(qDDlim(:,2)'-qDDlim(:,1)',size(QDD_ii,1),1);
        Q_ii2_norm = (Q_ii2 - repmat(qlim(:,1)',size(Q_ii2,1),1)) ./ ...
                    repmat(qlim(:,2)'-qlim(:,1)',size(Q_ii2,1),1);
        QD_ii2_norm = (QD_ii2 - repmat(qDlim(:,1)',size(QD_ii2,1),1)) ./ ...
                    repmat(qDlim(:,2)'-qDlim(:,1)',size(QD_ii2,1),1);
        QDD_ii2_norm = (QDD_ii2 - repmat(qDDlim(:,1)',size(QDD_ii2,1),1)) ./ ...
                    repmat(qDDlim(:,2)'-qDDlim(:,1)',size(QDD_ii2,1),1);
        Stats_ep.Q_norm = (Stats_ep.Q - repmat(qlim(:,1)',size(Stats_ep.Q,1),1)) ./ ...
                    repmat(qlim(:,2)'-qlim(:,1)',size(Stats_ep.Q,1),1);
        % Prüfe, ob Grenzen eingehalten werden. Ist teilweise nicht anders
        % möglich, wenn Grenzen in mehreren Gelenken gleichzeitig verletzt
        % werden.
        if any(QDD_ii_norm(:) < -1) || any(QDD_ii_norm(:)>2)
          warning(['Gelenkbeschleunigungsgrenzen werden in Traj. massiv ', ...
            'verletzt (qDD norm: %1.1f bis %1.1f)'], min(QDD_ii_norm(:)), ...
            max(QDD_ii_norm(:)));
          if ResStat.Error(ii_restab)<=3, ResStat.Error(ii_restab) = 4; end
        end
        if any(QD_ii_norm(:) < -0.2) || any(QD_ii_norm(:)>1.2)
          warning(['Gelenkgeschwindigkeitsgrenzen werden in Traj. massiv ', ...
            'verletzt (qD norm: %1.1f bis %1.1f)'], min(QD_ii_norm(:)), ...
            max(QD_ii_norm(:)));
          if ResStat.Error(ii_restab)<=3, ResStat.Error(ii_restab) = 5; end
        end
        if any(Q_ii_norm(:) < -0.1) || any(Q_ii_norm(:)>1.1)
          warning(['Gelenkpositionsgrenzen werden in Traj. massiv ', ...
            'verletzt (q norm: %1.1f bis %1.1f)'], min(Q_ii_norm(:)), ...
            max(Q_ii_norm(:)));
          if ResStat.Error(ii_restab)<=3, ResStat.Error(ii_restab) = 6; end
        end

        % Vergleich
        if usr_plot_debug || (raise_error_h&&usr_plot_on_err)
          progr_ep = (0:1:Stats_ep.iter)'/(Stats_ep.iter);
          progr_traj = Traj_t/Traj_t(end);
          % Bild: Gelenkpositionen
          change_current_figure(1);clf;set(1,'Name','q','NumberTitle','off');
          for i = 1:RP.NJ
            subplot(ceil(sqrt(RP.NJ)),ceil(sqrt(RP.NJ)),i); hold on;
            plot(100*progr_ep, Stats_ep.Q_norm(1:Stats_ep.iter+1,i));
            plot(100*progr_traj, Q_ii_norm(:,i));
            plot(100*progr_traj, Q_ii2_norm(:,i));
            ylabel(sprintf('q %d (norm)', i)); grid on;
            xlabel('Fortschritt in %');
          end
          linkxaxes
          legend({'EP', 'Traj (FullProj)', 'Traj (ActProj)'});
          sgtitle('Gelenkwinkel');
          if usr_save_figures
            saveas(1, fullfile(respath, [filename_pre,'Gelenke.fig']));
            saveas(1, fullfile(respath, [filename_pre,'Gelenke.png']));
          end
          
          % Bild: Gelenk-Geschwindigkeiten
          change_current_figure(11);clf;set(11,'Name','qD','NumberTitle','off');
          for i = 1:RP.NJ
            subplot(ceil(sqrt(RP.NJ)),ceil(sqrt(RP.NJ)),i); hold on;
            plot(100*progr_traj, QD_ii_norm(:,i));
            plot(100*progr_traj, QD_ii2_norm(:,i));
            ylabel(sprintf('qD %d (norm)', i)); grid on;
            xlabel('Fortschritt in %');
          end
          linkxaxes
          legend({'Traj (FullProj)', 'Traj (ActProj)'});
          sgtitle('Gelenkgeschwindigkeiten (Traj.-IK)');
          if usr_save_figures
            saveas(11, fullfile(respath, [filename_pre,'Gelenkgeschwindigkeit.fig']));
            saveas(11, fullfile(respath, [filename_pre,'Gelenkgeschwindigkeit.png']));
          end
          
          % Bild: Gelenk-Beschleunigungen
          change_current_figure(12);clf;set(12,'Name','qDD','NumberTitle','off');
          for i = 1:RP.NJ
            subplot(ceil(sqrt(RP.NJ)),ceil(sqrt(RP.NJ)),i); hold on;
            plot(100*progr_traj, QDD_ii_norm(:,i));
            plot(100*progr_traj, QDD_ii2_norm(:,i));
            ylabel(sprintf('qDD %d (norm)', i)); grid on;
            xlabel('Fortschritt in %');
          end
          linkxaxes
          legend({'Traj (FullProj)', 'Traj (ActProj)'});
          sgtitle('Gelenkbeschleunigungen (Traj.-IK)');
          if usr_save_figures
            saveas(12, fullfile(respath, [filename_pre,'Gelenkbeschleunigung.fig']));
            saveas(12, fullfile(respath, [filename_pre,'Gelenkbeschleunigung.png']));
          end
          
          % Bild: Zielkriterien (Zeitverlauf)
          change_current_figure(2);clf;set(2,'Name','h','NumberTitle','off');
          for i = 1:6
            subplot(2,3,i); hold on;
            plot(100*progr_ep(1:end-1), Stats_ep.h(1:Stats_ep.iter,1+I_wn_ep(i)));
            plot(100*progr_traj, Stats_traj.h(:,1+I_stats_traj(i)));
            plot(100*progr_traj, Stats_traj2.h(:,1+I_stats_traj(i)));
            ylabel(sprintf('h %d (%s) (wn=%1.1f)', i, hnames{i}, s_ep_ii.wn(I_wn_ep(i)))); grid on;
            xlabel('Fortschritt in %');
          end
          linkxaxes
          legend({'EP', 'Traj (FullProj)', 'Traj (ActProj)'});
          sgtitle('Zielfunktionen der IK');
          if usr_save_figures
            saveas(2, fullfile(respath, [filename_pre,'Zielfunktion.fig']));
            saveas(2, fullfile(respath, [filename_pre,'Zielfunktion.png']));
          end

          % Bild: Redundante Koordinate
          change_current_figure(25);clf; hold on; grid on;
          plot(100*progr_ep, 180/pi*Stats_ep.X(1:Stats_ep.iter+1,6));
          plot(100*progr_traj, 180/pi*X_ii(:,6));
          plot(100*progr_traj, 180/pi*X_ii2(:,6));
          if any(wn_traj(15:19))
            plot([0; 100], 180/pi*(repmat(xlim_abs(6,:)',1,2)'), 'r--');
          end
          legend({'EP', 'Traj (FullProj)', 'Traj (ActProj)'}); grid on;
          xlabel('Fortschritt in %');
          ylabel('Redundante Koordinate x6 in deg');
          sgtitle('Redundante Koordinate x6');
          set(25,'Name','x6','NumberTitle','off');
          if usr_save_figures
            saveas(25, fullfile(respath, [filename_pre,'RedKoordX.fig']));
            saveas(25, fullfile(respath, [filename_pre,'RedKoordX.png']));
          end
          
          % Bild: Konsistenz von Beschleunigung und Geschwindigkeit
%           Q_int = cumtrapz(Traj_t, QD_ii) + repmat(Q_ii(1,:),length(Traj_t),1);
%           QD_int = cumtrapz(Traj_t, QDD_ii) + repmat(QD_ii(1,:),length(Traj_t),1);
%           change_current_figure(4);clf;
%           for i = 1:RP.Leg(1).NJ % Für alle Beinketten reicht der Platz nicht
%             subplot(3,RP.Leg(1).NJ,sprc2no(3,RP.Leg(1).NJ,1,i)); hold on;
%             plot(Traj_t, Q_ii(:,i));
%             plot(Traj_t, Q_int(:,i));
%             title(sprintf('q %d', i)); grid on;
%             if i == RP.Leg(1).NJ, legend({'direkt', 'Integral'}); end
%             subplot(3,RP.Leg(1).NJ,sprc2no(3,RP.Leg(1).NJ,2,i)); hold on;
%             plot(Traj_t, QD_ii(:,i));
%             plot(Traj_t, QD_int(:,i));
%             if i ==RP.Leg(1).NJ, legend({'direkt', 'Integral'}); end
%             title(sprintf('qD %d', i)); grid on;
%             subplot(3,RP.Leg(1).NJ,sprc2no(3,RP.Leg(1).NJ,3,i)); hold on;
%             plot(Traj_t, QDD_ii(:,i));
%             title(sprintf('qDD %d', i)); grid on;
%             xlabel('Virt. Zeit in s');
%           end
%           linkxaxes
%           sgtitle('Konsistenz q');
        end
        
        % Bild: Roboter
        if usr_plot_robot || (raise_error_h&&usr_plot_on_err)
          change_current_figure(100);clf;
          set(100,'Name','Rob','NumberTitle','off');
          title(sprintf('Roboter in Punkt %d', k));
          hold on;grid on;
          xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
          view(3);
          s_plot = struct( 'straight', 0);
          RP.plot( qs, xs, s_plot );
          plot3(XL(:,1), XL(:,2), XL(:,3), 'k+');
          if usr_save_figures
            saveas(100, fullfile(respath, [filename_pre,'Robot.fig']));
            saveas(100, fullfile(respath, [filename_pre,'Robot.png']));
          end
        end

        % In Zielfunktions-Bild eintragen
        if usr_plot_objfun || (raise_error_h && usr_plot_on_err)
          change_current_figure(20);clf;set(20,'Name','h_x6','NumberTitle','off');
          for jj = 1:6
            subplot(2,3,jj); hold on; grid on;
            plot(x_test_ges(:,6)*180/pi, h_ges(:,I_wn_ep(jj)));
            ylabel(sprintf('h %d (%s)', jj, hnames{jj})); grid on;
            xlabel('x6 in deg');
          end
          for jj = 1:length(I_wn_ep)
            if s_ep_ii.wn(I_wn_ep(jj))==0, continue; end
            subplot(2,3,jj); hold on;
            % Debug: Ortskurve der Zwischenzustände einzeichnen
            hdl3=plot(X_ii(:,6)*180/pi,Stats_traj.h(:,1+I_stats_traj(jj)), 'r+-');
            hdl4=plot(Stats_ep.X(:,6)*180/pi,Stats_ep.h(:,1+I_wn_ep(jj)), 'gx-');
            
            hdl0=plot(180/pi*xs(6), hs(I_wn_ep(jj)), 'cs', 'MarkerSize', 16);
            hdl1=plot(x_traj_ii(6)*180/pi, h_traj_ii(end,1+I_stats_traj(jj)), 'r^', 'MarkerSize', 16);
            hdl2=plot(x_ep_ii(6)*180/pi, h_ep_ii(1+I_wn_ep(jj)), 'gv', 'MarkerSize', 16);
%             plot([xs(6);x_traj_ii(6)]*180/pi, [hs(jj);h_traj_ii(end,1+I_wn_traj(jj))], 'r-');
%             plot([xs(6);x_ep_ii(6)]*180/pi, [hs(jj);h_ep_ii(1+jj)], 'g-');
          end
          legend([hdl0;hdl1;hdl2;hdl3;hdl4], {'Start', 'Traj Erg.', ...
            'Einzel Erg.', 'Traj Dbg.', 'Einzel Dbg.'});
          linkxaxes
          if usr_save_figures
            saveas(20, fullfile(respath, [filename_pre,'Ortskurve_Ziel_RedKoord.fig']));
            saveas(20, fullfile(respath, [filename_pre,'Ortskurve_Ziel_RedKoord.png']));
          end
        end
        
        % Animation: Bewegung von Start zu Ziel
        if usr_save_anim
          for iii = 1:2 % Für beide Methoden Animation erzeugen
            change_current_figure(200);clf;hold all;
            if strcmp(get(200, 'windowstyle'), 'docked'), set(200, 'windowstyle', 'normal'); end
            set(200, 'name', 'Anim', ...
              'color','w', 'NumberTitle', 'off', 'units','normalized',...
              'outerposition',[0 0 1 1]); % Vollbild
            view(3); axis auto; hold on; grid on;
            xlabel('x in m');ylabel('y in m');zlabel('z in m');
            maxduration_animation = 5; % Dauer der Animation als mp4 (in s)
            if iii == 1 % Benutze Positions-IK für Animation
              t_Vid = (0:1/30*(Stats_ep.iter/maxduration_animation):Stats_ep.iter)';
              I_anim = knnsearch( (1:Stats_ep.iter)' , t_Vid ); % Berechne Indizes in Traj.-Zeitstempeln
              Q_anim = Stats_ep.Q(I_anim,:);
              X_anim = Stats_ep.X(I_anim,:);
              anim_name = 'Animation_Position';
            else % Benutze Trajektorien-IK für Animation
              t_Vid = (0:1/30*(Traj_t(end)/maxduration_animation):Traj_t(end))';
              I_anim = knnsearch( Traj_t , t_Vid );
              Q_anim = Q_ii(I_anim,:);
              X_anim = X_ii(I_anim,:);
              anim_name = 'Animation_Traj';
            end
            if length(I_anim) > max(I_anim)
              warning(['Es gibt nicht genug Zwischenschritte für ein ', ...
                'flüssiges Video (%d Video-Bilder, %d IK-Schritte)'], length(I_anim), max(I_anim));
            end
            s_plot = struct( 'ks_legs', [], 'straight', 0, 'mode', 4);
            s_anim = struct('mp4_name', [fullfile(respath, [filename_pre, anim_name]),'.mp4'] );
            RP.anim( Q_anim, X_anim, s_anim, s_plot);
          end
        end
        if raise_error_h && usr_raise_on_err
          error(['Punkt %d/ Orientierung %d: Beide Methoden kommen nicht ', ...
            'zum gleichen Ergebnis (h)'], k, l);
        end
      end % for ii (Testfälle der IK)
    end % for l (Drehung des Endeffektors)
    fprintf('%d/%d Posen für Punkt %d geprüft. Bei restlichen IK nicht erfolgreich\n', ...
      num_ik_qs_successfull, length(x6_l_range), k);
  end % for k (Punkte)
  fprintf('Berechnungen für Rob %d (%s) abgeschlossen\n', robnr, RP.mdlname);
  % Reduzieren auf tatsächliche Daten
  ResStat = ResStat(1:ii_restab,:);
  ResStat_Start = ResStat_Start(1:ii_restab_start,:);
  % Ergebnis für Roboter speichern
  writetable(ResStat, fullfile(respath, sprintf('Rob%d_%s_Stats_Nullspace.csv', ...
    robnr, RP.mdlname)), 'Delimiter', ';');
  writetable(ResStat_Start, fullfile(respath, sprintf('Rob%d_%s_Stats_IK.csv', ...
    robnr, RP.mdlname)), 'Delimiter', ';');
  % Legende eintragen
  fid = fopen(fullfile(respath, sprintf('Rob%d_%s_Stats_Nullspace.csv.txt', ...
    robnr, RP.mdlname)), 'w');
  fprintf(fid, ['Informationen zu Tabellenspalten:\n', ...
    'PktNr: Laufende Nummer des betrachteten Punkts\n', ...
    'OriNr: Nummer verschiedener z-Orientierungen als Startwert\n', ...
    'IK_Fall: Nummer bezüglich verwendeter Optimierungskriterien\n', ...
    'h_start: Anfangswert der Optimierungskriterien\n', ...
    'h_err_abs: Fehler zwischen Einzelpunkt- und Trajektorienmethode\n', ...
    'h_err_rel: Relativer Fehler\n', ...
    'h_step_traj: Änderung des Optimierungskriteriums bei Trajektorien-Methode (negativ ist gut)\n', ...
    'h_step_ep: Änderung bei Einzepunkt-Methode\n', ...
    'Error: Fehlercodes\n', ...
    '\t0: Kein Fehler. Alles in Ordnung\n', ...
    '\t1: Verschlechterung in Trajektorien-IK\n', ...
    '\t2: Verschlechterung in Einzelpunkt-IK\n', ...
    '\t3: Verschlechterung in beiden IK-Methoden\n', ...
    '\t4: Verletzung der Beschleunigungsgrenzen in Traj.-IK\n', ...
    '\t5: Verletzung der Geschwindigkeitsgrenzen in Traj.-IK\n', ...
    '\t6: Verletzung der Gelenkwinkelgrenzen in Traj.-IK\n', ...
    '\t10: Ungültiges Ergebnis in IK\n']);
  fclose(fid);
  % Ergebnisse prüfen: Bei zu vielen Fehlern Abbruch.
  IK_success_ratio_start = sum(ResStat_Start.Erfolg==1)/size(ResStat_Start,1);
  fprintf('Die IK-Erfolgsquote liegt bei %1.1f%% (%d/%d Konfigurationen)\n', ...
    100*IK_success_ratio_start, sum(ResStat_Start.Erfolg==1), size(ResStat_Start,1));
  IK_LimViolratio_start = sum(ResStat_Start.Grenzen==0)/size(ResStat_Start,1);
  fprintf('In %1.1f%% der IK-Fälle Verletzung der Grenzen (%d/%d Konfigurationen)\n', ...
    100*IK_LimViolratio_start, sum(ResStat_Start.Grenzen==0), size(ResStat_Start,1));
  if IK_success_ratio_start < 0.4
    % Bei 3RRR funktionieren viele Konfigurationen nicht, da die PKM nicht
    % so weit drehen kann (hat auch nichts mit Grenzen zu tun).
    error(['Die IK-Erfolgsquote für die Startpose liegt nur bei %1.1f%%. ', ...
      'Vermutlich noch systematischer Fehler'], 100*IK_success_ratio_start);
  end
  fprintf('Die Nullraumbewegung war in %1.1f%% der Fälle optimal (%d/%d)\n', ...
    100*sum(ResStat.Error==0)/size(ResStat,1), sum(ResStat.Error==0), size(ResStat,1));
  raiseerr = false;
  for ii = [2 4 6 8 9 10]
    ResStat_Filt = ResStat(ResStat.IK_Fall==ii,:);
    Nullspace_Error_ratio = sum(ResStat_Filt.Error~=0)/size(ResStat_Filt,1);
    if ~all(RP.I_EE == [1 1 1 0 0 1]) && Nullspace_Error_ratio > 0.25 || ...% TODO: Kann kleiner gewählt werden, wenn Parameter getuned sind.
        all(RP.I_EE == [1 1 1 0 0 1]) && Nullspace_Error_ratio > 0.4 % 3T1R funktioniert noch nicht gut.
      warning(['Für Fall %d wird in %1.1f%% der Untersuchungen nicht die ', ...
        '(lokal) optimale Lösung im Nullraum gefunden'], ii, 100*Nullspace_Error_ratio);
      raiseerr = true;
    end
  end
  if any(raiseerr)
    error('Konvergenz der Nullraumbewegung ist zu schlecht. Deutet auf Fehler hin.');
  end
end
