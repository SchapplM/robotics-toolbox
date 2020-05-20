% Benchmark-Test für Aufgabenredundanz bei 6UPS, 6PUS und 6RRRRRR
%
% Ablauf:
% * Beispiel-Parameter und Roboter definieren
% * Gelenkwinkel einstellen
% * Jacobi-Beispiel für 3T2R-Jacobi
% * Beispieltrajektorie definieren und berechnen mit zwei Verfahren
% * Auswertung
%
% Ergebnis:
% * Mit Nullraumbewegung werden die Gelenkwinkelgrenzen immer gehalten
% * Mit rein serieller Berechnung (ohne Nullraum) werden die Grenzen
%   wesentlich stärker angenähert, aber auch nicht verletzt.
%
% Siehe auch: 
% * ParRob_class_example_6UPS_3T2R.m
% * Schappler et al 2019: Modeling Parallel Robot Kinematics for 3T2R and 
%   3T3R Tasks using Reciprocal Sets of Euler Angles. MDPI Robotics.

% Junnan Li, HiWi bei Moritz Schappler, 2020-05
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc

%% Definitionen, Benutzereingaben
short_traj = false; % Trajektorie stark abkürzen, um prinzipielle Funktionalität zu zeigen
eckpunkte_berechnen = true; % Einzelpunkt-IK für alle Eckpunkte berechnen
test_class_methods = true; % Zusätzlich zweite Implementierung rechnen
debug_plot = false;

rob_path = fileparts(which('robotics_toolbox_path_init.m'));
respath = fullfile(rob_path, 'examples_tests', 'results', 'ParRob_3T3R_task_red_benchmark');
mkdirs(respath);

I_EE_3T3R = logical([1 1 1 1 1 1]);
I_EE_3T2R = logical([1 1 1 1 1 0]);
%% Klasse für PKM erstellen (basierend auf serieller Beinkette)
% Robotermodell aus PKM-Bibliothek laden.
for robnr = 1:3 % 1: 6UPS; 2: 6PUS; 3:6RRRRRR
  if isempty(which('serroblib_path_init.m'))
    warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
    return
  end
  %% Klasse für PKM erstellen (basierend auf PKM-Bibliothek)
  if isempty(which('parroblib_path_init.m'))
    warning('Repo mit parallelen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
    return
  end
  if robnr == 1
    RP = parroblib_create_robot_class('P6RRPRRR14V3G1P1A1', 0.5, 0.2);
    I_qa = false(36,1);
    I_qa(3:6:36) = true;
    RP.update_actuation(I_qa);
    RP.phi_P_B_all = zeros(3,6);
  elseif robnr == 2
    RP = parroblib_create_robot_class('P6PRRRRR6V2G8P1A1', [0.8;0.3;pi/3], 0.2);
    pkin_6_PUS = zeros(length(RP.Leg(1).pkin),1); % Namen, siehe RP.Leg(1).pkin_names
    pkin_6_PUS(strcmp(RP.Leg(1).pkin_names,'a2')) = 0.2;
    pkin_6_PUS(strcmp(RP.Leg(1).pkin_names,'theta1')) = -pi/2; % So drehen, dass a2 nach oben zeigt
    pkin_6_PUS(strcmp(RP.Leg(1).pkin_names,'a4')) = 0.5;
    pkin_6_PUS(4:5) = pi/2; % alpha2, alpha3
    for i = 1:RP.NLEG
      RP.Leg(i).update_mdh(pkin_6_PUS);
      % EE-KS mit Null-Rotation vorbelegen
      RP.Leg(i).update_EE(zeros(3,1),zeros(3,1));
    end
  elseif robnr == 3
    RP = parroblib_create_robot_class('P6RRRRRR10G1P1A1', 1.3, 0.3);
    pkin_gen = zeros(length(RP.Leg(1).pkin_names),1);
    % Nachbearbeitung einiger Kinematikparameter
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'d1')) = 0.0; % Ergibt kein Sinn für PKM
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'a2')) = 0.2;
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'a3')) = 0.65;
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'d4')) = 0.85;
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'a4')) = 0.2;
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'a5')) = 0.15;
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'alpha2')) = pi/2;
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'alpha4')) = pi/2;
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'a6')) = 0.1;
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'d6')) = 0.0; % ist für P4 ungünstig
    for i = 1:RP.NLEG
      RP.Leg(i).update_mdh(pkin_gen);
    end
    I_qa = false(36,1);
    I_qa(1:6:36) = true;
    RP.update_actuation(I_qa);
  end
  % Debug: Alle Vorlagen-Funktionen neu generieren:
  % serroblib_create_template_functions({RP.Leg(1).mdlname}, false, true);
  % parroblib_create_template_functions({RP.mdlname(1:end-2)}, false, true);
  RP.fill_fcn_handles(true,true);

  %% Grenzen für die Gelenkpositionen setzen
  % Dadurch wird die Schrittweite bei der inversen Kinematik begrenzt (auf 5%
  % der Spannbreite der Gelenkgrenzen) und die Konfiguration klappt nicht um.
  for i = 1:RP.NLEG
    % Begrenze die Winkel der Kugel- und Kardangelenke auf +/- 360°
    RP.Leg(i).qlim = repmat([-2*pi, 2*pi], RP.Leg(i).NQJ, 1);
    % Begrenze die Länge der Schubgelenke
    RP.Leg(i).qlim(RP.Leg(i).MDH.sigma==1,:) = ...
      repmat([0.1, 1.5], sum(RP.Leg(i).MDH.sigma==1), 1);
  end
  
  %% Startpose bestimmen
  % Mittelstellung im Arbeitsraum
  X0 = [ [0.00;0.00;0.6]; [0;0;0]*pi/180 ];
  for i = 1:10 % Mehrere Versuche für "gute" Pose
    q0 = 0.5+rand(36,1); % Startwerte für numerische IK (zwischen -0.5 und 0.5 rad)
    q0(RP.MDH.sigma==1) = 0.5; % mit Schubaktor größer Null anfangen (damit Konfiguration nicht umklappt)
    % Inverse Kinematik auf zwei Arten berechnen
    [q1, Phi] = RP.invkin1(X0, q0);
    if any(abs(Phi) > 1e-8)
      error('Inverse Kinematik konnte in Startpose nicht berechnet werden');
    end
    [qs, Phis] = RP.invkin_ser(X0, rand(36,1));
    if any(abs(Phis) > 1e-6)
      error('Inverse Kinematik (für jedes Bein einzeln) konnte in Startpose nicht berechnet werden');
    end
    if any(qs(RP.MDH.sigma==1) < 0)
      warning('Versuch %d: Start-Konfiguration ist umgeklappt mit Methode Seriell. Erneuter Versuch.', i);
      if i == 10
        return
      else
        continue;
      end
    else
      break;
    end
  end
  
  %% Zwangsbedingungen in Startpose testen
  % Initialisierung mit vollständigen FG (3T3R).
  RP.update_EE_FG(I_EE_3T3R, I_EE_3T3R);
  Phi1=RP.constr1(qs, X0);
  Phit1=RP.constr1_trans(qs, X0);
  Phir1=RP.constr1_rot(qs, X0);
  if any(abs(Phi1) > 1e-6)
    error('ZB in Startpose ungleich Null');
  end
  
  %% Gelenkwinkelgrenzen festlegen (für Optimierung
  for i = 1:RP.NLEG
    q_i = qs(RP.I1J_LEG(i):RP.I2J_LEG(i));
    qlim_i = repmat(q_i,1,2);
    % Grenzen um die aktuelle Konfiguration herum wählen
    I1 = RP.Leg(i).MDH.sigma==1;
    % Schubachsen: Maximaler Hub +-500mm
    qlim_i(I1,:) = repmat(q_i(I1),1,2) + repmat([-0.5, 0.5], sum(I1),1);
    I0 = RP.Leg(i).MDH.sigma==0;
    % Kippwinkel der Kardan- und Kugelgelenke max. 60° in jede Richtung
    qlim_i(I0,:) = repmat(q_i(I0),1,2) + repmat([-pi/3, pi/3], sum(I0),1);
    RP.Leg(i).qlim = qlim_i;
  end
  % qlim für gesamte PKM festlegen
  qlim = NaN(RP.NJ,2);
  J1 = 1;
  for i = 1:RP.NLEG
    J2 = J1+RP.Leg(i).NQJ-1;
    qlim(J1:J2,:) = RP.Leg(i).qlim;
    J1 = J2+1;
  end
  %% Initialisierung Teil 2
  % Roboter auf 3T2R einstellen
  RP.update_EE_FG(I_EE_3T3R, I_EE_3T2R);
  %% Jacobi-Matrizen auswerten
  
  [G_q_red,G_q_voll] = RP.constr3grad_q(qs, X0);
  [~,G_x_voll] = RP.constr3grad_x(qs, X0); 
  
  % Testen der Komponentenaufteilung
  G_q = G_q_voll(RP.I_constr_red,:);
  G_x = G_x_voll(RP.I_constr_red,:);
  if any(G_q_red(:)-G_q(:))
    error('Aufteilung der ZB-Komponenten stimmt nicht zwischen constr3grad_q/constr3grad_x/ParRob');
  end
  
  % Aufteilung der Ableitung nach den Gelenken in Gelenkklassen
  % * aktiv/unabhängig (a),
  % * passiv+schnitt/abhängig (d)
  G_a = G_q(:,RP.I_qa);
  G_d = G_q(:,RP.I_qd);
  % Jacobi-Matrix zur Berechnung der abhängigen Gelenke und EE-Koordinaten
  G_dx = [G_d, G_x];
  
  fprintf('%s: Rang der vollständigen Jacobi der inversen Kinematik: %d/%d\n', ...
    RP.mdlname, rank(G_q), RP.NJ);
  fprintf('%s: Rang der vollständigen Jacobi der direkten Kinematik: %d/%d\n', ...
    RP.mdlname, rank(G_dx), sum(RP.I_EE_Task)+sum(RP.I_qd));
  fprintf('%s: Rang der Jacobi der aktiven Gelenke: %d/%d\n', ...
    RP.mdlname, rank(G_a), sum(RP.I_EE_Task));
  
  %% Beispieltrajektorie berechnen und zeichnen
  % IK-Grundeinstellungen
  s = struct('Phit_tol', 1e-9, 'Phir_tol', 1e-9, ...
    'maxstep_ns', 1e-5, ... % Schrittweite für Nullraum-Inkremente gering halten
    'wn', [0;1], 'K', 7e-1*ones(RP.NJ,1), ...
    'Kn', 0.7*ones(RP.NJ,1), ...
    'scale_lim', 0.7, ...
    'retry_limit', 0, 'I_EE', I_EE_3T2R, ...
    'maxrelstep', 0.25); % Grenzen sind nicht so breit; nehme größere max. Schrittweite
  % Würfel-Trajektorie (Kantenlänge 300mm
  d1=0.15; h1=0.15;
  X1 = X0+[-d1/2,d1/2,h1/2,0,0,0]'; % Start so, dass Würfel genau mittig ist
  k=1; XL = X1';
  k=k+1; XL(k,:) = XL(k-1,:) + [ d1,0,0, 0,0,0];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,-d1,0  0,0, pi/4];
  k=k+1; XL(k,:) = XL(k-1,:) + [-d1,0,0, 0,0,-pi/4];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,0,-h1, pi/4,0, 0];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,0, h1, -pi/4,0,0];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,d1,0,  0,pi/4,0];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,0,-h1, 0,-pi/4,0];
  k=k+1; XL(k,:) = XL(k-1,:) + [ d1,0,0, pi/6,-pi/6,0];
  k=k+1; XL(k,:) = XL(k-1,:) + [ 0,0,h1, pi/6,-pi/6,0];
  k=k+1; XL(k,:) = XL(k-1,:) + [ 0,0,-h1, -pi/6,pi/3,0];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,-d1,0  -pi/6,pi/6,0];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,0, h1, -pi/4,pi/4,-pi/3];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,0,-h1, pi/2,-pi/6,-pi/3];
  k=k+1; XL(k,:) = XL(k-1,:) + [-d1,0,0, pi/12,-pi/6,pi/2];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,d1,0,  -pi/4,0,-pi/6];
  k=k+1; XL(k,:) = XL(k-1,:) + [0,0, h1, -pi/12,pi/12,pi/3];
  % Reduziere alle Schwenkwinkel gegenüber der Vorlage
  XL(:,4:6) = 0.1*XL(:,4:6);
  % Debug: Zeichne Eckpunkte
  if debug_plot
    figure(1);clf; hold all; view(3);
    plot3(1e3*X0(1), 1e3*X0(2), 1e3*X0(3), 'g^');
    plot3(1e3*X1(1), 1e3*X1(2), 1e3*X1(3), 'mv');
    plot3(1e3*XL(:,1), 1e3*XL(:,2), 1e3*XL(:,3), 'r-');
    for jj = 1:size(XL,1)
      text(1e3*XL(jj,1), 1e3*XL(jj,2), 1e3*XL(jj,3)+3*jj, sprintf('%d', jj));
    end
    xlabel('x in mm');ylabel('y in mm');zlabel('z in mm');
  end
  % Berechne IK zu den einzelnen Eckpunkten. Wenn das nicht geht, bringt die
  % Trajektorie sowieso nichts
  s_ep = s; % Einstellungen für die Eckpunkte-IK
  s_ep.wn = [1;0]; % Nehme Zielfunktion 1. Damit Überschreitung der Ränder in Zwischenständen möglich
  s_ep.n_max = 5000; % Mehr Versuche (Abstände zwischen Punkten größer als bei Traj.-IK)
  s_ep.maxrelstep_ns = 0.05; % Große Werte, Größere Nullraumbewegung pro Zeitschritt
  s_ep.retry_limit = 100; % Neuversuche erlauben (bei Einzelpunkt i.O.)
  if eckpunkte_berechnen
    t0 = tic();
    for i = 1:size(XL,1)
      t1 = tic();
      [q_i, Phi_i] = RP.invkin4(XL(i,:)', qs, s_ep);
      fprintf('Eckpunkt %d/%d berechnet. Dauer %1.1fs (tpl-Funktion). Bis hier %1.1fs.\n', ...
        i, size(XL,1), toc(t1), toc(t0));
      if max(abs(Phi_i)) > 1e-6
        error('Eckpunkt %d geht nicht', i);
      end
      h1 = invkin_optimcrit_limits1(q_i, qlim);
      h2 = invkin_optimcrit_limits2(q_i, qlim);
      if any(isinf(h2))
        warning('Grenzverletzung bei Eckpunkt %d (kann an Einstellungen liegen)', i);
      end
      if test_class_methods
        t2 = tic();
        [q_i_class, Phi_i_class] = RP.invkin3(XL(i,:)', qs, s_ep);
        fprintf('Eckpunkt %d/%d berechnet. Dauer %1.1fs (Klassen-Methode).\n', ...
          i, size(XL,1), toc(t2));
        % Prüfe, ob beide Methoden das gleiche Ergebnis haben
        phi_test = Phi_i - Phi_i_class;
        if max(abs(phi_test(:))) > 1e-6
          error('Eckpunkt %d: phi stimmt nicht wischen tpl und Klassenmethode überein', i);
        end
        q_test = q_i - q_i_class;
        if max(abs(q_test(:))) > 1e-6
          error('Eckpunkt %d: q stimmt nicht wischen tpl und Klassenmethode überein', i);
        end
        h2 = invkin_optimcrit_limits2(q_i_class, qlim);
      end
    end
  end
  
  [X_t,XD_t,XDD_t,t,IL] = traj_trapez2_multipoint(XL, 3, 0.05, 0.01, 2e-3, 1e-2);
  % Debug: Trajektorie reduzieren
  if short_traj
    n = 200;
  else
    n = length(t);
  end
  % II = length(t)-n+1:1:length(t);
  II = 1:n;
  t = t(II);
  X_t = X_t(II,:);
  XD_t = XD_t(II,:);
  XDD_t = XDD_t(II,:);
  
  % Inverse Kinematik berechnen;  Lösung der IK von oben als Startwert
  t0 = tic();
  s_start = s;
  % Toleranz maximal stark setzen, damit es keinen Sprung im Verlauf gibt
  % (durch die vielen Nullraumiterationen ist die Aufgabentoleranz später
  % sowieso ungefähr Null.
  s_start.Phit_tol = 1e-12;
  s_start.Phir_tol = 1e-12;
  s_start.normalize = false;
  s_start.maxstep_ns = 1e-10; % Nullraumbewegung sollte schon zum Optimum konvergiert sein
  % Berechne IK mit 3T2R
  RP.update_EE_FG(I_EE_3T3R, I_EE_3T2R);
  warning on

  [q1, Psi_num1] = RP.invkin3(X_t(1,:)', qs, s_start);
  if any(abs(Psi_num1) > 1e-4)
    warning('IK konvergiert nicht');
  end
  
  % Normiere die Start-Gelenkwinkel auf Bereich 0 bis 1
  qsnorm = (qs-qlim(:,1)) ./ (qlim(:,2) - qlim(:,1)); % Muss 0.5 sein per Definition
  q1norm = (q1-qlim(:,1)) ./ (qlim(:,2) - qlim(:,1));
  
  if any(q1norm > 1) || any(q1norm < 0) % Winkel mit +- 2*pi berücksichtigen
    warning('Anfangs-Konfiguration für Trajektorie verletzt bereits die Grenzen');
  end

  %% Roboter in Startpose plotten
  figure(20*(robnr-1)+2);clf;
  set(20*(robnr-1)+2, 'Name', sprintf('Rob%d_Skizze', robnr), 'NumberTitle', 'off');
  title(sprintf('Rob %d - %s', robnr, RP.mdlname));
  hold on;grid on;
  xlabel('xin m');ylabel('yin m');zlabel('zin m');
  view(3);
  s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
  RP.plot( qs, X0, s_plot );
  plot3(X_t(:,1), X_t(:,2), X_t(:,3));
  %% Trajektorie berechnen
  % Berechne Beispiel-Trajektorie für 3T2R
  % Einstellungen für Trajektorie: Kein Normalisieren, damit Traj. nicht
  % springt. Muss doch normalisieren, damit Gelenkwinkelgrenzen korrekt
  % erkannt werden.
  s_Traj = s;
  s_Traj.normalize = false; % Mit Kriterium 2 keine Normalisierung. Sonst können Koordinaten jenseits der Grenzen landen
  s_Traj.wn = [0;1];
  s_Traj.mode_IK = 1;
  % Referenzlösung ohne Nullraumoptimierung (nur serielle IK, aber auch mit
  % 3T2R)
  fprintf('3T2R Inverse Kinematik (seriell-IK) für Trajektorie berechnen: %d Bahnpunkte\n', length(t));

  [Q1_t, QD1_t, ~, Phi1_t] = RP.invkin2_traj(X_t, XD_t, XDD_t, t, q1, s_Traj);
  I_err = abs(Phi1_t) > max(s_Traj.Phit_tol,s_Traj.Phir_tol);
  if any(I_err(:))
    I1 = find(sum(I_err,2),1);
    error('Fehler in Trajektorie zu groß. Zuerst bei Zeitschritt %d (t=%1.3fs). IK nicht berechenbar', I1, t(I1));
  end
  
  if test_class_methods
    [Q1_t_kls, QD1_t_kls, ~, Phi1_t_kls] = RP.invkin_traj(X_t, XD_t, XDD_t, t, q1, s_Traj); 
    if max(abs(Phi1_t_kls(:))) > max(s.Phit_tol,s.Phir_tol)
      error('Fehler in Trajektorie zu groß. IK nicht berechenbar');
    end
  end
  % Parallele IK
  s_Traj.mode_IK = 2;
  s_Traj.debug = true;
  fprintf('3T2R Inverse Kinematik (parallel-IK) für Trajektorie berechnen: %d Bahnpunkte\n', length(t));

  [Q2_t, QD2_t, ~, Phi2_t] = RP.invkin2_traj(X_t, XD_t, XDD_t, t, q1, s_Traj);
  if max(abs(Phi2_t(:))) > max(s_Traj.Phit_tol,s_Traj.Phir_tol)
    warning('Fehler in Trajektorie zu groß. IK nicht berechenbar\n');
  end
  if test_class_methods
    [Q2_t_kls, QD2_t_kls, ~, Phi2_t_kls] = RP.invkin_traj(X_t, XD_t, XDD_t, t, q1, s_Traj);
    if max(abs(Phi2_t_kls(:))) > max(s.Phit_tol,s.Phir_tol)
      warning('Fehler in Trajektorie zu groß. IK nicht berechenbar\n');
    end
    phi_test = Phi2_t - Phi2_t_kls;
    if max(abs(phi_test(:))) > 1e-6
      error('Eckpunkt %d passt tpl mit Klasssen nicht', i);
    end
  end
  
  % Berechne Ist-EE-Traj.
  i_BiKS = 1+RP.Leg(1).NL+1;
  II_BiKS = i_BiKS:(RP.Leg(1).NL+1):(1+(RP.Leg(1).NL+1)*RP.NLEG);
  for jj = 1:2
    X_ist = NaN(n,6*RP.NLEG);
    if jj == 1
      Q_t = Q1_t;
    else
      Q_t = Q2_t;
    end
    for i = 1:n
      Tc_ges = RP.fkine(Q_t(i,:)', NaN(6,1));
      % Schnitt-KS aller Beinketten bestimmen
      T_Bi = Tc_ges(:,:,II_BiKS);
      for j = 1:RP.NLEG
        R_0_Bj = T_Bi(1:3,1:3,j);
        R_P_Bj = eulxyz2r(RP.phi_P_B_all(:,j));
        R_Bj_P = R_P_Bj.';
        R_0_P = R_0_Bj * R_Bj_P;
        r_0_0_Bj = T_Bi(1:3,4,j);
        r_P_P_Bj = RP.r_P_B_all(:,j);
        r_0_Bj_P = -R_0_P*r_P_P_Bj;
        r_0_Ej = r_0_0_Bj + r_0_Bj_P;
        if j == 1
          r_0_E_Legs = r_0_Ej;
        elseif any(abs(r_0_E_Legs-r_0_Ej)>2e-6) % muss größer als IK-Toleranz sein
          warning('i=%d: EE aus Beinkette %d stimmt nicht mit Beinkette 1 überein', i, j);
        end
        T_E_Leg_j = rt2tr(R_0_P, r_0_Ej);
        X_ist(i,6*(j-1)+1:6*j) = RP.t2x(T_E_Leg_j);
      end
      R_E_Legs = R_0_P;
    end
    if jj == 1
      X1_ist = X_ist;
    else
      X2_ist = X_ist;
    end
  end
  H11_t = NaN(length(t),1+RP.NLEG);
  H12_t = NaN(length(t),1+RP.NLEG);
  H21_t = NaN(length(t),1+RP.NLEG);
  H22_t = NaN(length(t),1+RP.NLEG);
  % Gütefunktion nochmal berechnen
  for i = 1:length(t)
    H11_t(i,1) = invkin_optimcrit_limits1(Q1_t(i,:)', qlim);
    H12_t(i,1) = invkin_optimcrit_limits1(Q2_t(i,:)', qlim);
    H21_t(i,1) = invkin_optimcrit_limits2(Q1_t(i,:)', qlim);
    H22_t(i,1) = invkin_optimcrit_limits2(Q2_t(i,:)', qlim);
    for j = 1:6
      H11_t(i,1+j) = invkin_optimcrit_limits1(Q1_t(i,RP.I1J_LEG(j):RP.I2J_LEG(j))', qlim(RP.I1J_LEG(j):RP.I2J_LEG(j),:));
      H12_t(i,1+j) = invkin_optimcrit_limits1(Q2_t(i,RP.I1J_LEG(j):RP.I2J_LEG(j))', qlim(RP.I1J_LEG(j):RP.I2J_LEG(j),:));
      H21_t(i,1+j) = invkin_optimcrit_limits2(Q1_t(i,RP.I1J_LEG(j):RP.I2J_LEG(j))', qlim(RP.I1J_LEG(j):RP.I2J_LEG(j),:));
      H22_t(i,1+j) = invkin_optimcrit_limits2(Q2_t(i,RP.I1J_LEG(j):RP.I2J_LEG(j))', qlim(RP.I1J_LEG(j):RP.I2J_LEG(j),:));
    end
  end
  % Gelenkkoordinaten normieren
  Q1_t_norm = (Q1_t - repmat(qlim(:,1)',n,1)) ./ repmat(qlim(:,2)'-qlim(:,1)',n,1);
  Q2_t_norm = (Q2_t - repmat(qlim(:,1)',n,1)) ./ repmat(qlim(:,2)'-qlim(:,1)',n,1);
  
  %% Ergebniss speichern
  save(fullfile(respath,sprintf('Rob%d_%s_result.mat',robnr,RP.mdlname)));
  
  %% Zeitverlauf der Trajektorie plotten
  % Gesamtbild
  figure(20*(robnr-1)+4);clf;
  set(20*(robnr-1)+4, 'Name', sprintf('Rob%d_Traj', robnr), 'NumberTitle', 'off');
  sgtitle('Trajektorie allgemein');
  subplot(3,2,sprc2no(3,2,1,1)); hold on;
  plot(t, X_t);set(gca, 'ColorOrderIndex', 1)
  % plot([0;t(end)],[X0';X0'],'o--')
  legend({'$x$', '$y$', '$z$', '$\varphi_x$', '$\varphi_y$', '$\varphi_z$'}, 'interpreter', 'latex')
  grid on;
  ylabel('x_E');
  subplot(3,2,sprc2no(3,2,2,1));
  plot(t, XD_t);
  grid on;
  ylabel('xD_E');
  subplot(3,2,sprc2no(3,2,3,1));
  plot(t, XDD_t);
  grid on;
  ylabel('xDD_E');
  subplot(3,2,sprc2no(3,2,1,2));
  plot(t, Q2_t);
  grid on;
  ylabel('Q');
  subplot(3,2,sprc2no(3,2,2,2)); hold on;
  plot(t, Phi2_t(:,RP.I_constr_t_red));
  plot(t([1 end]), s.Phit_tol*[1;1], 'r--');
  plot(t([1 end]),-s.Phit_tol*[1;1], 'r--');
  grid on;
  ylabel('\Phi_{trans}');
  subplot(3,2,sprc2no(3,2,3,2)); hold on;
  plot(t, Phi2_t(:,RP.I_constr_r_red));
  plot(t([1 end]), s.Phir_tol*[1;1], 'r--');
  plot(t([1 end]),-s.Phir_tol*[1;1], 'r--');
  grid on;
  ylabel('\Phi_{rot}');
  saveas(20*(robnr-1)+4, fullfile(respath,sprintf( ...
    'Rob%d_%s_Traj.fig',robnr, RP.mdlname)));
  
  % Übersicht über Zielfunktionen
  figure(20*(robnr-1)+5);clf;
  set(20*(robnr-1)+5, 'Name', sprintf('Rob%d_Zielf', robnr), 'NumberTitle', 'off');
  sgtitle('Zielfunktionen');
  subplot(2,2,1); hold on;
  plot(t, H11_t(:,1));
  plot(t, H12_t(:,1));
  title('Zielfunktion 1 (nicht opt.)');
  ylabel('Zielfkt 1'); grid on;
  subplot(2,2,3); hold on;
  plot(t, log10(H11_t(:,1)));
  plot(t, log10(H12_t(:,1)));
  ylabel('Log.-Zielfkt 1 (n.o.)'); grid on;
  subplot(2,2,2); hold on;
  plot(t, H21_t(:,1));
  plot(t, H22_t(:,1));
  title('Optimierungs-Zielfunktion 2');
  ylabel('Zielfkt 2'); grid on;
  subplot(2,2,4); hold on;
  plot(t, log10(H21_t(:,1)));
  plot(t, log10(H22_t(:,1)));
  ylabel('Log.-Zielfkt 2'); grid on;
  legend({'seriell', 'parallel'});
  saveas(20*(robnr-1)+5, fullfile(respath,sprintf( ...
    'Rob%d_%s_Zielf.fig',robnr, RP.mdlname)));
  
  % Übersicht über Trajektorie
  figure(20*(robnr-1)+6);clf;
  set(20*(robnr-1)+6, 'Name', sprintf('Rob%d_TrajX', robnr), 'NumberTitle', 'off')
  sgtitle('Trajektorie X (Details)');
  for i = 1:6
    subplot(3,2,i); hold on
    linhdl1=plot(t, X1_ist(:,i:6:end));
    set(gca, 'ColorOrderIndex', 1);
    linhdl2=plot(t, X2_ist(:,i:6:end), ':');
    linhdl3=plot(t, X_t(:,i), '--');
    if i < 4, unit = 'm';
    else, unit = 'rad';
    end
    ylabel(sprintf('x %d in %s', i, unit));
    grid on
    if i == 5
      legend([linhdl1(1), linhdl2(1), linhdl3(1)], {'Seriell-IK', 'Parallel-IK', 'Soll 3T3R'});
    end
  end
  linkxaxes
  saveas(20*(robnr-1)+6, fullfile(respath,sprintf( ...
    'Rob%d_%s_TrajX.fig',robnr, RP.mdlname)));
  
  % Bild für einzelne Beine
  for jj = 1:2
    if jj == 1
      Q_t = Q1_t; Q_t_norm = Q1_t_norm; H_t = H11_t; Phi_t = Phi1_t;
      Name = 'Seriell';
    else
      Q_t = Q2_t; Q_t_norm = Q2_t_norm; H_t = H12_t; Phi_t = Phi2_t;
      Name = 'Parallel';
    end
    figure(20*(robnr-1)+6+jj);clf;
    set(20*(robnr-1)+6+jj, 'Name', sprintf('Rob%d_Beine_%s', robnr, Name), 'NumberTitle', 'off')
    sgtitle('Trajektorie Q (Beine)');
    for i = 1:RP.NLEG
      % Gelenkkoordinaten
      subplot(4,RP.NLEG,sprc2no(4,RP.NLEG,1,i));
      plot(t, Q_t(:,RP.I1J_LEG(i):RP.I2J_LEG(i)));
      ylabel(sprintf('q_%d', i)); grid on;
      if i == 6
        l = {};
        for j = 1:6
          l = {l{:}, sprintf('q_%d', j)}; %#ok<SAGROW>
        end
        legend(l);
      end
      title(sprintf('Beinkette %d', i));
      
      % Normierte Gelenk-Koordinaten.
      subplot(4,RP.NLEG,sprc2no(4,RP.NLEG,2,i));
      plot(t, Q_t_norm(:,RP.I1J_LEG(i):RP.I2J_LEG(i)));
      ylabel(sprintf('q_%d (norm)', i)); grid on;
      
      % ZB
      subplot(4,RP.NLEG,sprc2no(4,RP.NLEG,3,i));
      plot(t, Phi_t(:,RP.I1constr_red(i):RP.I2constr_red(i)));
      ylabel('ZB Beinkette'); grid on;
      
      % Zielfunktion
      subplot(4,RP.NLEG,sprc2no(4,RP.NLEG,4,i));
      plot(t, H_t(:,1+i));
      ylabel(sprintf('Zielfunktion für Beingelenke')); grid on;
    end
    linkxaxes
    saveas(20*(robnr-1)+6+jj, fullfile(respath,sprintf( ...
    'Rob%d_%s_Trajektorie_Beine_%s.fig',robnr, RP.mdlname, Name)));
  end
  
  %% Animation des bewegten Roboters
  for jj = 1:2
    if jj == 1
      Q_t = Q1_t;
      Name = 'IK_Seriell';
    else
      Q_t = Q2_t;
      Name = 'IK_Parallel';
    end
    s_anim = struct( 'gif_name', fullfile(respath, ...
      sprintf('Rob%d_%s_%s.gif',robnr,RP.mdlname, Name)) );
    s_plot = struct( 'ks_legs', [], 'straight', 0);
    figure(20*(robnr-1)+8+jj);clf;hold all;
    set(20*(robnr-1)+8+jj, 'name', sprintf('Rob%d_Anim_%s', robnr, Name), 'NumberTitle', 'off', ...
      'units','normalized','outerposition',[0 0 1 1]); % Vollbild, damit GIF größer wird
    view(3);
    axis auto
    hold on;grid on;
    xlabel('xin m');ylabel('yin m');zlabel('zin m');
    plot3(X_t(:,1), X_t(:,2), X_t(:,3));
    RP.anim( Q_t(1:20:size(Q_t,1),:), X_t(1:20:size(X_t,1),:), s_anim, s_plot);
    fprintf('Animation der Bewegung gespeichert: %s\n', s_anim.gif_name);
  end
  fprintf('Test für %s beendet\n',RP.mdlname);
  
end
dockall
