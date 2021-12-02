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
% 
% Erzeugt Bilder: Nummerierung mit 100er-Stelle für Nr. des Roboters
% * Zielf_Einzelpunkte (Nr. 1):
%   Verschiedene Lösungen der IK zur Anfahrt aller Einzelpunkte (Eckpunkte)
%   der Trajektorie. Ergebnis: Aufgabenredundanz-Ansatz hat bestes Ergebnis
%   im Vergleich zur Brute-Force-Optimierung mit Berechnung aller
%   Kombinationen
% * Roboter in Startpose (Nr. 2)
% * Beingelenke nach Trajektorien-IK mit verschiedenen Methoden (Nr. 11-14)
% * Zielfunktionen im Verlauf der Trajektorie (Nr. 20-22)
% * Plattformbewegung (Nr. 24)
% * Zielfunktionen im Verlauf der Trajektorie mit Brute-Force-Opt. (Nr. 28)
% * Alle Gelenkwinkel mit Brute-Force-Opt. des red. FG (Nr. 23)
% * Animation des Roboters in Trajektorie (Nr. 31-34)
% * Konsistenz von Position/Geschwindigkeit/Beschleunigung (Nr. 41ff,51ff,61ff)
% * Verlauf von Pos./Geschw./Beschl. aller Methoden in einem Bild (Nr. 71-73)
% 
%
% Siehe auch: 
% * ParRob_class_example_6UPS_3T2R.m
% * Schappler et al 2019: Modeling Parallel Robot Kinematics for 3T2R and 
%   3T3R Tasks using Reciprocal Sets of Euler Angles. MDPI Robotics.

% Junnan Li, HiWi bei Moritz Schappler, 2020-05
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

if isempty(which('serroblib_path_init.m'))
  warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end
if isempty(which('parroblib_path_init.m'))
  warning('Repo mit parallelen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end
%% Definitionen, Benutzereingaben
short_traj = false; % Trajektorie stark abkürzen, um prinzipielle Funktionalität zu zeigen
usr_create_anim = false; % Zum Erstellen von Animationen der Trajektorien
eckpunkte_berechnen = true; % Einzelpunkt-IK für alle Eckpunkte berechnen
test_class_methods = false; % Zusätzlich zweite Implementierung rechnen. Aufgrund numerischer Abweichung aber oft falsch-positive Fehler
debug_plot = false;

rob_path = fileparts(which('robotics_toolbox_path_init.m'));
respath = fullfile(rob_path, 'examples_tests', 'results', 'ParRob_3T3R_task_red_benchmark');
mkdirs(respath);

% Formatierung der Linien der verschiedenen Methoden
format_mlines = { 'r', 'v', '-', 8; ...
                  'g', 'd', '-', 5; ...
                  'b', 's', '--', 7; ...
                  'k', 'x', '--', 9; ...
                  'm', 'o', ':', 6; ...
                  'c', '^', '-', 3; ...
                  'r', '+', ':', 6};
%% Klasse für PKM erstellen (basierend auf serieller Beinkette)
% Robotermodell aus PKM-Bibliothek laden.
for robnr = 1:5 % 1: 3RRR; 2: 6UPS; 3: 6PUS; 4:6RRRRRR; 5: 3T1R-PKM
  %% Klasse für PKM erstellen (basierend auf PKM-Bibliothek)
  if robnr == 1
    RP = parroblib_create_robot_class('P3RRR1G1P1A1', 1.0, 0.2);
    pkin_gen = zeros(length(RP.Leg(1).pkin_names),1);
    % Nachbearbeitung einiger Kinematikparameter
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'a2')) = 0.6;
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'a3')) = 0.6;
    for i = 1:RP.NLEG
      RP.Leg(i).update_mdh(pkin_gen);
    end
  elseif robnr == 2
    RP = parroblib_create_robot_class('P6RRPRRR14V3G1P1A1', 1.0, 0.2);
  elseif robnr == 3
    RP = parroblib_create_robot_class('P6PRRRRR6V4G8P1A1', [0.8;0.3;pi/3], 0.2);
    pkin_6_PUS = zeros(length(RP.Leg(1).pkin),1); % Namen, siehe RP.Leg(1).pkin_names
    pkin_6_PUS(strcmp(RP.Leg(1).pkin_names,'a2')) = 0.2;
    pkin_6_PUS(strcmp(RP.Leg(1).pkin_names,'theta1')) = -pi/2; % So drehen, dass a2 nach oben zeigt
    pkin_6_PUS(strcmp(RP.Leg(1).pkin_names,'a4')) = 0.5;
    pkin_6_PUS(strcmp(RP.Leg(1).pkin_names,'alpha2')) = pi/2;
    for i = 1:RP.NLEG
      RP.Leg(i).update_mdh(pkin_6_PUS);
      % EE-KS mit Null-Rotation vorbelegen
      RP.Leg(i).update_EE(zeros(3,1),zeros(3,1));
    end
  elseif robnr == 4
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
  elseif robnr == 5
    % Parameter aus Maßsynthese
    RP = parroblib_create_robot_class('P4RRRRR5V1G1P1A1', 0.6399, 0.2316);
    pkin_gen = [-0.4235   -0.4619   -0.5137         0   -0.4207    0.1396         0]';
    for i = 1:RP.NLEG
      RP.Leg(i).update_mdh(pkin_gen);
    end
  end
  % Debug: Alle Vorlagen-Funktionen neu generieren:
  serroblib_create_template_functions({RP.Leg(1).mdlname}, false, false);
  parroblib_create_template_functions({RP.mdlname(1:end-2)}, false, false);
  matlabfcn2mex({[RP.mdlname(1:end-6), '_invkin_traj']});
  matlabfcn2mex({[RP.mdlname(1:end-6), '_invkin3']});
  matlabfcn2mex({[RP.mdlname(1:end-6), '_invkin']});
  matlabfcn2mex({[RP.Leg(1).mdlname, '_invkin_eulangresidual']});
  RP.fill_fcn_handles(true,true);

  I_EE_full = RP.I_EE;
  if all(RP.I_EE == [1 1 1 1 1 1])
    I_EE_red = logical([1 1 1 1 1 0]);
  elseif  all(RP.I_EE == [1 1 0 0 0 1])
    I_EE_red = logical([1 1 0 0 0 0]);
  elseif  all(RP.I_EE == [1 1 1 0 0 1])
    I_EE_red = logical([1 1 1 0 0 0]);
  else
    error('EE-FG des Roboters nicht vorgesehen');
  end
  I_EE_full_str = sprintf('%dT%dR', sum(I_EE_full(1:3)), sum(I_EE_full(4:6)));
  I_EE_red_str = sprintf('%dT%dR', sum(I_EE_red(1:3)), sum(I_EE_red(4:6)));
  %% Grenzen für die Gelenkpositionen und -geschwindigkeiten setzen
  % Dadurch wird die Schrittweite bei der inversen Kinematik begrenzt (auf 5%
  % der Spannbreite der Gelenkgrenzen) und die Konfiguration klappt nicht um.
  for i = 1:RP.NLEG
    % Begrenze die Winkel der Kugel- und Kardangelenke auf +/- 360°
    RP.Leg(i).qlim = repmat([-2*pi, 2*pi], RP.Leg(i).NQJ, 1);
    % Begrenze die Länge der Schubgelenke
    RP.Leg(i).qlim(RP.Leg(i).MDH.sigma==1,:) = ...
      repmat([0.1, 1.5], sum(RP.Leg(i).MDH.sigma==1), 1);
    
    RP.Leg(i).qDlim = repmat(2*[-2*pi, 2*pi], RP.Leg(i).NQJ, 1); % 720deg/s
    RP.Leg(i).qDlim(RP.Leg(i).MDH.sigma==1,:) = ...
      repmat([-2, 2], sum(RP.Leg(i).MDH.sigma==1), 1); % 2m/s
    
    RP.Leg(i).qDDlim = repmat([-100, 100], RP.Leg(i).NQJ, 1); % sind 10g auf 1m Länge
    RP.Leg(i).qDDlim(RP.Leg(i).MDH.sigma==1,:) = ...
      repmat([-50, 50], sum(RP.Leg(i).MDH.sigma==1), 1); % 50m/s² sind ca. 5g
  end
  RP.xDlim = [NaN(5,2); [-pi, pi]]; % 180°/s max. für EE-Drehung (sehr schnell)
  RP.xDDlim = RP.xDlim / 0.200; % Max. Geschw. in 200ms aufbauen
  %% Startpose bestimmen
  % Mittelstellung im Arbeitsraum
  X0 = [ [0.00;0.00;0.6]; [0;0;0]*pi/180 ];
  if all(I_EE_full == [1 1 0 0 0 1]), X0(3) = 0; end
  q0 = 0.5+rand(RP.NJ,1); % Startwerte für numerische IK (zwischen 0.5 und 1.5 rad)
  q0(RP.MDH.sigma==1) = 0.5; % mit Schubaktor größer Null anfangen (damit Konfiguration nicht umklappt)
  % Inverse Kinematik auf zwei Arten berechnen
  [qs, Phis, ~, Stats] = RP.invkin_ser(X0, q0, struct('retry_on_limitviol',true));
  if any(abs(Phis) > 1e-6)
    error('Inverse Kinematik (für jedes Bein einzeln) konnte in Startpose nicht berechnet werden');
  end
  if any(qs(RP.MDH.sigma==1) < 0)
    error('Start-Konfiguration ist umgeklappt mit Methode Seriell. Darf nicht sein.');
  end
  
  %% Zwangsbedingungen in Startpose testen
  % Initialisierung mit vollständigen FG (3T3R).
  RP.update_EE_FG(I_EE_full, I_EE_full);
  Phi1=RP.constr1(qs, X0);
  Phit1=RP.constr1_trans(qs, X0);
  Phir1=RP.constr1_rot(qs, X0);
  if any(abs(Phi1) > 1e-6)
    error('ZB in Startpose ungleich Null');
  end
  if ~all(I_EE_full == [1 1 1 0 0 1]) % TODO: Auch 3T1R prüfen
    assert(all(size(Phi1)==[sum(I_EE_full)*RP.NLEG 1]), ...
      'ZB Phi1 hat die falsche Dimension');
    assert(all(size(Phit1)==[sum(I_EE_full(1:3))*RP.NLEG 1]), ...
      'ZB Phit1 hat die falsche Dimension');
    assert(all(size(Phir1)==[sum(I_EE_full(4:6))*RP.NLEG 1]), ...
      'ZB Phir1 hat die falsche Dimension');
  end
  %% Gelenkwinkelgrenzen festlegen (für Optimierung)
  for i = 1:RP.NLEG
    q_i = qs(RP.I1J_LEG(i):RP.I2J_LEG(i));
    qlim_i = repmat(q_i,1,2);
    % Grenzen um die aktuelle Konfiguration herum wählen
    I1 = RP.Leg(i).MDH.sigma==1;
    % Schubachsen: Maximaler Hub +-500mm
    qlim_i(I1,:) = repmat(q_i(I1),1,2) + repmat([-0.5, 0.5], sum(I1),1);
    I0 = RP.Leg(i).MDH.sigma==0;
    % Kippwinkel der Kardan- und Kugelgelenke max. 60° in jede Richtung
    % TODO: Das nur für Roboter machen, die auch Kardan haben!
    qlim_i(I0,:) = repmat(q_i(I0),1,2) + repmat([-pi/3, pi/3], sum(I0),1);
    RP.Leg(i).qlim = qlim_i;
  end
  % qlim für gesamte PKM festlegen
  qlim = cat(1, RP.Leg.qlim);
  qDlim = cat(1, RP.Leg.qDlim);
  qDDlim = cat(1, RP.Leg.qDDlim);
  %% Initialisierung Teil 2
  % Roboter auf 3T2R einstellen
  RP.update_EE_FG(I_EE_full, I_EE_red);
  %% Jacobi-Matrizen auswerten
  [G_q_red, G_q_voll] = RP.constr3grad_q(qs, X0);
  G_q_red(abs(G_q_red(:))<1e-10)=0;
  if all(I_EE_full == [1 1 1 0 0 1])
    % Für jede Beinkette werden fünf ZB aufgestellt. Annahme: 5 Gelenke in
    % Kette. Dadurch mit 6FG überbestimmt. Benutze kinematische Modellierung
    n_legconstr_used = 5;
  else
    % Für 2T1R- oder 3T3R-PKM: Entweder 3 oder 6 ZB für Beinketten
    n_legconstr_used = sum(I_EE_full);
  end
  assert(all(size(G_q_red)==[RP.NLEG*n_legconstr_used-1, RP.NJ]), ...
    'ZB-matrix G_q_red hat die falsche Dimension');
  assert(all(size(G_q_voll)==[6*RP.NLEG RP.NJ]), ...
    'ZB-matrix G_q_voll hat die falsche Dimension');
  [G_x_red, G_x_voll] = RP.constr3grad_x(qs, X0);
  assert(all(size(G_x_red)==[RP.NLEG*n_legconstr_used-1, sum(I_EE_full)]), ...
    'ZB-matrix G_x_red hat die falsche Dimension');
  assert(all(size(G_x_voll)==[6*RP.NLEG 6]), ...
    'ZB-matrix G_x_voll hat die falsche Dimension');
  % Testen der Komponentenaufteilung
  G_q = G_q_voll(RP.I_constr_red,:);
  G_x = G_x_voll(RP.I_constr_red,:);
  if ~all(I_EE_full == [1 1 1 0 0 1]) && any(abs(G_q_red(:)-G_q(:))>1e-10)
    % Für 3T1R werden die reduzierten ZB nicht durch Auswahl von Zeilen
    % gebildet, sondern teilweise durch Betragssumme (X- und Y-Rotation)
    test_Gq = G_q - G_q_red;
    test_Gq(abs(test_Gq)<1e-8)=0;
    error('Aufteilung der ZB-Komponenten stimmt nicht zwischen constr3grad_q/constr3grad_x/ParRob');
  end

  % Aufteilung der Ableitung nach den Gelenken in Gelenkklassen
  % * aktiv/unabhängig (a),
  % * passiv+schnitt/abhängig (d)
  G_a = G_q(:,RP.I_qa);
  G_d = G_q(:,RP.I_qd);
  % Jacobi-Matrix zur Berechnung der abhängigen Gelenke und EE-Koordinaten
  G_dx = [G_d, G_x];
  
  fprintf('%s: Rang der vollständigen %dx%d-Jacobi der inversen Kinematik: %d/%d (Cond. %1.2g)\n', ...
    RP.mdlname, size(G_q,1), size(G_q,2), rank(G_q), min(size(G_q)), cond(G_q));
  fprintf('%s: Rang der vollständigen Jacobi der direkten Kinematik: %d/%d (Cond. %1.2g)\n', ...
    RP.mdlname, rank(G_dx), sum(RP.I_EE_Task)+sum(RP.I_qd), cond(G_dx));
  fprintf('%s: Rang der Jacobi der aktiven Gelenke: %d/%d\n', ...
    RP.mdlname, rank(G_a), sum(RP.I_EE_Task));
  
  %% Eckpunkte für Beispiel-Trajektorie bestimmen und IK prüfen
  % IK-Grundeinstellungen
  s = struct('Phit_tol', 1e-12, 'Phir_tol', 1e-12, ... % sehr genau rechnen
    'maxstep_ns', 1e-5, ... % Schrittweite für Nullraum-Inkremente gering halten
    'wn', [0;1], ... % keine Vorgabe von K oder Kn (Standard-Werte)
    'scale_lim', 0.7, ...
    'retry_limit', 0, ...
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
  if ~short_traj
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
  end
  % Reduziere alle Schwenkwinkel gegenüber der Vorlage
  XL(:,4:6) = 0.1*XL(:,4:6);
  if ~all(RP.I_EE == [1 1 1 1 1 1])
    XL(:, ~RP.I_EE) = 0; XL = unique(XL, 'rows');
  end
  % Debug: Zeichne Eckpunkte
  if debug_plot
    change_current_figure(1);clf; hold all; view(3);
    plot3(1e3*X0(1), 1e3*X0(2), 1e3*X0(3), 'g^');
    plot3(1e3*X1(1), 1e3*X1(2), 1e3*X1(3), 'mv');
    plot3(1e3*XL(:,1), 1e3*XL(:,2), 1e3*XL(:,3), 'r-');
    for ii = 1:size(XL,1)
      text(1e3*XL(ii,1), 1e3*XL(ii,2), 1e3*XL(ii,3)+3*ii, sprintf('%d', ii));
    end
    xlabel('x in mm');ylabel('y in mm');zlabel('z in mm');
  end
  % Berechne IK zu den einzelnen Eckpunkten. Wenn das nicht geht, bringt die
  % Trajektorie sowieso nichts. Benutze die IK mit Aufgabenredundanz 
  s_ep = s; % Einstellungen für die Eckpunkte-IK
  s_ep.wn = [1;0]; % Nehme Zielfunktion 1. Damit Überschreitung der Ränder in Zwischenständen möglich
  s_ep.n_max = 5000; % Mehr Versuche (Abstände zwischen Punkten größer als bei Traj.-IK)
  s_ep.maxrelstep_ns = 0.05; % Große Werte, Größere Nullraumbewegung pro Zeitschritt
  s_ep.retry_limit = 100; % Neuversuche erlauben (bei Einzelpunkt i.O.)
  s_ep.normalize = false;
  if eckpunkte_berechnen
    h1 = NaN(size(XL,1),1); h2 = h1; % Zielfunktion für Aufgabenredundanz
    nsteps_angle = 180; % Anzahl der Diskretisierung für globale Optimierung
    h1_go = NaN(size(XL,1),nsteps_angle); h2_go = h1_go; % ZF. für globale Opt.
    t0 = tic();
    for i = 1:size(XL,1)
      t1 = tic();
      % Berechne IK mit reduziertem FG
      RP.update_EE_FG(I_EE_full, I_EE_red);
      % Berechnung mit aus Vorlagendatei generierter Funktion
      [q_i, Phi_i, ~, Stats_i] = RP.invkin4(XL(i,:)', qs, s_ep);
      assert(all(size(Phi_i)==[RP.NLEG*n_legconstr_used-1 1]), ...
        'ZB Phi_i aus ParRob/invkin4 hat die falsche Dimension');
      % Prüfe nochmals die Richtigkeit mit anderer Modellierung
      x_i = RP.fkineEE_traj(q_i'); % tatsächliche EE-Drehung (Freiheitsgrad der Aufg.Red.)
      [~,Phi_i_voll] = RP.constr1(q_i, x_i(:));
      assert(all(abs(Phi_i_voll)<1e-8), ...
        sprintf('Ergebnis der %s-IK (Parallel) ist falsch', I_EE_red_str));
      % Berechne die IK mit Seriell-Methode (zum Testen).
      [q_i_test, Phi_i_test] = RP.invkin_ser(XL(i,:)', qs, rmfield(s_ep,{'maxstep_ns','maxrelstep_ns'}));
      assert(all(size(Phi_i_test)==[RP.NLEG*n_legconstr_used-1 1]), ...
        'ZB Phi_i aus ParRob/invkin_ser hat die falsche Dimension');
      % Prüfe auch hiervon die Richtigkeit
      x_i_test = RP.fkineEE_traj(q_i_test');
      [~,Phi_i_voll] = RP.constr1(q_i_test, x_i_test(:));
      assert(all(abs(Phi_i_voll)<1e-9), ...
        sprintf('Ergebnis der %s-IK (Seriell) ist falsch', I_EE_red_str));
      
      fprintf('Eckpunkt %d/%d berechnet. Dauer %1.1fs (tpl-Funktion). Bis hier %1.1fs.\n', ...
        i, size(XL,1), toc(t1), toc(t0));
      if max(abs(Phi_i)) > 1e-6
        error('Eckpunkt %d geht nicht', i);
      end
      h1(i) = invkin_optimcrit_limits1(q_i, qlim);
      h2(i) = invkin_optimcrit_limits2(q_i, qlim);
      if any(isinf(h2(i)))
        warning('Grenzverletzung bei Eckpunkt %d (kann an Einstellungen liegen)', i);
      end
      % Berechnung mit Klassenmethode (inhaltlich identisch)
      if test_class_methods
        t2 = tic();
        [q_i_class, Phi_i_class, ~, Stats_class] = RP.invkin3(XL(i,:)', qs, s_ep);
        assert(all(size(Phi_i_class)==[RP.NLEG*n_legconstr_used-1 1]), ...
          'ZB Phi_i aus ParRob/invkin3 hat die falsche Dimension');
        fprintf('Eckpunkt %d/%d berechnet. Dauer %1.1fs (Klassen-Methode).\n', ...
          i, size(XL,1), toc(t2));
        % Prüfe, ob beide Methoden das gleiche Ergebnis haben
        phi_test = Phi_i - Phi_i_class;
        if max(abs(phi_test(:))) > 1e-6
          error('Eckpunkt %d: phi stimmt nicht zwischen tpl und Klassenmethode überein', i);
        end
        q_test = q_i - q_i_class;
        if max(abs(q_test(:))) > 1e-4
          if Stats_class.retry_number > 0 && Stats_i.retry_number == 0
            % In diesem Fall ist es zu erwarten, dass das Ergebnis unter-
            % schiedlich ist. Sollte aber eigentlich nicht passieren.
            warning('Eckpunkt %d: IK hat Fehlversuch mit Klassenmethode, aber nicht mit tpl');
          else
            error('Eckpunkt %d: q stimmt nicht zwischen tpl und Klassenmethode überein', i);
          end
        end
      end
      % Bestimme best- und schlechtmöglichstes IK-Ergebnis (ohne Aufgabenredundenz)
      t3 = tic();
      RP.update_EE_FG(I_EE_full, I_EE_full); % keine Aufgabenredundanz
      s_ep_3T3R = s_ep; % Neue Konfiguration für Einzelpunkt-IK mit 3T3R
      s_ep_3T3R.scale_lim = 0; % Grenzen ignorieren. Gibt eh nur eine Lösung
      s_ep_3T3R.retry_limit = 0; % erster Versuch muss stimmen, da hier die Anfangswerte gut sind
      qs_globopt = q_i; % Start-Konfiguration für IK in globaler Optimierung
      for j = 1:nsteps_angle % in 2-Grad-Schritten durchgehen
        x = XL(i,:)';
        x(6) = 360/nsteps_angle*(j-1)*pi/180; % EE-Drehung vorgeben
        [q_i_kls, Phi_i_kls] = RP.invkin4(x, qs_globopt, s_ep_3T3R);
        qs_globopt = q_i_kls;
        if max(abs(Phi_i_kls)) > 1e-6
          % Diese Orientierung ist nicht erreichbar. Ist eigentlich kein
          % Fehler, da technisch nicht jede Orientierung möglich ist.
%           warning(['Eckpunkt %d geht nicht mit %s-IK für Rotationswinkel ', ...
%             '%d/%d (%1.1f deg)'], i, I_EE_full_str, j, nsteps_angle, 180/pi*x(6));
          continue; % Für nicht lösbare Posen wird die Lösung weiter unten verworfen
        end
        % Berechne Zielkriterien
        h1_go(i,j) = invkin_optimcrit_limits1(q_i_kls, qlim);
        h2_go(i,j) = invkin_optimcrit_limits2(q_i_kls, qlim);
        if any(isinf(h2_go(i)))
          warning('Grenzverletzung (kls) bei Eckpunkt %d (kann an Einstellungen liegen)', i);
        end
      end
      RP.update_EE_FG(I_EE_full, I_EE_red); % Aufgabenredundenz zurücksetzen
      fprintf(['Eckpunkt %d/%d berechnet. Dauer %1.1fs (Globale Optimierung %d Schritte, ', ...
        '%s, Brute-Force).\n'], i, size(XL,1), toc(t3), nsteps_angle, I_EE_full_str);
    end
  end
  %% Plot: Zielfunktion für die Eckpunkte mit verschiedenen IK-Einstellungen
  if eckpunkte_berechnen
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
    legend([hdl1(1), hdl2, hdl3], {'Glob. Opt., 2°-Schritte.', 'Glob. Opt., bester', 'Mit Aufgabenredundanz'});
    saveas(100*robnr+1, fullfile(respath,sprintf( ...
      'Rob%d_%s_EinzelTraj_Zielf.fig',robnr, RP.mdlname)));
  end
  %% Zeitverlauf der Trajektorie generieren
  [X_t,XD_t,XDD_t,t,IL] = traj_trapez2_multipoint(XL, 3, 0.05, 0.01, 2e-3, 0);
  % Debug: Trajektorie reduzieren
%   if short_traj
%     n = 200;
%   else
    n = length(t);
%   end
  % II = length(t)-n+1:1:length(t);
  II = 1:n;
  t = t(II);
  X_t = X_t(II,:);
  XD_t = XD_t(II,:);
  XDD_t = XDD_t(II,:);
  
  %% Inverse Kinematik zum Startpunkt der Trajektorie
  % Inverse Kinematik berechnen;  Lösung der IK von oben als Startwert
  % Wird nicht für die Trajektorien-IK benutzt, da die optimale Startkon-
  % figuration von den benutzten Nebenbedingungen abhängt.
  tic();
  s_start = s;
  % Toleranz maximal stark setzen, damit es keinen Sprung im Verlauf gibt
  % (durch die vielen Nullraumiterationen ist die Aufgabentoleranz später
  % sowieso ungefähr Null.
  s_start.Phit_tol = 1e-12;
  s_start.Phir_tol = 1e-12;
  s_start.normalize = false;
  s_start.maxstep_ns = 1e-10; % Nullraumbewegung sollte schon zum Optimum konvergiert sein
  % Berechne IK mit 3T2R
  RP.update_EE_FG(I_EE_full, I_EE_red);
  warning on
  % Berechne Ersten Punkt der Trajektorie mit Aufgabenredundanz.
  % Dadurch bestmögliche Startkonfiguration
  [q1, Psi_num1] = RP.invkin3(X_t(1,:)', qs, s_start);
  if any(abs(Psi_num1) > 1e-4)
    error('IK konvergiert nicht für Startpunkt der Trajektorie');
  end
  
  % Normiere die Start-Gelenkwinkel auf Bereich 0 bis 1
  qsnorm = (qs-qlim(:,1)) ./ (qlim(:,2) - qlim(:,1)); % Muss 0.5 sein per Definition
  if any(abs(qsnorm-0.5)>1e-6)
    error('Die erste Startkonfiguration für die IK ist nicht in der Mitte des Gelenkraums');
  end
  % Prüfe den Anfangswert für die IK (Optimal im Startpunkt)
  q1norm = (q1-qlim(:,1)) ./ (qlim(:,2) - qlim(:,1));
  if any(q1norm > 1) || any(q1norm < 0) % Winkel mit +- 2*pi berücksichtigen
    error('Anfangs-Konfiguration für Trajektorie verletzt bereits die Grenzen');
  end

  %% Roboter in Startpose plotten
  change_current_figure(100*robnr+2);clf;
  set(100*robnr+2, 'Name', sprintf('Rob%d_Skizze', robnr), 'NumberTitle', 'off');
  title(sprintf('Rob %d - %s', robnr, RP.mdlname));
  hold on;grid on;
  xlabel('x in m');ylabel('y in m');zlabel('z in m');
  view(3);
  s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
  RP.plot( qs, X0, s_plot );
  plot3(X_t(:,1), X_t(:,2), X_t(:,3));
  
  %% IK für Trajektorie berechnen (Vorbereitung)
  RP.update_EE_FG(I_EE_full, I_EE_red);
  % Berechne Beispiel-Trajektorie für 3T2R
  % Einstellungen für Trajektorie: Kein Normalisieren, damit Traj. nicht
  % springt. Muss doch normalisieren, damit Gelenkwinkelgrenzen korrekt
  % erkannt werden.
  s_Traj = struct('Phit_tol', 1e-12, 'Phir_tol', 1e-12);
  % Abspeichern der Gelenkwinkel für verschiedene Varianten der Berechnung
  Namen_Methoden = cell(1,7);
  Q_t_all = NaN(length(t), RP.NJ, length(Namen_Methoden)); QD_t_all = Q_t_all; QDD_t_all = Q_t_all;
  Q_t_norm_all = Q_t_all;
  H1_all = NaN(length(t), 1+RP.NLEG, length(Namen_Methoden)); H2_all = H1_all;
  H1D_all = H1_all; H2D_all = H1_all;
  Hcond_all = NaN(length(t), 2, length(Namen_Methoden));
  XE_all = NaN(length(t), 6, length(Namen_Methoden));
  XDE_all = XE_all; XDDE_all = XE_all;
  %% IK für Trajektorie berechnen (verschiedene Methoden)
  qDlim_backup = cat(1, RP.Leg.qDlim); % damit überschriebene Werte wieder ...
  qDDlim_backup = cat(1, RP.Leg.qDDlim); % ... hergestellt werden können
  for kk = 1:length(Namen_Methoden)
    s_kk = s_Traj;
    s_kk.debug = true;
    s_kk.wn = zeros(10,1);
    s_kk.wn(2) = 1; % Benutze Kriterium 2 (hyperbolische Grenzen für Position)
    s_kk.wn(8) = 0.1; % auch als D-Anteil für schnellere Reaktion bei Annäherung an Grenzen
    s_kk.wn(3) = 0.5; % lineare Grenzen für Geschwindigkeit bzw. Dämpfung
    s_kk.normalize = false; % Mit Kriterium 2 keine Normalisierung. Sonst können Koordinaten jenseits der Grenzen landen
    for j = 1:RP.NLEG
      RP.Leg(j).qDlim = qDlim_backup(RP.I1J_LEG(j):RP.I2J_LEG(j),:);
      RP.Leg(j).qDDlim = qDDlim_backup(RP.I1J_LEG(j):RP.I2J_LEG(j),:);
    end
    switch kk
      case 1
        name_method=sprintf('%s-IK (mit Opt.)', I_EE_red_str);
        I_EE_Task_kk = I_EE_red;
      case 2
        name_method=sprintf('%s-IK ohne qD lim.', I_EE_red_str);
        for j = 1:RP.NLEG, RP.Leg(j).qDlim(:) = NaN; end % Dadurch Grenzen nicht aktiv
        s_kk.wn(3:4) = 0; % Zielfunktionen basierend auf qDlim deaktivieren
        I_EE_Task_kk = I_EE_red;
      case 3
        name_method=sprintf('%s-IK ohne qDD lim.', I_EE_red_str);
        for j = 1:RP.NLEG, RP.Leg(j).qDDlim(:) = NaN; end % Dadurch Grenzen nicht aktiv
        I_EE_Task_kk = I_EE_red;
      case 4
        name_method=sprintf('%s-IK ohne Opt.', I_EE_red_str);
        s_kk.wn(:) = 0;
        I_EE_Task_kk = I_EE_red;
      case 5
        name_method=sprintf('%s-IK mit IK-cond.-Opt.', I_EE_red_str);
        s_kk.wn(5) = 1; % IK-Konditionszahl auch verbessern (P-Anteil)
        s_kk.wn(9) = 0.2; % mit D-Anteil
        I_EE_Task_kk = I_EE_red;
      case 6
        name_method=sprintf('%s-IK mit PKM-cond.-Opt.', I_EE_red_str);
        s_kk.wn(6) = 1; % PKM-Jacobi optimieren (P-Anteil)
        s_kk.wn(10) = 0.2; % sehr kleiner D-Anteil
        I_EE_Task_kk = I_EE_red;
      case 7
        name_method=sprintf('%s-IK', I_EE_full_str);
        I_EE_Task_kk = I_EE_full;
        s_kk.wn(:) = 0; % Keine zusätzliche Optimierung möglich
      otherwise
        error('Fall %d noch nicht definiert', kk);
    end
    RP.update_EE_FG(I_EE_full, I_EE_Task_kk);
    % Positions-IK zum Startpunkt der Trajektorie mit genau den gleichen
    % Optimierungs-Nebenbedingungen wie in der Trajektorie. Dadurch keine
    % Nullraumbewegung am Anfang (Start in lokalem Optimum)
    s_pik_kk = struct('scale_lim',0.5); % Grenzen dürfen nicht überschritten werden
    % s_pik_kk.wn = s_kk.wn([1 2 5]); % Positions-Grenzen und Kondition
    % Wähle immer die gleichen Nebenbedingungen, damit alle mit gleicher
    % Konfiguration starten (besser vergleichbar)
    [qs_kk, Phi_s, ~, Stats_s] = RP.invkin3(X_t(1,:)', qs, s_pik_kk);
    if any(abs(Phi_s)>1e-6)
      error(['Zusätzliche Nullraumbewegung am Beginn der Trajektorie ', ...
        'fehlgeschlagen']);
    end

    Namen_Methoden{kk} = name_method;
    t1=tic();
    [Q_t_kk, QD_t_kk, QDD_t_kk, Phi_t_kk,~,~,~,Stats_kk] = RP.invkin2_traj(X_t, XD_t, XDD_t, t, qs_kk, s_kk);
    I_err = abs(Phi_t_kk) > max(s_kk.Phit_tol,s_kk.Phir_tol) | isnan(Phi_t_kk);
    % Prüfe, ob die Trajektorie vorzeitig abbricht. Das ist kein Fehler, da
    % bei ungünstiger Parametrierung des IK-Algorithmus keine Lösung
    % gefunden werden kann. Bei guter Parametrierung ist dann eine Lösung
    % möglich.
    n_iO = n; % Anzahl der i.O. Trajektorienpunkte
    if any(I_err(:))
      I1 = find(sum(I_err,2),1);
      n_iO = I1-1; % Bis einen Punkt vor dem Fehler war es noch i.O.
      warning(['Fehler in Trajektorie zu groß. Zuerst bei Zeitschritt %d/%d ', ...
        '(t=%1.3fs). Traj.-IK nicht vollständig berechenbar'], I1, size(Q_t_kk,1), t(I1));
    end
    fprintf(['Traj.-IK Fall %d (%s) berechnet. Dauer: %1.1fs für %d/%d ', ...
      'Bahnpunkte. %1.1fms pro i.O.-Punkt\n']', kk, name_method, toc(t1), n_iO, ...
      n, 1e3*toc(t1)/n_iO);
    Q_t_all(:,:,kk) = Q_t_kk;
    QD_t_all(:,:,kk) = QD_t_kk;
    QDD_t_all(:,:,kk) = QDD_t_kk;
    % Das gleiche nochmal mit der Klassenmethode
    if test_class_methods
      t1=tic();
      [Q_t_tpl, QD_t_tpl, QDD_t_tpl, Phi_t_kls] = RP.invkin_traj( ...
        X_t(1:n_iO,:), XD_t(1:n_iO,:), XDD_t(1:n_iO,:), t(1:n_iO), qs_kk, s_kk);
      fprintf(['Traj.-IK Fall %d (%s) berechnet. Dauer: %1.1fs für %d ', ...
        'Bahnpunkte; %1.1fms pro Punkt (Klassen-Methode)\n'], kk, ...
        name_method, toc(t1), n_iO, 1e3*toc(t1)/n_iO);
      if max(abs(Phi_t_kls(:))) > max(s.Phit_tol,s.Phir_tol)
        error('Fehler in Trajektorie zu groß. IK nicht berechenbar');
      end
      Phi_test = Phi_t_kk(1:n_iO,:) - Phi_t_kls;
      Q_test_abs = Q_t_kk(1:n_iO,:) - Q_t_tpl;
      QD_test_abs = QD_t_kk(1:n_iO,:) - QD_t_tpl;
      QDD_test_abs = QDD_t_kk(1:n_iO,:) - QDD_t_tpl;
      if max(abs(Phi_test(:))) > 1e-6
        % Debug-Plots für vergleich
        figure(3000);clf;
        ii = 0;
        for i = 1:RP.NLEG
          for j = 1:RP.Leg(i).NJ
            ii = ii + 1;
            subplot(RP.NLEG,RP.Leg(1).NJ,sprc2no(RP.NLEG, RP.Leg(1).NJ, i, j));
            hold on; grid on;
            stairs(t(1:n_iO), QDD_t_tpl(1:n_iO,ii));
            stairs(t(1:n_iO), QDD_t_kk(1:n_iO,ii), '--');
            if j == 1, ylabel(sprintf('BK %d', i)); end
            if i == 1, title(sprintf('qDD %d', j)); end
          end
        end
        linkxaxes
        error('Traj.-IK Fall %d: IK-Ergebnis der Trajektorie (Phi) passt nicht für invkin_traj vs invkin2_traj', kk);
      end
      if max(abs(Q_test_abs(:))) > 1e-6
        I_firstviol = find(any(abs(Q_test_abs) > 1e-6,2), 1, 'first');
        Q_test_abs(abs(Q_test_abs) < 1e-6) = 0; % Zur besseren Erkennung der Abweichungen im Debug-Modus
        warning(['Traj.-IK Fall %d: IK-Ergebnis der Trajektorie (Q) passt nicht ', ...
          'für invkin_traj vs invkin2_traj. Zuerst in Zeitschritt %d. Max Fehler %1.1e.'], ...
          kk, I_firstviol, max(abs(Q_test_abs(:))));
      end
      QD_test_rel = QD_test_abs./QD_t_tpl;
      I_abs = abs(QD_test_abs) > 1e-2;
      I_rel = abs(QD_test_rel) > 1e-1;
      I_err = I_abs & I_rel;
      if any(I_err(:))
        I_firstviol = find(any(I_err,2), 1, 'first');
        msg = sprintf(['Traj.-IK Fall %d: IK-Ergebnis der Trajektorie (QD) passt nicht ', ...
          'für invkin_traj vs invkin2_traj. Zuerst in Zeitschritt %d. Max Fehler %1.1e.'], ...
          kk, I_firstviol, max(abs(QD_test_abs(:))));
        if n_iO ~= n % Berechnung unvollständig. Daher am Ende Fehler möglich
          warning(msg); %#ok<SPWRN>
        else % Berechnung Fehlerfrei bis zum Ende. Bei Abweichung Fehler.
          error(msg); %#ok<SPERR>
        end
      end
      if max(abs(QDD_test_abs(:))) > 1e-3
        I_firstviol = find(any(abs(QDD_test_abs) > 1e-3,2), 1, 'first');
        warning(['Traj.-IK Fall %d: IK-Ergebnis der Trajektorie (QDD) passt nicht ', ...
          'für invkin_traj vs invkin2_traj. Zuerst in Zeitschritt %d. Max Fehler %1.1e.'], ...
          kk, I_firstviol, max(abs(QDD_test_abs(:))));
      end
    end
    % Wiederherstellung der Grenzwerte, die temporär zurückgesetzt wurden
    for j = 1:RP.NLEG
      RP.Leg(j).qDlim = qDlim_backup(RP.I1J_LEG(j):RP.I2J_LEG(j),:);
    end

    % Berechne Ist-EE-Traj.
    i_BiKS = 1+RP.Leg(1).NL+1;
    II_BiKS = i_BiKS:(RP.Leg(1).NL+1):(1+(RP.Leg(1).NL+1)*RP.NLEG);
    X_ist = NaN(n,6*RP.NLEG);
    Q_t = Q_t_all(:,:,kk);
    for i = 1:n_iO
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
        R_0_E = R_0_P * RP.T_P_E(1:3,1:3);
        if j == 1
          % die EE-Position aus der ersten Kette muss auch für folgende
          % Ketten eingehalten werden. Daher Speicherung.
          r_0_E_Legs = r_0_Ej;
          R_0_E_Legs = R_0_E;
          % TODO: Auch für Orientierung prüfen
        else
          if any(abs(r_0_E_Legs-r_0_Ej)>2e-6) % muss größer als IK-Toleranz sein
            error('i=%d: EE-Position aus Beinkette %d stimmt nicht mit Beinkette 1 überein', i, j);
          end
          test_R = R_0_E_Legs\R_0_E-eye(3);
          if any(abs(test_R(:)) > 1e-3)
            change_current_figure(978);clf;
            hold on; grid on; view(3);
            xlabel('x in m');ylabel('y in m');zlabel('z in m');
            s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
            RP.plot( Q_t(i,:)', X_t(i,:)', s_plot );
            error('i=%d: EE-Rotation aus Beinkette %d stimmt nicht mit Beinkette 1 überein', i, j);
          end
        end
        T_E_Leg_j = rt2tr(R_0_P, r_0_Ej);
        X_ist(i,6*(j-1)+1:6*j) = RP.t2x(T_E_Leg_j);
      end
      R_E_Legs = R_0_P;
    end
    % Plattform-Pose aus direkter Kinematik abspeichern
    [XE_all(:,:,kk), XDE_all(:,:,kk), XDDE_all(:,:,kk)] = RP.fkineEE2_traj(Q_t_kk, QD_t_kk, QDD_t_kk);
    test2_XEall = X_ist(:,1:6) - XE_all(:,:,kk);
    assert(all(abs(test2_XEall(:)) < 1e-10), 'Neuberechnung mit fkineEE2_traj falsch');
    XE_all(:,6,kk) = denormalize_angle_traj(XE_all(:,6,kk), XDE_all(:,6,kk), t);
    
    % Zielfunktionen für Position
    h1 = NaN(size(H1_all(:,:,kk))); h2=h1;
    for ii = 1:length(t)
      h1(ii,1) = invkin_optimcrit_limits1(Q_t_kk(ii,:)', qlim);
      h2(ii,1) = invkin_optimcrit_limits2(Q_t_kk(ii,:)', qlim);
      for kkk = 1:RP.NLEG % Kriterien für jedes Bein einzeln berechnen
        h1(ii,1+kkk) = invkin_optimcrit_limits1(Q_t_kk(ii,RP.I1J_LEG(kkk):RP.I2J_LEG(kkk))', RP.Leg(kkk).qlim);
        h2(ii,1+kkk) = invkin_optimcrit_limits2(Q_t_kk(ii,RP.I1J_LEG(kkk):RP.I2J_LEG(kkk))', RP.Leg(kkk).qlim);
      end
    end
    H1_all(:,:,kk) = h1;
    H2_all(:,:,kk) = h2;
    % Zielfunktionen für Geschwindigkeit
    h1D = NaN(size(H1D_all(:,:,kk))); h2D=h1;
    for ii = 1:length(t)
      h1D(ii,1) = invkin_optimcrit_limits1(QD_t_kk(ii,:)', cat(1, RP.Leg.qDlim));
      h2D(ii,1) = invkin_optimcrit_limits2(QD_t_kk(ii,:)', cat(1, RP.Leg.qDlim));
      for kkk = 1:RP.NLEG % Kriterien für jedes Bein einzeln berechnen
        h1D(ii,1+kkk) = invkin_optimcrit_limits1(QD_t_kk(ii,RP.I1J_LEG(kkk):RP.I2J_LEG(kkk))', RP.Leg(kkk).qDlim);
        h2D(ii,1+kkk) = invkin_optimcrit_limits2(QD_t_kk(ii,RP.I1J_LEG(kkk):RP.I2J_LEG(kkk))', RP.Leg(kkk).qDlim);
      end
    end
    H1D_all(:,:,kk) = h1D;
    H2D_all(:,:,kk) = h2D;
    % Zielfunktion für Konditionszahl
    for jj = 1:n_iO
      % Konditionszahl der Gesamt-Matrix
      Phi_q = RP.constr3grad_q(Q_t_kk(jj,:)', X_t(jj,:)');
      Hcond_all(jj,1, kk) = cond(Phi_q);
%       % Konditionszahl der Jacobi jeder einzelnen Beinkette
%       for kkk = 1:RP.NLEG % Kriterien für jedes Bein einzeln berechnen
%         Hcond_all(jj,2+kkk, kk) = cond(RP.Leg(kkk).jacobig(Q_t_kk(jj,RP.I1J_LEG(kkk):RP.I2J_LEG(kkk))'));
%       end
      % PKM-Jacobi-Matrix
      [~,Phi_q_voll] = RP.constr4grad_q(Q_t_kk(jj,:)');
      [~,Phi_x_voll] = RP.constr4grad_x(X_ist(jj,1:6)');
      Jinv_voll = -Phi_q_voll\Phi_x_voll;
      Hcond_all(jj,2, kk) = cond(Jinv_voll(RP.I_qa,RP.I_EE));
    end
    
    % Gelenkkoordinaten normieren
    Q_t_norm_all(:,:,kk) = (Q_t_all(:,:,kk) - repmat(qlim(:,1)',n,1)) ... % untere Grenze abziehen
                            ./ repmat(qlim(:,2)'-qlim(:,1)',n,1); % geteilt durch Spannweite
  end
  %% Konsistenz von Position, Geschwindigkeit und Beschleunigung testen
  for kk = 1:length(Namen_Methoden)
    Q = Q_t_all(:,:,kk);
    assert(any(~isnan(Q(:))), 'Keine Daten vorliegend. Fehler beim Debuggen?');
    QD = QD_t_all(:,:,kk);
    QDD = QDD_t_all(:,:,kk);
    % Integriere die Beschleunigung und Geschwindigkeit
    QD_int = cumtrapz(t, QDD) + repmat(QD(1,:),n,1);
    % Integriere die Geschwindigkeit zur Position
    Q_int = cumtrapz(t, QD) + repmat(Q(1,:),n,1);
    % Differenziere die Geschwindigkeit zur Beschleunigung
    QD_diff = [diff(QD)./repmat(diff(t),1,size(QDD,2));NaN(1,size(QDD,2))];
    % Vergleiche die Verläufe graphisch: Position
    change_current_figure(100*robnr+40+kk);clf;
    set(100*robnr+40+kk, 'Name', sprintf('Rob%d_M%d_Kons_q_qD', robnr, kk), 'NumberTitle', 'off');
    sgtitle(sprintf('Konsistenz q-int(qD) M%d (%s)', kk, Namen_Methoden{kk}));
    ii = 0;
    for i = 1:RP.NLEG
      for j = 1:RP.Leg(i).NJ
        ii = ii + 1;
        subplot(RP.NLEG,RP.Leg(1).NJ,sprc2no(RP.NLEG, RP.Leg(1).NJ, i, j));
        hold on; grid on;
        hdl1=stairs(t, Q(:,ii));
        hdl2=stairs(t, Q_int(:,ii), '--');
        ylim([-0.1, +0.1]+minmax2([Q(:,ii);Q_int(:,ii)]')); % Gelenkgrenzen nicht für Plot-Grenzen berücksichtigen
        plot(t([1,end]), repmat(RP.Leg(i).qlim(j,:),2,1), 'r-');
        if j == 1, ylabel(sprintf('BK %d', i)); end
        if i == 1, title(sprintf('q %d', j)); end
        if ii == RP.NJ, legend([hdl1;hdl2],{'direkt', 'integral'}); end
      end
    end
    linkxaxes
    saveas(100*robnr+40+kk, fullfile(respath,sprintf('Rob%d_M%d_Konsistenz_q', robnr, kk)));
    % Vergleiche die Verläufe graphisch: Geschwindigkeit
    change_current_figure(100*robnr+50+kk);clf;
    set(100*robnr+50+kk, 'Name', sprintf('Rob%d_M%d_Kons_qD_qDD', robnr, kk), 'NumberTitle', 'off')
    sgtitle(sprintf('Konsistenz qD-int(qDD) M%d (%s)', kk, Namen_Methoden{kk}));
    ii = 0;
    for i = 1:RP.NLEG
      for j = 1:RP.Leg(i).NJ
        ii = ii + 1;
        subplot(RP.NLEG,RP.Leg(1).NJ,sprc2no(RP.NLEG, RP.Leg(1).NJ, i, j));
        hold on; grid on;
        hdl1=stairs(t, QD(:,ii));
        hdl2=stairs(t, QD_int(:,ii), '--');
        plot(t([1,end]), repmat(RP.Leg(i).qDlim(j,:),2,1), 'r-');
        ylim([-0.1, +0.1]+minmax2([QD(:,ii);QD_int(:,ii)]')); % Gelenkgrenzen nicht für Plot-Grenzen berücksichtigen
        if j == 1, ylabel(sprintf('BK %d', i)); end
        if i == 1, title(sprintf('qD %d', j)); end
        if ii == RP.NJ, legend([hdl1;hdl2],{'direkt', 'integral'}); end
      end
    end
    linkxaxes
    saveas(100*robnr+50+kk, fullfile(respath,sprintf('Rob%d_M%d_Konsistenz_qD', robnr, kk)));
    % Vergleiche die Verläufe graphisch: Beschleunigung
    change_current_figure(100*robnr+60+kk);clf;
    set(100*robnr+60+kk, 'Name', sprintf('Rob%d_M%d_Kons_qDD', robnr, kk), 'NumberTitle', 'off')
    sgtitle(sprintf('Konsistenz diff(qD)-qDD M%d (%s)', kk, Namen_Methoden{kk}));
    ii = 0;
    for i = 1:RP.NLEG
      for j = 1:RP.Leg(i).NJ
        ii = ii + 1;
        subplot(RP.NLEG,RP.Leg(1).NJ,sprc2no(RP.NLEG, RP.Leg(1).NJ, i, j));
        hold on; grid on;
        hdl1=stairs(t, QDD(:,ii));
        hdl2=stairs(t, QD_diff(:,ii), '--');
        plot(t([1,end]), repmat(RP.Leg(i).qDDlim(j,:),2,1), 'r-');
        if j == 1, ylabel(sprintf('BK %d', i)); end
        if i == 1, title(sprintf('qDD %d', j)); end
        if ii == RP.NJ, legend([hdl1;hdl2],{'direkt', 'differenz'}); end
      end
    end
    linkxaxes
    saveas(100*robnr+50+kk, fullfile(respath,sprintf('Rob%d_M%d_Konsistenz_qDD', robnr, kk)));
  end
  %% Trajektorie: Bild für einzelne Beine
  fprintf('Zeichne Bilder für Zusammenfassung von einzelnen Beinketten (Trajektorien-IK)\n');
  for kk = 1:length(Namen_Methoden)
    % Definitionen für die Ergebnisse dieser Methode laden
    Q_t = Q_t_all(:,:,kk);
    QD_t = QD_t_all(:,:,kk);
    QDD_t = QDD_t_all(:,:,kk);
    Q_t_norm = Q_t_norm_all(:,:,kk);
    Name = Namen_Methoden{kk};
    H1_t = H1_all(:,:,kk);
    H2_t = H2_all(:,:,kk);
    % Zeichnen
    change_current_figure(100*robnr+10+kk);clf;
    set(100*robnr+10+kk, 'Name', sprintf('Rob%d_M%d_Beine', robnr, kk), 'NumberTitle', 'off')
    sgtitle(sprintf('Trajektorie Q (Beine); %s', Name));
    axhdl = NaN(5,RP.NLEG);
    for i = 1:RP.NLEG
      % Gelenkkoordinaten
      axhdl(1,i)=subplot(5,RP.NLEG,sprc2no(5,RP.NLEG,1,i));
      plot(t, Q_t(:,RP.I1J_LEG(i):RP.I2J_LEG(i)));
      ylabel(sprintf('q_%d', i)); grid on;
      if i == RP.NLEG % letzte Spalte der Subplots
        l = {};
        for j = 1:RP.Leg(1).NJ
          l = [l, {sprintf('q_%d', j)}];
        end
        legend(l);
      end
      title(sprintf('Beinkette %d', i));
      % Gelenkgeschwindigkeiten
      axhdl(2,i)=subplot(5,RP.NLEG,sprc2no(5,RP.NLEG,2,i));
      plot(t, QD_t(:,RP.I1J_LEG(i):RP.I2J_LEG(i)));
      ylabel(sprintf('qD_%d', i)); grid on;
      % Gelenkbeschleunigungen
      axhdl(3,i)=subplot(5,RP.NLEG,sprc2no(5,RP.NLEG,3,i));
      plot(t, QDD_t(:,RP.I1J_LEG(i):RP.I2J_LEG(i)));
      ylabel(sprintf('qDD_%d', i)); grid on;
      % Debug: Normierte Gelenk-Koordinaten.
      % plot(t, Q_t_norm(:,RP.I1J_LEG(i):RP.I2J_LEG(i)));
      % ylabel(sprintf('q_%d (norm)', i)); grid on;
      % Zielfunktion 1
      axhdl(4,i)=subplot(5,RP.NLEG,sprc2no(5,RP.NLEG,4,i));
      plot(t, H1_t(:,1+i)); grid on;
      ylabel(sprintf('ZF 1 BK %d', i));
      % Zielfunktion 2
      axhdl(5,i)=subplot(5,RP.NLEG,sprc2no(5,RP.NLEG,5,i));
      plot(t, H2_t(:,1+i)); grid on;
      ylabel(sprintf('ZF 2 BK %d', i));
    end
    linkxaxes
    remove_inner_labels(axhdl,'x')
    saveas(100*robnr+10+kk, fullfile(respath,sprintf( ...
    'Rob%d_M%d_%s_Trajektorie_Beine_%s.fig',robnr, kk, RP.mdlname, Name)));
  end
  
  %% Trajektorie: Bild für Gesamt-Zielfunktionen
  fprintf('Zeichne Bilder für Zielfunktionen (Trajektorien-IK)\n');
  % Zielfunktionen bezogen auf Positions-Grenzen
  change_current_figure(100*robnr+20); clf;
  set(100*robnr+20, 'Name', sprintf('Rob%d_Zielf_q', robnr), 'NumberTitle', 'off');
  sgtitle('Zielfunktionen (Position)');
  H1_PKM_all = reshape(H1_all(:,1,:), n, length(Namen_Methoden));
  H2_PKM_all = reshape(H2_all(:,1,:), n, length(Namen_Methoden));
  subplot(2,2,1); hold on;
  hdl={};
  hdl{1}=plot(t, H1_PKM_all);
  title('Zielfunktion 1 (nicht opt.)');
  ylabel('Zielfkt 1'); grid on;
  subplot(2,2,3); hold on;
  hdl{2}=plot(t, log10(H1_PKM_all));
  ylabel('Log.-Zielfkt 1 (n.o.)'); grid on;
  subplot(2,2,2); hold on;
  hdl{3}=plot(t, H2_PKM_all);
  title('Optimierungs-Zielfunktion 2');
  ylabel('Zielfkt 2'); grid on;
  subplot(2,2,4); hold on;
  hdl{4}=plot(t, log10(H2_PKM_all));
  ylabel('Log.-Zielfkt 2'); grid on;
  saveas(100*robnr+20, fullfile(respath,sprintf('Rob%d_Zielf_Pos', robnr)));
  % Linien nachträglich neu formatieren (bessere Lesbarkeit)
  for k = 1:4, leghdl=line_format_publication(hdl{k}, format_mlines); end
  legend(leghdl, Namen_Methoden);
  
  % Zielfunktionen bezogen auf Geschwindigkeits-Grenzen
  change_current_figure(100*robnr+21); clf;
  set(100*robnr+21, 'Name', sprintf('Rob%d_Zielf_qD', robnr), 'NumberTitle', 'off');
  sgtitle('Zielfunktionen (Geschwindigkeit)');
  H1D_PKM_all = reshape(H1D_all(:,1,:), n, length(Namen_Methoden));
  H2D_PKM_all = reshape(H2D_all(:,1,:), n, length(Namen_Methoden));
  subplot(2,2,1); hold on;
  hdl={};
  hdl{1}=plot(t, H1D_PKM_all);
  title('Zielfunktion 1 (nicht opt.)');
  ylabel('Zielfkt 1'); grid on;
  subplot(2,2,3); hold on;
  hdl{2}=plot(t, log10(H1D_PKM_all));
  ylabel('Log.-Zielfkt 1 (n.o.)'); grid on;
  subplot(2,2,2); hold on;
  hdl{3}=plot(t, H2D_PKM_all);
  title('Optimierungs-Zielfunktion 2');
  ylabel('Zielfkt 2'); grid on;
  subplot(2,2,4); hold on;
  hdl{4}=plot(t, log10(H2D_PKM_all));
  ylabel('Log.-Zielfkt 2'); grid on;
  saveas(100*robnr+21, fullfile(respath,sprintf('Rob%d_Zielf_Geschw', robnr)));
  % Linien nachträglich neu formatieren (bessere Lesbarkeit)
  for k = 1:4, leghdl=line_format_publication(hdl{k}, format_mlines); end
  legend(leghdl, Namen_Methoden);
  
  % Zielfunktionen bezogen auf Konditionszahl
  change_current_figure(100*robnr+22); clf;
  set(100*robnr+22, 'Name', sprintf('Rob%d_Zielf_cond', robnr), 'NumberTitle', 'off');
  subplot(2,1,1); hold on;
  Hcond_IKJac_all = reshape(Hcond_all(:,1,:), n, length(Namen_Methoden));
  hdl=plot(t, Hcond_IKJac_all);
  leghdl=line_format_publication(hdl, format_mlines);
  legend(leghdl, Namen_Methoden);
  grid on; ylabel('cond(Phi_q)', 'interpreter', 'none');
  subplot(2,1,2); hold on;
  Hcond_PKMJac_all = reshape(Hcond_all(:,2,:), n, length(Namen_Methoden));
  hdl=plot(t, Hcond_PKMJac_all);
  leghdl=line_format_publication(hdl, format_mlines);
  legend(leghdl, Namen_Methoden);
  grid on; ylabel('cond(J)', 'interpreter', 'none');
  sgtitle('Konditionszahl (mögliche Opt. Zielf.)');
  linkxaxes
  saveas(100*robnr+22, fullfile(respath,sprintf('Rob%d_Zielf_cond', robnr)));
  %% Trajektorie: Bild für Plattformbewegung
  change_current_figure(100*robnr+24);clf;
  set(100*robnr+24, 'Name', sprintf('Rob%d_TrajX', robnr), 'NumberTitle', 'off')
  sgtitle('Trajektorie X (Details)');
  for i = 1:6
    subplot(3,2,i); hold on
    linhdl1=plot(t, reshape(XE_all(:,i,:),n,length(Namen_Methoden)));
    linhdl3=plot(t, X_t(:,i), ':');
    % Eckpunkte einzeichnen
    plot(t(IL), X_t(IL,i), 'ko');
    if i < 4, unit = 'm';
    else, unit = 'rad';
    end
    ylabel(sprintf('x %d in %s', i, unit));
    grid on
    leghdl1=line_format_publication(linhdl1, format_mlines);
%     if i == 5, legend([leghdl1;linhdl3], [Namen_Methoden,{'Soll 3T3R'}]); end
  end
  linkxaxes

  saveas(100*robnr+24, fullfile(respath,sprintf( ...
    'Rob%d_%s_TrajX.fig',robnr, RP.mdlname)));

  % Bild für Drehung der redundanten Koordinate
  change_current_figure(100*robnr+25); clf;
  set(100*robnr+25, 'Name', sprintf('Rob%d_phiz', robnr), 'NumberTitle', 'off');
  sgtitle(sprintf('Redundante Koordinate'));
  hdl = NaN(length(Namen_Methoden),3); leghdl = hdl;
  subplot(1,3,1); hold on;
  hdl(:,1)=plot(t, reshape(XE_all(:,6,:),n,length(Namen_Methoden)));
  grid on; ylabel('phi z');
  
  subplot(1,3,2); hold on;
  hdl(:,2)=plot(t, reshape(XDE_all(:,6,:),n,length(Namen_Methoden)));
  plot(t([1,end]), repmat(RP.xDlim(6,:),2,1), 'r-');
  grid on; ylabel('phiD z');
  ylim([-0.1, +0.1]+RP.xDlim(6,:));
  
  subplot(1,3,3); hold on;
  hdl(:,3)=plot(t, reshape(XE_all(:,6,:),n,length(Namen_Methoden)));
  grid on; ylabel('phiDD z');
  plot(t([1,end]), repmat(RP.xDDlim(6,:),2,1), 'r-');
  ylim([-0.1, +0.1]+RP.xDDlim(6,:));
  for i = 1:3
    leghdl(:,i)=line_format_publication(hdl(:,i), format_mlines);
  end
  legend(leghdl(:,3), Namen_Methoden);
  linkxaxes
  saveas(100*robnr+25, fullfile(respath,sprintf( ...
    'Rob%d_%s_Traj_phiz.fig',robnr, RP.mdlname)));
  %% Trajektorie: Vergleich Gelenkverlauf nach verschiedenen Methoden
  for Dtype = 1:3
    change_current_figure(100*robnr+70+Dtype); clf;
    if     Dtype == 1, Dstring = ''; %#ok<ALIGN>
    elseif Dtype == 2, Dstring = 'D';
    elseif Dtype == 3, Dstring = 'DD'; end
    set(100*robnr+70+Dtype, 'Name', sprintf('Rob%d_Q%s', robnr, Dstring), 'NumberTitle', 'off');
    sgtitle(sprintf('Q %s', Dstring));
    hdl = NaN(RP.NLEG,RP.Leg(1).NJ,length(Namen_Methoden));
    axhdl = NaN(RP.NLEG,RP.Leg(1).NJ);
    for kk = 1:length(Namen_Methoden)
      Q = Q_t_all(:,:,kk);
      QD = QD_t_all(:,:,kk);
      QDD = QDD_t_all(:,:,kk);
      ii = 0;
      for i = 1:RP.NLEG
        for j = 1:RP.Leg(i).NJ
          ii = ii + 1;
          axhdl(i,j)=subplot(RP.NLEG,RP.Leg(1).NJ,sprc2no(RP.NLEG, RP.Leg(1).NJ, i, j));
          hold on; grid on;
          if Dtype == 1
            hdl(i,j,kk)=stairs(t, Q(:,ii));
            plot(t([1,end]), repmat(RP.Leg(i).qlim(j,:),2,1), 'r-');
            ylim([-0.1, +0.1]+qlim(ii,:));
          elseif Dtype == 2
            hdl(i,j,kk)=stairs(t, QD(:,ii));
            plot(t([1,end]), repmat(RP.Leg(i).qDlim(j,:),2,1), 'r-');
            ylim([-0.1, +0.1]+qDlim(ii,:));
          elseif Dtype == 3
            hdl(i,j,kk)=stairs(t, QDD(:,ii));
            plot(t([1,end]), repmat(RP.Leg(i).qDDlim(j,:),2,1), 'r-');
            ylim([-0.1, +0.1]+qDDlim(ii,:));
          end
          if j == 1, ylabel(sprintf('BK %d', i)); end
          if i == 1, title(sprintf('q%s %d', Dstring, j)); end
        end
      end
    end
    linkxaxes
    % Linien nachträglich neu formatieren (bessere Lesbarkeit)
    for i = 1:RP.NLEG
      for j = 1:RP.Leg(i).NJ
        leghdl=line_format_publication(hdl(i,j,:), format_mlines);
      end
    end
    remove_inner_labels(axhdl,1);
    legend(leghdl, Namen_Methoden);
    saveas(100*robnr+70+Dtype, fullfile(respath,sprintf('Rob%d_Q%s', robnr, Dstring)));
  end
  %% Ergebniss speichern
  save(fullfile(respath,sprintf('Rob%d_%s_result.mat',robnr,RP.mdlname)));

  %% Trajektorie mit verschiedene z Komponente berechnen
  RP.update_EE_FG(I_EE_full, I_EE_full);
  s_Traj_z = struct( ...
    'normalize', false, ...
    'wn', [0;0;0;0]);
  nzE = 360/10;
  Q_z_all = NaN(n, RP.NJ, nzE);
  h1_z_all = zeros(n,nzE);
  h2_z_all = zeros(n,nzE);
  X_t_z = X_t;
  t0 = tic();
  for zz = 1:nzE
    X_t_z(:,6) = 2*pi/nzE * zz;
    t1 = tic();
    [Q1_t_ns, QD1_t_ns, QDD1_t_ns, Phi1_t_ns] = RP.invkin2_traj(X_t_z, XD_t, XDD_t, t, q1, s_Traj_z);   
    fprintf('Trajektorie mit konstanter EE-Rotation %d/%d berechnet. Dauer: %1.1fs. Verbleibend ca. %1.1fs\n', ...
      zz, nzE, toc(t1), toc(t0)/zz*(nzE-zz));
    for kk = 1:n % Kennzahlen dafür berechnen
      h1_z_all(kk,zz) = invkin_optimcrit_limits1(Q1_t_ns(kk,:)', qlim);
      h2_z_all(kk,zz) = invkin_optimcrit_limits2(Q1_t_ns(kk,:)', qlim);
    end
    Q_z_all(:,:,zz) = Q1_t_ns;
  end
  %% Trajektorie mit verschiedene z Komponente auswerten
  hdl={};
  change_current_figure(100*robnr+28); clf;
  set(100*robnr+28, 'Name', sprintf('Rob%d_Zielf_z', robnr), 'NumberTitle', 'off');
  sgtitle('Zielfunktionen bei verschiedenen zE-Drehungen');
  
  subplot(2,2,1); hold on;
  plot(t, h1_z_all, 'c:');
  plot(t, min(h1_z_all,[],2), 'r--');
  hdl{1}=plot(t, H1_PKM_all);
  title('Zielfunktion 1 ');
  ylabel('Zielfkt 1'); grid on;
  ylim([0.8*min(H1_PKM_all(:)), 2.5*max(H1_PKM_all(:))]);
 
  subplot(2,2,3); hold on;
  plot(t, log10(h1_z_all), 'c:');
  plot(t, log10(min(h1_z_all,[],2)), 'r--');
  hdl{2}=plot(t, log10(H1_PKM_all));
  ylim([log10(0.8*min(H1_PKM_all(:))), log10(2.5*max(H1_PKM_all(:)))]);
  ylabel('Log.-Zielfkt 1 (n.o.)'); grid on;
  
  subplot(2,2,2); hold on;
  plot(t, h2_z_all, 'c:');
  plot(t, min(h2_z_all,[],2), 'r--');
  hdl{3}=plot(t, H2_PKM_all);
  ylim([0.8*min(H2_PKM_all(:)), 2.5*max(H2_PKM_all(:))]);
  title('Optimierungs-Zielfunktion 2');
  ylabel('Zielfkt 2'); grid on;
  
  subplot(2,2,4); hold on;
  plot(t, log10(h2_z_all), 'c:');
  plot(t, log10(min(h2_z_all,[],2)), 'r--');
  hdl{4}=plot(t, log10(H2_PKM_all));
  ylim([log10(0.8*min(H2_PKM_all(:))), log10(2.5*max(H2_PKM_all(:)))]);
  ylabel('Log.-Zielfkt 2'); grid on;
  for k = 1:4, leghdl=line_format_publication(hdl{k}, format_mlines); end

  % Alle Gelenkwinkel vergleichen
  change_current_figure(100*robnr+29);clf;
  set(100*robnr+29, 'Name', sprintf('Rob%d_Gelenkw_vgl', robnr), 'NumberTitle', 'off');
  sgtitle('Gelenkwinkel bei verschiedenen zE-Drehungen');
  ii = 0;
  for i = 1:RP.NLEG
    for j = 1:RP.Leg(i).NJ
      ii = ii + 1;
      subplot(RP.NLEG,RP.Leg(1).NJ,sprc2no(RP.NLEG, RP.Leg(1).NJ, i, j));
      hold on;
      qz_data = reshape(Q_z_all(:,ii,:), n, nzE);
      I_io = qz_data >= qlim(ii,1) & qz_data <= qlim(ii,2);
      qz_data_iO = qz_data;
      qz_data_iO(~I_io) = NaN; % damit nicht im Plot sichtbar
      qz_data_niO = qz_data;
      qz_data_niO(I_io) = NaN;
      ghdl=plot(t, qz_data_iO, 'g:');
      bhdl=plot(t, qz_data_niO, 'r:');
      hdl=plot(t, reshape(Q_t_all(:,ii,:), n, length(Namen_Methoden)));
      leghdl=line_format_publication(hdl, format_mlines);
      if j == 1, ylabel(sprintf('BK %d', i)); end
      if i == 1, title(sprintf('q %d', j)); end
      if ii == RP.NJ
        legend([ghdl(1);bhdl(1);leghdl],[{'gut'},{'schlecht'},Namen_Methoden]);
      end
    end
  end
  
  
  %% Animation des bewegten Roboters
  if usr_create_anim
  RP.I_EE_Task = I_EE_red; % zum Zeichnen, damit x(6) ignoriert wird
  for kk = 1:length(Namen_Methoden)
    Q_t = Q_t_all(:,:,kk);
    Name = Namen_Methoden{kk};
    anim_filename = fullfile(respath, sprintf('Rob%d_M%d_%s_%s',robnr, kk,RP.mdlname, Name));
    s_anim = struct( 'gif_name', [anim_filename,'.gif'], ...
                     'mp4_name', [anim_filename,'.mp4'] );
    s_plot = struct( 'ks_legs', [], 'straight', 0);
    change_current_figure(100*robnr+30+kk);clf;hold all;
    set(100*robnr+30+kk, 'name', sprintf('Rob%d_Anim_M%d', robnr, kk), ...
      'color','w', 'NumberTitle', 'off', 'units','normalized',...
      'outerposition',[0 0 1 1]); % Vollbild, damit GIF größer wird
    view(3);
    axis auto
    hold on;grid on;
    xlabel('x in m');ylabel('y in m');zlabel('z in m');
    title(Namen_Methoden{kk});
    plot3(X_t(:,1), X_t(:,2), X_t(:,3));
    maxduration_animation = 10; % Dauer der Animation als mp4 (in s)
    t_Vid = (0:1/30*(t(end)/maxduration_animation):t(end))';
    I_anim = knnsearch( t , t_Vid ); % Berechne Indizes in Traj.-Zeitstempeln
    RP.anim( Q_t(I_anim,:), X_t(I_anim,:), s_anim, s_plot);
    fprintf('Animation der Bewegung gespeichert: %s\n', s_anim.gif_name);
  end
  end

  fprintf('Test für %s beendet\n',RP.mdlname);
  close all % sonst sind zu viele Bilder offen.
end
dockall
