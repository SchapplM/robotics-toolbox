% Inverse Kinematik für Trajektorie mit Dynamischer Programmierung
% (für Aufgabenredundanz mit einem Redundanz-FG)
% Zwischen gegebenen Rastpunkten der Trajektorie werden optimale Werte für
% die redundante Koordinate gefunden. Optimiert wird die Drehung um die
% z-Achse des Endeffektors.
% 
% Eingabe:
% R
%   Roboter-Klasse
% X_t_in [NT x 6], XD_t_in [NT x 6], XDD_t_in [NT x 6], t [NT x 1]
%   Endeffektor-Trajektorie (Orientierung als Euler-Winkel).
%   Die Trajektorie muss eine Rast-zu-Rast-Bewegung mit NI Stützstellen
%   sein
% q0
%   Start-Konfiguration der Gelenkkoordinaten
% s_in
%   Struktur mir Einstellungswerten (siehe Quelltext)
% 
% Ausgabe:
% XE [NI x 6]
%   Modifizierte Stützstellen der Trajektorie (Spalte 6 ist
%   Optimierungsvariable, phiz-Euler-Winkel). Spalte 1-5 wie Eingabe.
% DPstats
%   Struktur mit Details zum Ablauf der Dynamischen Programmierung. Felder:
%   Siehe Quelltext
% TrajDetail
%   Detail-Struktur zu der final generierten Trajektorie. Felder:
%   .Q, .QD, .QDD [NT x NJ]: Gelenkwinkel-Trajektorie
%   .X6, XD6, XDD6 [NT x 1]: Trajektorie für Optimierungsvariable
%   .JP: Gelenkpositionen, Ausgabe der invkin-Funktion der Roboter-Klasse
%   .Jinv_ges: PKM-Jacobi-Matrix. Ausgabe der Roboter-Klasse. 
% 
% Siehe auch: cds_debug_taskred_perfmap, SerRob/invkin2_traj, 
%   ParRob/invkin2_traj, @RobBase/perfmap_taskred_ik

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-02
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [XE, DPstats, TrajDetail] = dynprog_taskred_ik(R, X_t_in, XD_t_in, ...
  XDD_t_in, t, q0, s_in)
%% Initialisierung der Einstellungen
s = struct( ...
  'phi_min', -pi, ... % Untere Grenze für Optimierungsvariable
  'phi_max', pi, ... % Obere Grenze
  'phi_lim_x0_dependent', true, ... % Diskrete Werte für phi hängen von Anfangswert ab
  'xDlim', R.xDlim, ... % Minimale und maximale EE-Geschw. (6x2). Letzte Zeile für Optimierungsvariable
  'xDDlim', R.xDDlim, ... % Minimale und maximale EE-Beschleunigung.
  'constraint_qDlim', true, ... % Breche ab bei Verletzung der Gelenk-Geschw.-Grenze
  'constraint_qDDlim', true, ... % Breche ab bei Verletzung der Gelenk-Beschl.-Grenze
  ... % Zusammensetzung der Kostenfunktion der Dynamischen Programmierung
  ... % über die Stufen. Möglich: average, max, RMStime, RMStraj
  'cost_mode', 'RMStime', ... 
  ... % Kriterium zur Entscheidung, falls mehrere Pfade hinsichtlich 
  ... % cost_mode gleich gut sind. Möglich: motion_redcoord, cond
  'cost_mode2', 'cond', ...
  ... % Schwellwert zum Abbruch der Trajektorien-Kinematik
  'abort_thresh_h', inf(R.idx_ik_length.hntraj, 1), ... % Standardmäßig alle Kriterien nutzen, die berechnet werden
  ... % Freie Bewegung zusätzlich zu vorgegebenen Ziel-Intervallen zulassen
  'use_free_stage_transfer', true, ...
  'overlap', false, ... % Die Zustände bei Redundanz-Optimierung werden zusätzlich überlappend gewählt
  'stageopt_posik', false, ... % Jeder Zustand wird nochmal mit der Positions-Kinematik optimiert nur erneut als Ziel probiert
  ... % Redundanzkarte für schönere Debug-Bilder. Ausgabe von perfmap_taskred_ik
  'PM_H_all', [], 'PM_s_ref', [], 'PM_s_tref', [], 'PM_phiz_range', [], ...
  'PM_Q_all', [], ... % optional Gelenkwinkel für Redundanzkarte zum Debuggen
  'PM_limit', false, ... % Begrenzung der Farbskala der Redundanzkarte
  ...'wn_names', {{}}, ... % Zusätzliche Optimierungskriterien (Benennung siehe idx_ikpos_wn aus Roboter-Klasse)
  ... % Gewichtung der einzelnen Zielkriterien in der Kostenfunktion.
  'wn', ones(R.idx_ik_length.wnpos, 1), ... % Bezieht sich auf Positions-IK
  'settings_ik', struct(''), ... % Einstellungen für Trajektorien-IK-Funktion
  'n_phi', 6, ... % Anzahl der Diskretisierungsschritte für Optimierungsvariable (kann um 1 abweichen, je nach Anfangswert)
  'T_dec_ns', 0.1, ... % Verzögerungszeit zum Abbremsen der Nullraumbewegung bei DP-Stufen
  'Tv', 1/3, ... % Verzögerungszeit für Aufgabenbewegung (Nullraum-Abbremsen schon vorher)
  ... % Indizes für Eckpunkte der Trajektorie. Eckpunkte sind Rastpunkte und
  ... % Optimierungs-Zustände in der DP. Indizes bezogen auf `X_t_in` usw.
  'IE', [1, size(X_t_in,1)], ... 
  'debug_dir', '', ... % Verzeichnis zum Speichern aller Zwischenzustände
  'continue_saved_state', false, ... % Lade gespeicherten Zustand aus debug_dir um nach Fehler schnell weiterzumachen
  'debug', false, ... % Zusätzliche Rechnungen zur Fehlersuche
  'fastdebug', false, ... % Einige Prüfungen wieder deaktivieren, damit es schneller geht
  'verbose', 0); % Stufen: 1=Text, 2=Zusammenfassungs-Bilder, 3=alle Bilder
% Vorgegebene Einstellungen laden
if nargin == 7
  for f = fields(s_in)'
    if isfield(s, f{1})
      s.(f{1}) = s_in.(f{1});
    else % Fall soll eigentlich nicht vorkommen. Daher Prüfung als zweites
      error('Feld %s aus s_in kann nicht übergeben werden', f{1});
    end
  end
end
% Prüfe die Eingaben
assert(length(s.wn)==R.idx_ik_length.wnpos, 's.wn muss Dimension von wnpos haben');
assert(length(s.IE) > 1, 'Mindestens zwei Rastpunkte müssen angegeben werden');
assert(size(X_t_in,1) == length(t), 'Anzahl Bahnpunkte X/t muss übereinstimmen');
assert(size(XD_t_in,1) == length(t), 'Anzahl Bahnpunkte XD/t muss übereinstimmen');
assert(size(XDD_t_in,1) == length(t), 'Anzahl Bahnpunkte XD/t muss übereinstimmen');
assert(size(q0,1) == R.NJ, 'Anzahl Gelenkkoordinaten NJ in q0 muss stimmen');
assert(s.IE(1) == 1, 'Erster Eintrag von IE muss 1 sein');
assert(s.IE(end) == size(X_t_in,1), 'Letzter Eintrag von IE muss Trajektorienlänge sein');
assert(all(diff(s.IE)>0), 'Rastpunkt-Indizes müssen aufsteigend sein');
assert(length(unique(s.IE))==length(s.IE), 'Rastpunkt-Indizes müssen eindeutig sein');
if s.continue_saved_state && isempty(s.debug_dir)
  warning('Laden von gespeichertem Zustand nicht möglich, da kein Ordner gegeben.');
  s.continue_saved_state = false;
end
if strcmp(s.cost_mode, 'RMStraj') && isempty(s.PM_s_ref)
  error('Für Integration über Bahnkoordinate muss PM_s_ref gegeben sein');
end
if s.stageopt_posik && all(s.wn==0)
  warning('Modus stageopt_posik erfordert Angabe von IK-Nebenbedingungen in s.wn');
  s.stageopt_posik = false;
end
%% Roboter-bezogene Variablen initialisieren
if R.Type == 0
  qlim = R.qlim;
  qDlim = R.qDlim;
  qDDlim = R.qDDlim;
else
  qlim = cat(1,R.Leg.qlim);
  qDlim = cat(1,R.Leg.qDlim);
  qDDlim = cat(1,R.Leg.qDDlim);
end
if s.verbose > 1 || s.debug % Notwendig für Debug-Bilder
  if ~isempty(s.PM_H_all)
    % Skaliere die Zeit-Trajektorie zu den normalisierten Koordinaten
    [s_tref, I_unique] = unique(s.PM_s_tref); % bei doppelter Belegung Fehler in interp1
    t_tref = interp1(s_tref, t(I_unique), s.PM_s_ref);
    assert(abs(t_tref(end)-t(end)) < 1e-6, ['Zeitbasis zwischen Traj. ', ...
      'und Redundanzkarte stimmt nicht']);
    if s.debug && ~isempty(s.PM_Q_all) && ~s.fastdebug
      t1 = tic();
      % Prüfe, ob die eingegebene Gelenkwinkel-Trajektorie aus der 
      % Redundanzkarte zu der EE-Trajektorie passt
      for kk = 1:length(s.PM_phiz_range) % Gehe scheibenweise Karte durch
        Q_kk = squeeze(s.PM_Q_all(kk,:,:))'; % Alle Gelenkkoordinaten zu Winkel kk
        X_kk = R.fkineEE2_traj(Q_kk); % EE-Lage dazu neu bestimmen
        assert(all(abs(s.PM_phiz_range(kk)-X_kk(:,6))<1e-6 | isnan(X_kk(:,6))), ...
          'Eingegebene Redundanzkarte ist inkonsistent. Winkel-Iteration %d', kk);
        for ii = 1:length(t_tref) % Alle Zeitschritte der Redundanzkarte prüfen
          % Finde die Entsprechung zwischen Redundanzkarte und Trajektorie
          [t_err, I_t] = min(abs(t_tref(ii) - t));
          % Prüfe die Übereinstimmung zwischen den EE-Koordinaten
          if abs(t_err) < 1e-8 && ... % Zeit-Basis muss übereinstimmen für Vergleich
              ~any(isnan(X_kk(ii,1:5))) % Bei fehlendem Wert NaN. Dann nicht prüfbar
            % Zeitpunkt stimmt genau überein. Prüfe Schritt der Trajektorie
            test_x_ii = X_kk(ii,1:5)' - X_t_in(I_t,1:5)';
            assert(all(abs(test_x_ii) < 1e-2), sprintf(['X-Trajektorie ', ...
              'stimmt nicht zwischen Eingabe (Schritt %d) und Redundanzkarte ', ...
              '(Schritt %d). Abweichung Zeitbasis %1.1es. delta x: [%s]'], ...
              I_t, ii, t_err, disp_array(test_x_ii', '%1.2g')));
          end
        end % for ii
      end % for kk
      if s.verbose > 0
        fprintf('Redundanzkarte gegen Eingabe-Trajektorie geprüft (in %1.1fs)\n', toc(t1));
      end
    end
  end
end
if ~isempty(s.debug_dir)
  % Speichere detaillierte Information zu den Eingabewerten ab
  save(fullfile(s.debug_dir, 'dp_init.mat'), 's', 'X_t_in', 'XD_t_in', ...
    'XDD_t_in', 't', 'q0');
end
%% Einstellungen für Trajektorien-IK vorbereiten
s_Traj = struct('enforce_qlim', false, ... % hier nicht relevant bzw. anderweitig abgedeckt
  ... % Plattform-Trajektorie in Toleranzband erzwingen. Sollte eigentlich
  ... % durch Kriterium xlim_hyp erreicht werden. Ist nur Rückfallebene
  'enforce_xlim', true, ... 
  'Phit_tol', 1e-12, 'Phir_tol', 1e-12); % hohe Genauigkeit für Positionskorrektur
% Gewichtungen der Nullraumoptimierung einstellen.
s_Traj.wn = zeros(R.idx_ik_length.wntraj,1);
wn_names = {}; % Namen der aktiven Nebenbedingungen
wn_hcost = zeros(1,R.idx_ik_length.hntraj); % Gewichtungen für die Ausgabe aus Stats.h
for f = fields(R.idx_ikpos_wn)'
  % Gehe alle vorgesehenen Kriterien der Eingabe durch. Diese sind bezogen
  % auf die Positions-IK und werden auf die Trajektorien-IK umgerechnet
  if s.wn(R.idx_ikpos_wn.(f{1})) ~= 0
    % Setze alle Kriterien standardmäßig auf moderat gedämpfte PD-Regler.
    % Benutze die Gewichtung des P-Anteils, die vorgegeben wird.
    % Dann sind die Gewichtungen konsistent mit der aufrufenden Funktion.
    s_Traj.wn(R.idx_iktraj_wnP.(f{1})) = 1.0 * s.wn(R.idx_ikpos_wn.(f{1}));
    s_Traj.wn(R.idx_iktraj_wnD.(f{1})) = 0.7 * s.wn(R.idx_ikpos_wn.(f{1}));
    wn_names = [wn_names, f{1}]; %#ok<AGROW>
    wn_hcost(R.idx_iktraj_hn.(f{1})) = s_Traj.wn(R.idx_iktraj_wnP.(f{1}));
  end
end
s_Traj.wn(R.idx_iktraj_wnP.qDlim_par) = 0.8; % Dämpfung     
% Begrenzung der Plattform-Drehung. Wird in DP vorgegeben. Ohne diese
% Begrenzung würde der DP-Ansatz nicht funktionieren, da die vorgesehenen
% Zustände für die Optimierungsvariable nicht im Toleranzband eingehalten
% werden können.
% Quadratische Gewichtung: Zieht schwach in die Mitte zum Toleranzband
s_Traj.wn(R.idx_iktraj_wnP.xlim_par) = 1; %   P-Regler
s_Traj.wn(R.idx_iktraj_wnD.xlim_par) = 0.7; % D-Regler
% Hyperbolische Gewichtung des Abstands zu den Grenzen: Stößt stark von
% Grenzen ab. Muss sehr hoch skaliert werden, da Zielfunktion erst kurz vor
% Grenze wirklich stark ansteigt und dann der Bremsweg oft zu kurz ist.
% Sonst dominiert die Konditionszahl
s_Traj.wn(R.idx_iktraj_wnP.xlim_hyp) = 100*1; %   P-Regler
s_Traj.wn(R.idx_iktraj_wnD.xlim_hyp) = 100*0.7; % D-Regler
% Dämpfung xlim quadratisch (bzw. Bestrafung der Abweichung von Ref.-Geschw.)
s_Traj.wn(R.idx_iktraj_wnP.xDlim_par) = 0.8;
% Einhaltung der Geschwindigkeitsgrenzen erzwingen (mit Nullraumbewegung)
s_Traj.enforce_xDlim = true;
% Sofort abbrechen, wenn eine der Nebenbedingungen verletzt wurde. Dadurch
% schnellere Berechnung. Konfiguration über Eingabe möglich.
s_Traj.abort_thresh_h = s.abort_thresh_h;
% Einhaltung der Grenzen der Optimierungsvariable nicht erzwingen. Wird mit
% enforce_xlim gemacht und so wird eine kurze, geringfügige Überschreitung
% erlaubt. Falls inf gesetzt wird, erfolgt sofort ein Abbruch wenn die Traj.
% das Toleranzband überschreitet.
s_Traj.abort_thresh_h(R.idx_iktraj_hn.xlim_hyp) = NaN;
% Nehme nicht die IK-Lösung mit minimaler Norm von qDD, sondern benutze die
% Trajektorie aus der Dynamischen Optimierung als Vorsteuerung.
% Sonst können die Toleranzbänder für die Zustandswechsel oft nicht
% eingehalten werden, da die Minimum-Norm-Lösung eine starke Richtungsvor-
% gabe macht.
s_Traj.ik_solution_min_norm = false;
% Spezifische Einstellungen für einige Optimierungskriterien übernehmen.
% Betrifft solche, die den Wert der Kriterien beeinflussen. Sollte konsis-
% tent mit den Vorgaben aus der aufrufenden Funktion (Maßsynthese) sein
for f = fields(s.settings_ik)'
  if any(strcmp(f{1}, {'cond_thresh_ikjac', 'optimcrit_limits_hyp_deact', ...
      'collbodies_thresh', 'installspace_thresh', 'cond_thresh_jac', ...
      'thresh_ns_qa'}))
    s_Traj.(f{1}) = s.settings_ik.(f{1});
  end
end
% Übernehme maximale Geschwindigkeit und Beschleunigung der Plattform aus
% Eingabe. Kann von in Matlab-Klasse gespeichertem Wert abweichen.
s_Traj.xDlim = s.xDlim;
s_Traj.xDDlim = s.xDDlim;
task_redundancy = true;
if R.I_EE_Task(6) == R.I_EE(6)
  % Benutze keine lokale Optimierung mit Aufgabenredundanz, sondern die
  % vorab bestimmte Trajektorie zwischen den gewählten Zuständen für die
  % redundante Koordinate. Setze nur die wn-Komponenten auf ungleich Null, 
  % für die der Abbruch-Schwellwert geprüft werden soll.
  task_redundancy = false;
  s_Traj.wn(:) = 0;
  for f = fields(R.idx_iktraj_hn)'
    if ~isnan(s_Traj.abort_thresh_h(R.idx_iktraj_hn.(f{1})))
      s_Traj.wn(R.idx_iktraj_wnP.(f{1})) = 1;
%       fprintf('Setze wn(%s) auf 1.\n', f{1});
    end
  end
  s_Traj.wn(R.idx_iktraj_wnP.xlim_hyp) = 0; % irrelevant bei 3T3R
  s_Traj.wn(R.idx_iktraj_wnP.qDlim_hyp) = 0; % irrelevant bei 3T3R
  % Freier Transfer zwischen Zuständen ist nicht sinnvoll ohne Redundanz
  s.use_free_stage_transfer = false;
  % Überlappende Intervalle ebenfalls nicht sinnvoll
  s.overlap = false;
end
% Debug-Einstellung auch in Trajektorie übernehmen
s_Traj.debug = s.debug; % Damit viele Test-Rechnungen in Traj.-IK

%% Dynamische Programmierung vorbereiten
x0 = R.fkineEE2_traj(q0')'; % Start-Pose für Zustand auf erster Stufe
% Erzeuge die Werte für die diskreten Stufen der dynamischen Programmierung
phi_range_fix = linspace(s.phi_min, s.phi_max, s.n_phi); % Theoretische Diskretisierung der Optimierungsvariablen
if length(phi_range_fix) == 1
  delta_phi = 0; % Sonderfall: Nur ein Zustand
else
  delta_phi = phi_range_fix(2)-phi_range_fix(1); % Abstand zwischen zwei Zuständen
end
% Erzeuge den Bereich so, dass der Anfangswert mit vorkommt. Damit ist
% prinzipiell ein Beibehalten des Anfangswertes in der nicht-redundanten
% Version möglich.
if s.phi_lim_x0_dependent
  phi_range_all = [x0(6):delta_phi:s.phi_max, x0(6):-delta_phi:s.phi_min];
  [~, I] = sort(abs(phi_range_all-x0(6)));
  phi_range = phi_range_all(I(2:end)); % doppelten Startwert entfernen
  phi_range = sort(phi_range); % Reihenfolge wieder aufsteigend machen (besser nachvollziehbar)
else
  % Diskretes Intervall benutzen (Ersteinreichung des LNEE-Papers).
  phi_range = linspace(s.phi_min, s.phi_max, s.n_phi);
end
if isempty(phi_range), phi_range = x0(6); end % Falls es nur einen Zustand gibt
% Setze redundante Koordinate der Eingabe-Trajektorie auf NaN. Muss
% weiter unten überschrieben werden.
X_t_in(:,6) = NaN;  XD_t_in(:,6) = NaN; XDD_t_in(:,6) = NaN;
test_x0 = [x0(1:3)-X_t_in(1,1:3)'; angleDiff(x0(4:6),X_t_in(1,4:6)')];
assert(all(abs(test_x0(1:5)) < 1e-4), sprintf(['q0 und x0 ', ...
  'stimmen nicht. Fehler: [%s]mm, [%s]°'], disp_array(1e3*test_x0(1:3)', '%1.2e'), ...
  disp_array(180/pi*test_x0(4:5)', '%1.1f')));
% Weitere Variablen vorbereiten
IE = s.IE; % Indizes der Eckpunkte (DP-Zustände)
XE = X_t_in(IE,:); % EE-Pose für die Eckpunkte
N = size(XE,1)-1; % Anzahl der Entscheidungsstufen (erste Stufe schon festgelegt)
z = length(phi_range); % Anzahl unterschiedlicher Orientierungen
z2 = z; % Anzahl der Ende-Zustände auf der jeweils nächsten Stufe
if s.use_free_stage_transfer
  % Zusätzlicher (virtueller) Ziel-Zustand für freie Bewegung. Nur einen pro
  % Start-Zustand.
  z2 = z2 + 1;
end
if s.overlap
  % Weitere virtuelle Ziel-Zustände genau versetzt zu den ursprünglichen
  z2 = z2 + z - 1;
elseif s.stageopt_posik
  % Zusätzliche Ziel-Zustände aus Optimierung der ursprünglichen
  z2 = z2 + z;
end
z1 = z2; % Anzahl der Start-Zustände auf jeder Stufe. Entspricht End-Zuständen der vorherigen
% Einstellungen für Positions-IK, falls Zustände damit berechnet werden
if s.stageopt_posik
  % Übernehme die wesentlichen Einstellungen von Traj.-IK
  s_Pos = struct('wn', s.wn, 'cond_thresh_ikjac', s_Traj.cond_thresh_ikjac, ...
    'cond_thresh_jac', s_Traj.cond_thresh_jac, 'Phit_tol', s_Traj.Phit_tol, ...
    'Phir_tol', s_Traj.Phir_tol);
  % Begrenzung der redundanten Koordinate wieder einsetzen
  s_Pos.wn(R.idx_ikpos_wn.xlim_hyp) = 1e4; % sehr stark von Grenzen abstoßen
  s_Pos.wn(R.idx_ikpos_wn.xlim_par) = 0; %  ohne linearen Term
  % Die Grenzen xlim müssen unten in Abhängigkeit des Startwertes
  % festgelegt werden, da die Angabe relativ dazu ist.
end
% Endwerte für die Gelenkkonfiguration der optimalen Teilstrategie für jede
% Stufe (aus Vorwärts-Iteration). Setze eine virtuelle Stufe 0 in die
% Variable für einfachere Indizierung bezogen auf Startwert
QE_all = NaN(R.NJ, z, N+1);
QE_all(:, 1, 1) = q0; % virt. Endwert Stufe 0, ist Anfangswert der Stufe 1
% Endwert der EE-Drehung (Optimierungsvariable) für die optimalen
% Teilstrategien für jede Stufe.
XE_all = NaN(N+1, z2);
XE_all(1,1) = x0(6); % virt. Endwert Stufe 0
% Kumulierte Zielfunktionswerte bis zur aktuellen Stufe, basierend auf der
% teil-optimalen Politik bis hier hin
F_all = inf(N+1, z2);
F_all(1,1) = 0; % Startwert für Stufe 1
F_all(1,2:end) = inf; % nur ein Startwert. Andere Einträge damit deaktiviert
% Nummer des Zustands der vorherigen Stufe, der zur optimalen Teilpolitik
% bis zum jeweiligen Zustand führt
I_all = zeros(N+1, z2);
% Alle Endwerte der IK für die berechneten Transitionen auf der aktuellen
% Stufe. Wird für z Anfangs-Drehungen und z End-Drehungen gemacht.
% Index 2: Start-Zustand, Index 3: End-Zustand
QE_stage = NaN(R.NJ, z1, z2);
% Das gleiche für die Orientierung
% Index 1: Start-Zustand, Index 2: End-Zustand
YE_stage = NaN(z1, z2);
% Das gleiche für den Kostenterm für den Stufenübergang
F_stage = NaN(z1, z2);
% Zusätzlichen Kostenterm aus der Konditionszahl berechnen. Wird bei
% Gleichheit der sonstigen Kosten benutzt
F_stage_cond = NaN(z1, z2); F_stage_range = F_stage_cond;
% Vorbereitung statistischer Größen für die Ausgabe
n_statechange_succ = 0; % Anzahl erfolgreicher Stufenübergänge
n_statechange_total = 0; % Anzahl aller Stufenübergänge
% Speichere die Anzahl der erfolgreichen Stufenübergänge für jede Stufe
Stats_statechange_succ_all = zeros(N+1,z2);
% Anzahl insgesamt simulierter Trajektorien-Zeitschritte (für alle
% Übergänge und insgesamt).
Stats_nt_ik_all = zeros(N+1,z2);
Stats_comptime_ik_all = zeros(N+1,z2); % Rechenzeit dafür
nt_ik = 0;
%% Debug-Bild vorbereiten
if s.verbose > 1
  DbgLineHdl = NaN(z,z); % Handle mit Linien für alle Zwischenschritte
  figikhdl = change_current_figure(500);clf;hold on;
  sgtitle(figikhdl, sprintf('Redundanzkarte für alle Stufen'));
  set(figikhdl, 'Name', sprintf('DynProg_PerfMap'), 'NumberTitle', 'off');
  % Handles für Linien und Marker vorbereiten. Für Legende ganz am Ende.
  PM_hdl = NaN(10,1);
  % Dummy-Einträge für eingezeichnete Linien in Redundanzkarte
  PM_hdl(1) = plot(NaN,NaN,'c-');
  PM_hdl(2) = plot(NaN,NaN,'k-');
  PM_hdl(3) = plot(NaN,NaN,'m-');
  PM_hdl(4) = plot(NaN,NaN,'g-');
  PM_hdl(5) = plot(NaN,NaN,'rs');
  PM_legtxt = {'Local Opt.', 'Invalid', 'Valid', 'Optimal', ... % für Trajektorien
    'DP', ... % Eckpunkte für DP (Endergebnis)
    'Joint Lim', 'Act. Sing.', 'IK Sing.', ... % Marker für Nebenbedingungs-Verletzung ...
    'Collision', 'Install. Space', 'Out of Range'}; % ... aus Redundanzkarte
  PM_formats = {'bx', 'g*', 'g^', 'co', 'gv', 'm+'}; % NB-Marker
  %% Redundanzkarte einzeichen (falls gegeben)
  if ~isempty(s.PM_H_all) && ~s.fastdebug
    abort_thresh_hpos = NaN(R.idx_ik_length.hnpos, 1);
    for f = fields(R.idx_ikpos_hn)'
      if isfield(R.idx_iktraj_hn, f{1})
        abort_thresh_hpos(R.idx_ikpos_hn.(f{1})) = ...
          s_Traj.abort_thresh_h(R.idx_iktraj_hn.(f{1}));
      end
    end
    [Hdl_all, ~, PlotData] = R.perfmap_plot(s.PM_H_all, s.PM_phiz_range, ...
      t_tref, struct( ...
      'reference', 'time', 'wn', s.wn, 'abort_thresh_h', abort_thresh_hpos, ...
      'PM_limit', s.PM_limit, ...
      'markermindist', [max(t)/200, 1])); % leichtes Ausdünnen der Marker
    % Grenzen für Zustände einzeichnen
    for v = [s.phi_min, s.phi_max]*180/pi
      plot(t_tref([1; end]), repmat(v,2,1), 'k--');
    end
    cbtext = sprintf('h=%s(%s)', s.cost_mode, disp_array(wn_names, '%s'));
    if any(PlotData.I_exc(:))
      cbtext = [cbtext, sprintf('; Log: h>%1.0f', PlotData.condsat_limit_rel)];
    end
    if any(PlotData.I_colorlim(:))
      cbtext = [cbtext, sprintf('; h>%1.1e black', PlotData.colorlimit_rel)];
    end
    ylabel(Hdl_all.cb, cbtext, 'Rotation', 90, 'interpreter', 'none');
      PM_hdl(5+(1:6)) = Hdl_all.VM;
  end
  xlabel('Zeit in s');
  ylabel('Redundante Koordinate in deg');
end
%% Dynamische Programmierung berechnen
t0 = tic();
if s.verbose
  fprintf('Starte Dynamische Programmierung über %d Stufen\n', N);
end
% Schleife über alle Stufen (= Eckpunkte der Trajektorie)
for i = 2:size(XE,1) % von der zweiten Position an, bis letzte Position
  if i > 2 && s.verbose
    fprintf(['Starte Berechnungen für Stufe %d. Zeit bis hier: %1.1fs. ', ...
      'Verbleibend: ca. %1.1fs\n'], i, toc(t0), (size(XE,1)-i)*toc(t0)/(i-2));
  end
  if all(isnan(XE_all(i-1,:)))
    if s.verbose
      fprintf(['Stufe %d kann nicht erreicht werden. Kein gültiger ', ...
        'Zustand auf Stufe %d.\n'], i, i-1);
    end
    break;
  end
  % Trajektorien-Zeiten für den Übergang von i-1 nach i.
  t_i = t(IE(i-1):IE(i)); % Ausschnitt aus der Gesamt-Trajektorie
  if strcmp(s.cost_mode, 'RMStraj') % Bahnkoordinate für Ausschnitt
    s_i = s.PM_s_tref(IE(i-1):IE(i));
  end
  % Variablen für diesen Stufenübergang vorbelegt initialisieren
  F_stage(:) = inf;
  F_stage_cond(:) = inf;
  F_stage_range(:) = inf;
  YE_stage(:) = NaN;
  QE_stage(:) = NaN;
  if false && s.continue_saved_state % TODO: Prüfung fehlt, Tmp-Plot geht nicht
    % Prüfe, ob ein gespeicherter Zustand geladen werden kann
    d = load(fullfile(s.debug_dir, sprintf('dp_stage%d_final.mat', i-1)), ...
      'F_stage_sum', 'F_stage', 'F_stage_cond', 'F_stage_range', 'QE_stage', 'YE_stage');
    F_stage = d.F_stage;
    F_stage_cond = d.F_stage_cond;
    F_stage_range = d.F_stage_range;
    YE_stage = d.YE_stage;
    QE_stage = d.QE_stage;
    if s.verbose
      fprintf('Zustand %d aus Datei geladen. Überspringe Berechnung\n', i);
    end
  end
  % Gehe die Zustände nicht nach der Reihe durch, sondern nach Ähnlichkeit
  % zum Startwert. Wähle nur erreichbare Startwerte (nicht unendliche Kosten)
  [~, I_phidistasc] = sort(abs(XE_all(i-1,:)-x0(6)));
  I_phidistasc = I_phidistasc(~isinf(F_all(i-1,I_phidistasc)));
  if s.verbose
    fprintf('Prüfe %d gültige Anfangs-Zustände für Transfer zu Stufe %d: [%s]\n', ...
      length(I_phidistasc), i, disp_array(I_phidistasc, '%d'));
  end
  % Summe der auf dieser Stufe insgesamt zu prüfenden Übergänge
  n_statechange_total = n_statechange_total + length(I_phidistasc)*z2;
  for k = I_phidistasc % max. z1 unterschiedliche Orientierungen für vorherige Stufe
    t0_k = tic();
    % Eingabe der Stufen. Wert wählen, je nachdem welcher Wert vorher
    % in diesem Intervall geendet hat. Ohne Redundanzoptimierung sind das
    % genau die vorgegebenen diskreten Werte aus phi_range
    z_k = XE_all(i-1, k);
    if isnan(z_k) % dürfte gar nicht mehr vorkommen aufgrund Filter vor Schleife
      if s.verbose
        fprintf(['Überspringe Zustand %d. Keine Teilpolitik führt von ', ...
          'Stufe %d dahin\n'], k, i-1);
      end
      continue
    end
    if s.verbose
      fprintf('Stufe %d: Prüfe Start-Orientierung %d (%d/%d) (%1.1f°)\n', ...
        i, k, find(k==I_phidistasc), length(I_phidistasc), 180/pi*z_k);
    end
    % Anzahl der zu prüfenden Zustands-Transfers auf nächste Stufe.
    % Je nach Modus anders (siehe oben für Initialisierung der Variablen).
    if s.use_free_stage_transfer
      nz2_ik = z + 1;
    else
      nz2_ik = z;
    end
    if s.overlap
      nz2_ik = nz2_ik + (z-1);
    elseif s.stageopt_posik
      nz2_ik = nz2_ik + z;
    end
    % Merke, ob auf Stufe k bereits ein freier Transfer zur nächsten Stufe
    % durchgeführt wurde, ohne dass die Einstellung explizit genutzt wurde
    unconstrained_transfer_done_k = false;
    % Ziel-Zustände auch nicht der Reihe nach durchgehen, sondern nach
    % Ähnlichkeit zu z_k
    [~, I_phidistasc2] = sort(abs(phi_range-z_k));
    I_l = [I_phidistasc2, max(I_phidistasc2)+1:nz2_ik];
    for l = I_l % Zustand auf nächster Stufe. Reihenfolge: Normale Intervalle, Überlappende Intervalle bzw. Optimierung auf normales Intervall, freie Bewegung
      t0_l = tic();
      if s.verbose
        fprintf('Stufe %d, Start-Orientierung %d: Prüfe Aktion %d/%d\n', i, k, l, nz2_ik);
      end
      % Die zweite Hälfte der Prüfungen bezieht sich auf eine freie Bewegung
      if s.use_free_stage_transfer && l == nz2_ik % ist immer der letzte
        free_stage_transfer = true;
        % Wenn die freie Bewegung schon so stattgefunden hat, überspringen
        if unconstrained_transfer_done_k
          if s.verbose
            fprintf('Unbeschränkte Bewegung wurde bereits durchgeführt. Überspringe.\n');
          end
          continue;
        end
      else
        free_stage_transfer = false;
      end
      % Anfangswert aus Endwert aus vorherigen Iterationen
      qs = QE_all(:,k,i-1);
      assert(all(~isnan(qs)), 'Logik-Fehler. qs ist NaN');
      if ~free_stage_transfer
        if l <= z % Normales Intervall / Zustand
          z_l = phi_range(l);
        elseif s.stageopt_posik && l > z
          % vorläufiger Zielwert
          z_l = phi_range(l-z);
          % Erzeuge den Zielwert durch eine Optimierung mit der Positions-IK
          % Benutze den Zielwert der Stufe der vorherigen Ergebnisse
          if isnan(YE_stage(k,l-z))
            if s.verbose
              fprintf(['Nachbearbeitung von Transfer %d/%d nicht möglich, ', ...
                'keine Lösung\n'], i, l-z);
            end
            if z_l == z_k && s.verbose % Ziel-Wert entspricht dem Start-Wert. Einmal probieren, ob es doch geht.
              fprintf('Benutze den Wert der vorherigen Stufe als Startwert\n');
            else
              continue
            end
          end
          % Aufgabenredundanz für Positions-IK benutzen (Annahme: Ist nicht
          % schon gesetzt) und dann wieder zurücksetzen
          if ~task_redundancy
            I_EE_red = R.I_EE_Task; I_EE_red(end) = false;
            R.update_EE_FG(R.I_EE, I_EE_red);
          end
          xl_posik = X_t_in(IE(i),:)'; % Letzter Wert der Trajektorie (Rotation wird ignoriert)
          % Anfangswert aus allen schon berechneten Zuständen dieser Stufe
          % (der Reihe nach durchprobieren, anfangen bei nächsten).
          [~,I_distsort] = sort(abs(YE_stage(k,1:z)-z_l));
          I_distsort = I_distsort(~isnan(YE_stage(k,I_distsort)));
          if isempty(I_distsort) % Es gibt keine validen Zustände. Nehme Startwert von vorheriger Stufe oder vom Anfang
            q0_posik = [qs, q0];
          else
            q0_posik = [squeeze(QE_stage(:,k,I_distsort)), qs, q0];
          end
          s_Pos.retry_limit = size(q0_posik,2); % Keine darüber hinausgehenden zufälligen Neuversuche
          xl_posik(end) = z_l; % Ist-Rotation. Wird als Mittelpunkt für Grenzen xlim benutzt. Sonst keine Auswirkung, da mit 3T2R gerechnet wird
          % Intervall für redundante Koordinate. Muss aktualisiert werden.
          if length(phi_range_fix) == 1 % Sonderfall: Nur ein Zustand (in Haupt-Modus)
            % Nehme den kompletten erlaubten Bereich als Grenze
            s_Pos.xlim = [NaN(5,2); xl_posik(6)-s.phi_max, xl_posik(6)-s.phi_min];
          else
            % Bis zum Rand des Nachbar-Intervalls
            s_Pos.xlim = [NaN(5,2); [-1, 1]* 1.2 * delta_phi/2];
          end
          if R.Type == 0 % Seriell
            [q_i, Phi_i, ~, Stats_PosIK] = R.invkin2(R.x2tr(xl_posik), q0_posik, s_Pos);
          else % PKM
            [q_i, Phi_i, ~, Stats_PosIK] = R.invkin4(xl_posik, q0_posik, s_Pos);
            if false % Debug Pos.-IK
              figure(2000);clf;
              Iter = 1:1+Stats_PosIK.iter;
              if R.Type == 0, I_constr_red = [1 2 3 5 6];
              else,           I_constr_red = R.I_constr_red; end
              subplot(3,3,1);
              plot(Stats_PosIK.condJ(Iter,:));
              xlabel('Iterationen'); grid on;
              ylabel('cond(J)');
              legend({'IK-Jacobi', 'Jacobi'});
              subplot(3,3,2);
              plot([diff(Stats_PosIK.Q(Iter,:),1,1);NaN(1,R.NJ)]);
              xlabel('Iterationen'); grid on;
              ylabel('diff q');
              subplot(3,3,3); hold on;
              Stats_X = R.fkineEE2_traj(Stats_PosIK.Q(Iter,:));
              Stats_X(:,6) = denormalize_angle_traj(Stats_X(:,6));
              I_Phi_iO = all(abs(Stats_PosIK.PHI(Iter,I_constr_red))<1e-6,2);
              I_Phi_med = all(abs(Stats_PosIK.PHI(Iter,I_constr_red))<1e-3,2)&~I_Phi_iO;
              I_Phi_niO = ~I_Phi_iO & ~I_Phi_med;
              legdhl = NaN(3,1);
              if any(I_Phi_iO)
                legdhl(1)=plot(Iter(I_Phi_iO), 180/pi*Stats_X(I_Phi_iO,6), 'gv');
              end
              if any(I_Phi_med)
                legdhl(2)=plot(Iter(I_Phi_med), 180/pi*Stats_X(I_Phi_med,6), 'mo');
              end
              if any(I_Phi_niO)
                legdhl(3)=plot(Iter(I_Phi_niO), 180/pi*Stats_X(I_Phi_niO,6), 'rx');
              end
              plot(Iter, 180/pi*Stats_X(:,6), 'k-');
              legdhl(4) = plot(Iter([1, end])', 180/pi*(xl_posik(end) + s_Pos.xlim(6,1))*[1;1], 'k--');
              plot(Iter([1, end])', 180/pi*(xl_posik(end) + s_Pos.xlim(6,2))*[1;1], 'k--');
              legstr = {'Phi<1e-6', 'Phi<1e-3', 'Phi>1e-3', 'Limit'};
              legend(legdhl(~isnan(legdhl)), legstr(~isnan(legdhl)));
              xlabel('Iterationen'); grid on;
              ylabel('phi_z in deg');
              qlim = cat(1,R.Leg(:).qlim);
              qlim_range = qlim(:,2)-qlim(:,1);
              Stats_Q_norm = (Stats_PosIK.Q(Iter,:)-repmat(qlim(:,1)',1+Stats_PosIK.iter,1))./ ...
                              repmat(qlim_range',1+Stats_PosIK.iter,1);
              subplot(3,3,4);
              plot([Stats_Q_norm(Iter,:);NaN(1,R.NJ)]);
              xlabel('Iterationen'); grid on;
              ylabel('q norm');
              subplot(3,3,5);
              plot([diff(Stats_PosIK.PHI(Iter,I_constr_red));NaN(1,length(I_constr_red))]);
              xlabel('Iterationen'); grid on;
              ylabel('diff Phi');
              subplot(3,3,6);
              plot(Stats_PosIK.PHI(Iter,I_constr_red));
              xlabel('Iterationen'); grid on;
              ylabel('Phi');
              subplot(3,3,7);
              plot(diff(Stats_PosIK.h(Iter,[true,s_Pos.wn'~=0])));
              xlabel('Iterationen'); grid on;
              ylabel('diff h');
              subplot(3,3,8);
              plot(Stats_PosIK.h(Iter,[true,s_Pos.wn'~=0]));
              critnames = fields(R.idx_ikpos_hn)';
              legend(['w.sum',critnames(s_Pos.wn'~=0)], 'interpreter', 'none');
              xlabel('Iterationen'); grid on;
              ylabel('h');
              linkxaxes
            end
          end
          if ~task_redundancy % Wieder auf nicht-redundant zurücksetzen
            R.update_EE_FG(R.I_EE, R.I_EE);
          end
          % Prüfe das Ergebnis der Positions-IK.
          if any(abs(Phi_i) > 1e-8)
            if s.verbose
              fprintf('Keine Optimierung mit Positions-IK möglich bei %d/%d\n', i, l-z);
            end
            continue
          end
          if any(isinf(Stats_PosIK.h(1+Stats_PosIK.iter,:)))
            if s.verbose
              critviol = {};
              for f = fields(R.idx_ikpos_hn)'
                if isinf(Stats_PosIK.h(1+Stats_PosIK.iter,1+R.idx_ikpos_hn.(f{1})))
                  critviol = [critviol, f{1}]; %#ok<AGROW> 
                end
              end
              fprintf(['Nebenbedingung (%s) in zusätzlicher Optimierung ', ...
                'überschritten bei %d/%d\n'], disp_array(critviol, '%s'), i, l);
            end
            continue
          end
          if Stats_PosIK.iter == 1 && all(abs(q_i-q0_posik(:,1))<1e-10)
            if s.verbose
              fprintf('Optimierung mit Positions-IK direkt abgebrochen %d/%d\n', i, l-z);
            end
            continue
          end
          x_i_neu = R.fkineEE2_traj(q_i');
          if x_i_neu(end) < xl_posik(end) + s_Pos.xlim(6,1) || ...
              x_i_neu(end) > xl_posik(end) + s_Pos.xlim(6,2)
            if s.verbose
              fprintf(['Optimierung mit Positions-IK führt zu unzulässigem ', ...
                'Wert für Koord. bei %d/%d: Bereich: %1.1f°...%1.1f°. Wert: %1.1f°\n'], ...
                i, l, 180/pi*(xl_posik(end) + s_Pos.xlim(6,1)), ...
                180/pi*(xl_posik(end) + s_Pos.xlim(6,2)), 180/pi*x_i_neu(end));
            end
            continue
          end
          % Nehme mit Positions-IK berechneten neuen Wert als Zielwert
          if s.verbose
            fprintf(['Optimaler Zustand %1.1f° für Stufe %d im Bereich ', ...
              '%1.1f°...%1.1f° mit Positions-IK bestimmt. Ursprünglich: %1.1f°\n'], 180/pi*x_i_neu(end), k, ...
              180/pi*(xl_posik(end) + s_Pos.xlim(6,1)), 180/pi*(xl_posik(end) + s_Pos.xlim(6,2)), 180/pi*xl_posik(end));
          end
          z_l = x_i_neu(end);
        else % if l > z % Überlappend
          z_l = phi_range(l-z)+delta_phi/2;
        end
        assert(~isnan(z_l), 'Logik-Fehler: z_l ist NaN');
      else % Freier Übergang
        z_l = NaN; % Ziel-Zustand ist undefiniert.
      end
      % Trajektorie als Teil der eingegebenen Trajektorie bestimmen.
      % Redundante Koordinate überschreiben. Bildet dann Mitte des
      % Toleranzbandes
      X_t_l = X_t_in(IE(i-1):IE(i),:);
      XD_t_l = XD_t_in(IE(i-1):IE(i),:);
      XDD_t_l = XDD_t_in(IE(i-1):IE(i),:);
      if ~free_stage_transfer
      % Finde den Index bis zum Zeitpunkt, ab dem die Bewegung aufhören
      % soll, damit noch die Verzögerungszeiten eingehalten werden können
      ii1 = floor( interp1(t_i, 1:length(t_i), t_i(end)-s.Tv-s.T_dec_ns) );
      % Prüfe, ob die Geschwindigkeit überhaupt realisierbar ist. Gehe nicht vom
      % günstigsten Fall aus, dass das obere und untere Ende des Toleranz-
      % bandes genutzt werden. Für Geschwindigkeits-Profil wichtig.
      v = (abs(z_l - z_k))/(t_i(end)-t_i(1));
      if v > s.xDlim(6,2)
        if s.verbose
          fprintf(['Transfer %d/%d (%1.2f°) nach %d/%d (%1.2f°) in %1.1fs ', ...
            'erfordert Geschw. %1.1f rad/s größer als Grenze %1.1f rad/s.\n'], ...
            i-1, k, 180/pi*z_k, i, l, 180/pi*z_l, (t_i(end)-t_i(1)), v, s.xDlim(6,2));
        end
        continue
      end
      % Prüfe, ob bei maximalem Bang-Bang-Beschleunigungs-Rechteck die Zeit
      % ausreichen würde. Halbe Zeitspanne +amax, andere hälfte -amax.
      deltaphi_bb = 2 * 0.5*s.xDDlim(6,2)*((t_i(ii1)-t_i(1))/2)^2; % 1/2*a*t²
      if deltaphi_bb < abs(z_l - z_k)
        if s.verbose
          fprintf(['Transfer %d/%d (%1.2f°) nach %d/%d (%1.2f°) in %1.1fs ', ...
            'aufgrund Beschleunigungsgrenze %1.1fs rad/s² nicht möglich. ', ...
            'Erlaubt nur Distanz %1.1f°\n'], i-1, k, 180/pi*z_k, i, l, ...
            180/pi*z_l, (t_i(end)-t_i(1)), s.xDDlim(6,2), deltaphi_bb*180/pi);
        end
        continue
      end
      % Berechne eine Geschwindigkeits-Trapez-Trajektorie zur Annäherung an
      % eine Vorsteuerungs-Trajektorie (für die Mitte des Toleranzbandes).
      % Damit muss die Transition zwischen zwei Winkeln nicht nur über die
      % Nullraumbewegung gemacht werden. Darauf aufbauend sind trotzdem
      % Nullraum-Bewegungen möglich.
      if abs(z_k-z_l) < 1e-12 % bei 3T3R-IK auch exakt gleiche Werte
        % Funktion trapveltraj erzeugt Fehler bei gleichem Start und Ziel
        X_t_l(1:ii1,6) = z_k;
        XD_t_l(1:ii1,6) = 0;
        XDD_t_l(1:ii1,6) = 0;
      else
        [X_t_l(1:ii1,6),XD_t_l(1:ii1,6),XDD_t_l(1:ii1,6),tSamples] = trapveltraj([z_k, z_l],ii1,...
          'EndTime',t_i(ii1)-t_i(1), 'Acceleration', s.xDDlim(6,2)); % 'PeakVelocity', s.xDlim(6,2));
        if ~all(abs(t_i(1)+tSamples(:)-t_i(1:ii1)) < 1e-10) % Teste Ausgabe
          if ~isempty(s.debug_dir)
            save(fullfile(s.debug_dir, sprintf(['dp_stage%d_state%d_', ...
              'to%d_error_trapveltrajsampletime.mat'], i-1, k, l)));
          end
          error('Profilzeiten stimmen nicht');
        end
        % Fehlerkorrektur der Funktion. Manchmal sehr kleine Imaginärteile
        if any(~isreal(X_t_l(:))) || any(~isreal(XD_t_l(:))) || any(~isreal(XDD_t_l(:)))
          X_t_l = real(X_t_l); XD_t_l = real(XD_t_l); XDD_t_l = real(XDD_t_l);
        end
      end
      X_t_l(ii1+1:end,6) = X_t_l(ii1,6); % letzten Wert halten
      XD_t_l(ii1+1:end,6) = 0; % ... beim Halten des Wertes keine Geschw. ... 
      XDD_t_l(ii1+1:end,6) = 0; % ... und Beschleunigung
      assert(~any(isnan(X_t_l(:))), 'X_t_l enthält NaN. Logik-Fehler');
      assert(~any(isnan(XD_t_l(:))), 'XD_t_l enthält NaN. Logik-Fehler');
      assert(~any(isnan(XDD_t_l(:))), 'XDD_t_l enthält NaN. Logik-Fehler');
      % Alternativ (alte Version): Konstante Geschwindigkeit
%       X_t_l(1:ii1,6) = linspace(z_k, z_l, ii1); % size(X_t_l,1)
%       % Steuere die Geschwindigkeit der redundanten Koordinate vor.
%       XD_t_l(1:ii1,6) = (z_l-z_k) / (t_i(ii1)-t_i(1));
      
      if max(abs(XD_t_l(:,6))) > s.xDlim(6,2)
        if s.verbose
          fprintf(['Transfer %d/%d (%1.2f°) nach %d/%d (%1.2f°) in %1.1fs ',...
            '(nach Berücksichtigung der Geschwindigkeits-Trapez-Traj.) ', ...
            'erfordert Geschw. %1.1f rad/s größer als Grenze %1.1f rad/s.\n'], ...
            i-1, k, 180/pi*z_k, i, l, 180/pi*z_l, (t_i(ii1)-t_i(1)), ...
            max(abs(XD_t_l(:,6))), s.xDlim(6,2));
        end
        continue
      end
      % Begrenze die Nullraumgeschwindigkeit am Ende der Bewegung
      nullspace_maxvel_interp = NaN(2,4); % Spalten: Zeitschritte
      nullspace_maxvel_interp(:,1) = [t_i(1); 1]; % Am Anfang volle Geschw.
      % Volle Geschwindigkeit bis zum Anfang der Abbremsphase behalten
      nullspace_maxvel_interp(:,2) = [max(t_i(end)-s.T_dec_ns-s.Tv, ...
        mean(t_i([1 end]))); 1]; % Am Anfang volle Geschw.
      % Auf Null Abbremsen kurz vor Traj.-Ende
      nullspace_maxvel_interp(:,3) = [max(t_i(end)-s.Tv, nullspace_maxvel_interp(1,2)+eps); 0];
      % Bei Null bleiben bis zum Ende
      nullspace_maxvel_interp(:,4) = [t_i(end); 0];
      s_Traj.nullspace_maxvel_interp = nullspace_maxvel_interp;
      % Grenzen für die redundante Koordinate erst schmal, dann weit und
      % dann wieder eng
      xlim6_interp = NaN(3,5); % Spalten: Zeitschritte
      % Schmale Grenzen (Breite wie Abstand zwischen DO-Zuständen)
      xlim6_interp(:,1) = [t_i(1);-delta_phi/2; delta_phi/2];
      % Direkt am Anfang zwei Punkte einfügen, damit Ableitung des Spline
      % am Anfang Null ist. Sonst zu viel oszillieren.
      xlim6_interp(:,2) = [t_i(1)+1e-6;-delta_phi/2; delta_phi/2];
      % Aufweiten der Grenzen: drei mal so breit wie am Anfang.
      xlim6_interp(:,3) = [mean([t_i(1),nullspace_maxvel_interp(1,2)]); ...
        xlim6_interp(2:3,1)*3];
      % Toleranzband darf sich nicht mehr ändern, während die
      % Nullraumgeschwindigkeit auf Null gebremst wird
      xlim6_interp(:,4) = [nullspace_maxvel_interp(1,2);-delta_phi/2 + 1e-3; delta_phi/2 - 1e-3];
      % Danach Grenzen konstant lassen. Die Aufgabenbewegung darf nicht
      % dazu führen, dass die Trajektorie wieder das Band verlässt.
      % Die Bewegung die in der phiz-Variable der Trajektorie ist, muss
      % ausgeglichen werden (siehe Debug-Plot)
      xlim6_interp(:,5) = [t_i(end); xlim6_interp(2:3,4)]; % keine Modifikation hier wenn Traj. oben korrigiert
      % Debug: Plotten des Bereichs
%       figure(99);clf;hold on;
%       plot(t_i, 180/pi*X_t_l(:,6));
%       plot(t_i, 180/pi*(X_t_l(:,6)+interp1(xlim6_interp(1,:), xlim6_interp(2,:), t_i, 'spline')));
%       plot(t_i, 180/pi*(X_t_l(:,6)+interp1(xlim6_interp(1,:), xlim6_interp(3,:), t_i, 'spline')));
%       plot(t_i, 180/pi*(X_t_l(:,6)+interp1(xlim6_interp(1,:), xlim6_interp(2,:), t_i)));
%       plot(t_i, 180/pi*(X_t_l(:,6)+interp1(xlim6_interp(1,:), xlim6_interp(3,:), t_i)));
%       grid on;
      s_Traj.xlim6_interp = xlim6_interp;
      end % ~free_stage_transfer
      s_Traj_l = s_Traj;
      if free_stage_transfer
        % Setze Einstellungen wieder zurück, die eine freie Bewegung verhindern
        s_Traj_l.wn(R.idx_iktraj_wnP.xlim_hyp) = 0;
        s_Traj_l.wn(R.idx_iktraj_wnD.xlim_hyp) = 0;
        s_Traj_l.wn(R.idx_iktraj_wnP.xlim_par) = 0;
        s_Traj_l.wn(R.idx_iktraj_wnD.xlim_par) = 0;
        s_Traj_l.wn(R.idx_iktraj_wnP.xDlim_par) = 0;
        s_Traj_l.enforce_xlim = false;
        s_Traj_l.enforce_xDlim = false;
        s_Traj_l.abort_thresh_h = s.abort_thresh_h;
        s_Traj_l.abort_thresh_h(R.idx_iktraj_hn.xlim_hyp) = NaN;
        s_Traj_l.ik_solution_min_norm = true;
      end
      % Trajektorie berechnen
      t0_traj = tic();
      calc_traj = true;
      if s.continue_saved_state
        % Prüfe, ob ein gespeicherter Zustand geladen werden kann
        resfile_l = fullfile(s.debug_dir, sprintf(['dp_stage%d_state%d_', ...
          'to%d_result.mat'], i-1, k, l));
        if exist(resfile_l, 'file')
          d = load(resfile_l);
          if ~isfield(d, 'z_l') || ... % Altes Speicherformat. Daten fehlen.
              size(d.Q,1) ~= length(t_i) || ... % Länge ist falsch.
              abs(d.X6_traj(1) - z_k) > 1e-6 || ... % Anfangswert ist anders. Passt nicht.
              any(abs(d.Q(1,:) - qs')>1e-8) || ... % Startkonfiguration stimmt nicht überein
              ~isnan(z_l) && d.z_l ~= z_l
            if s.verbose && calc_traj
              fprintf(['Geladene Daten passen nicht zu der Trajektorie. ', ...
                'Kein weiteres Laden von Ergebnissen versuchen.\n']);
            end
            s.continue_saved_state = false;
          else
            Q = d.Q;
            QD = d.QD;
            QDD = d.QDD;
            Stats = d.Stats;
            calc_traj = false;
          end
        else % ~exist(resfile_l, 'file')
          if s.verbose
            fprintf(['Datei für Übergang %d/%d-%d/%d nicht vorhanden. ', ...
              'Kein Laden weiterer Ergebnisse versuchen.\n'], i-1, k, i, l);
          end
          s.continue_saved_state = false;
        end
        if s.verbose && ~calc_traj
          fprintf(['Zustand %d/%d-%d/%d aus Datei geladen. Überspringe ', ...
            'Berechnung der Trajektorie\n'], i-1, k, i, l);
        end
      end
      t4 = tic();
      if calc_traj
        if R.Type == 0 % Seriell
          [Q, QD, QDD, ~, ~, Stats] = R.invkin2_traj(X_t_l, XD_t_l, XDD_t_l, t_i, qs, s_Traj_l);
        else % PKM
          [Q, QD, QDD, ~, ~, ~, ~, Stats] = R.invkin2_traj(X_t_l, XD_t_l, XDD_t_l, t_i, qs, s_Traj_l);
        end
        if s.verbose
          fprintf('Traj. für %d/%d Bahnpunkte. Dauer: %1.1fs. Pro Bahnpunkt: %1.1fms\n', ...
            Stats.iter, length(t_i), toc(t0_traj), 1e3*toc(t0_traj)/Stats.iter);
        end
        if isinf(s_Traj_l.abort_thresh_h(R.idx_iktraj_hn.poserr_ee)) && ... % s.debug && 
            any(Stats.h(1:Stats.iter,1+R.idx_iktraj_hn.poserr_ee)==0) % Zum Debuggen bei Unstimmigkeiten
          error('Positionsfehler soll berechnet werden, ist aber Null. Darf nicht sein.')
        end
      end
      assert(all(~isnan(Stats.h(1:Stats.iter,1))), 'Nebenbedingung NaN. Logik-Fehler');
      % Anzahl der insgesamt durchführten IK-Schritte speichern
      nt_ik = nt_ik + Stats.iter;
      Stats_nt_ik_all(i,l) = Stats_nt_ik_all(i,l) + Stats.iter;
      Stats_comptime_ik_all(i,l) = Stats_comptime_ik_all(i,l) + toc(t4);
      if any(Stats.errorcode == [0 1 2])
        % Entweder kein Abbruch der IK (Code 0), oder Abbruch der IK
        % aufgrund unmöglicher Berechnung von q/qD/qDD (Code 1/2).
        % Benutze den letzten (funktionierenden) Wert
        iter_l = Stats.iter; % Index auf die Trajektorie bis Ende
      else
        % Abbruch aufgrund der Verletzung einer Nebenbedingung. Nehme den
        % Wert, der zur Verletzung geführt hat für den Plot.
        iter_l = min(size(Q,1), Stats.iter+1);
      end
      X_l = R.fkineEE2_traj(Q(1:iter_l,:));
      % Winkel >180° in Trajektorie zulassen
      X_l(1,6) = z_k; % Zuerst Start-Winkel setzen als Referenz für De-Norm.
      X_l(:,4:6) = denormalize_angle_traj(X_l(:,4:6));
      % Setze den ersten Wert nochmal auf den Anfangswert. Sonst ist ein
      % Startwert >180° nicht möglich, der aber vorgegeben werden kann.
      assert(angleDiff(z_k, X_l(1,6)) < 1e-6, 'Startwert inkonsistent');
      if s.debug && ~task_redundancy
        test_X = X_l - X_t_l(1:iter_l,:);
        assert(all(abs(test_X(:)) < 1e-6), sprintf( ...
          'X-Traj. stimmt nicht. Fehler max %1.1e', max(abs(test_X(:)))));
      end
      % Prüfe freie Bewegung anhand der Werte für xlim_hyp
      if s.use_free_stage_transfer && ~unconstrained_transfer_done_k && ...
          all(Stats.h(1:iter_l,1+R.idx_iktraj_hn.xlim_hyp)==0)
        % Alle Werte für xlim_hyp sind Null. Das Kriterium war also nie
        % aktiv und die Bewegung war bereits ohne Einschränkungen, frei
        unconstrained_transfer_done_k = true;
      end
      
      % Prüfe, ob die Gelenk-Konfigurationen noch denen aus der
      % Redundanzkarte entsprechen
      if s.debug && ~isempty(s.PM_Q_all)
        [dbg_delta_t1, I_tstart] = min(abs(t_tref-t_i(1)));
        [dbg_delta_phi1, I_phistart] = min(abs(s.PM_phiz_range-z_k));
        q_start_PM = s.PM_Q_all(I_phistart,:,I_tstart);
        dbg_delta_q1 = Q(1,:) - q_start_PM;
        fprintf(['Vergleich Startpunkt (phiz=%1.1f°) mit Werten aus ', ...
          'Diskretisierung (%1.1f°): Delta phi %1.1g°. Max delta q %1.3f. delta t %1.1gs.\n'], ...
          180/pi*z_k, 180/pi*s.PM_phiz_range(I_phistart), 180/pi*dbg_delta_phi1, ...
          max(abs(dbg_delta_q1)), dbg_delta_t1);
        [dbg_delta_t2, I_tend] = min(abs(t_tref-t_i(iter_l)));
        [dbg_delta_phi2, I_phiend] = min(abs(s.PM_phiz_range-X_l(iter_l,6)));
        q_end_PM = s.PM_Q_all(I_phiend,:,I_tend)';
        dbg_delta_q2 = Q(iter_l,:)' - q_end_PM;
        fprintf(['Vergleich Endpunkt (phiz=%1.1f°; Soll %1.1f°) mit Werten aus ', ...
          'Diskretisierung (%1.1f°): Delta phi %1.1g°. Max delta q %1.3f. delta t %1.1gs.\n'], ...
          180/pi*X_l(iter_l,6), 180/pi*z_l, 180/pi*s.PM_phiz_range(I_phiend), ...
          180/pi*dbg_delta_phi2, max(abs(dbg_delta_q2)), dbg_delta_t2);
        if any(abs([dbg_delta_phi1; dbg_delta_phi2]) > 10*pi/180)
          % Zeichne den Roboter in beiden Fällen
          change_current_figure(800);clf;
          for iii = 1:2
            if iii == 1
              q_iii = q_end_PM;
              x_iii = R.fkineEE_traj(q_end_PM')';
              assert(abs(x_iii(6)-s.PM_phiz_range(I_phiend)) < 1e-6, ...
                'Logik-Fehler bei Auswahl der EE-Koordinaten aus Redundanzkarte');
            else
              q_iii = Q(iter_l,:)';
              x_iii = X_l(iter_l,:)';
            end
            subplot(1,2,iii); hold on; grid on;
            title(sprintf('Pose %d', iii));
            xlabel('x in m');ylabel('y in m');zlabel('z in m');
            s_plot = struct(  'ks_legs', [], 'straight', 0, 'mode', 4);
            R.plot( q_iii, x_iii, s_plot );
          end
          fprintf('Fehler ist sehr groß. Vermutlich inkonsistente Daten\n');
        end
      end
      % Berechne Verletzung der Geschw.- oder Beschleunigungs-Grenzen.
      % Positionsgrenzen werden über eigene Zielkriterien beachtet.
      limviol_qD = false;
      if s.constraint_qDlim
        if any(max(abs(QD))' > qDlim(:,2))
          limviol_qD = true;
        end
      end
      limviol_qDD = false;
      if s.constraint_qDDlim
        if any(max(abs(QDD))' > qDDlim(:,2))
          limviol_qDD = true;
        end
      end
      if Stats.iter < length(t_i) % Trajektorie nicht erfolgreich
        % Setze Strafterm unendlich
        F_stage(k,l) = inf;
      elseif isinf(Stats.h(iter_l,1+R.idx_iktraj_hn.xlim_hyp))
        % Wert für die redundante Koordinate ist außerhalb des Zielbereichs
        % Kann gelegentlich passieren, wenn enforce_xlim nicht richtig funktioniert.
        F_stage(k,l) = inf;
        Stats.errorcode = 3; % entspricht dem Wert aus der Funktion
      elseif limviol_qD
        F_stage(k,l) = inf;
        Stats.errorcode = 4; % zur korrekten Verarbeitung weiter unten
      elseif limviol_qDD
        F_stage(k,l) = inf;
        Stats.errorcode = 5; % zur korrekten Verarbeitung weiter unten
      else % Erfolgreich berechnet
        % Kostenfunktion für DP aus IK-Zielfunktion bestimmen:
        % Bedingung: Kriterium ist addierbar über die gestückelte Trajektorie
        % Es wird nicht die Summe `Stats.h(:,1)` aus der
        % Trajektorien-Funktion benutzt, da diese auch die DP-Kriterien 
        % xlim_hyp und xlim_par enthält. Diese sind später bedeutungslos.
        hsum = sum(repmat(wn_hcost, Stats.iter, 1) .* Stats.h(:,2:end), 2);
        if strcmp(s.cost_mode, 'average')
          F_stage(k,l) = mean(hsum);
        elseif strcmp(s.cost_mode, 'RMStime') % Integration über Zeit
          F_stage(k,l) =  sqrt(trapz(t_i, hsum.^2)/t_i(end));
        elseif strcmp(s.cost_mode, 'RMStraj') % Integration über Bahnkoordinate
          F_stage(k,l) =  sqrt(trapz(s_i, hsum.^2)/s_i(end));
        else
         F_stage(k,l) = max(hsum);
        end
        % Mittlere Konditionszahl als weiteres Entscheidungskriterium
        F_stage_cond(k,l) = mean(Stats.condJ(:,2)); % Aktuierungs-J. / geom. J.
        % Bewegung der redundanten Koordinate als weiteres Kriterium
        F_stage_range(k,l) = sum(abs(diff(X_l(:,6))));
        % Statistik: Anzahl der erfolgreichen Übergänge speichern
        n_statechange_succ = n_statechange_succ + 1;
        Stats_statechange_succ_all(i,l) = Stats_statechange_succ_all(i,l)+1;
      end
      %% Debugge Ergebnis der Berechnung. Vergleich gegen Redundanzkarte.
      % Kann inkonsistentes Verhalten aufdecken
      if s.debug && ~isempty(s.PM_H_all)
        % Interpoliere die Trajektorie über die Redundanzkarte. Ergebnis
        % sollte mit den tatsächlichen Leistungsmerkmalen übereinstimmen
        % Kriterium wählen. TODO: Mehr Logik für Wahl des Kriteriums
        Icrit1 = find(strcmp(fields(R.idx_ikpos_hn), 'jac_cond'));
        Icrit2 = find(strcmp(fields(R.idx_iktraj_hn), 'jac_cond'));
        Hcrit = s.PM_H_all(:,:,Icrit1)';
        [t_tref_unique, I_t_unique] = unique(t_tref); % Sonst Fehler bei interp2
        h_interp = interp2(t_tref_unique, s.PM_phiz_range, ...
          Hcrit(:,I_t_unique), t_i(1:iter_l,:), X_l(:,6));
        h_traj = Stats.h(1:iter_l,1+Icrit2);
        % Korrelation berechnen auf Basis der Werte fast bis zum Schluss.
        % Am Ende nicht aussagekräftig durch Abbruch bei sehr großen Werten
        iter_mc = max(1, iter_l-5);
        corr_PM_traj = corr(h_interp(1:iter_mc), h_traj(1:iter_mc));
        if corr_PM_traj < 0.8
          fprintf(['Zielkriterium %s stimmt nicht zwischen Trajektorie ', ...
            'und Redundanzkarte überein. Korrelation %1.3f\n'], 'jac_cond', corr_PM_traj)
          if s.verbose > 2
            change_current_figure(400); clf; hold on;
            plot(t_i(1:iter_l,:), [h_traj, h_interp]);
            grid on;
            xlabel('t in s'); ylabel('h cond');
            legend({'traj', 'perfmap interp'});
            title(sprintf('%d/%d -> %d/%d; corr(%s)=%1.3f', i-1, k, i, l, ...
              'jac_cond', corr_PM_traj), 'interpreter', 'none');
          end
        end
      end
      % Prüfe Konsistenz von zurückgegebenen Daten
      if s.debug
        % Position neu mit Trapezregel berechnen (Integration)
        Q_num = repmat(Q(1,:),iter_l,1)+cumtrapz(t_i(1:iter_l), QD(1:iter_l,:));
        % das gleiche für die Geschwindigkeit
        QD_num = cumtrapz(t_i(1:iter_l), QDD(1:iter_l,:));
        QDD_num = zeros(size(Q_num));
        QDD_num(2:iter_l,:) = diff(QD(1:iter_l,:))./...
          repmat(diff(t_i(1:iter_l)), 1, size(Q,2)); % Differenzenquotient
        
        % Bestimme Korrelation zwischen den Verläufen (1 ist identisch)
        corrQD = diag(corr(QD_num, QD(1:iter_l,:)));
        corrQ = diag(corr(Q_num, Q(1:iter_l,:)));
        if any(corrQD < 0.95) || any(corrQ < 0.98)
          fprintf(['Keine konsistenten Gelenkverläufe. Korrelation q ', ...
            '%1.2f...%1.2f, qD %1.2f...%1.2f\n'], min(corrQ), max(corrQ), ...
            min(corrQD), max(corrQD));
          if s.verbose > 2 && false % nur manuell untersuchen
            change_current_figure(56);clf;
            for kkk = 1:R.NJ
              subplot(ceil(sqrt(R.NJ)), ceil(sqrt(R.NJ)), kkk); hold on;
              plot(t_i(1:iter_l), [QD(1:iter_l,kkk), QD_num(:,kkk)]);
              ylabel(sprintf('qD %d', kkk)); grid on;
            end
            legend({'traj.', 'int.'});
            linkxaxes
            change_current_figure(57);clf;
            for kkk = 1:R.NJ
              subplot(ceil(sqrt(R.NJ)), ceil(sqrt(R.NJ)), kkk); hold on;
              plot(t_i(1:iter_l), [QDD(1:iter_l,kkk), QDD_num(:,kkk)]);
              ylabel(sprintf('qDD %d', kkk)); grid on;
            end
            legend({'traj.', 'int.'});
            linkxaxes
          end
        end
      end
      %% Debugge einzelne Trajektorie
      if isinf(F_stage(k,l)) && s.verbose > 2
        [~, XD_l, XDD_l] = R.fkineEE2_traj(Q(1:iter_l,:), ...
          QD(1:iter_l,:), QDD(1:iter_l,:));
        fighdl_trajdbg = change_current_figure(999);clf;
        subplot(3,3,1);hold on;
        plot(t_i(1:iter_l,:), 180/pi*X_l(:,6));
        if ~free_stage_transfer
        plot(t_i, 180/pi*X_t_l(:,6));
        plot(t_i, 180/pi*(X_t_l(:,6)+interp1(xlim6_interp(1,:), xlim6_interp(2,:), t_i, 'spline')), '-^');
        plot(t_i, 180/pi*(X_t_l(:,6)+interp1(xlim6_interp(1,:), xlim6_interp(3,:), t_i, 'spline')), '-v');
        legend({'ist', 'Ref', 'Unten', 'Oben'});
        end
        grid on
        ylabel('phiz');
        subplot(3,3,2);hold on;
        plot(t_i(1:iter_l,:), XD_l(:,6));
        if ~free_stage_transfer
        plot(t_i, XD_t_l(:,6));
        plot(t_i([1;end]), s.xDlim(6,1)*[1;1], 'r-^');
        plot(t_i([1;end]), s.xDlim(6,2)*[1;1], 'r-v');
        plot(t_i, interp1(nullspace_maxvel_interp(1,:),s.xDlim(6,2)*nullspace_maxvel_interp(2,:), t_i));
        legend({'ist', 'Ref', 'Unten', 'Oben', 'Max Vel'});
        end
        ylabel('phizD'); grid on;
        subplot(3,3,3);hold on;
        plot(t_i(1:iter_l,:), XDD_l(:,6));
        if ~free_stage_transfer
        plot(t_i, XDD_t_l(:,6));
        plot(t_i([1;end]), s.xDDlim(6,1)*[1;1], 'r-^');
        plot(t_i([1;end]), s.xDDlim(6,2)*[1;1], 'r-v');
        legend({'ist', 'Ref', 'Unten', 'Oben'});
        end
        ylabel('phizDD'); grid on;
        Q_norm = (Q - repmat(qlim(:,1)', size(Q,1), 1)) ./ ...
                  repmat(qlim(:,2)'-qlim(:,1)', size(Q,1), 1);
        QD_norm = (QD - repmat(qDlim(:,1)', size(QD,1), 1)) ./ ...
                  repmat(qDlim(:,2)'-qDlim(:,1)', size(QD,1), 1);
        QDD_norm = (QDD - repmat(qDDlim(:,1)', size(QDD,1), 1)) ./ ...
                  repmat(qDDlim(:,2)'-qDDlim(:,1)', size(QDD,1), 1);
        subplot(3,3,4);hold on;
        plot(t_i, Q_norm);
        ylabel('q (norm.)'); grid on;
        subplot(3,3,5);hold on;
        plot(t_i, QD_norm);
        ylabel('qD (norm.)'); grid on;
        subplot(3,3,6);hold on;
        plot(t_i, QDD_norm);
        ylabel('qDD (norm.)'); grid on;
        subplot(3,3,7);hold on;
        plot(t_i, Stats.condJ);
        legend({'IK Jac.', 'Act. Jac.'});
        ylabel('cond(J)'); grid on;
        for kkk = 0:1
        subplot(3,3,8+kkk);hold on;
        plot(t_i, Stats.h(:,1));
        % Plotte die Optimierungskriterien, die aktiv sind
        leg_dbg = {'sum'};
        for f = fields(R.idx_iktraj_hn)'
          if s_Traj.wn(R.idx_iktraj_wnP.(f{1})) ~= 0 && ...
              any(Stats.h(:,1+R.idx_iktraj_hn.(f{1}))~=0) % nur aktive Kriterien
            plot(t_i, Stats.h(:,1+R.idx_iktraj_hn.(f{1})));
            leg_dbg = [leg_dbg, f{1}]; %#ok<AGROW>
          end
        end
        legend(leg_dbg, 'interpreter', 'none');
        ylabel('h'); grid on;
        if kkk == 1
          set(gca, 'yscale', 'log'); ylabel('log(h)');ylim([1,inf]);
        end
        end
        linkxaxes
        if ~free_stage_transfer
          sgtitle(fighdl_trajdbg, sprintf(['Stufe %d->%d. Transition %d->%d (%1.2f° -> ', ...
            '%1.2f°; bzw. %1.2f°)'], i-1, i, k, l, 180/pi*X_t_l(1,6), ...
            180/pi*z_l, 180/pi*X_l(end,6)));
        else
          sgtitle(fighdl_trajdbg, sprintf(['Stufe %d->%d. Transition %d->%d (%1.2f° -> ', ...
            '%1.2f°; freie Bewegung)'], i-1, i, k, l, 180/pi*X_l(1,6), ...
            180/pi*X_l(end,6)));
        end
        set(fighdl_trajdbg, 'NumberTitle', 'off', 'Name', sprintf(...
          'i%d_k%d_l%d_TrajDbg', i, k, l));
      end
      % Speichere Ergebnis bei Erfolg der Trajektorie ab. Sonst NaN lassen.
      if ~isinf(F_stage(k,l)) % || any(Stats.errorcode==[3 4 5]) % evtl. doch speichern, falls Nutzung geplant
        QE_stage(:, k, l) = Q(end,:)'; % Gelenkkonfiguration (Vermeidung von Umklappen)
        YE_stage(k, l) = X_l(end,6); % EE-Drehung / kontinuierliche Optimierungsvariable
      end
      if ~isempty(s.debug_dir)
        % Speichere detaillierten Zwischen-Zustand ab
        X6_traj = X_l(:,6);
        X6_traj_ref = X_t_l(:,6);
        save(fullfile(s.debug_dir, sprintf('dp_stage%d_state%d_to%d_result.mat', ...
          i-1, k, l)), 'Q', 'QD', 'QDD', 'Stats', 'X6_traj', 'i', 'k', 'l', ...
          's_Traj_l', 'z_l', 'X6_traj_ref', 'xlim6_interp');
      end
      % Eintragen der Linie in die Redundanzkarte
      if s.verbose > 1 && iter_l > 0 % Debug-Plot
        change_current_figure(figikhdl);
        linestyle = '-';
        if l > z % Kennzeichnung von stage optimization oder overlap  
          linestyle = '--';
        end
        if free_stage_transfer % Besondere Kennzeichnung bei freier Bewegung
          linestyle = ':';
        end
        if isinf(F_stage(k,l))
          linecolor = 'k'; % fehlgeschlagen: Schwarz
        else
          linecolor = 'm'; % Erfolgreich: Magenta. Kann später in cyan umgewandelt werden, wenn optimal für Ziel-Zustand
        end
        if l == 1 && k == 1 % Initialisierung
          DbgLineHdl(:) = NaN;
        end
        DbgLineHdl(k,l) = plot(t_i(1:iter_l), 180/pi*X_l(:,6), [linecolor,linestyle], 'linewidth', 2);
        % Zeichne Verbindung zur Ideallinie ein. Macht das Bild unüber-
        % sichtlich, daher immer nur eine Linie einzeichnen und die alte löschen
        if task_redundancy % Nur im Fall von Redundanz sinnvoll
        if exist('DbgLine2Hdl', 'var'), delete(DbgLine2Hdl); end
        if ~free_stage_transfer % Nur einzeichnen, falls Begrenzung gegeben
        DbgLine2Hdl(1) = plot(t_i, 180/pi*X_t_l(:,6), 'g-');
        I_quiver = round(linspace(1, iter_l, 7));
        DbgLine2Hdl(2) = quiver(t_i(I_quiver), 180/pi*X_l(I_quiver,6), ...
          zeros(length(I_quiver), 1), 180/pi*(X_t_l(I_quiver,6)-X_l(I_quiver,6)), 'off', 'b-');
        set(DbgLine2Hdl(2), 'MaxHeadSize', 0.03); % Sonst Pfeil zu breit
        DbgLine2Hdl(3) = plot(t_i, 180/pi*(X_t_l(:,6)+...
          interp1(xlim6_interp(1,:), xlim6_interp(2,:), t_i, 'spline')), 'k-');
        DbgLine2Hdl(4) = plot(t_i, 180/pi*(X_t_l(:,6)+...
          interp1(xlim6_interp(1,:), xlim6_interp(3,:), t_i, 'spline')), 'k-');
        end % if ~free_stage_transfer
        end % if task_redundancy
        % Zeichne Referenz ein, falls eine Nachoptimierung mit Positions-IK
        if s.stageopt_posik && l > z
        if exist('DbgLine2Hdl', 'var'), delete(DbgLine2Hdl); end
        DbgLine2Hdl = quiver(t_i(iter_l), 180/pi*YE_stage(k,l-z), 0, ...
          180/pi*(X_l(end,6) - YE_stage(k,l-z)), 'k-');
        end % s.stageopt_posik
        if isinf(F_stage(k,l)) % nur im Fall des Scheiterns der Trajektorie
          % "Erkunde" zusätzlich die Zielfunktion und zeichne die Marker für
          % die Verletzung ein. Dann ist der Abbruchgrund der Trajektorie
          % direkt erkennbar. Es muss nur der letzte Bahnpunkt geprüft werden
          for kk = 1:7 % über mögliche Fälle für den Abbruchgrund
            switch kk % siehe gleiche Schleife oben
              case 1, hname = 'qlim_hyp';
              case 2, hname = 'jac_cond';
              case 3, hname = 'ikjac_cond';
              case 4, hname = 'coll_hyp';
              case 5, hname = 'instspc_hyp';
              case 6, hname = ''; % IK ungültig / nicht lösbar (Reichweite)
              case 7 % Zum Abfangen sonstiger Abbruchgründe 
            end
            if any(Stats.errorcode == [1 2]) % IK-Fehler
              % Steht im Zusammenhang zu einer IK-Singularität, aber
              % anderer Marker
              plot(t_i(iter_l), 180/pi*X_l(iter_l,6), 'gh');
              break;
            elseif any(Stats.errorcode == [4 5]) % Geschw.-Beschl.-Verletzung
              plot(t_i(iter_l), 180/pi*X_l(iter_l,6), 'kx');
              break;
            elseif kk < 6 && Stats.errorcode == 3 && ... % Abbruchgrund muss die Überschreitung sein
                Stats.h(iter_l,1+R.idx_iktraj_hn.(hname)) >= ...
                s_Traj.abort_thresh_h(R.idx_iktraj_hn.(hname))
              % Kriterium kk wurde überschritten. Weiter zum Plot.
            elseif kk < 6 && Stats.errorcode == 3 && ... % Abbruchgrund muss die Überschreitung sein
                ( Stats.h(iter_l,1+R.idx_iktraj_hn.(hname)) < ...
                s_Traj.abort_thresh_h(R.idx_iktraj_hn.(hname)) || ...
                isnan(s_Traj.abort_thresh_h(R.idx_iktraj_hn.(hname))) )
              % Kriterium kk war nicht dasjenige, das den Abbruch erzeugte.
              % Entweder es wurde unterschritten, oder gar nicht gesetzt.
              continue
            elseif isinf(Stats.h(iter_l,1+R.idx_iktraj_hn.xlim_hyp))
              % Toleranzband im letzten Zeitschritt wurde verlassen. Kein
              % Abbruchgrund für Trajektorie (da abort_thresh auf xlim_hyp
              % deaktiviert). Nachträgliche Erkennung.
              % Zeichne einen anderen Marker zur Kennzeichnung
              % Verhältnis Ist-/Soll-Koordinate zum Zeitpunkt des Abbruchs:
              if X_l(iter_l,6) > X_t_l(iter_l,6)
                marker = 'k^'; % Verletzung nach oben
              else
                marker = 'kv'; % nach unten
              end
              plot(t_i(iter_l), 180/pi*X_l(iter_l,6), marker);
              break;
            elseif kk == 6 && ~isnan(Stats.h(iter_l,1))
              continue
            else % Keiner der vorherigen Fälle hat gepasst. Unklar, was passiert ist
              if ~isempty(s.debug_dir)
                save(fullfile(s.debug_dir, sprintf(['dp_stage%d_state%d_', ...
                  'to%d_error_abortreason.mat'], i-1, k, l)));
              end
              error('Abbruchgrund konnte nicht gefunden werden. Logik-Fehler.');
            end
            % Trage kritischen Bahnpunkt mit aktuellem Marker in Bild ein.
            % `1+iter`, da Ausgabe `iter` der letzte vollständige war.
            PM_hdl(5+kk) = plot(t_i(iter_l), 180/pi*X_l(iter_l,6), ...
              PM_formats{kk}, 'MarkerSize', 6); % Größerer Marker als oben, damit Herkunft erkennbar
            break; % es kann nur einen Abbruchgrund geben
          end % for kk (Fälle für Plot)
        end % if isinf...
        drawnow(); % Sonst wird das Bild nicht aktualisiert
        %% Debug-Bild für Redundanzkarte speziell für xlim_hyp und xlim_par
        if s.debug && ~isempty(s.PM_H_all) && false % manuell untersuchen
          s.PM_H_all(:,:,R.idx_ikpos_hn.xlim_hyp) = inf;
          s.PM_H_all(:,:,R.idx_ikpos_hn.xlim_par) = inf;
          % Erzeuge neue Werte für die Zielkriterien
          for ii1 = 1:size(s.PM_H_all,1) % Zeit-Stützstellen
            for ii2 = 1:size(s.PM_H_all,2) % Winkel-Stützstellen
              % Trage Kriterien direkt in vorhandene Variable ein
              % Index auf Stützstellen aus Teil der Trajektorie
              s_ref_ii1 = s.PM_s_ref(ii1);
              [~, iii1] = min(abs(s.PM_s_tref - s_ref_ii1));
              if t(iii1) > t_i(end)
                continue % Außerhalb des Bereichs für diesen Teil der Traj.
              end
              [~,I_t_ii1] = min(abs(t(iii1)-t_i));
              % Wert für redundante Koordinate in Karte
              phi_iii = s.PM_phiz_range(ii2);
              % Wert für Zielkriterien bestimmen
              delta_phi_z = X_t_l(I_t_ii1,6) - phi_iii; % Soll - Ist
              phi_lim_ii1 = [... % Grenzen bestimmen
                interp1(s_Traj.xlim6_interp(1,:), s_Traj.xlim6_interp(2,:), t_i(I_t_ii1), 'spline'), ...
                interp1(s_Traj.xlim6_interp(1,:), s_Traj.xlim6_interp(3,:), t_i(I_t_ii1), 'spline')];
              % Aktivierungsschwellwert so wie in Funktionbestimmen
              delta_phiz_thr = 1.2 * 0.5 * s_Traj.xDlim(6,2)^2/s_Traj.xDDlim(6,2);
              philim_thr = phi_lim_ii1 + [+1, -1]*delta_phiz_thr;
              if philim_thr(1) > philim_thr(2), philim_thr(:) = 0; end
              % Kriterien berechnen
              s.PM_H_all(ii1,ii2,R.idx_ikpos_hn.xlim_hyp) = ...
                invkin_optimcrit_limits2(-delta_phi_z, phi_lim_ii1, philim_thr);
              s.PM_H_all(ii1,ii2,R.idx_ikpos_hn.xlim_par) = ...
                invkin_optimcrit_limits1(-delta_phi_z, phi_lim_ii1);
            end
          end
          wn_plot = zeros(R.idx_ik_length.hnpos, 1);
          for f = fields(R.idx_ikpos_hn)'
            if isfield(R.idx_iktraj_wnP, f{1})
              wn_plot(R.idx_ikpos_hn.(f{1})) = s_Traj.wn(R.idx_iktraj_wnP.(f{1}));
            end
          end
          wn_plot(R.idx_ikpos_hn.xlim_hyp) = 100;
          pmtrajdbghdl = change_current_figure(501);clf;hold on;
          Hdl_all = R.perfmap_plot(s.PM_H_all, s.PM_phiz_range, ...
            t_tref, struct( ...
            'reference', 'time', 'wn', wn_plot, 'abort_thresh_h', abort_thresh_hpos, ...
            'PM_limit', false, 'log', true));
          sgtitle(pmtrajdbghdl, sprintf('Redundanzkarte für Stufe %d Aktion %d-%d', i, k, l));
          set(pmtrajdbghdl, 'Name', sprintf('DynProg_PerfMap_Detail'), 'NumberTitle', 'off');
          xlabel('Zeit in s');
          ylabel('Redundante Koordinate in deg');
          ylabel(Hdl_all.cb, 'log10(h)', 'Rotation', 90, 'interpreter', 'none');
          % Linien einzeichnen
          hdl = NaN(4,1);
          hdl(1) = plot(t_i, 180/pi*X_t_l(:,6), 'k-', 'LineWidth', 3);
          hdl(2) = plot(t_i, 180/pi*(X_t_l(:,6)+interp1(xlim6_interp(1,:), ...
            xlim6_interp(2,:), t_i, 'spline')), '-m', 'LineWidth', 2);
          hdl(3) = plot(t_i, 180/pi*(X_t_l(:,6)+interp1(xlim6_interp(1,:), ...
            xlim6_interp(3,:), t_i, 'spline')), '-m', 'LineWidth', 2);
          hdl(4) = plot(t_i(1:iter_l,:), 180/pi*X_l(:,6), 'c-', 'LineWidth', 3);
          legend(hdl, {'Ref', 'Grenze Unten', 'Grenze Oben', 'ist'});
        end
      end % if s.verbose (Debug-Plot)
      if s.verbose
        fprintf('Stufe %d, Start-Orientierung %d: Aktion %d/%d. Dauer: %1.1fs\n', ...
          i, k, l, z, toc(t0_l));
      end
    end % for l (Zustände Stufe i)
    if s.verbose
      fprintf('Stufe %d: Start-Orientierung %d/%d geprüft. Dauer: %1.1fs\n', ...
        i, k, z, toc(t0_k));
    end
  end % for k (Zustände Stufe i-1)
  F_stage_filt = F_stage;
  % Prüfe, ob in den Transfers Zustände innerhalb eines Intervalls mehrfach
  % vorkommen. Damit die Anzahl der Zustände nicht immer weiter steigt,
  % werden diese hier reduziert.
  if s.use_free_stage_transfer || s.overlap || s.stageopt_posik
    % Gehe die möglichen Intervalle durch für den Ziel-Zustand. Von jedem Start-
    % Zustand nur eine Verbindung zu jedem Ziel-Zustand möglich.
    for k = find(~all(isinf(F_stage),2))' % nur Start-Zustände k betrachten, die zu mindestens einem Ziel führen
      for j = 1:length(phi_range) % Gehe das Intervall durch
        I_inrangej = YE_stage(k,:) >  phi_range(j) - delta_phi/2 & ...
                     YE_stage(k,:) <= phi_range(j) + delta_phi/2;
        if sum(I_inrangej) > 1
          % Mehr als ein Wert im Intervall. Behalte nur den besten
          II_inrangej = find(I_inrangej);
          [~, III_best] = min(F_stage(k,II_inrangej));
          I_remove = I_inrangej; % alle entfernen ...
          I_remove(II_inrangej(III_best)) = false; % ... bis auf den besten
          F_stage_filt(k,I_remove) = inf; % Entferne durch setzen auf inf.
          if s.verbose
            fprintf(['Transfer von Stufe %d, Zustand %d nach Stufe %d (Intervall %1.1f°...%1.1f°): %d ', ...
              'doppelt: [%s] (%s). Behalten: %1.1f° (%d)\n'], i-1, k, i, 180/pi*(phi_range(j)-delta_phi/2), ...
              180/pi*(phi_range(j)+delta_phi/2), sum(I_inrangej), ...
              disp_array(180/pi*YE_stage(k,I_inrangej), '%1.1f°'), disp_array(find(I_inrangej), '%d'), ...
              180/pi*YE_stage(k,II_inrangej(III_best)), II_inrangej(III_best));
          end
          continue;
        end
      end % for j
    end % for k
  end % if
  % Füge die Kosten der vorherigen Stufen dazu (kumulierte Kosten)
  F_stage_sum = NaN(size(F_stage_filt));
  F_stage_max = F_stage_sum;
  for k = 1:z1 % vorherige Stufe
    for l = 1:z2 % nächste Stufe
      F_stage_sum(k,l) = F_stage_filt(k,l) + F_all(i-1,k);
      F_stage_max(k,l) = max(F_stage_filt(k,l), F_all(i-1,k));
    end
  end
  % Rekursion über die Betrachtung der kumulierten Kosten:
  % Bestimme optimale Teilpolitik bis hier hin. Dazu müssen die Vorwärts-
  % Rechnungen für alle Zustände der vorherigen Stufe gemacht worden sein.
  for l = 1:z2
    % Bestimme für jede der Zustände l für die nächste Stufe, welcher der
    % beste vorherige Zustand ist, um dort hin zu kommen
    F_best_kl_sum = 0; % Init.
    if all(isinf(F_stage_max(:, l)))
      F_best_kl = inf;
    elseif strcmp(s.cost_mode, 'max') % Max.-Kriterium
      [F_best_kl, k_best] = min(F_stage_max(:, l));
      % Prüfe den Fall, dass das Maximum nicht eindeutig ist. Benutze dann
      % von den gleich guten Ergebnissen den besseren Mittelwert
      I_equal1 = F_stage_max(:, l) == F_best_kl;
      if sum(I_equal1) > 1
        [F_best_kl_sum, ii_best_sum] = min(F_stage_sum(I_equal1, l));
        II_equal = find(I_equal1); % Diejenigen Zustände, die gleich gut sind
        k_best = II_equal(ii_best_sum); % Index des nach Summe besten Zustands
      end
    else % Durchschnitts- oder RMS-Kriterium
      I_equal1 = true(z,1); % Platzhalter
      [F_best_kl_sum, k_best] = min(F_stage_sum(:, l));
      F_best_kl = F_best_kl_sum;
    end
    if isinf(F_best_kl) % Dieser Zustand l ist nicht erreichbar
      % Trage Strafterm ein. Übergang wird damit verworfen
      F_all(i,l) = inf;
    else
      % Prüfe ob der vorher gefundene Zustand eindeutig der beste ist. Hier
      % machen, damit doppelte Prüfung für Max.-Kriterium möglich ist
      % Prüfe, ob die Kosten jetzt eindeutig sind. Falls die Zielfunktionen
      % unterhalb ihrer Schwellwerte (z.B. Konditionszahl) sind, ist
      % konstant h=0 möglich. In dem Fall entscheidet Krit. aus s.cost_mode2
      I_equal2 = (F_stage_sum(:, l) == F_best_kl_sum);
      if sum(I_equal2) > 1
        % Zustände, deren Summen gleich gut sind und falls vorher Maximum-
        % Kriterium angewendet wurde, auch nach diesem optimal sind
        II_equal2 = find(I_equal1 & I_equal2);
        if strcmp(s.cost_mode2, 'motion_redcoord')
          [~, ii_best2] = min(F_stage_range(II_equal2, l));
        elseif strcmp(s.cost_mode2, 'cond') % Jacobi-Kondition
          [~, ii_best2] = min(F_stage_cond(II_equal2, l));
        else
          error('Modus %s für costmode2 nicht definiert', s.cost_mode2);
        end
        k_best = II_equal2(ii_best2); % Index des Zustands mit bester Kondition
      end
      if s.verbose
        fprintf(['Zustand %d (St. %d) bester Vorgänger für Zustand %d ', ...
          '(St. %d). %d Übergänge waren gleich gut, %d in Ordnung.\n'], ...
          k_best, i-1, l, i, sum(I_equal2), sum(~isinf(F_stage_sum(:, l))));
        if s.verbose > 1 && ~isnan(DbgLineHdl(k_best,l))
          % Ändere nochmal die Farbe im Bild
          set(DbgLineHdl(k_best,l), 'Color', 'c'); % cyan für lokal optimal
          ZOrderSet(DbgLineHdl(k_best,l), 1); % Linie ist voll sichtbar
        end
      end
      % Nummer zum besten Zustand der Vor-Stufe
      I_all(i,l) = k_best;
      % Kosten für besten Zustand der Stufe eintragen
      if strcmp(s.cost_mode, 'max')
        F_all(i,l) = max(F_all(i-1,k_best), F_stage_filt(k_best, l));
      else % average, RMStime, RMStraj
        F_all(i,l) = F_all(i-1,k_best) + F_stage_filt(k_best, l);
      end
      % Endwerte für den Roboter-Zustand eintragen. Aufgrund des Prinzips
      % der lokalen Optimierung nicht nur aus dem diskreten Zustand ableit-
      % bar. EE-Rotation ändert sich durch die Trajektorie.
      QE_all(:, l, i) = QE_stage(:, k_best, l); % Gelenkwinkel
      XE_all(i, l) = YE_stage(k_best, l); % EE-Drehung
    end
  end % for l
  % Nochmalige Prüfung auf doppelte Vorkommnisse in Intervall. In jedem
  % Intervall darf nur ein Startwert für die nächste Stufe liegen.
  % Bis hier werden zusätzliche Intervalle noch als eigene Zustände gezählt
  % und nicht mit ursprünglichen Intervallen zusammengelegt (außer für
  % Transfer von dem selben Start-Zustand aus). Betrifft Modus "overlap".
  for jj = 1:length(phi_range)
    I_in_jj = phi_range(jj)-delta_phi/2 < XE_all(i, :) & ...
              phi_range(jj)+delta_phi/2 > XE_all(i, :);
    if sum(I_in_jj) > 1
      % Schlechteren Wert auf unendlich setzen, um ihn zu deaktivieren
      II_in_jj = find(I_in_jj); % Zähl-Indizes der Zustände im Intervall
      [~,II_min] = min(F_all(i,I_in_jj)); % bester Wert dafür
      I_delete_in_jj = I_in_jj; % Binär-Indizes der zu löschenden Zustände
      I_delete_in_jj(II_in_jj(II_min)) = false; % Behalten des besten
      if s.verbose
        fprintf('Mehr als ein Startwert in Intervall %d (%1.1f°...%1.1f°): [%s] (%s)\n', jj, ...
          180/pi*(phi_range(jj)-delta_phi/2), 180/pi*(phi_range(jj)+delta_phi/2), ...
          disp_array(180/pi*XE_all(i, I_in_jj), '%1.1f°'), disp_array(II_in_jj, '%d'));
        fprintf('Behalte Nr. %d (fval=%1.1f). Verwerfe Nr. [%s] mit fval=[%s]\n', ...
          II_in_jj(II_min), F_all(i,II_in_jj(II_min)), disp_array(find(I_delete_in_jj),'%d'), ...
          disp_array(F_all(i,I_delete_in_jj), '%1.1f'));
      end
      F_all(i,I_delete_in_jj) = inf;
    end
  end
  if ~isempty(s.debug_dir)
    % Speichere detaillierten Zwischen-Zustand ab
    save(fullfile(s.debug_dir, sprintf('dp_stage%d_final.mat', i-1)), ...
      'F_stage_sum', 'F_stage_filt', 'F_stage', 'F_stage_cond', 'QE_stage', 'YE_stage');
  end
end % for i (Schleife über Stufen)
if s.verbose
  fprintf('Schleife durchgelaufen. Bis hier %1.1fs.\n', toc(t0));
end

%% Ergebnis nachverarbeiten, gestückelte Trajektorie zusammensetzen.
% Trajektorie neu generieren anstatt für alle Zwischenschritte die Größen
% zu speichern
% Stützstellen des optimalen Verlaufs für die redundante Koordinate
% eintragen
I_best = NaN(size(XE,1), 1);
I_best(1) = 1; % Muss immer über Start-Zustand gehen.
I_validstates = find(~all(isinf(F_all),2))'; % Liste bezogen auf Zustänze der Stufen
for i = fliplr(I_validstates) % Rekursiv von Ende zu Anfang
  if i == I_validstates(end)
    [~, k_best] = min(F_all(i,:));
    % Bei mehreren gleichwertigen Lösungen, nehme die mit dem geringsten
    % Abstand zum Startwert (kann vorkommen, wenn Zielfunktion nicht aktiv
    % ist und immer 0 ist, weil Schwellwerte nicht überschritten werden).
    I_equal3 = find(F_all(i,:) == F_all(i,k_best));
    if length(I_equal3) > 1
      % Ignoriere hier die Option s.cost_mode2, da für Kondition schwer
      % umsetzbar (nur Betrachtung des letzten Zustands reicht eigtl. nicht)
      [~, ii_best2] = min(abs(XE_all(i,I_equal3) - x0(6)));
      k_best = I_equal3(ii_best2);
    end
    XE(i,6) = XE_all(i,k_best);
    k_to_best = I_all(i,k_best);
    I_best(i) = k_best;
  else
    XE(i,6) = XE_all(i,k_to_best);
    % Rekursiv weitergehen, um den Vorgänger zu finden, der als optimale
    % Teilpolitik zu dieser Gesamtpolitik führt.
    I_best(i) = k_to_best;
    k_to_best = I_all(i,k_to_best);
  end
end
if s.verbose
  if any(~isinf(F_all(end,:)))
    fprintf('Bestes Ergebnis bei Zuständen [%s]. Zielfunktion: %1.1f\n', ...
      disp_array(I_best', '%d'), F_all(end,I_best(end)));
  else
    fprintf('Trajektorie nicht erfolgreich. Zustände [%s]\n', ...
      disp_array(I_best(I_validstates)', '%d'));
  end
end
assert(~any(isnan(XE(I_validstates,6))), 'Logik-Fehler. angeblich erfolgreiche Stufen in XE enthalten NaN.');
if ~isempty(s.debug_dir)
  % Speichere detaillierten Zwischen-Zustand ab
  save(fullfile(s.debug_dir, 'dp_final.mat'), 'I_best', 'XE_all', 'QE_all', 'I_all');
end
% if s.debug && ~isempty(s.debug_dir)
%   % Speichere kompletten Zustand ab (nur für Debuggen, Datei zu groß)
%   save(fullfile(s.debug_dir, 'dp_final_all.mat'));
% end
%% Trajektorie aus den optimierten Stützstellen berechnen
% Referenztrajektorie für redundante Koordinate (wie oben). Darum wird noch
% die Nullraumbewegung durchgeführt.
for i = 1:size(XE,1)-1
  % Referenzwert für die redundante Koordinate bestimmen. Genau wie oben,
  % damit das Ergebnis möglichst identisch ist.
  i1 = IE(i); % Start-Index für Stufe i
  i2 = IE(i+1); % End-Index
  % Index für den Zeitschritt, an dem der Wert nur noch gehalten wird
  ii1 = i1-1+floor( interp1(t(i1:i2), 1:(i2-i1+1), t(i2)-s.Tv-s.T_dec_ns) );
  if isnan(I_best(i+1))
    break; % Keine Lösung gefunden.
  end
  % Start- und Ziel-Koordinate für diese Stufe (bezogen auf diskrete Werte)
  % Nehme die gefundenen Stützstellen und nicht die Mittelpunkte der Inter-
  % valle
  phi1 = XE_all(i,I_best(i));
  phi2 = XE_all(i+1,I_best(i+1));
  % Referenzverlauf bestimmen.
  % Alternative 1: Benutze die angepassten Ist-Werte der Optimierungsvariablen.
%   [X_t_in(i1:i2,6),XD_t_in(i1:i2,6),XDD_t_in(i1:i2,6)] = trapveltraj(XE(i:i+1,6)', i2-i1+1,...
%     'EndTime',t(i2)-t(i1), 'Acceleration', s.xDDlim(6,2)); % 'PeakVelocity', s.xDlim(6,2));
  % Alternative 2: Benutze die gleichen Referenzwerte wie oben.
  if abs(phi2-phi1) < 1e-12 % Funktion trapveltraj erzeugt Fehler bei gleichem Start und Ziel
    X_t_in(i1:ii1,6) = phi1;
    XD_t_in(i1:ii1,6) = 0;
    XDD_t_in(i1:ii1,6) = 0;
  else
    [X_t_in(i1:ii1,6),XD_t_in(i1:ii1,6),XDD_t_in(i1:ii1,6)] = trapveltraj([phi1,phi2], ii1-i1+1,...
      'EndTime',t(ii1)-t(i1), 'Acceleration', s.xDDlim(6,2)); % 'PeakVelocity', s.xDlim(6,2));
  end
  X_t_in(ii1+1:i2,6) = X_t_in(ii1,6); % letzten Wert halten
  XD_t_in(ii1+1:i2,6) = 0;
  XDD_t_in(ii1+1:i2,6) = 0;
  
  % Prüfe durch Integration die Konsistenz der Trajektorie
  X6_int_ii = X_t_in(i1,6) + cumtrapz(t(i1:i2)-t(i1), XD_t_in(i1:i2,6));
  test_x6 = X6_int_ii-X_t_in(i1:i2,6);
  if ~all(abs(test_x6) < 1e-2) || any(~isreal(test_x6))
    warning('Fehler bei Bestimmung der Referenztrajektorie für Punkt %d bis %d', i, i+1);
    % Benutze eine konstante Geschwindigkeit ohne Beschleunigung
    X_t_in(i1:ii1,6) = interp1(t([i1;ii1]), [phi1,phi2], t(i1:ii1)); % Einfachere Variante
    XD_t_in(i1:ii1,6) = (XE(i+1,6) - XE(i,6))/(t(i2)-t(i1));
    XDD_t_in(i1:ii1,6) = 0;
  end
end
% % Fehlerkorrektur der Funktion. Manchmal sehr kleine Imaginärteile
% if any(~isreal(X_t_in(:))) || any(~isreal(XD_t_in(:))) || any(~isreal(XDD_t_in(:)))
%   X_t_in = real(X_t_in); XD_t_in = real(XD_t_in); XDD_t_in = real(XDD_t_in);
% end
% Wenn der zweite Rastpunkt bereits nicht erreicht werden kann, bleibt
% komplett NaN gesetzt. Damit dann unten kein Fehler aufgeworfen wird, überschreiben
if all(isnan(X_t_in(:,6)))
  X_t_in(:,6) = x0(6);
  XD_t_in(:,6) = 0;
  XDD_t_in(:,6) = 0;
end
% Toleranzband für die Koordinate: Muss genauso wie oben sein, damit
% ungefähr das gleiche rauskommt.
xlim6_interp = NaN(3,1+3*(length(IE)-1)); % Spalten: Zeitschritte
xlim6_interp(:,1) = [t(1);-delta_phi/2; delta_phi/2];
delta_phi_i = delta_phi;
for i = 1:size(XE,1)-1
  if ~any(I_validstates==i)
    % Kein gültiger Zustand in DP gefunden. Keine Beschränkung der
    % Koordinate sinnvoll für Gesamt-Trajektorie. Dadurch Kriterium ab dem
    % Zeitpunkt deaktiviert, ab dem keine Aussage mehr möglich ist.
    delta_phi_i = inf;
  end
  % Zweiter Punkt direkt am Anfang für Steitigkeit (funktioniert nicht bei
  % zusammengesetzten Einzelbewegungen)
%   xlim6_interp(:,1+3*i-2) = [t(IE(i))+1e-6;-delta_phi/2; delta_phi/2];
  % Aufweitung des Toleranzbandes zur Mitte
  xlim6_interp(:,1+3*i-1) = [mean(t(IE([i,i+1])));-delta_phi_i; delta_phi_i];
  % Zusammenführen am Ende
  xlim6_interp(:,1+3*i) = [t(IE(i+1));-delta_phi_i/2 + 1e-3; delta_phi_i/2 - 1e-3];
end
xlim6_interp = xlim6_interp(:, ~any(isnan(xlim6_interp)));
if s.verbose > 2 % Debug-Plot für die Referenztrajektorie
  change_current_figure(300);clf;
  subplot(3,1,1); hold on;
  plot(t, 180/pi*X_t_in(:,6));
  plot(t(IE), 180/pi*X_t_in(IE,6), 'rs');
  plot(t, 180/pi*(X_t_in(:,6)+interp1(xlim6_interp(1,:), xlim6_interp(2,:), t, 'spline')));
  plot(t, 180/pi*(X_t_in(:,6)+interp1(xlim6_interp(1,:), xlim6_interp(3,:), t, 'spline')));
  subplot(3,1,2);
  plot(t, XD_t_in(:,6));
  subplot(3,1,3);
  plot(t, XDD_t_in(:,6));
end
% Eintragen der Verläufe von Optimierungsvariable und ihrer Geschwindigkeit
s_Traj.xlim6_interp = xlim6_interp;
nullspace_maxvel_interp = nullspace_maxvel_from_tasktraj(t, ...
  IE, s.Tv, s.T_dec_ns , diff(t(1:2)));
s_Traj.nullspace_maxvel_interp = nullspace_maxvel_interp;
% Kein Abbrechen der Trajektorie mehr, wenn Toleranzband verletzt wird.
% Dieses war nur Mittel zum Zweck für die DP (bzw. dort auch deaktiviert)
% TODO: enforce_xlim hier wieder deaktivieren?
s_Traj.abort_thresh_h(R.idx_iktraj_hn.xlim_hyp) = NaN;
if R.Type == 0 % Seriell
  [Q, QD, QDD, PHI, JP, Stats] = R.invkin2_traj(X_t_in, XD_t_in, XDD_t_in, t, q0, s_Traj);
  Jinv_ges = [];
else % PKM
  [Q, QD, QDD, PHI, Jinv_ges, ~, JP, Stats] = R.invkin2_traj(X_t_in, XD_t_in, XDD_t_in, t, q0, s_Traj);
end
[X_neu, XD_neu, XDD_neu] = R.fkineEE2_traj(Q, QD, QDD);
X_neu(:,6) = denormalize_angle_traj(X_neu(:,6));

if Stats.iter < length(t) && s.verbose
  fprintf(['Trajektorie nicht erfolgreich bei komplettem Durchlauf. ', ...
    'Bis %d/%d gekommen.\n'], Stats.iter, length(t));
end
if Stats.iter == 0
  warning('Sofortiger Abbruch der Trajektorie. Ungültige Startbedingung.');
end
if s.verbose > 1 && length(I_validstates) > 1 % Bild nur Sinnvoll, wenn mind. eine Stufe geschafft
  %% Debug: Ergebnis-Bild für Dynamische Programmierung
  dpfinalhdl1 = change_current_figure(199);clf;hold on;
  subplot(2,2,1);hold on;
  % Alle Einzelnen Schritte einzeichnen
  k_list = find(~isnan(XE_all(end,:)));
  for i = size(XE,1):-1:2 % Rekursiv
    k_list_prev = I_all(i,k_list);
    for jj = 1:length(k_list)
      plot([i-1;i], 180/pi*[XE_all(i-1,k_list_prev(jj)); XE_all(i,k_list(jj))], 'k-');
    end
    % Listen tauschen
    k_list = k_list_prev;
    plot(i, 180/pi*phi_range, 'bo');
    % Zeichne Antennen (nicht gut erkennbar). Sollen Intervall darstellen.
    quiver(i*ones(1,length(phi_range)), 180/pi*phi_range, ...
      zeros(1,length(phi_range)), 180/pi*delta_phi/2*ones(1,length(phi_range)), 'off', '.');
    quiver(i*ones(1,length(phi_range)), 180/pi*phi_range, ...
      zeros(1,length(phi_range)), -180/pi*delta_phi/2*ones(1,length(phi_range)), 'off', '.');
  end
  xlabel('Stufe (Rastpose)');
  ylabel('Zustand (Winkel)');
  grid on;
  XE_best = NaN(length(I_best), 1);
  for i = 1:size(XE,1)
    if isnan(I_best(i)), break;end
    XE_best(i) = XE_all(i,I_best(i));
  end
  subplot(2,2,2);hold on;
  plot(t, 180/pi*X_t_in(:,6));
  plot(t, 180/pi*(X_t_in(:,6)+interp1(xlim6_interp(1,:), xlim6_interp(2,:), t, 'spline')));
  plot(t, 180/pi*(X_t_in(:,6)+interp1(xlim6_interp(1,:), xlim6_interp(3,:), t, 'spline')));
  plot(t(IE), 180/pi*XE(:,6), 'rs');
  plot(t(IE(I_validstates(2:end))), 180/pi*XE_best(2:I_validstates(end)), 'co');
  legend({'Ref', 'Max', 'Min', 'Result DP', 'Keypoint'});
  grid on; ylabel('phi z optimal, Stützstellen');
  xlabel('Zeit in s');
  subplot(2,2,3);hold on;
  plot(t, interp1(nullspace_maxvel_interp(1,:),nullspace_maxvel_interp(2,:), t));
  plot(t(IE), zeros(length(IE),1), 'rs');
  grid on; ylabel('Geschwindigkeitsgrenze, Nullraum, norm');
  sgtitle(dpfinalhdl1, 'Soll-Trajektorie aus Dynamischer Programmierung');
  xlabel('Zeit in s');
  %% Debug-Bild für Trajektorie
  dpfinalhdl2 = change_current_figure(250);clf;
  subplot(2,2,1);hold on;
  plot(t, 180/pi*X_t_in(:,6));
  plot(t, 180/pi*X_neu(:,6));
  plot(t, 180/pi*(X_t_in(:,6)+interp1(xlim6_interp(1,:), xlim6_interp(2,:), t, 'spline')), '-^');
  plot(t, 180/pi*(X_t_in(:,6)+interp1(xlim6_interp(1,:), xlim6_interp(3,:), t, 'spline')), '-^');
  legend({'Ref', 'Traj', 'Traj-Lim.', 'Traj-Lim.'});
  grid on; ylabel('phiz'); xlabel('Zeit in s');
  subplot(2,2,2);hold on;
  plot(t, XD_t_in(:,6));
  plot(t, XD_neu(:,6));
  plot(t(IE), zeros(length(IE),1), 'rs');
  grid on; ylabel('phiz D'); xlabel('Zeit in s'); legend({'Ref', 'Traj'});
  subplot(2,2,3);hold on;
  plot(t, XDD_t_in(:,6));
  plot(t, XDD_neu(:,6));
  plot(t(IE), zeros(length(IE),1), 'rs');
  grid on; ylabel('phiz DD'); xlabel('Zeit in s');
  legend({'Ref', 'Traj'});
  sgtitle(dpfinalhdl2, 'Ist-Trajektorie aus Dynamischer Programmierung (durchgängig)');
  linkxaxes
  %% In großes Debug-Bild der Redundanzkarte einzeichnen
  if s.verbose > 1
    change_current_figure(figikhdl);
    plot(t, 180/pi*X_neu(:,6), 'g-', 'linewidth', 3);
    % Bildausschnitt ändern
    ylim(180/pi*minmax2([s.PM_phiz_range;XE_all(:);X_neu(:,6)]'));
    % Zeichne Stützstellen aus DP ein. Dadurch Abweichung der finalen
    % Trajektorie zu Rast-zu-Rast-Trajektorie aus DP deutlich
    plot(t(IE(I_validstates)), 180/pi*XE(I_validstates,6), 'rs');
    % Legende erst hier einzeichnen, damit sie nicht automatisch gefüllt wird
    I_hdl = ~isnan(PM_hdl); % nur im Plot aktive Nebenbedingungen in Legende
    legend(PM_hdl(I_hdl), PM_legtxt(I_hdl), 'Location', 'South', 'Orientation', 'horizontal');
  end
end
%% Ausgabe schreiben
DPstats = struct('F_all', F_all, 'n_statechange_succ', n_statechange_succ, ...
  'n_statechange_total', n_statechange_total, 'nt_ik', nt_ik, ...
  'nt_ik_all', Stats_nt_ik_all, 'Stats_comptime_ik_all', Stats_comptime_ik_all, ...
  'statechange_succ_all', Stats_statechange_succ_all, ...
  'phi_range', phi_range, 'z1', z1, 'z2', z2, 'delta_phi', delta_phi);
TrajDetail = struct('Q', Q, 'QD', QD, 'QDD', QDD, 'PHI', PHI, 'JP', JP, ...
  'Jinv_ges', Jinv_ges, 'Stats', Stats, 'X6', X_neu(:,6), ...
  'XD6', XD_neu(:,6), 'XDD6', XDD_neu(:,6));
if s.verbose > 1 && ~isempty(s.debug_dir)
  saveas(figikhdl, fullfile(s.debug_dir, 'dp_perfmap_traj.fig'));
end
if ~isempty(s.debug_dir)
  save(fullfile(s.debug_dir, 'dp_output.mat'), 'DPstats', 'TrajDetail');
end
