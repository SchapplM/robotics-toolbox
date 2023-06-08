% Inverse Kinematik für allgemeinen Roboter
% % Variante 3:
% * Translation mit Vektor 0-E statt A-B
% * Absolute Rotation ausgedrückt bspw. in XYZ-Euler-Winkeln
%   (statt XYZ wird die Konvention aus `phiconv_W_E` genommen)
% * Rotationsfehler mit Orientierungsfehler ZYX-Rotation um festes KS
%   (Linksmultiplikation)
% Numerische Berechnung mit Inverser Jacobi-Matrix der inversen Kinematik.
% Dadurch Berechnung aller Gelenkwinkel aller Beine auf einmal
% 
% Eingabe:
% xE_soll [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS (Soll)
% Q0 [N x N_init]
%   Anfangs-Gelenkwinkel für Algorithmus (werden nacheinander ausprobiert)
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% s_in
%   Struktur mit Eingabedaten. Felder, siehe Quelltext.
%   Auswahl der Einstellungen:	
%   .wn [11x1] Gewichtungen der Zielfunktionen für Nullraumbewegung
%     (1) Quadratischer Abstand der Gelenkkoordinaten von ihrer Mitte
%     (2) Hyperbolischer Abstand der Gelenkkoordinaten von ihren Grenzen
%     (3) Konditionszahl der geometrischen Matrix der Inv. Kin.
%     (4) Konditionszahl der PKM-Jacobi-Matrix (Antriebe zu Plattform)
%     (5) Abstand der Kollisionskörper voneinander (hyperbolisch gewertet)
%     (6) Abstand von Prüfkörpern des Roboters zur Bauraumgrenze (hyperbolisch)
%     (7) Abstand von phiz zu xlim (quadratisch gewertet)
%     (8) Abstand von phiz zu xlim (hyperbolisch gewertet)
%     (9) Abstand der Kollisionskörper voneinander (quadratisch gewertet)
%    (10) Abstand von Prüfkörpern des Roboters zur Bauraumgrenze (quadratisch)
%    (11) Positionsfehler am Endeffektor (max. Wert für Position)
% 
% Ausgabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM als Lösung der IK
% Phi
%   Kinematische Zwangsbedingungen für die Lösung. Bei korrekter Berechnung
%   muss dieser Wert Null sein.
% Tc_stack_PKM 
%   Gestapelte Transformationsmatrizen der PKM. Im Basis-KS.
%   Entspricht mit Abwandlung der Anordnung wie in fkine:
%   * PKM-Basis
%   * Für jede Beinkette: Basis und alle bewegten Körper-KS. Ohne
%     virtuelles EE-KS
%   * Plattform-KS
%   * EE-KS (damit Kollisionskörper zugeordnet werden können)
% Stats
%   Struktur mit Detail-Ergebnissen für den Verlauf der Berechnung. Felder:
%   .mode (jedes gesetzte Bit entspricht einem Programmpfad der IK ("Modus"))
%   .condJ (n+1x2): (1.) Konditionszahl der IK-Jacobi-Matrix (Ableitung
%     des Euler-Winkel-Residuums mit reduzierten FG. (2.) Konditionszahl 
%     der analytischen PKM-Jacobi-Matrix ohne Betrachtung von Aufgaben-Red.
%   .maxcolldepth (n+1x2): Eindringtiefe der Kollisionen. Erste Spalte alle
%     Prüfungen. Zweite Spalte nur mit Redundanz beeinflussbare Prüfungen.
%     Negative Werte sind keine Kollision (gut)
%   .instspc_mindst (n+1x2): Überschreitungsdistanz des Bauraums. Erste Spalte
%     alle, zweite nur beeinflussbare Prüfungen. Negative Werte sind im Bauraum (gut)
% Jinv_out
%   Inverse PKM-Jacobi-Matrix
%   (Jacobi zwischen allen Gelenkgeschwindigkeiten qD und EE-geschwindigkeit xDE)
%   (Nicht: Nur Bezug zu Antriebsgeschwindigkeiten qaD)
% 
% Quelle:
% [SchapplerTapOrt2019] Schappler, M. et al.: Modeling Parallel Robot Kine-
% matics for 3T2R and 3T3R Tasks using Reciprocal Sets of Euler Angles (2019)
% [NakamuraHan1986] Y. Nakamura, H. Hanafusa: Inverse Kinematic Solutions 
% With Singularity Robustness for Robot Manipulator Control, 1986
% [CorkeIK] Peter Corke, Robotics Toolbox, ikine.m
% [SchapplerOrt2021] Schappler, M. et al.: Singularity Avoidance of Task-
% Redundant Robots in Pointing Tasks (...); ICINCO 2021
% 
% Siehe auch: SerRob/invkin2 (bzw. robot_invkin_eulangresidual.m.template)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07/2019-06
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [q, Phi, Tc_stack_PKM, Stats, Jinv_out] = invkin3(Rob, xE_soll, Q0, s_in)

%% Initialisierung
% Variablen so initialisieren, dass Code gleich mit Vorlagen-Funktion
Rob_I_EE = Rob.I_EE;
NJ = Rob.NJ;
NL = Rob.NL;
NLEG = Rob.NLEG;
I_qa = Rob.I_qa;
assert(isreal(xE_soll) && all(size(xE_soll) == [6 1]), ...
  'ParRob/invkin3: xE_soll muss 6x1 sein');
assert(isreal(Q0) && all(size(Q0,1) == [NJ]), ...
  'ParRob/invkin3: Q0 muss %dxn sein', NJ);

%% Definitionen
% Variablen zum Speichern der Zwischenergebnisse
sigma_PKM = Rob.MDH.sigma; % Marker für Dreh-/Schubgelenk
s = struct(...
  'K', ones(NJ,1), ... % Verstärkung Aufgabenbewegung
  'Kn', ones(NJ,1), ... % Verstärkung Nullraumbewegung
  'wn', zeros(Rob.idx_ik_length.wnpos,1), ... % Gewichtung der Nebenbedingung
  'xlim', Rob.xlim, ... % Begrenzung der Endeffektor-Koordinaten
  'q_poserr', Rob.update_q_poserr(), ... % Positionsfehler an den Antrieben
  'maxstep_ns', 1e-10, ... % Maximale Schrittweite für Nullraum zur Konvergenz (Abbruchbedingung)
  'normalize', true, ...
  'condlimDLS', 1, ... % Grenze der Konditionszahl, ab der die Pseudo-Inverse gedämpft wird (1=immer)
  'lambda_min', 2e-4, ... % Untergrenze für Dämpfungsfaktor der Pseudo-Inversen
  'n_min', 0, ... % Minimale Anzahl Iterationen
  'n_max', 1000, ... % Maximale Anzahl Iterationen
  'rng_seed', NaN, ...  % Initialwert für Zufallszahlengenerierung
  'scale_lim', 0, ... % Herunterskalierung bei Grenzüberschreitung
  'scale_coll', 0, ... % Herunterskalierung bei Kollision
  'collbodies_thresh', 1.5, ... % Vergrößerung der Kollisionskörper für Aktivierung des Ausweichens
  'collision_thresh', NaN, ... % absoluter Schwellwert für die Aktivierung der Kollisions-Kennzahl (hyperbolisch)
  'installspace_thresh', 0.100, ... % Ab dieser Nähe zur Bauraumgrenze Nullraumbewegung zur Einhaltung des Bauraums
  'Phit_tol', 1e-8, ... % Toleranz für translatorischen Fehler
  'Phir_tol', 1e-8,... % Toleranz für rotatorischen Fehler
  'maxrelstep', 0.1, ... % Maximale Schrittweite relativ zu Grenzen
  'maxrelstep_ns', 0.005, ... % Maximale Schrittweite der Nullraumbewegung
  'avoid_collision_finish', false,... % Nullraumbewegung am Ende zur Vermeidung von Kollisionen
  'finish_in_limits', false, ... % Führe am Ende eine Nullraumoptimierung zur Wiederherstellung der Grenzen durch
  ... % Bei hyperbolischen Grenzen kann z.B. mit Wert 0.9 erreicht werden, 
  ... % dass in den mittleren 90% der Gelenkwinkelspannweite das Kriterium 
  ... % deaktiviert wird (Stetigkeit durch Spline). Deaktivierung mit NaN.
  'optimcrit_limits_hyp_deact', NaN, ... 
  'cond_thresh_ikjac', 1, ... % Schwellwert zur Aktivierung der IK-Jacobi-Optimierung
  'cond_thresh_jac', 1, ... % Schwellwert zur Aktivierung der PKM-Jacobi-Optimierung
  'retry_on_limitviol', false, ... % Bei Grenzverletzung neu versuchen mit true
  'retry_limit', 100, ...; % Anzahl der Neuversuche
  'debug', false); % Zusätzliche Test-Berechnungen
if nargin == 4
  % Prüfe Felder der Einstellungs-Struktur und belasse Standard-Werte, 
  % falls Eingabe nicht gesetzt
  for f = fields(s_in)'
    if isfield(s, f{1}) % Feld in Eingabe gesetzt
      s.(f{1}) = s_in.(f{1});
    else
      error('Feld %s aus s_in kann nicht übergeben werden', f{1});
    end
  end
end

% Variablen aus Einstellungsstruktur holen
K = s.K; 
Kn = s.Kn; 
n_min = s.n_min;
n_max = s.n_max;
s.wn = [s.wn;zeros(Rob.idx_ik_length.wnpos-length(s.wn),1)]; % Fülle mit Nullen auf, falls altes Eingabeformat
wn = s.wn;
condlimDLS = s.condlimDLS;
lambda_min = s.lambda_min;
scale_lim = s.scale_lim;
scale_coll = s.scale_coll;
Phit_tol = s.Phit_tol;
Phir_tol = s.Phir_tol;
retry_limit = s.retry_limit;
maxrelstep = s.maxrelstep;
maxrelstep_ns = s.maxrelstep_ns;
maxstep_ns = s.maxstep_ns;
finish_in_limits = s.finish_in_limits;
break_when_in_limits = false;
avoid_collision_finish = s.avoid_collision_finish;
success = false;

nsoptim = false;
if sum(Rob_I_EE) > sum(Rob.I_EE_Task)
  % Nullraumoptimierung nur möglich, falls FG da sind. TODO: Das
  % berücksichtigt noch nicht den Fall von 3T3R-PKM in 3T0R-Aufgaben.
  if any(wn ~= 0)
    nsoptim = true;
  end
else
  % Keine zusätzlichen Optimierungskriterien
  finish_in_limits = false; % Alle Nullraumbewegungen nicht möglich
end
% Prüfe, ob der Fall von 1FG-Aufgabenredundanz vorliegt.
if sum(Rob_I_EE) - sum(Rob.I_EE_Task) == 1 && ~Rob.I_EE_Task(end)
  taskred_rotsym = true;
else
  taskred_rotsym = false;
end
idx_wn = Rob.idx_ikpos_wn;
idx_hn = Rob.idx_ikpos_hn;
qlim = NaN(NJ,2);
J1 = 1;
for i = 1:NLEG
  J2 = J1+Rob.Leg(i).NQJ-1;
  qlim(J1:J2,:) = Rob.Leg(i).qlim;
  J1 = J2+1;
end
qmin = qlim(:,1);
qmax = qlim(:,2);
limits_set = false;
if all(~isnan(qlim(:)))
  limits_set = true;
else
  % Grenzen sind nicht wirksam
  qmin = -Inf(NJ,1);
  qmax =  Inf(NJ,1);
  retry_limit = size(Q0,2)-1; % keine zufällige Neubestimmung möglich.
  finish_in_limits = false;
end
% Grenzen für die Neubestimmung der Anfangswerte (falls unendl. vorkommt).
% Annahme: Betrifft nur Drehgelenke. Dort dann zwischen -pi und pi.
% Die Variable qmin/qmax wird für Nebenbedingungen benutzt.
qmin_norm = qmin; qmax_norm = qmax;
qmin_norm(isinf(qmin)) = sign(qmin_norm(isinf(qmin)))*(pi);
qmax_norm(isinf(qmax)) = sign(qmax_norm(isinf(qmax)))*(pi);
delta_qlim = qmax - qmin;
% Schwellwert in Gelenkkoordinaten für Aktivierung des Kriteriums für 
% hyperbolisch gewichteten Abstand von den Grenzen.
qlim_thr_h2 = repmat(mean(qlim,2),1,2) + repmat(delta_qlim,1,2).*...
  repmat([-0.5, +0.5]*s.optimcrit_limits_hyp_deact,NJ,1);
% Schwellwert der Z-Rotation (3T2R) für Aktivierung des Kriteriums für 
% hyperbolisch gewichteten Abstand von den Grenzen.
xlim_thr_h8 = repmat(mean(s.xlim,2),1,2) + repmat(s.xlim(:,2)-s.xlim(:,1),1,2).*...
  repmat([-0.5, +0.5]*0.8,6,1); % vorläufig auf 80% der Grenzen in xlim
if any(isnan(s.xlim(6,:))) % Kriterium nicht bestimmbar
  s.wn([idx_wn.xlim_par, idx_wn.xlim_hyp]) = 0;
end
I_constr_t_red = Rob.I_constr_t_red;
I_constr_r_red = Rob.I_constr_r_red;
% Variablen für Dämpfung der Inkremente
delta_q_alt = zeros(NJ,1); % Altwert für Tiefpassfilter
delta_q_N_alt = zeros(NJ,1); % Altwert für Nullraum-Tiefpassfilter
damping_active = false; % Standardmäßig noch nicht aktiviert
q0 = Q0(:,1);
Phi_alt = zeros(length(Rob.I_constr_red),1); % Altwert für Tiefpassfilter
delta_Phi_alt = Phi_alt;
N = NaN(NJ,NJ); % Nullraum-Projektor
% Gradient von Nebenbedingung 3 bis 5
h3dq = zeros(1,NJ); h4dq = h3dq; h5dq = h3dq; h6dq = h3dq; h9dq = h3dq; h10dq = h3dq; h11dq = h3dq;
h = zeros(Rob.idx_ik_length.hnpos,1); h_alt = inf(Rob.idx_ik_length.hnpos,1); % Speicherung der Werte der Nebenbedingungen
bestcolldepth = inf; currcolldepth = inf; % Speicherung der Schwere von Kollisionen
bestinstspcdist = inf; currinstspcdist = inf; % Speicherung des Ausmaßes von Bauraum-Verletzungen
% Definitionen für die Kollisionsprüfung
collbodies_ns = Rob.collbodies;
maxcolldepth = 0;
collobjdist_thresh = 0;
if isempty(collbodies_ns.type) % Keine Kollisionskörper
  s.wn([idx_wn.coll_hyp,idx_wn.coll_par]) = 0; % Deaktivierung der Kollisionsvermeidung
  scale_coll = 0;
  avoid_collision_finish = false;
end
if isempty(Rob.collbodies_instspc.type) % Keine Kollisionskörper
  s.wn([idx_wn.instspc_hyp,idx_wn.instspc_par]) = 0; % Deaktivierung der Bauraumprüfung
end
wn = s.wn;
if scale_coll || wn(idx_wn.coll_hyp) || wn(idx_wn.coll_par) || avoid_collision_finish
  % Kollisionskörper für die Kollisionserkennung z.B. 50% größer machen.
  % Ist zusammen mit dem Schwellwert für die Kollisionsvermeidung wirksam.
  collbodies_ns.params(collbodies_ns.type==6,1) = ... % Kapseln (Direktverbindung)
    s.collbodies_thresh*collbodies_ns.params(collbodies_ns.type==6,1);
  collbodies_ns.params(collbodies_ns.type==13,7) = ... % Kapseln (Basis-KS)
    s.collbodies_thresh*collbodies_ns.params(collbodies_ns.type==13,7);
  collbodies_ns.params(collbodies_ns.type==4|collbodies_ns.type==15,4) = ... % Kugeln
    s.collbodies_thresh*collbodies_ns.params(collbodies_ns.type==4|collbodies_ns.type==15,4);
  % Maximal mögliche Eindringtiefe der Warnungs-Ersatzkörper bestimmen um
  % daraus die Grenzen der hyperbolischen Kollisionsfunktion zu bestimmen.
  % Ist eine etwas größere Schätzung (abhängig von relativer Größe von
  % Kugeln und Zylindern)
  maxcolldepth = max([0; collbodies_ns.params(collbodies_ns.type==6,1);  ... % 1. Eintrag damit nicht leer
                         collbodies_ns.params(collbodies_ns.type==13,7); ...
                         collbodies_ns.params(collbodies_ns.type==4|collbodies_ns.type==15,4)]);
  % Abstand der Objekte, ab dem die Zielfunktion anfängt (bei größeren
  % Abständen ist sie Null). Dies Wert muss kleiner sein als der, ab dem die
  % Erkennung beginnt (sonst Sprung). Unklar, ob dieser Wert immer passt.
  % Die Erkennung wird durch `collbodies_ns` bestimmt. Diese müssen also
  % eher zu groß gewählt werden (über Parameter collbodies_thresh).
  % je kleiner der Wert wird, desto später wird die Vermeidung wirksam
  collobjdist_thresh = 0.3 * maxcolldepth;
end
if ~isnan(s.collision_thresh)
  collobjdist_thresh = s.collision_thresh;
end
% Übergreifende Variable zur Speicherung aktiver Kollisionsprüfungen.
I_collcheck_nochange_all = false(1,size(Rob.collchecks,1));
I_instspccheck_nochange = false(1,size(Rob.collchecks_instspc,1));

% Zählung in Rob.NL: Starrkörper der Beinketten, Gestell und Plattform. 
% Hier werden nur die Basis-KS der Beinketten und alle bewegten Körper-KS
% der Beine angegeben.
Tc_stack_PKM = NaN((NL+NLEG+1)*3,4); % siehe fkine_legs; dort aber leicht anders
rejcount = 0; % Zähler für Zurückweisung des Iterationsschrittes, siehe [CorkeIK]
scale = 1; % Skalierung des Inkrements (kann reduziert werden durch scale_lim)
condJ = NaN; condJik = NaN;
Jinv_out = NaN(NJ, sum(Rob_I_EE));

Stats_default = struct('file', 'pkm_invkin_eul', 'Q', NaN(1+n_max, NJ), ...
  'PHI', NaN(1+n_max, 6*NLEG), 'iter', n_max, 'retry_number', retry_limit, ...
  'condJ', NaN(1+n_max,2), 'lambda', NaN(n_max,2), 'rejcount', NaN(n_max,1), ...
  'h', NaN(1+n_max,1+Rob.idx_ik_length.hnpos), 'coll', false, 'instspc_mindst', NaN(1+n_max,2), ...
  'maxcolldepth', NaN(1+n_max,2), 'h_instspc_thresh', NaN, 'h_coll_thresh', ...
  NaN, 'mode', uint32(zeros(n_max,1)));
Stats = Stats_default;

%% Iterative Berechnung der inversen Kinematik
for rr = 0:retry_limit % Schleife über Neu-Anfänge der Berechnung
  q1 = q0;
  % Grad der Nicht-Erfüllung der Zwangsbedingungen (Fehler)
  [Phi, Phi_voll] = Rob.constr3(q1, xE_soll);
  if nargout >= 4 % Anfangswerte eintragen
    % Zurücksetzen der Statistik (falls mehrere Wiederholungen gemacht werden
    Stats = Stats_default;
    Stats.PHI(1,:) = Phi_voll;
    Stats.Q(1,:) = q1;
  end
  lambda_mult = lambda_min; % Zurücksetzen der Dämpfung
  lambda = 0.0;
  rejcount = 0; % Zurücksetzen des Zählers für Fehlversuche
  bestPhi = inf; % Merker für bislang bestes Residuum
  % Zurücksetzen des Modus für Abbruch in Grenzen (falls Neuversuch unter-
  % nommen wurde, weil die Grenzen doch verletzt wurden).
  if limits_set % Siehe unten bei Fallabfrage finish_in_limits
    finish_in_limits = s.finish_in_limits;
    nsoptim = false;
    wn = s.wn;
    if any(wn ~= 0), nsoptim = true; end
    break_when_in_limits = false;
  end
  for jj = 1:n_max
    % Gesamt-Jacobi bilden (reduziert um nicht betrachtete EE-Koordinaten)
    Jik=Rob.constr3grad_q(q1, xE_soll);
    %% Nullstellensuche für Positions- und Orientierungsfehler
    condJik = cond(Jik);
    % (Optimierung der Aufgabe)
    % Benutze das Damped Least Squares Verfahren je nach Konditionszahl.
    % Bei Redundanz immer benutzen, nicht nur, falls Nullraumprojektion erfolglos.
    if condJik > condlimDLS% && (~nsoptim || nsoptim && rejcount > 0)
      % Pseudo-Inverse mit Dämpfung:
      % Passe die Dämpfung lambda im DLS-Verfahren an. Wähle die Kon-
      % ditionszahl als Kriterium, da z.B. Grenzen für Singulärwerte
      % und Manipulierbarkeit nicht bekannt sind.
      % Skalierung zwischen 0 (z.B. Grenzfall cond=60) und 1 (komplett singulär).
      % Nehme einen Mindestwert für die Dämpfung und einen sich bei Stagnation
      % erhöhenden Aufschlag. Mit Aufschlag wird immer lambda_min benutzt.
      lambda = (-1+2*2/pi*atan(condJik/condlimDLS))*(lambda_min+lambda_mult)/2;
      % [NakamuraHan1986], Gl. 22. Kleinere Dimension bei Redundanz als andere pinv.
      delta_q_T = ((Jik')/(Jik*Jik' + lambda*eye(length(Phi))))*(-Phi);
    else
      % Normale (Pseudo-)Invertierung der Jacobi-Matrix der seriellen Kette
      delta_q_T = Jik \ (-Phi);
      lambda = 0.0;
      lambda_mult = lambda_min; % Zurücksetzen. Alles (wieder) i.O.
    end
    %% Optimierung der Nebenbedingungen (Nullraum)
    delta_q_N = zeros(size(delta_q_T));
    if nsoptim && ... % Nullraum muss vorhanden sein und Kriterien gesetzt
        ... %% falls vorherige Iterationen erfolglos, keine Nullraumbewegung. 
        ... % Annahme: Schädlich für Konvergenz. Nur, falls Stagnation 
        ... % nicht durch Gelenkgrenzen (scale_lim) verursacht wurde.
        (rejcount == 0 || rejcount~=0 && scale == 0)
      N = (eye(NJ) - pinv(Jik)* Jik); % Nullraum-Projektor
      % Berechne Inkremente für alle Gelenkwinkel um Nullraumbewegungen mit
      % Differenzenquotienten zu bestimmen. Notwendig, wenn Nullraumbewe-
      % gung durchgeführt wird, wenn die Zwangsbedingungen nicht erfüllt
      % sind.
      qD_test_all = NaN(NJ,NJ);
      I_qD_test_all = false(NJ,1); % Indizes relevanter Gelenke
      scale_qD_test_all = NaN(NJ,1);
      for kkk = 1:NJ
        qD_test = zeros(NJ,1);
        qD_test(kkk) = 1e-6; % kleines Inkrement
        % Nullraumbewegung mit diesem Inkrement. Isolierte Bewegung
        % Eines Gelenks bei dieser Kennzahl nicht möglich. In Null-
        % raumbewegung findet auch Bewegung des anderen Kollisions-
        % objektes statt. TODO: Saubere Herleitung fehlt noch.
        qD_test_N = N*qD_test;
        if abs(qD_test_N(kkk)) < 1e-10, continue; end % Gelenk hat keinen Einfluss auf Nullraum
        I_qD_test_all(kkk) = true; % erst jetzt Gelenk für Differenzenquotient unten benutzen
        % Normierung größtes Element auf 1e-6.
        scale_qD_test_all(kkk) = 1e-6 / qD_test_N(kkk);
        qD_test_N = qD_test_N * scale_qD_test_all(kkk);
        qD_test_all(:,kkk) = qD_test_N;
         % Ignoriere den Rest. Es sind sowieso immer alle gleich. Der
         % Ansatz dient hauptsächlich dazu, Gelenke ohne Einfluss
         % auszusortieren.
        break;
      end

      % Berechne Gradienten der zusätzlichen Optimierungskriterien
      h(:) = 0;
      v = zeros(NJ, 1);
      %% Einhaltung der Gelenkwinkel-Grenzen
      if wn(idx_wn.qlim_par) ~= 0
        Stats.mode(jj) = bitset(Stats.mode(jj),4);
        [h(idx_hn.qlim_par), h1dq] = invkin_optimcrit_limits1(q1, qlim);
        v = v - wn(idx_wn.qlim_par)*h1dq'; % [SchapplerTapOrt2019], Gl. (44)
      end
      if wn(idx_wn.qlim_hyp) ~= 0
        Stats.mode(jj) = bitset(Stats.mode(jj),5);
        [h(idx_hn.qlim_hyp), h2dq] = invkin_optimcrit_limits2(q1, qlim, qlim_thr_h2);
        v = v - wn(idx_wn.qlim_hyp)*h2dq'; % [SchapplerTapOrt2019], Gl. (45)
      end
      %% Singularitätsvermeidung Jacobi-Matrix (IK- und PKM-Jacobi) und Positionsfehler
      Jinv = NaN(NJ, 6);
      % Bestimme Ist-Lage der Plattform (bezogen auf erste Beinkette).
      % Benutze dies für die Berechnung der PKM-Jacobi. Nicht aussage-
      % kräftig, wenn Zwangsbedingungen grob verletzt sind. Dafür wird
      % die Rotation korrekt berücksichtigt.
      xE_1 = xE_soll + [zeros(5,1); Phi_voll(4)];
      % Bestimme PKM-Jacobi für Iterationsschritt (falls benötigt)
      if wn(idx_wn.jac_cond) || wn(idx_wn.poserr_ee) || any(wn(3:end)) && taskred_rotsym && all(abs(Phi)<1e-6) || s.debug
        % Benutze einfache Jacobi-Matrix und nicht die constr3grad-
        % Funktionen. Jinv ist zwischen beiden nur identisch, wenn Phi
        % exakt Null ist.
        [~, Phi4_x_voll] = Rob.constr4grad_x(xE_1);
        [~, Phi4_q_voll] = Rob.constr4grad_q(q1);
        Jinv = -Phi4_q_voll\Phi4_x_voll;
        % Entferne Elemente rechengenauigkeitsbedingt nahe bei Null, die sehr
        % große Gradienten erzeugen, die aber effektiv keine Wirkung haben.
        Jinv(abs(Jinv)/max(abs(Jinv(:)))<1e-6) = 0;
        condJ = cond(Jinv(I_qa,Rob_I_EE)); % bezogen auf Antriebe (nicht: Passive Gelenke)
      end
      if wn(idx_wn.ikjac_cond) ~= 0 && condJik > s.cond_thresh_ikjac || ...
         wn(idx_wn.jac_cond) ~= 0 && condJ > s.cond_thresh_jac || ... % Singularitäts-Kennzahl aus Konditionszahl
         wn(idx_wn.poserr_ee) ~= 0
        h(idx_hn.ikjac_cond) = invkin_optimcrit_condsplineact(condJik, ... %  Spline-Übergang
          1.5*s.cond_thresh_ikjac, s.cond_thresh_ikjac); %  zwischen Null
        h(idx_hn.jac_cond) = invkin_optimcrit_condsplineact(condJ, ... und der
          1.5*s.cond_thresh_jac, s.cond_thresh_jac); %      Konditionszahl
        if wn(idx_wn.poserr_ee) ~= 0
          dx_poserr = abs(inv(Jinv(I_qa,Rob_I_EE))) * s.q_poserr(I_qa);
          h(idx_hn.poserr_ee) = norm(dx_poserr(Rob_I_EE(1:3)));
        end
        % Zwei verschiedene Arten zur Berechnung der Nullraumbewegung, je
        % nachdem, ob die Beinketten schon geschlossen sind, oder nicht.
        % Beide Varianten werden im Debug-Modus gegeneinander geprüft.
        if all(abs(Phi)<1e-6) && taskred_rotsym || s.debug % Variante 1
          Stats.mode(jj) = bitset(Stats.mode(jj),16);
          % Bestimme Nullraumbewegung durch Differenzenquotient für die 
          % redundante Koordinate. Dadurch nur eine neue Funktions- 
          % auswertung
          xD_test_3T3R = [zeros(5,1);1e-8];
          xD_test = xD_test_3T3R; % Hier werden für 2T1R nicht die Koordinaten reduziert
          qD_test = Jinv * xD_test;
          if wn(idx_wn.ikjac_cond) && condJik > s.cond_thresh_ikjac % Kennzahl bezogen auf Jacobi-Matrix der inversen Kinematik
            Stats.mode(jj) = bitset(Stats.mode(jj),8);
            % Einfacher Differenzenquotient für Kond. der IK-Jacobi-Matrix
            % Zusätzlich Spline-Übergang für Aktivierungsschwelle
            Phi_q_test = Rob.constr3grad_q(q1+qD_test, xE_soll);
            h3_test = invkin_optimcrit_condsplineact(cond(Phi_q_test), ...
              1.5*s.cond_thresh_ikjac, s.cond_thresh_ikjac);
            h3dq = (h3_test-h(idx_hn.ikjac_cond))./qD_test';
            % Zur Fehlertoleranz (falls bei überbestimmten PKM ein Gelenk keinen Einfluss hat)
            h3dq(isnan(h3dq)) = 0; h3dq(isinf(h3dq)) = 0;
          else
            h3dq(:) = 0;
          end
          if wn(idx_wn.jac_cond) && condJ < 1e10 && condJ > s.cond_thresh_jac || ... % bezogen auf PKM-Jacobi (geht numerisch nicht in Singularität)
             wn(idx_wn.poserr_ee) ~= 0 % Positionsfehler ähnlich wie Jacobi-Singularität zu rechnen
            Stats.mode(jj) = bitset(Stats.mode(jj),9);
            [~,Phi4_x_voll_test] = Rob.constr4grad_x(xE_1+xD_test);
            [~,Phi4_q_voll_test] = Rob.constr4grad_q(q1+qD_test);
            Jinv_test = -Phi4_q_voll_test\Phi4_x_voll_test;
            h4_test = invkin_optimcrit_condsplineact(cond(Jinv_test(I_qa,Rob_I_EE)), ...
              1.5*s.cond_thresh_jac, s.cond_thresh_jac);
            h4dq = (h4_test-h(idx_hn.jac_cond))./qD_test';
            h4dq(isnan(h4dq)) = 0; h4dq(isinf(h4dq)) = 0;
            if wn(idx_wn.poserr_ee) ~= 0
              dx_poserr_test = abs(inv(Jinv_test(I_qa,Rob_I_EE))) * s.q_poserr(I_qa);
              h11_test = norm(dx_poserr_test(Rob_I_EE(1:3)));
              h11dq = (h11_test-h(idx_hn.poserr_ee))./qD_test';
              h11dq(isnan(h11dq)) = 0; h11dq(isinf(h11dq)) = 0;
            end
            if s.debug
              % Benutze Formel für Differential des Matrix-Produkts mit 
              % Matrix-Invertierung zur Bildung des PKM-Jacobi-Inkrements
              Phi4D_x_voll = Phi4_x_voll_test-Phi4_x_voll;
              Phi4D_q_voll = Phi4_q_voll_test-Phi4_q_voll;
              Jinv_test_2 = Jinv + ...
                Phi4_q_voll\Phi4D_q_voll/Phi4_q_voll*Phi4_x_voll + ...
                -Phi4_q_voll\Phi4D_x_voll;
              h4_test_2 = cond(Jinv_test_2(I_qa,Rob_I_EE));
              if abs(h4_test_2 - h4_test)/h4_test > 1e-6 && abs(h4_test_2-condJ) < 1
                error('Beide Ansaetze fuer Delta Jinv stimmen nicht ueberein');
              end
            end
          else
            h4dq(:) = 0;
          end
          if s.debug
            h3dq_var1 = h3dq;
            h4dq_var1 = h4dq;
          end
        end % kein if-else für Debug
        if ~all(abs(Phi)<1e-6) && taskred_rotsym || s.debug % Variante 2
          % Bestimme Nullraumbewegung durch Differenzenquotient für jede
          % Gelenkkoordinate.
          if wn(idx_wn.ikjac_cond) && condJik > s.cond_thresh_ikjac % Kennzahl bezogen auf Jacobi-Matrix der inversen Kinematik
            Stats.mode(jj) = bitset(Stats.mode(jj),8);
            h3dq_all = NaN(NJ,NJ);
            for kkk = 1:NJ % Alternativ: find(I_qD_test_all)'
              if ~I_qD_test_all(kkk), continue; end % zur Vereinheitlichung mit Mex-Funktion
              q_test = q1 + qD_test_all(:,kkk); % Test-Konfiguration mit Inkrement
              Jik_kkk=Rob.constr3grad_q(q_test, xE_soll); % Berechnung identisch mit oben
              h3_kkk = invkin_optimcrit_condsplineact(cond(Jik_kkk), ...
                1.5*s.cond_thresh_ikjac, s.cond_thresh_ikjac);
              % Skaliere mit Gelenkzahl des Roboters (TODO: Mathematische
              % Begründung fehlt. Empirisch ermittelt. Ziel: var1=var2).
              h3dq_all(kkk,:) = NJ * (h3_kkk-h(idx_hn.ikjac_cond))./qD_test_all(:,kkk)';
            end
            h3dq = mean(h3dq_all(I_qD_test_all,:),1);
          end
          if wn(idx_wn.jac_cond) && condJ < 1e10 && condJ > s.cond_thresh_jac || ...% bezogen auf PKM-Jacobi (geht numerisch nicht in Singularität)
             wn(idx_wn.poserr_ee) ~= 0
            Stats.mode(jj) = bitset(Stats.mode(jj),9);
            h4dq_all = NaN(NJ,NJ);
            h11dq_all = NaN(NJ,NJ);
            for kkk = 1:NJ
              if ~I_qD_test_all(kkk), continue; end
              % Nullraumbewegung mit Inkrement. Inkrementelle Bewegung 
              % eines Gelenks erzeugt später Bewegung aller Gelenke.
              % Dadurch wird auch nur die phiz-Rotation durchgeführt. Hier
              % erfolgt eine Mittelwertbildung über alle Gelenke, da die
              % kinematischen Ketten nicht geschlossen sind
              q_test = q1 + qD_test_all(:,kkk); % Test-Konfiguration mit Inkrement
              % Geometrische Matrizen mit Inkrement bestimmen. Auch für
              % Phi_dx. 
              [~, Phi4_q_voll_kkk] = Rob.constr4grad_q(q_test);
              % Inkrement für erste Beinkette. Müsste egal sein, da Null-
              % raumprojektion durchgeführt wurde, auch wenn qD_test einer
              % anderen Beinkette angehört
              xD_test = zeros(6,1); % 3T3R-Formulierung. Eintragung z.B. bei 2T1R nur relevante FG
              xD_test(Rob_I_EE) = Jinv(1:Rob.I2J_LEG(1),Rob_I_EE) \ qD_test_all(1:Rob.I2J_LEG(1),kkk);
              xE_test = xE_1 + xD_test;
              [~, Phi4_x_voll_kkk] = Rob.constr4grad_x(xE_test);
              Jinv_kkk = -Phi4_q_voll_kkk\Phi4_x_voll_kkk;
              h4_kkk = invkin_optimcrit_condsplineact(cond(Jinv_kkk(I_qa,Rob_I_EE)), ...
                1.5*s.cond_thresh_jac, s.cond_thresh_jac);
              h4dq_all(kkk,:) = (h4_kkk-h(idx_hn.jac_cond))./qD_test_all(:,kkk)';
              if wn(idx_wn.poserr_ee) ~= 0
                dx_poserr_kkk = abs(inv(Jinv_kkk(I_qa,Rob_I_EE))) * s.q_poserr(I_qa);
                h11_kkk = norm(dx_poserr_kkk(Rob_I_EE(1:3)));
                h11dq_all(kkk,:) = (h11_kkk-h(idx_hn.poserr_ee))./qD_test_all(:,kkk)';
              end
            end
            h4dq = mean(h4dq_all(I_qD_test_all,:),1);
            h11dq = mean(h11dq_all(I_qD_test_all,:),1);
          end
          if s.debug
            h3dq_var2 = h3dq;
            h4dq_var2 = h4dq;
          end
        end
        if s.debug && taskred_rotsym
          % Prüfe, ob die Berechnung des Gradienten mit beiden Ansätzen gleich ist.
          qN3_var1 = N * h3dq_var1';
          qN3_var2 = N * h3dq_var2';
          test_qN3 = qN3_var1 ./ qN3_var2;
          qN4_var1 = N * h4dq_var1';
          qN4_var2 = N * h4dq_var2';
          test_qN4 = qN4_var1 ./ qN4_var2;
          if any( abs(qN3_var1)>1e-4 & abs(test_qN3-1)>0.1 ) || ...
              any(abs(qN4_var1)>1e-4 & abs(test_qN4-1)>0.1 ) || ...  
              any(abs(qN3_var1)>1e-4) && all(sign(qN3_var1)==-sign(qN3_var2)) || ...
              any(abs(qN4_var1)>1e-4) && all(sign(qN4_var1)==-sign(qN4_var2))
            error(['jj=%d. Beide Varianten der Gradientenbildung fuer Kriterium 3 ', ...
              'oder 4 (Sing.) stimmen nicht ueberein. Phi=%1.1e. ratio h3: %1.1e...%1.1e, ratio h4: %1.1e...%1.1e'], ...
                jj, max(abs(Phi)), min(test_qN3), max(test_qN3), min(test_qN4), max(test_qN4));
          end
        end
        if wn(idx_wn.ikjac_cond), v = v - wn(idx_wn.ikjac_cond)*h3dq'; end
        if wn(idx_wn.jac_cond), v = v - wn(idx_wn.jac_cond)*h4dq'; end
        if wn(idx_wn.poserr_ee), v = v - wn(idx_wn.poserr_ee)*h11dq'; end
      end
      %% Kollisionsvermeidung
      if wn(idx_wn.coll_hyp) || wn(idx_wn.coll_par) % Kollisionsprüfung
        % Direkte Kinematik aller Beinketten (Datenformat für Kollision)
        [~, JP] = Rob.fkine_coll(q1);
        % Kollisionserkennung im vergrößerten Warnbereich
        colldet_warn = false;
        colldet = true(1,size(Rob.collchecks,1));
        if wn(idx_wn.coll_hyp)
          colldet = check_collisionset_simplegeom_mex(collbodies_ns, Rob.collchecks, ...
            JP, struct('collsearch', true));
          if any(colldet)
            colldet_warn = true;
          end
        end
        if wn(idx_wn.coll_par) && ~colldet_warn
          % Prüfe im Folgenden Schritt alle Kollisionen
          colldet(:) = true;
        end
        if wn(idx_wn.coll_hyp) && colldet_warn || wn(idx_wn.coll_par)
          Stats.mode(jj) = bitset(Stats.mode(jj),10);
          % Zwei verschiedene Arten zur Berechnung der Nullraumbewegung, je
          % nachdem, ob die Beinketten schon geschlossen sind, oder nicht.
          % Siehe Berechnung für vorheriges Kriterium
          if all(abs(Phi)<1e-6) && taskred_rotsym || s.debug % Variante 1
            Stats.mode(jj) = bitset(Stats.mode(jj),16);
            % Bestimme Nullraumbewegung durch Differenzenquotient für die 
            % redundante Koordinate. Dadurch nur eine neue Funktions- 
            % auswertung
            xD_test_3T3R = [zeros(5,1);1e-6];
            xD_test = xD_test_3T3R; % Hier werden für 2T1R nicht die Koordinaten reduziert
            qD_test = Jinv * xD_test;
            JP_test = [JP; NaN(1, size(JP,2))];
            [~, JP_test(2,:)] = Rob.fkine_coll(q1+qD_test);
            % Kollisionsprüfung für alle Gelenkpositionen auf einmal. Prüfe
            % nur die Fälle, bei denen die vergrößerten Objekte bereits eine
            % Kollision angezeigt haben.
            [~, colldist_test] = check_collisionset_simplegeom_mex( ...
              Rob.collbodies, Rob.collchecks(colldet,:), JP_test, struct('collsearch', false));
            % Prüfe, welche Kollisionsprüfungen durch die Gelenkbewegung
            % beeinflusst werden
            I_collcheck_nochange = abs(colldist_test(1,:)-colldist_test(2,:)) < 1e-12;
            % Benutze nur die zur Bildung des Gradienten
            % Kollisions-Kriterium berechnen: Tiefste Eindringtiefe (positiv)
            % Falls keine Kollision vorliegt (mit den kleineren
            % Kollisionskörpern), dann Abstände negativ angeben.
            if all(I_collcheck_nochange) % Keine Kollision nennenswert geändert
              mincolldist_test = repmat(colldist_test(1), 2, 1);
            else
              mincolldist_test = min(colldist_test(:,~I_collcheck_nochange),[],2); % Schlimmste Kollision für jeden Körper bestimmen
            end
            I_collcheck_nochange_all( colldet) = I_collcheck_nochange;
            I_collcheck_nochange_all(~colldet) = false;
            if colldet_warn % nur im Warnbereich aktiv
              h(idx_hn.coll_hyp) = invkin_optimcrit_limits3(-mincolldist_test(1), ... % zurückgegebene Distanz ist zuerst negativ
                [-5*collobjdist_thresh, 0], -collobjdist_thresh);
            else
              h(idx_hn.coll_hyp) = 0;
            end
            if wn(idx_wn.coll_hyp) ~= 0 % hyperbolisches Kriterium
              if h(idx_hn.coll_hyp) == 0 || ... % nichts tun. Noch im Toleranzbereich
                all(I_collcheck_nochange) % Gradient nicht bestimmbar
                h5dq(:) = 0;
              elseif ~isinf(h(idx_hn.coll_hyp))
                h5_test = invkin_optimcrit_limits3(-mincolldist_test(2), ... % zurückgegebene Distanz ist zuerst negativ
                  [-5*collobjdist_thresh, 0], -collobjdist_thresh);
                % Einfacher Differenzenquotient
                h5dq = (h5_test-h(idx_hn.coll_hyp))./qD_test';
              else % Kollision so groß, dass Wert inf ist. Dann kein Gradient aus h bestimmbar.
                % Indirekte Bestimmung über die betragsmäßige Verkleinerung der (negativen) Eindringtiefe
                h5dq = 1e3*(-mincolldist_test(2)-(-mincolldist_test(1)))./qD_test';
                currcolldepth = -mincolldist_test(1);
                if all(abs(Phi) < 1e-3)
                  bestcolldepth = min(bestcolldepth,currcolldepth);
                end
              end
              h5dq(isnan(h5dq)) = 0;
            end
            h(idx_hn.coll_par) = invkin_optimcrit_limits1(-mincolldist_test(1), ...
              [-10*maxcolldepth, 0]); % immer aktiv
            if wn(idx_wn.coll_par) && ~all(I_collcheck_nochange) % quadratisches Kriterium
              h9_test = invkin_optimcrit_limits1(-mincolldist_test(2), ... % zurückgegebene Distanz ist zuerst negativ
                [-10*maxcolldepth, 0]);
              % Einfacher Differenzenquotient
              h9dq = (h9_test-h(idx_hn.coll_par))./qD_test';
              h9dq(isnan(h9dq)) = 0;
            else
              h9dq(:) = 0;
            end
            if s.debug
              h5dq_var1 = h5dq;
              h9dq_var1 = h9dq;
              I_collcheck_nochange_var1 = I_collcheck_nochange;
            end
          end % Kein if-else, sondern neues if mit inverser Bedingung (wegen Debug)
          if ~(all(abs(Phi)<1e-6) && taskred_rotsym) || s.debug % Variante 2
            % Bestimme Nullraumbewegung durch Differenzenquotient für jede
            % Gelenkkoordinate.
            JP_test = [JP; NaN(NJ, size(JP,2))];
            for kkk = 1:NJ
              if ~I_qD_test_all(kkk), continue; end
              q_test = q1 + qD_test_all(:,kkk); % Test-Konfiguration mit Inkrement
              % Bestimme die Veränderung der Gelenkpositionen
              [~, JP_test(1+kkk,:)] = Rob.fkine_coll(q_test);
            end
            % Kollisionsprüfung für alle Gelenkpositionen auf einmal.
            [~, colldist_test] = check_collisionset_simplegeom_mex( ...
              Rob.collbodies, Rob.collchecks(colldet,:), JP_test, struct('collsearch', false));
            % Prüfe, welche Kollisionsprüfungen sich nicht verändern.
            % Bestimme die Änderung für jedes einzelne Inkrement
            cdtdiff=repmat(colldist_test(1,:),NJ,1)-colldist_test(2:end,:);
            % Benutze die Skalierungen, damit ein absolut großes Inkrement
            % nicht numerisches Rauschen verstärkt und den Schwellwert
            % damit überschreitet.
            I_collcheck_nochange_voll2 = abs(cdtdiff) < 1e-12*repmat(1./scale_qD_test_all, 1, size(cdtdiff,2));
            % Berücksichtige nur Kollisionsprüfungen, die jetzt von min.
            % einem Gelenk beeinflusst werden.
            I_collcheck_nochange = any(I_collcheck_nochange_voll2);
            if all(I_collcheck_nochange) % Keine Kollision nennenswert geändert
              % Setze alle auf exakt den gleichen Wert. Dann Gradient Null.
              mincolldist_test = repmat(colldist_test(1), size(JP_test,1),1);
            else
              mincolldist_test = min(colldist_test(:,~I_collcheck_nochange),[],2);
            end
            I_collcheck_nochange_all( colldet) = I_collcheck_nochange;
            I_collcheck_nochange_all(~colldet) = false;
            if colldet_warn
              h(idx_hn.coll_hyp) = invkin_optimcrit_limits3(-mincolldist_test(1), ... % zurückgegebene Distanz ist zuerst negativ
                [-5*collobjdist_thresh, 0], -collobjdist_thresh);
            else
              h(idx_hn.coll_hyp) = 0;
            end
            if wn(idx_wn.coll_hyp) % Hyperbolisches Kriterium
              if h(idx_hn.coll_hyp) == 0 || ... % nichts tun. Noch im Toleranzbereich
                all(I_collcheck_nochange)
                h5dq(:) = 0;
              elseif ~isinf(h(idx_hn.coll_hyp))
                % Differenzenquotient für jedes Inkrement getrennt und dann Mittelwert.
                % Damit stimmt das Ergebnis mit Variante 1 überein. Ohne nicht.
                h5dq_all = NaN(NJ,NJ);
                for kkk = 1:NJ
                  if ~I_qD_test_all(kkk), continue; end
                  h5_test = invkin_optimcrit_limits3(-mincolldist_test(1+kkk), ... % zurückgegebene Distanz ist zuerst negativ
                    [-5*collobjdist_thresh, 0], -collobjdist_thresh);
                  h5dq_all(kkk,:) = (h5_test-h(idx_hn.coll_hyp))./(qD_test_all(:,kkk)');
                end
                h5dq = mean(h5dq_all(I_qD_test_all,:),1);
              else % Kollision so groß, dass Wert inf ist. Dann kein Gradient aus h bestimmbar.
                % Indirekte Bestimmung über die betragsmäßige Verkleinerung der (negativen) Eindringtiefe
                h5dq_all = NaN(NJ,NJ);
                for kkk = 1:NJ
                  if ~I_qD_test_all(kkk), continue; end
                  % Muss stetiges Kriterium sein, damit es nicht zu Dauer-
                  % schwingungen kommt. Wähle relativ großen Wert als Faktor
                  h5dq_all(kkk,:) = 1e3*(-mincolldist_test(1+kkk)-(-mincolldist_test(1)))./(qD_test_all(:,kkk)');
                end
                h5dq = mean(h5dq_all(I_qD_test_all,:),1);
                currcolldepth = -mincolldist_test(1);
                if all(abs(Phi) < 1e-3)
                  bestcolldepth = min(bestcolldepth,currcolldepth);
                end
              end
            end
            h(idx_hn.coll_par) = invkin_optimcrit_limits1(-mincolldist_test(1), ...
              [-10*maxcolldepth, 0]);
            if wn(idx_wn.coll_par) && ~all(I_collcheck_nochange) % quadratisches Kriterium
              h9dq_all = NaN(NJ,NJ);
              for kkk = 1:NJ
                if ~I_qD_test_all(kkk), continue; end
                h9_test = invkin_optimcrit_limits1(-mincolldist_test(1+kkk), ... % zurückgegebene Distanz ist zuerst negativ
                  [-10*maxcolldepth, 0]);
                h9dq_all(kkk,:) = (h9_test-h(idx_hn.coll_par))./(qD_test_all(:,kkk)');
              end
              h9dq = mean(h9dq_all(I_qD_test_all,:),1);
            else
              h9dq(:) = 0;
            end
            if s.debug
              I_collcheck_nochange_var2 = I_collcheck_nochange;
              h5dq_var2 = h5dq;
              h9dq_var2 = h9dq;
            end
          end
          if nargout >= 4
            Stats.maxcolldepth(jj,:) = [-min(colldist_test(1,:)),-mincolldist_test(1)];
          end
          if s.debug && all(abs(Phi)<1e-6) && taskred_rotsym && ...
              ~all(I_collcheck_nochange_var1==I_collcheck_nochange_var2) % Wenn beide ungleich sind, ist eine Abweichung zu erwarten
            % Prüfe, ob die Berechnung des Gradienten mit beiden Ansätzen gleich ist.
            qN5_var1 = N * h5dq_var1';
            qN5_var2 = N * h5dq_var2';
            test_qN5 = qN5_var1 ./ qN5_var2;
            qN9_var1 = N * h9dq_var1';
            qN9_var2 = N * h9dq_var2';
            test_qN9 = qN9_var1 ./ qN9_var2;
            if any( abs(qN5_var1)>1e-4 & abs(test_qN5-1)>0.1 ) || ...
                any(abs(qN9_var1)>1e-4 & abs(test_qN9-1)>0.1 ) || ...  
                any(abs(qN5_var1)>1e-4) && all(sign(qN5_var1)==-sign(qN5_var2)) || ...
                any(abs(qN9_var1)>1e-4) && all(sign(qN9_var1)==-sign(qN9_var2))
              error(['jj=%d. Beide Varianten der Gradientenbildung fuer Kriterium 5 ', ...
                'oder 9 (Koll.) stimmen nicht ueberein. Phi=%1.1e. ratio h5: %1.1e...%1.1e, ratio h9: %1.1e...%1.1e'], ...
                jj, max(abs(Phi)), min(test_qN5(1)), max(test_qN5(1)), min(test_qN9(1)), max(test_qN9(1)));
            end
          end
          if wn(idx_wn.coll_hyp)
            v = v - wn(idx_wn.coll_hyp)*h5dq';
          end
          if wn(idx_wn.coll_par)
            v = v - wn(idx_wn.coll_par)*h9dq';
          end
        end
      end
      %% Einhaltung der Bauraum-Grenzen
      h([idx_hn.instspc_par, idx_hn.instspc_hyp]) = 0;
      if wn(idx_wn.instspc_hyp) || wn(idx_wn.instspc_par) % Bauraumprüfung
        Stats.mode(jj) = bitset(Stats.mode(jj),11);
        % Direkte Kinematik aller Beinketten (Datenformat für Kollision)
        [~, JP] = Rob.fkine_coll(q1);
        % Zwei verschiedene Arten zur Berechnung der Nullraumbewegung, je
        % nachdem, ob die Beinketten schon geschlossen sind, oder nicht.
        % Siehe Berechnung für vorheriges Kriterium
        if all(abs(Phi)<1e-6) && taskred_rotsym || s.debug % Variante 1
          Stats.mode(jj) = bitset(Stats.mode(jj),16);
          % Bestimme Nullraumbewegung durch Differenzenquotient für die 
          % redundante Koordinate. Dadurch nur eine neue Funktions- 
          % auswertung
          xD_test_3T3R = [zeros(5,1);1e-6];
          xD_test = xD_test_3T3R; % Hier werden für 2T1R nicht die Koordinaten reduziert
          qD_test = Jinv * xD_test;
          JP_test = [JP; NaN(1, size(JP,2))];
          [~, JP_test(2,:)] = Rob.fkine_coll(q1+qD_test);
          % Bauraumprüfung für alle Gelenkpositionen auf einmal
          [~, absdist] = check_collisionset_simplegeom_mex(Rob.collbodies_instspc, ...
            Rob.collchecks_instspc, JP_test, struct('collsearch', false));
          I_instspccheck_nochange = abs(absdist(1,:)-absdist(2,:)) < 1e-12;
          if all(I_instspccheck_nochange) % Keine Bauraumprüfung nennenswert geändert
            % Setze alle auf exakt den gleichen Wert. Dann Gradient Null.
            mindist_h_all = repmat(absdist(1), 2, 1);
          else
            % Prüfe, ob alle beweglichen Kollisionsobjekte in mindestens einem
            % Bauraumkörper enthalten sind (falls Prüfung gefordert)
            mindist_h_all = -inf(size(JP_test,1),1);
            for i = 1:size(Rob.collbodies_instspc.link,1)
              % Indizes aller Kollisionsprüfungen mit diesem (Roboter-)Objekt i
              I = Rob.collchecks_instspc(:,1) == i & ... % erste Spalte für Roboter-Obj.
                ~I_instspccheck_nochange'; % Nur solche Objektprüfungen berücksichtigen, die hier beeinflusst werden
              if ~any(I), continue; end % Bauraum-Objekte nicht direkt prüfen. Sonst leeres Array
              % Falls mehrere Bauraum-Objekte, nehme das mit dem besten Wert
              mindist_i = min(absdist(:,I),[],2);
              % Nehme den schlechtesten Wert von allen Objekten
              mindist_h_all = max([mindist_i,mindist_h_all],[],2);
            end
          end
          % Bauraum-Kriterium berechnen: Negativer Wert ist im Bauraum (gut),
          % positiver ist außerhalb (schlecht). Größter positiver Wert
          % maßgeblich
          if wn(idx_wn.instspc_hyp)
            h(idx_hn.instspc_hyp) = invkin_optimcrit_limits3(mindist_h_all(1), ... % Wert bezogen auf aktuelle Pose
              [-100.0, 0], ... % obere Grenze: Bei Schwelle zur Bauraumverletzung ist Wert inf
              -s.installspace_thresh); % obere Grenze: z.B. ab 100mm Nähe zum Rand Kriterium aktiv
            h6_test = invkin_optimcrit_limits3(mindist_h_all(2), ... % Wert bezogen auf Test-Pose
              [-100.0, 0], -s.installspace_thresh);
            if h(idx_hn.instspc_hyp) == 0 || ...% nichts unternehmen (im Bauraum, mit Sicherheitsabstand)
                all(I_instspccheck_nochange)
              h6dq(:) = 0;
            elseif ~isinf(h(idx_hn.instspc_hyp))
              h6dq = (h6_test-h(idx_hn.instspc_hyp))./qD_test';
            else % Verletzung so groß, dass Wert inf ist. Dann kein Gradient bestimmbar.
              % Indirekte Bestimmung über die Verkleinerung des (positiven) Abstands
              h6dq = 1e3*(mindist_h_all(2)-mindist_h_all(1))./qD_test';
              currinstspcdist = mindist_h_all(1);
              if all(abs(Phi) < 1e-3)
                bestinstspcdist = min(bestinstspcdist,currinstspcdist);
              end
            end
            h6dq(isnan(h6dq)) = 0; % Falls ein qD_test exakt Null ist
            if s.debug
              h6dq_var1 = h6dq;
            end
          end
          if wn(idx_wn.instspc_par)
            h(idx_hn.instspc_par) = invkin_optimcrit_limits1(mindist_h_all(1), ...
              [-100.0, 0]);
            h10_test = invkin_optimcrit_limits1(mindist_h_all(2), [-100, 0]);
            h10dq = (h10_test-h(idx_hn.instspc_par))./qD_test';
            h10dq(isnan(h6dq)) = 0; % Falls ein qD_test exakt Null ist
            if s.debug
              h10dq_var1 = h10dq;
            end
          end
        end % Kein if-else, sondern neues if mit inverser Bedingung (wegen Debug)
        if ~all(abs(Phi)<1e-6) && taskred_rotsym || s.debug % Variante 2
          % Bestimme Nullraumbewegung durch Differenzenquotient für jede
          % Gelenkkoordinate.
          JP_test = [JP; NaN(NJ, size(JP,2))];
          for kkk = 1:NJ
            if ~I_qD_test_all(kkk), continue; end
            q_test = q1 + qD_test_all(:,kkk); % Test-Konfiguration mit Inkrement
            % Bestimme die Veränderung der Gelenkpositionen
            [~, JP_test(1+kkk,:)] = Rob.fkine_coll(q_test);
          end
          % Bauraumprüfung für alle Gelenkpositionen auf einmal.
          [~, absdist] = check_collisionset_simplegeom(Rob.collbodies_instspc, ...
            Rob.collchecks_instspc, JP_test, struct('collsearch', false));
          I_instspccheck_nochange = abs(diff(minmax2(absdist')')) < 1e-12;
          if all(I_instspccheck_nochange) % Keine Bauraumprüfung nennenswert geändert
            % Setze alle auf exakt den gleichen Wert. Dann Gradient Null.
            mindist_h_all = repmat(absdist(1), size(JP_test,1),1);
          else
            mindist_h_all = -inf(size(JP_test,1),1); % gleiche Rechnung wie oben
            for i = 1:size(Rob.collbodies_instspc.link,1)
              I = Rob.collchecks_instspc(:,1) == i & ~I_instspccheck_nochange';
              if ~any(I), continue; end
              mindist_i = min(absdist(:,I),[],2);
              mindist_h_all = max([mindist_i,mindist_h_all],[],2);
            end
          end
          if wn(idx_wn.instspc_hyp)
            h(idx_hn.instspc_hyp) = invkin_optimcrit_limits3(mindist_h_all(1), ... % Wert bezogen auf aktuelle Pose
              [-100.0, 0], -s.installspace_thresh);
            if h(idx_hn.instspc_hyp) == 0 || ...% nichts unternehmen (im Bauraum, mit Sicherheitsabstand)
              all(I_instspccheck_nochange)
              h6dq(:) = 0;
            elseif ~isinf(h(idx_hn.instspc_hyp))
              h6dq_all = NaN(NJ,NJ);
              for kkk = 1:NJ
                if ~I_qD_test_all(kkk), continue; end
                h6_test = invkin_optimcrit_limits3(mindist_h_all(1+kkk), ... % Wert bezogen auf Test-Pose dieses Gelenks
                  [-100.0, 0], -s.installspace_thresh);
                h6dq_all(kkk,:) = (h6_test-h(idx_hn.instspc_hyp))./(qD_test_all(:,kkk)'); % Differenzenquotient bzgl. Inkrement
              end
              h6dq = mean(h6dq_all(I_qD_test_all,:),1);
            else % Verletzung so groß, dass Wert inf ist. Dann kein Gradient aus h bestimmbar.
              % Indirekte Bestimmung über Abstand
              h6dq_all = NaN(NJ,NJ);
              for kkk = 1:NJ
                if ~I_qD_test_all(kkk), continue; end
                % Skaliere mit Gelenkzahl des Roboters (TODO: Mathematische
                % Begründung fehlt. Empirisch ermittelt. Ziel: var1=var2).
                h6dq_all(kkk,:) = NJ * 1e3*(mindist_h_all(1+kkk)-mindist_h_all(1))./(qD_test_all(:,kkk)');
              end
              h6dq = mean(h6dq_all(I_qD_test_all,:),1);
              currinstspcdist = mindist_h_all(1);
              if all(abs(Phi) < 1e-3)
                bestinstspcdist = min(bestinstspcdist,currinstspcdist);
              end
            end
            if s.debug
              h6dq_var2 = h6dq;
            end
          end
          if wn(idx_wn.instspc_par)
            h(idx_hn.instspc_par) = invkin_optimcrit_limits1(mindist_h_all(1), [-100.0, 0]);
            h10dq_all = NaN(NJ,NJ);
            for kkk = 1:NJ
              if ~I_qD_test_all(kkk), continue; end
              h10_test = invkin_optimcrit_limits1(mindist_h_all(1+kkk), [-100.0, 0]);
              h10dq_all(kkk,:) = (h10_test-h(idx_hn.instspc_par))./(qD_test_all(:,kkk)'); % Differenzenquotient bzgl. Inkrement
            end
            h10dq = mean(h10dq_all(I_qD_test_all,:),1);
            if s.debug
              h10dq_var2 = h10dq;
            end
          end
        end
        if s.debug && taskred_rotsym
          % Prüfe, ob die Berechnung des Gradienten mit beiden Ansätzen gleich ist.
          if wn(idx_wn.instspc_hyp)
            qN6_var1 = N * h6dq_var1';
            qN6_var2 = N * h6dq_var2';
            test_qN6 = qN6_var1 ./ qN6_var2;
            if any(abs(qN6_var1)>1e-4) && (any(abs(test_qN6-1)>0.1) || ...  % max. 10% Abweichung
                all(sign(qN6_var1)==-sign(qN6_var2))) % Bewegung darf nicht entgegengesetzt sein.
              error(['jj=%d. Beide Varianten der Gradientenbildung fuer Kriterium 6 ', ...
                '(Bauraum) stimmen nicht ueberein. Phi=%1.1e. ratio h6: %1.1e...%1.1e'], ...
                  jj, max(abs(Phi)), min(test_qN6), max(test_qN6));
            end
          end
          if wn(idx_wn.instspc_par)
            qN10_var1 = N * h10dq_var1';
            qN10_var2 = N * h10dq_var2';
            test_qN10 = qN10_var1 ./ qN10_var2;
            if any(abs(qN10_var1)>1e-4) && (any(abs(test_qN10-1)>0.1) || ...  % max. 10% Abweichung
                all(sign(qN10_var1)==-sign(qN10_var2))) % Bewegung darf nicht entgegengesetzt sein.
              error(['jj=%d. Beide Varianten der Gradientenbildung fuer Kriterium 10 ', ...
                '(par. Bauraum) stimmen nicht ueberein. Phi=%1.1e. ratio h10: %1.1e...%1.1e'], ...
                  jj, max(abs(Phi)), min(test_qN10), max(test_qN10));
            end
          end
        end
        if nargout >= 4
          % Bestimme auch den Abstand ohne Berücksichtigung der
          % beeinflussbaren Prüfungen
          mindist_all = -inf; % ähnliche Rechnung wie oben
          for i = 1:size(Rob.collbodies_instspc.link,1)
            I = Rob.collchecks_instspc(:,1) == i;
            if ~any(I), continue; end
            mindist_i = min(absdist(1,I));
            mindist_all = max([mindist_i,mindist_all]);
          end
          Stats.instspc_mindst(jj,:) = [mindist_all, mindist_h_all(1)];
        end
        if wn(idx_wn.instspc_hyp)
          v = v - wn(idx_wn.instspc_hyp)*h6dq';
        end
        if wn(idx_wn.instspc_par)
          v = v - wn(idx_wn.instspc_par)*h10dq';
        end
      end
      %% Einhaltung der Grenzen der redundanten Koordinate
      if wn(idx_wn.xlim_par) ~= 0 || wn(idx_wn.xlim_hyp) ~= 0 % Jacobi-Matrizen für wn(idx_wn.xlim_par) und/oder wn(idx_wn.xlim_hyp)
        xD_test_limred = [zeros(5,1);1e-6];
        qD_test = Jinv * xD_test_limred;
      end
      if wn(idx_wn.xlim_par) ~= 0 && all(abs(Phi)<1e-6) % quadratische Limitierung der redundanten Koordinate (3T2R)
        Stats.mode(jj) = bitset(Stats.mode(jj),12);
        Stats.mode(jj) = bitset(Stats.mode(jj),16);
        h(idx_hn.xlim_par) = invkin_optimcrit_limits1(Phi_voll(4), s.xlim(6,1:2));
        h7_test = invkin_optimcrit_limits1(Phi_voll(4)+1e-6, s.xlim(6,1:2));
        h7dq = (h7_test-h(idx_hn.xlim_par))./qD_test'; % Siehe [SchapplerOrt2021], Gl. 28
        h7dq(isnan(h7dq)) = 0;
        v = v - wn(idx_wn.xlim_par)*h7dq';
      end
      if wn(idx_wn.xlim_hyp) ~= 0 && all(abs(Phi)<1e-6) % hyperbolische Limitierung der redundanten Koordinate (3T2R)
        Stats.mode(jj) = bitset(Stats.mode(jj),13);
        Stats.mode(jj) = bitset(Stats.mode(jj),16);
        h(idx_hn.xlim_hyp) = invkin_optimcrit_limits2(Phi_voll(4), s.xlim(6,1:2), xlim_thr_h8(6,:));
        h8_test = invkin_optimcrit_limits2(Phi_voll(4)+1e-6, s.xlim(6,1:2), xlim_thr_h8(6,:));
        if isinf(h(idx_hn.xlim_hyp)) || isinf(h8_test)
          if Phi_voll(4) <= s.xlim(6,1) + 1e-6
            h8dq = -1e6*qD_test';
          elseif Phi_voll(4) >= s.xlim(6,2) - 1e-6
            h8dq = +1e6*qD_test';
          else
            error('Fall sollte eigentlich nicht vorkommen');
          end
          % Mache das Kriterium stärker als die bisher aufsummierten Kriterien.
          % Annahme: xlim_hyp kommt als letztes und muss dominieren, wenn verletzt.
          if any(v ~= 0) % nur notwendig, falls es andere Kriterien gibt
            h8dq = h8dq / min(abs(h8dq'./v));
          end
        else
          h8dq = (h8_test-h(idx_hn.xlim_hyp))./qD_test'; % Siehe [SchapplerOrt2021], Gl. 28
        end
        h8dq(isnan(h8dq)) = 0;
        v = v - wn(idx_wn.xlim_hyp)*h8dq';
      end
      % Begrenze die Werte zur Vermeidung numerischer Probleme
       % unendlich kann nicht normiert werden. Begrenze auf Maximalwert
       % (ohne Rücksicht auf Relation)
      v(isinf(v)) = sign(v(isinf(v)))*1e8;
      if any(abs(v)>1e8)
        Stats.mode(jj) = bitset(Stats.mode(jj),15);
        v = v* 1e8/max(abs(v));
      end
      delta_q_N = N * v;
    end

    % Reduziere Schrittweite auf einen absoluten Wert. Annahme: Newton-
    % Raphson-Verfahren basiert auf Linearisierung. Kleinwinkelnäherung
    % wird verlassen, wenn Gelenkwinkel mehr als 3° drehen. Hier keine
    % Betrachtung der Summe, wie bei seriellen Robotern.
    % Führe das getrennt für delta_q_T und delta_q_N durch, damit die 
    % Nullraumbewegung nicht die Aufgabenbewegung dominieren kann.
    delta_q_T = K.*delta_q_T;
    delta_q_N = Kn.*delta_q_N;
    abs_delta_qTrev = abs(delta_q_T(sigma_PKM==0)); % nur Drehgelenke
    if any(abs_delta_qTrev > 0.5) % 0.5rad=30°
      Stats.mode(jj) = bitset(Stats.mode(jj),17);
      % Reduziere das Gelenk-Inkrement so, dass das betragsgrößte
      % Winkelinkrement danach 30° hat.
      delta_q_T = delta_q_T .* 0.5/max(abs_delta_qTrev);
    end
    abs_delta_qNrev = abs(delta_q_N(sigma_PKM==0)); % nur Drehgelenke
    if any(abs_delta_qNrev > 0.05*(1-jj/n_max)) % 0.05rad=3°
      Stats.mode(jj) = bitset(Stats.mode(jj),18);
      % Reduziere das Gelenk-Inkrement so, dass das betragsgrößte
      % Winkelinkrement danach 3° hat.
      % Verkleinere die Schritte mit fortlaufenden Iterationen, um even-
      % tuellen Oszillationen auszugleichen.
      delta_q_N = delta_q_N .* 0.05*(1-jj/n_max)/max(abs_delta_qNrev);
    end
    
    % Reduziere die einzelnen Komponenten bezüglich der Winkelgrenzen
    % Bei nur gemeinsamer Reduzierung kann die Nullraumbewegung zu groß
    % werden; Dokumentation siehe unten
    if limits_set && ~isnan(maxrelstep)
      abs_delta_q_T_rel = abs(delta_q_T ./ delta_qlim);
      if any(abs_delta_q_T_rel > maxrelstep)
        Stats.mode(jj) = bitset(Stats.mode(jj),19);
        delta_q_T = delta_q_T .* maxrelstep / max(abs_delta_q_T_rel);
      end
    end
    if nsoptim && limits_set && ~isnan(maxrelstep_ns)
      abs_delta_q_N_rel = abs(delta_q_N ./ delta_qlim);
      if any(abs_delta_q_N_rel > maxrelstep_ns)
        Stats.mode(jj) = bitset(Stats.mode(jj),20);
        delta_q_N = delta_q_N .* maxrelstep_ns / max(abs_delta_q_N_rel);
      end
    end
    % Dämpfung der Nullraumbewegung. Verlangsamt die Konvergenz, reduziert
    % dafür Schwingungen zwischen delta_q_T und delta_q_N, die durch keine
    % der anderen Stabilisierungsmaßnahmen erfasst werden. Nur machen, wenn
    % auch Bewegung im Nullraum (nicht wenn q_N deaktiviert wurde)
    if nsoptim && damping_active && any(delta_q_N)
      Stats.mode(jj) = bitset(Stats.mode(jj),21);
      % Korrigiere den Altwert, da bezogen auf andere Gelenkkonfig.
      delta_q_N_alt_N = N*delta_q_N_alt;
      % Benutze diskretes PT1-Filter mit T=2 (Schritte der IK) und K=1
      delta_q_N = delta_q_N_alt_N + 1/(1+2)*(1*delta_q_N-delta_q_N_alt_N);
      delta_q_N_alt = delta_q_N;
    end
    
    % Prüfe ob mit dem Inkrement eine Kollision erzeugt wird und reduziere
    % die Bewegung darauf hin. Ist nur bezogen auf die Aufgabe, da die
    % Nullraumbewegung keine Kollision erzeugt (falls das Kriterium benutzt wird)
    if scale_coll
      % Bestimme Abstand der Objekte vor und nach dem Schritt
      [~, JP_pre] = Rob.fkine_coll(q1);
      [~, JP_post] = Rob.fkine_coll(q1+delta_q_T);
      [colldet_pp,colldist_pp] = check_collisionset_simplegeom_mex(Rob.collbodies, ...
        Rob.collchecks, [JP_pre;JP_post], struct('collsearch', false));
      % Prüfe, ob eine Kollision passieren würde und berechne die
      % Reduktion des Schrittes, die notwendig ist, um das zu verhindern
      if any(colldet_pp(2,:)) % Mit dem geplanten Schritt tritt eine Kollision auf
        mindist_pre = min(colldist_pp(1,:));
        mindist_post = min(colldist_pp(2,:));
        if mindist_pre ~= mindist_post % Nur sinnvoll, wenn Gelenkinkrement die Kollisions-Kennzahl ändert. Sonst entweder Inkrement Null oder Kollisionskörper passen nicht hierzu.
          Stats.mode(jj) = bitset(Stats.mode(jj),22);
          % Mit `scale` wird genau die Grenze zur Kollision erreicht
          % (Wert 0; in linearer Näherung)
          scale = (0-mindist_pre)/(mindist_post-mindist_pre);
          % Durch `scale_coll` wird dieses Erreichen weiter nach "innen" gezogen
          delta_q_T = scale_coll * scale * delta_q_T;
        end
      end
    end
    
    % Inkrement der Gelenkwinkel; [SchapplerTapOrt2019], Gl. (43)
    % Verstärkungsfaktoren K und Kn oben bereits angewendet.
    delta_q = delta_q_T + delta_q_N;
    
    % Reduziere Schrittweite (qT+qN gemeinsam) auf einen Maximalwert
    % bezogen auf Winkelgrenzen (bzw. auf die mögliche Spannweite)
    if limits_set && ~isnan(maxrelstep)
      Stats.mode(jj) = bitset(Stats.mode(jj),23);
      % Bestimme Inkrement relativ zur Spannbreite der Grenzen
      abs_delta_q_rel = abs(delta_q ./ delta_qlim);
      if any(abs_delta_q_rel > maxrelstep)
        % Ein Element hat ein zu großes Inkrement. Normiere den
        % Inkrement-Vektor damit
        delta_q = delta_q .* maxrelstep / max(abs_delta_q_rel);
      end
    end
    % Zusätzlicher Dämpfungsterm (gegen Oszillationen insbesondere mit der
    % Nullraumbewegung). Aktiviere, sobald Oszillationen erkannt werden
    delta_Phi = Phi - Phi_alt;
    if damping_active
      Stats.mode(jj) = bitset(Stats.mode(jj),24);
      % Benutze diskretes PT1-Filter mit T=2 (Schritte der IK) und K=1
      % Zusätzlich zu obiger Filterung von delta_q_N.
      delta_q = delta_q_alt + 1/(1+2)*(1*delta_q-delta_q_alt);
    elseif all(sign(delta_q) == -sign(delta_q_alt)) || ...
        all(sign(delta_Phi) == -sign(delta_Phi_alt))
      damping_active = true; % ab jetzt aktiviert lassen.
    end
    delta_q_alt = delta_q;
    Phi_alt = Phi;
    delta_Phi_alt = delta_Phi;

    % Gelenkwinkel-Schritt anwenden
    q2 = q1 + delta_q;
    
    % Prüfe, ob die Gelenkwinkel ihre Grenzen überschreiten und reduziere
    % die Schrittweite, falls das der Fall ist; [SchapplerTapOrt2019], Gl. (47)
    scale = 1;
    if scale_lim
      delta_ul_rel = (qmax - q2)./(qmax-q1); % Überschreitung der Maximalwerte: <0
      delta_ll_rel = (-qmin + q2)./(q1-qmin); % Unterschreitung Minimalwerte: <0
      if any([delta_ul_rel;delta_ll_rel] < 0)
        Stats.mode(jj) = bitset(Stats.mode(jj),25);
        % Berechne die prozentual stärkste Überschreitung
        % und nutze diese als Skalierung für die Winkeländerung
        % Reduziere die Winkeländerung so, dass die gröte Überschreitung auf
        % das Erreichen der Grenzen reduziert wird.
        if min(delta_ul_rel)<min(delta_ll_rel)
          % Verletzung nach oben ist die größere
          [~,I_max] = min(delta_ul_rel);
          scale = (qmax(I_max)-q1(I_max))./(delta_q(I_max));
        else
          % Verletzung nach unten ist maßgeblich
          [~,I_min] = min(delta_ll_rel);
          scale = (qmin(I_min)-q1(I_min))./(delta_q(I_min));
        end
        % Mit `scale` werden die Grenzen direkt für ein Gelenk erreicht.
        % Durch `scale_lim` wird dieses Erreichen weiter nach "innen" gezogen
        delta_q = scale_lim*scale*delta_q;
        q2 = q1 + delta_q;
      end
    end

    if any(isnan(q2)) || any(isinf(q2))
      if nargout >= 4
        Stats.iter = jj;
      end
      break; % ab hier kann das Ergebnis nicht mehr besser werden wegen NaN/Inf
    end

    % Fehlermaß für aktuelle Iteration (wird auch in nächster Iteration benutzt)
    [Phi_neu, Phi_voll] = Rob.constr3(q2, xE_soll);
    % Prüfe, ob Wert klein genug ist. Bei kleinen Zahlenwerten, ist
    % numerisch teilweise keine Verbesserung möglich.
    Phi_iO = all(abs(Phi_neu(I_constr_t_red)) < Phit_tol) && ...
             all(abs(Phi_neu(I_constr_r_red)) < Phir_tol);
    if Phi_iO && any(delta_q_N) && sum(wn.*h)>=sum(wn.*h_alt) && ~any(isinf(h))
      % Prüfe, ob sich die Nebenbedingungen überhaupt noch verbessern. Wenn
      % nicht, kann auch abgebrochen werden. Variable delta_q_N dient zur
      % Ablaufsteuerung für folgende Abfragen (nicht für Ergebnis selbst).
      % Wert von unendlich führt dazu, dass sehr große Gradienten erzeugt
      % werden. Dann wird nicht abgebrochen
      delta_q_N(:) = 0;
    end
    if scale_lim && scale == 0
      % Die Bewegung wurde komplett herunterskaliert, da die Grenzen
      % überschritten wurden. Erhöhe die Nullraumbewegung zur Vermeidung
      % der Grenzen. Sonst wird die IK sowieso nicht konvergieren. Annahme:
      % Keine Singularität, die mit DLS beseitigt wird.
      wn(idx_wn.qlim_hyp) = wn(idx_wn.qlim_hyp) + 0.1;
    end
    % Prüfe, ob Schritt erfolgreich war (an dieser Stelle, da der 
    % Altwert von Phi noch verfügbar ist). Siehe [CorkeIK].
    Phi_neu_norm = norm(Phi_neu);
    Delta_Phi = Phi_neu_norm - norm(Phi); % "neu" - "alt";
    bestPhi = min([bestPhi;Phi_neu_norm]);
    if any(delta_q_N) && (Delta_Phi > 0 || Phi_neu_norm > bestPhi) && ...  % zusätzlich prüfen gegen langsamere Oszillationen
        (~any(isinf(h)) && sum(wn.*h)>=sum(wn.*h_alt) || ... % Verschlechterung (bzw. keine Verbesserung) der Opt.-Kriterien
         isinf(h(idx_hn.coll_hyp)) && currcolldepth > bestcolldepth || ... % gegen langsame Oszillation für Kollisionsvermeidung aus Eindringtiefe
         isinf(h(idx_hn.instspc_hyp)) && currinstspcdist > bestinstspcdist) % das gleiche für Bauraumverletzung
      % Zusätzliches Optimierungskriterium hat sich verschlechtert und
      % gleichzeitig auch die IK-Konvergenz. Das deutet auf eine
      % Konvergenz mit Oszillationen hin. Reduziere den Betrag der
      % Nullraumbewegung. Annahme: Bewegung so groß, dass keine
      % Linearisierungsfehler (außerhalb des Nullraums).
      % Nicht bei Kriterium unendlich (separate Gradientenberechnung).
      % Dort ist das Kriterium "keine Verschlechterung" mit > statt >=.
      % Der Wert bestcolldepth/bestinstspcdist ist kein Altwert wie h_alt.
      Kn = Kn*0.8;
    end
    if Phi_iO || Delta_Phi < 0 ... % Verbesserung des Residuums
        || any(delta_q_N) && all(abs(Phi_neu)<1e-3) && Delta_Phi~=0 % Bei Nullraumbewegung auch Verschlechterung möglich, wenn noch im "guten" Bereich
      if condJik>1e4 && Delta_Phi > -Phit_tol % Singularität mit Mini-Schritten
        % Erhöhe Dämpfung um von Singularität wegzukommen. Falls das nicht
        % funktioniert, führt das immerhin zu einem relativ frühen Abbruch,
        % da lambda gegen unendlich geht und dann sich Phi nicht mehr mit
        % Minimal-Schritten numerisch verbessert.
        lambda_mult = lambda_mult*2;
      else
        % Erfolgreich. Verringere lambda bis auf Minimalwert.
        lambda_mult = max(lambda_mult/2, lambda_min); % Singularität überwunden
      end
      % Behalte Ergebnis der Iteration für weitere Iterationen.
      q1 = q2;
      h_alt = h;
      % Behalte Wert für Phi nur in diesem Fall. Dadurch wird auch die Verbesserung
      % gegenüber der Iteration messbar, bei der zuletzt eine Verschlechterung eintrat.
      Phi = Phi_neu;
      rejcount = 0;
    else
      % Kein Erfolg. Erhöhe die Dämpfung. Mache den Schritt nicht.
      % Setzt voraus, dass die Konditionszahl so schlecht ist, dass
      % oben das DLS-Verfahren benutzt wird. Ansonsten Stillstand.
      lambda_mult = lambda_mult*2;
      rejcount = rejcount + 1;
      if condJik <= condlimDLS && ~nsoptim || ... % Keine Verbesserung der Konvergenz trotz guter Konditionszahl.
       rejcount > 50 % Stillstand zu lange trotz exponentieller Erhöhung von lambda.
        % Abbruch. Keine Verbesserung mit Algorithmus möglich.
        if nargout >= 4
          Stats.iter = jj;
        end
        break;
      end
    end
    if nargout >= 4
      Stats.Q(1+jj,:) = q1;
      Stats.PHI(1+jj,:) = Phi_voll;
      Stats.h(jj,:) = [sum(wn.*h),h'];
      Stats.condJ(jj,:) = [condJik, condJ];
      Stats.lambda(jj,:) = [lambda, lambda_mult];
      Stats.rejcount(jj) = rejcount;
    end
    % Prüfe Abbruchbedingung für Einhaltung der Winkelgrenzen. Ist dies bei
    % "finish_in_limits" der Fall, muss die IK nur noch konvergiert sein.
    if break_when_in_limits && (all(q1 >= qlim(:,1)) && all(q1 <= qlim(:,2)))
      delta_q_N(:) = 0; % unvollständige Nullraumbewegung wird hiernach ignoriert.
      % hiernach dürfen die Grenzen nicht mehr verlassen werden (falls
      % noch weitere Iterationen gemacht werden)
      scale_lim = 0.7;
      % Reduziere die Nullraumbewegung ab hier ganz stark. Damit können die
      % Grenzen noch leicht nachkorrigiert werden, es dominiert aber die
      % Aufgabenbewegung.
      Kn = Kn*0.8;
    end
    % Abbruchbedingungen prüfen
    if jj >= n_min ... % Mindestzahl Iterationen erfüllt
     && Phi_iO && ... % Haupt-Bedingung ist erfüllt
     ( ~nsoptim || ...%  und keine Nebenoptimierung läuft
        nsoptim && all(abs(delta_q_N) <= maxstep_ns) ) % oder die Nullraumoptimierung läuft noch. "<=" für Fall 0.
      success = true;
      if finish_in_limits && (any(q1 < qlim(:,1)) || any(q1 > qlim(:,2)))
        % Es soll eigentlich abgebrochen werden. Die Grenzen wurden aber 
        % nicht eingehalten. Mache noch mit dem Algorithmus weiter und 
        % optimiere nur noch die Grenzen (und nicht z.B. Konditionszahl)
        finish_in_limits = false; % Modus ist damit aktiviert
        nsoptim = true;
        wn(:) = 0;
        % Nutze nur die hyperbolische Funktion des Abstands (und limred, falls vorher aktiv)
        wn(idx_wn.qlim_hyp) = 1;
        wn(idx_wn.xlim_par) = s.wn(idx_wn.xlim_par);
        wn(idx_wn.xlim_hyp) = s.wn(idx_wn.xlim_hyp);
        % Mache diese Optimierung nicht mehr zu Ende, sondern höre auf, 
        % wenn die Grenzen erreicht sind.
        break_when_in_limits = true;
        success = false; % Bereits gesetzten Wert wieder zurücknehmen
        Kn = s.Kn; % Änderungen an Standard-Werten zurücksetzen
        continue
      end
      if avoid_collision_finish
        % Eigentlich soll abgebrochen werden. Prüfe nochmal auf Kollisionen
        [~, JP] = Rob.fkine_coll(q1);
        colldet = check_collisionset_simplegeom_mex(Rob.collbodies, ...
          Rob.collchecks, JP, struct('collsearch', false));
        if any(colldet)
          wn(:) = 0; % Deaktiviere alle anderen Nebenbedingungen
          wn(idx_wn.coll_hyp) = 1e3; % Starke Kollisionsvermeidung
          avoid_collision_finish = false; % Nur einmal versuchen
          success = false; % Bereits gesetzten Wert wieder zurücknehmen
          Kn = s.Kn; % Änderungen an Standard-Werten zurücksetzen
          continue
        end
      end
      if nargout >= 4
        Stats.iter = jj;
      end
      break;
    end
    if jj == n_max-10
      % die letzten 10 Iterationen sind zum Ausgleich des Positionsfehlers (ohne Nullraum)
      nsoptim = false;
      h(:) = NaN; % sonst wird der bisherige Wert gehalten. Missverständlich.
    end
  end
  if success
    if nargout >= 4
      Stats.retry_number = rr;
    end
    if ~s.retry_on_limitviol || s.retry_on_limitviol && all(q1>=qmin) && all(q1<=qmax)
      break;
    else
      % Neuversuch, da Endergebnis nicht in den Grenzen.
      success = false; % Bereits gesetzten Wert wieder zurücknehmen
    end
  end
  % Beim vorherigen Durchlauf kein Erfolg. Generiere neue Anfangswerte
  if rr == 0 && ~isnan(s.rng_seed)
    rng(s.rng_seed); % Initialisiere Zufallszahlen, falls gewünscht
  end
  if rr < size(Q0,2) % versuche eine weitere der vorgegebenen Konfigurationen
    q0 = Q0(:,rr+1);
  else % benutze eine zufällige Konfiguration
    q0 = qmin_norm + rand(NJ,1).*(qmax_norm-qmin_norm); 
  end
end
if nargout >= 3 || nargout >= 4 && (wn(idx_wn.coll_hyp) ~= 0 || wn(idx_wn.coll_par) ~= 0)
  Tc_stack_PKM = Rob.fkine_coll(q1);
end
if nargout >= 4 % Berechne Leistungsmerkmale für letzten Schritt
  h(:)=0; % Damit kein NaN bleibt, was die Gesamtsumme NaN werden lässt.
  if wn(idx_wn.qlim_par) ~= 0, h(idx_hn.qlim_par) = invkin_optimcrit_limits1(q1, qlim); end
  if wn(idx_wn.qlim_hyp) ~= 0, h(idx_hn.qlim_hyp) = invkin_optimcrit_limits2(q1, qlim, qlim_thr_h2); end
  Jik=Rob.constr3grad_q(q1, xE_soll);
  if ~any(isnan(Jik(:))) && ~any(isinf(Jik(:)))
    condJik = cond(Jik);
  else
    condJik = NaN;
  end
  if wn(idx_wn.ikjac_cond) && condJik > s.cond_thresh_ikjac
    h(idx_hn.ikjac_cond) = invkin_optimcrit_condsplineact(condJik, ...
              1.5*s.cond_thresh_ikjac, s.cond_thresh_ikjac);
  end
  if wn(idx_wn.jac_cond) || wn(idx_wn.poserr_ee) % Bestimme PKM-Jacobi für Iterationsschritt
    % Benutze die einfachen Zwangsbedingungen, da vollständige FG.
    xE_1 = xE_soll + [zeros(5,1); Phi_voll(4)];
    [~, Phi4_x_voll] = Rob.constr4grad_x(xE_1);
    [~, Phi4_q_voll] = Rob.constr4grad_q(q1);
    Jinv = -Phi4_q_voll\Phi4_x_voll; % bezogen auf 3T3R
    Jinv_out = Jinv(:, Rob_I_EE);
    if ~any(isnan(Jinv(:))) && ~any(isinf(Jinv(:)))
      condJ = cond(Jinv(I_qa,Rob_I_EE));
    else
      condJ = NaN;
    end
    if condJ > s.cond_thresh_jac
      h(idx_hn.jac_cond) = invkin_optimcrit_condsplineact(condJ, ...
              1.5*s.cond_thresh_jac, s.cond_thresh_jac);
    end
    if wn(idx_wn.poserr_ee)
      dx_poserr = abs(inv(Jinv(I_qa,Rob_I_EE))) * s.q_poserr(I_qa);
      h(idx_hn.poserr_ee) = norm(dx_poserr(Rob_I_EE(1:3)));
    end
  end
  h([idx_hn.coll_hyp, idx_hn.coll_par]) = 0;
  if wn(idx_wn.coll_hyp) ~= 0 || wn(idx_wn.coll_par) ~= 0 % Berechnung muss genauso sein wie oben
    colldet1 = check_collisionset_simplegeom_mex(collbodies_ns, Rob.collchecks, ...
          Tc_stack_PKM(:,4)', struct('collsearch', true));
    if any(colldet1) % Jetzt Betrachtung der eigentlichen Kollisionskörper
      [colldet,colldist] = check_collisionset_simplegeom_mex(Rob.collbodies, ...
        Rob.collchecks(colldet1,:), Tc_stack_PKM(:,4)', struct('collsearch', false)); % "false" gibt auch Abstände ohne Kollision
      if all(I_collcheck_nochange_all(colldet1))
        mincolldist_h = -colldist(1);
      else
        mincolldist_h = -min(colldist(~I_collcheck_nochange_all(colldet1)));
      end
      if any(colldet)
        Stats.coll = true;
      end
      Stats.maxcolldepth(Stats.iter+1,:) = [-min(colldist), mincolldist_h];
      if wn(idx_wn.coll_hyp) ~= 0
        h(idx_hn.coll_hyp) = invkin_optimcrit_limits3(mincolldist_h, ...
                [-5*collobjdist_thresh, 0], -collobjdist_thresh);
        % Trage den Wert ein, ab dem eine Kollision vorliegt
        Stats.h_coll_thresh = invkin_optimcrit_limits3(0, ...
                  [-5*collobjdist_thresh, 0], -collobjdist_thresh);
      end
      if wn(idx_wn.coll_par) ~= 0
        h(idx_hn.coll_par) = invkin_optimcrit_limits1(mincolldist_h, ...
          [-10*maxcolldepth, 0]);
      end
    end
  end
  if wn(idx_wn.instspc_hyp) ~= 0 || wn(idx_wn.instspc_par) ~= 0 % Berechnung muss genauso sein wie oben
    [~, absdist] = check_collisionset_simplegeom_mex(Rob.collbodies_instspc, ...
      Rob.collchecks_instspc, Tc_stack_PKM(:,4)', struct('collsearch', false));
    mindist_h_all = -inf;
    for i = 1:size(Rob.collbodies_instspc.link,1)
      I = Rob.collchecks_instspc(:,1) == i & ~I_instspccheck_nochange';
      if ~any(I), continue; end
      mindist_h_i = min(absdist(1,I));
      mindist_h_all = max([mindist_h_i,mindist_h_all]);
    end
    mindist_all = -inf;
    for i = 1:size(Rob.collbodies_instspc.link,1)
      I = Rob.collchecks_instspc(:,1) == i;
      if ~any(I), continue; end
      mindist_i = min(absdist(1,I));
      mindist_all = max([mindist_i,mindist_all]);
    end
    if wn(idx_wn.instspc_hyp) ~= 0
      h(idx_hn.instspc_hyp) = invkin_optimcrit_limits3(mindist_h_all, ...
        [-100.0, 0], -s.installspace_thresh);
    end
    if wn(idx_wn.instspc_par) ~= 0
      h(idx_hn.instspc_par) = invkin_optimcrit_limits1(mindist_h_all, ...
        [-100.0, 0]);
    end
    Stats.instspc_mindst(Stats.iter+1,:) = [mindist_all,mindist_h_all(1)];
    % Trage den Wert ein, ab dem eine Bauraumverletzung vorliegt
    Stats.h_instspc_thresh = invkin_optimcrit_limits3(0, ...
      [-100.0, 0], -s.installspace_thresh);
  end
  if wn(idx_wn.xlim_par) ~= 0
    h(idx_hn.xlim_par) = invkin_optimcrit_limits1(Phi_voll(4), s.xlim(6,1:2));
  end
  if wn(idx_wn.xlim_hyp) ~= 0
    h(idx_hn.xlim_hyp) = invkin_optimcrit_limits2(Phi_voll(4), s.xlim(6,1:2), xlim_thr_h8(6,:));
  end
  Stats.h(1+Stats.iter,:) = [sum(wn.*h),h'];
  Stats.condJ(1+Stats.iter,:) = [condJik, condJ];
end
q = q1;
if s.normalize
  q(sigma_PKM==0) = normalize_angle(q(sigma_PKM==0)); % nur Winkel normalisieren
end
