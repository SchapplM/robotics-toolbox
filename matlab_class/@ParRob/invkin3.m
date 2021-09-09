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
%   * Kein Plattform-KS
% Stats
%   Struktur mit Detail-Ergebnissen für den Verlauf der Berechnung
% 
% Quelle:
% [SchapplerTapOrt2019] Schappler, M. et al.: Modeling Parallel Robot Kine-
% matics for 3T2R and 3T3R Tasks using Reciprocal Sets of Euler Angles (2019)
% [NakamuraHan1986] Y. Nakamura, H. Hanafusa: Inverse Kinematic Solutions 
% With Singularity Robustness for Robot Manipulator Control, 1986
% [CorkeIK] Peter Corke, Robotics Toolbox, ikine.m
% 
% Siehe auch: SerRob/invkin2 (bzw. robot_invkin_eulangresidual.m.template)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07/2019-06
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [q, Phi, Tc_stack_PKM, Stats] = invkin3(Rob, xE_soll, Q0, s_in)

%% Initialisierung
assert(isreal(xE_soll) && all(size(xE_soll) == [6 1]), ...
  'ParRob/invkin3: xE_soll muss 6x1 sein');
assert(isreal(Q0) && all(size(Q0,1) == [Rob.NJ]), ...
  'ParRob/invkin3: Q0 muss %dxn sein', Rob.NJ);

%% Definitionen
% Variablen zum Speichern der Zwischenergebnisse
sigma_PKM = Rob.MDH.sigma; % Marker für Dreh-/Schubgelenk
s = struct(...
  'K', ones(Rob.NJ,1), ... % Verstärkung Aufgabenbewegung
  'Kn', ones(Rob.NJ,1), ... % Verstärkung Nullraumbewegung
  'wn', zeros(6,1), ... % Gewichtung der Nebenbedingung
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
s.wn = [s.wn;zeros(6-length(s.wn),1)]; % Fülle mit Nullen auf, falls altes Eingabeformat
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
avoid_collision_finish = s.avoid_collision_finish;
break_when_in_limits = false;
success = false;

nsoptim = false;
if sum(Rob.I_EE) > sum(Rob.I_EE_Task)
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
if sum(Rob.I_EE) - sum(Rob.I_EE_Task) == 1 && ~Rob.I_EE_Task(end)
  taskred_rotsym = true;
else
  taskred_rotsym = false;
end

qlim = NaN(Rob.NJ,2);
J1 = 1;
for i = 1:Rob.NLEG
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
  qmin = -Inf(Rob.NJ,1);
  qmax =  Inf(Rob.NJ,1);
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
  repmat([-0.5, +0.5]*s.optimcrit_limits_hyp_deact,Rob.NJ,1);
I_constr_t_red = Rob.I_constr_t_red;
I_constr_r_red = Rob.I_constr_r_red;
% Variablen für Dämpfung der Inkremente
delta_q_alt = zeros(Rob.NJ,1); % Altwert für Tiefpassfilter
delta_q_N_alt = zeros(Rob.NJ,1); % Altwert für Nullraum-Tiefpassfilter
damping_active = false; % Standardmäßig noch nicht aktiviert
N = NaN(Rob.NJ,Rob.NJ); % Nullraum-Projektor
% Gradient von Nebenbedingung 3 bis 5
h3dq = zeros(1,Rob.NJ); h4dq = h3dq; h5dq = h3dq; h6dq = h3dq;
h = zeros(6,1); h_alt = inf(6,1); % Speicherung der Werte der Nebenbedingungen
% Definitionen für die Kollisionsprüfung
collbodies_ns = Rob.collbodies;
maxcolldepth = 0;
collobjdist_thresh = 0;
if scale_coll || wn(5) || avoid_collision_finish
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
q0 = Q0(:,1);
% Zählung in Rob.NL: Starrkörper der Beinketten, Gestell und Plattform. 
% Hier werden nur die Basis-KS der Beinketten und alle bewegten Körper-KS
% der Beine angegeben.
Tc_stack_PKM = NaN((Rob.NL-1+Rob.NLEG)*3,4); % siehe fkine_legs; dort aber leicht anders
rejcount = 0; % Zähler für Zurückweisung des Iterationsschrittes, siehe [CorkeIK]
scale = 1; % Skalierung des Inkrements (kann reduziert werden durch scale_lim)
condJpkm = NaN;
if nargout == 4
  Stats = struct('Q', NaN(1+n_max, Rob.NJ), 'PHI', NaN(1+n_max, 6*Rob.NLEG), ...
    'iter', n_max, 'retry_number', retry_limit, 'condJ', NaN(1+n_max,1), 'lambda', ...
    NaN(n_max,2), 'rejcount', NaN(n_max,1), 'h', NaN(1+n_max,1+6), 'coll', false, ...
    'instspc_mindst', NaN(1+n_max,1), 'h_instspc_thresh', NaN, 'h_coll_thresh', NaN);
end

%% Iterative Berechnung der inversen Kinematik
for rr = 0:retry_limit % Schleife über Neu-Anfänge der Berechnung
  q1 = q0;
  % Grad der Nicht-Erfüllung der Zwangsbedingungen (Fehler)
  [Phi, Phi_voll] = Rob.constr3(q1, xE_soll);
  if nargout == 4 % Anfangswerte eintragen
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
    condJ = cond(Jik);
    % (Optimierung der Aufgabe)
    % Benutze das Damped Least Squares Verfahren je nach Konditionszahl.
    % Bei Redundanz immer benutzen, nicht nur, falls Nullraumprojektion erfolglos.
    if condJ > condlimDLS% && (~nsoptim || nsoptim && rejcount > 0)
      % Pseudo-Inverse mit Dämpfung:
      % Passe die Dämpfung lambda im DLS-Verfahren an. Wähle die Kon-
      % ditionszahl als Kriterium, da z.B. Grenzen für Singulärwerte
      % und Manipulierbarkeit nicht bekannt sind.
      % Skalierung zwischen 0 (z.B. Grenzfall cond=60) und 1 (komplett singulär).
      % Nehme einen Mindestwert für die Dämpfung und einen sich bei Stagnation
      % erhöhenden Aufschlag. Mit Aufschlag wird immer lambda_min benutzt.
      lambda = (-1+2*2/pi*atan(condJ/condlimDLS))*(lambda_min+lambda_mult)/2;
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
        jj < n_max-10 && ...% die letzten Iterationen sind zum Ausgleich des Positionsfehlers (ohne Nullraum)
        ... %% falls vorherige Iterationen erfolglos, keine Nullraumbewegung. 
        ... % Annahme: Schädlich für Konvergenz. Nur, falls Stagnation 
        ... % nicht durch Gelenkgrenzen (scale_lim) verursacht wurde.
        (rejcount == 0 || rejcount~=0 && scale == 0)
      % Berechne Gradienten der zusätzlichen Optimierungskriterien
      v = zeros(Rob.NJ, 1);
      if wn(1) ~= 0
        [h(1), h1dq] = invkin_optimcrit_limits1(q1, qlim);
        v = v - wn(1)*h1dq'; % [SchapplerTapOrt2019], Gl. (44)
      end
      if wn(2) ~= 0
        [h(2), h2dq] = invkin_optimcrit_limits2(q1, qlim, qlim_thr_h2);
        v = v - wn(2)*h2dq'; % [SchapplerTapOrt2019], Gl. (45)
      end
      % Bestimme Ist-Lage der Plattform (bezogen auf erste Beinkette).
      % Benutze dies für die Berechnung der PKM-Jacobi. Nicht aussage-
      % kräftig, wenn Zwangsbedingungen grob verletzt sind. Dafür wird
      % die Rotation korrekt berücksichtigt.
      xE_1 = xE_soll + [zeros(5,1); Phi_voll(4)];
      if wn(4) || any(wn(3:6)) && taskred_rotsym && all(abs(Phi)<1e-3) % Bestimme PKM-Jacobi für Iterationsschritt
        % Benutze einfache Jacobi-Matrix und nicht die constr3grad-
        % Funktionen. Jinv ist zwischen beiden nur identisch, wenn Phi
        % exakt Null ist.
        [~, Phi4_x_voll] = Rob.constr4grad_x(xE_1);
        [~, Phi4_q_voll] = Rob.constr4grad_q(q1);
        Jinv = -Phi4_q_voll\Phi4_x_voll;
        condJpkm = cond(Jinv(Rob.I_qa,Rob.I_EE)); % bezogen auf Antriebe (nicht: Passive Gelenke)
        h(4) = condJpkm;
      end
      if wn(3) ~= 0 || wn(4) ~= 0 % Singularitäts-Kennzahl aus Konditionszahl
        % Zwei verschiedene Arten zur Berechnung der Nullraumbewegung, je
        % nachdem, ob die Beinketten schon geschlossen sind, oder nicht.
        if all(abs(Phi)<1e-3) && taskred_rotsym
          % Bestimme Nullraumbewegung durch Differenzenquotient für die 
          % redundante Koordinate. Dadurch nur eine neue Funktions- 
          % auswertung
          xD_test_3T3R = [zeros(5,1);1e-8];
          xD_test = xD_test_3T3R; % Hier werden für 2T1R nicht die Koordinaten reduziert
          qD_test = Jinv * xD_test;
          if wn(3) % Kennzahl bezogen auf Jacobi-Matrix der inversen Kinematik
            % Einfacher Differenzenquotient für Kond. der IK-Jacobi-Matrix
            Phi_q_test = Rob.constr3grad_q(q1+qD_test, xE_soll);
            condJ_test = cond(Phi_q_test);
            h3dq = (condJ_test-condJ)./qD_test';
          end
          if wn(4) && condJpkm < 1e10 % bezogen auf PKM-Jacobi (geht numerisch nicht in Singularität)
            [~,Phi4_x_voll_test] = Rob.constr4grad_x(xE_1+xD_test);
            [~,Phi4_q_voll_test] = Rob.constr4grad_q(q1+qD_test);
            Jinv_test = -Phi4_q_voll_test\Phi4_x_voll_test;
            h4_test = cond(Jinv_test(Rob.I_qa,Rob.I_EE));
            h4dq = (h4_test-h(4))./qD_test';
            if s.debug
              % Benutze Formel für Differential des Matrix-Produkts mit 
              % Matrix-Invertierung zur Bildung des PKM-Jacobi-Inkrements
              Phi4D_x_voll = Phi4_x_voll_test-Phi4_x_voll;
              Phi4D_q_voll = Phi4_q_voll_test-Phi4_q_voll;
              Jinv_test_2 = Jinv + ...
                Phi4_q_voll\Phi4D_q_voll/Phi4_q_voll*Phi4_x_voll + ...
                -Phi4_q_voll\Phi4D_x_voll;
              h4_test_2 = cond(Jinv_test_2(Rob.I_qa,Rob.I_EE));
              if abs(h4_test_2 - h4_test)/h4_test > 1e-6 && abs(h4_test_2-condJpkm) < 1
                error('Beide Ansätze für Delta Jinv stimmen nicht überein');
              end
            end
          end
        else
          % Bestimme Nullraumbewegung durch Differenzenquotient für jede
          % Gelenkkoordinate.
          if wn(3) % Kennzahl bezogen auf Jacobi-Matrix der inversen Kinematik
            for kkk = 1:Rob.NJ
              q_test = q1; % ausgehend von aktueller Konfiguration
              q_test(kkk) = q_test(kkk) + 1e-6; % minimales Inkrement
              Jik_kkk=Rob.constr3grad_q(q_test, xE_soll);% Berechnung identisch mit oben
              condJik_kkk = cond(Jik_kkk);
              % Differenzenquotient aus Log-Kond. scheint bei hohen Konditions-
              % zahlen numerisch etwas besser zu dämpfen (sonst dort sofort
              % maximal große Sprünge der Gelenkwinkel). Dafür Gradient dort gering.
              h3dq(kkk) = (condJik_kkk-condJ)/1e-6;
            end
          end
          if wn(4) && condJpkm < 1e10 % bezogen auf PKM-Jacobi (geht numerisch nicht in Singularität)
            for kkk = 1:Rob.NJ
              q_test = q1; % ausgehend von aktueller Konfiguration
              q_test(kkk) = q_test(kkk) + 1e-6; % minimales Inkrement
              [~, Phi4_q_voll_kkk] = Rob.constr4grad_q(q_test);
              Jinv_kkk = -Phi4_q_voll_kkk\Phi4_x_voll;
              condJpkm_kkk = cond(Jinv_kkk(Rob.I_qa,Rob.I_EE));
              h4dq(kkk) = (condJpkm_kkk-condJpkm)/1e-6;
            end
          end
        end
        if wn(3), v = v - wn(3)*h3dq'; end
        if wn(4), v = v - wn(4)*h4dq'; end
        h(3) = condJ;
      end
      if wn(5) % Kollisionsprüfung
        % Direkte Kinematik aller Beinketten (Datenformat für Kollision)
        [~, JP] = Rob.fkine_coll(q1);
        % Kollisionserkennung im vergrößerten Warnbereich
        colldet = check_collisionset_simplegeom_mex(collbodies_ns, Rob.collchecks, ...
          JP, struct('collsearch', true));
        if any(colldet)
          % Zwei verschiedene Arten zur Berechnung der Nullraumbewegung, je
          % nachdem, ob die Beinketten schon geschlossen sind, oder nicht.
          % Siehe Berechnung für vorheriges Kriterium
          if all(abs(Phi)<1e-3) && taskred_rotsym
            % Bestimme Nullraumbewegung durch Differenzenquotient für die 
            % redundante Koordinate. Dadurch nur eine neue Funktions- 
            % auswertung
            xD_test_3T3R = [zeros(5,1);1e-8];
            xD_test = xD_test_3T3R; % Hier werden für 2T1R nicht die Koordinaten reduziert
            qD_test = Jinv * xD_test;
            JP_test = [JP; NaN(1, size(JP,2))];
            [~, JP_test(2,:)] = Rob.fkine_coll(q1+qD_test);
            % Kollisionsprüfung für alle Gelenkpositionen auf einmal. Prüfe
            % nur die Fälle, bei denen die vergrößerten Objekte bereits eine
            % Kollision angezeigt haben.
            [~, colldist_test] = check_collisionset_simplegeom_mex( ...
              Rob.collbodies, Rob.collchecks(colldet,:), JP_test, struct('collsearch', false));
            % Kollisions-Kriterium berechnen: Tiefste Eindringtiefe (positiv)
            % Falls keine Kollision vorliegt (mit den kleineren
            % Kollisionskörpern), dann Abstände negativ angeben.
            mincolldist_test = min(colldist_test,[],2); % Schlimmste Kollision für jeden Körper bestimmen
            h(5) = invkin_optimcrit_limits2(-mincolldist_test(1), ... % zurückgegebene Distanz ist zuerst negativ
              [-100*maxcolldepth, 0], [-80*maxcolldepth, -collobjdist_thresh]);
            if h(5) == 0 % nichts tun. Noch im Toleranzbereich
              h5dq(:) = 0;
            elseif ~isinf(h(5))
              h5_test = invkin_optimcrit_limits2(-mincolldist_test(2), ... % zurückgegebene Distanz ist zuerst negativ
                [-100*maxcolldepth, 0], [-80*maxcolldepth, -collobjdist_thresh]);
              % Einfacher Differenzenquotient
              h5dq = (h5_test-h(5))./qD_test';
            else % Kollision so groß, dass Wert inf ist. Dann kein Gradient aus h bestimmbar.
              % Indirekte Bestimmung über die betragsmäßige Verkleinerung der (negativen) Eindringtiefe
              h5dq = (-mincolldist_test(2)-(-mincolldist_test(1)))./qD_test';
              if max(abs(h5dq)) > 100*eps % Normiere auf Wert 1e3 für größtes Gelenk
                h5dq = h5dq/max(abs(h5dq)) * 1e3; % wird weiter unten reduziert (für delta_q)
              end
            end
          else
            % Bestimme Nullraumbewegung durch Differenzenquotient für jede
            % Gelenkkoordinate.
            JP_test = [JP; NaN(Rob.NJ, size(JP,2))];
            for kkk = 1:Rob.NJ
              q_test = q1; % ausgehend von aktueller Konfiguration
              q_test(kkk) = q_test(kkk) + 1e-6; % minimales Inkrement
              [~, JP_test(1+kkk,:)] = Rob.fkine_coll(q_test);
            end
            % Kollisionsprüfung für alle Gelenkpositionen auf einmal.
            [~, colldist_test] = check_collisionset_simplegeom_mex( ...
              Rob.collbodies, Rob.collchecks(colldet,:), JP_test, struct('collsearch', false));
            mincolldist_test = min(colldist_test,[],2);
            h(5) = invkin_optimcrit_limits2(-mincolldist_test(1), ... % zurückgegebene Distanz ist zuerst negativ
              [-100*maxcolldepth, 0], [-80*maxcolldepth, -collobjdist_thresh]);
            if h(5) == 0 % nichts tun. Noch im Toleranzbereich
              h5dq(:) = 0;
            elseif ~isinf(h(5))
              for kkk = 1:Rob.NJ
                h5_test = invkin_optimcrit_limits2(-mincolldist_test(1+kkk), ... % zurückgegebene Distanz ist zuerst negativ
                  [-100*maxcolldepth, 0], [-80*maxcolldepth, -collobjdist_thresh]);
                h5dq(kkk) = (h5_test-h(5))/1e-6;
              end
            else % Kollision so groß, dass Wert inf ist. Dann kein Gradient aus h bestimmbar.
              % Indirekte Bestimmung über die betragsmäßige Verkleinerung der (negativen) Eindringtiefe
              for kkk = 1:Rob.NJ
                h5dq(kkk) = (-mincolldist_test(1+kkk)-(-mincolldist_test(1)));
              end
              if max(abs(h5dq)) > 100*eps % Normiere auf Wert 1e3 für größtes Gelenk
                h5dq = h5dq/max(abs(h5dq)) * 1e3; % wird weiter unten reduziert (für delta_q)
              end
            end
          end
          v = v - wn(5)*h5dq';
        end
      end
      if wn(6) % Bauraumprüfung
        % Direkte Kinematik aller Beinketten (Datenformat für Kollision)
        [~, JP] = Rob.fkine_coll(q1);
        % Zwei verschiedene Arten zur Berechnung der Nullraumbewegung, je
        % nachdem, ob die Beinketten schon geschlossen sind, oder nicht.
        % Siehe Berechnung für vorheriges Kriterium
        if all(abs(Phi)<1e-3) && taskred_rotsym
          % Bestimme Nullraumbewegung durch Differenzenquotient für die 
          % redundante Koordinate. Dadurch nur eine neue Funktions- 
          % auswertung
          xD_test_3T3R = [zeros(5,1);1e-8];
          xD_test = xD_test_3T3R; % Hier werden für 2T1R nicht die Koordinaten reduziert
          qD_test = Jinv * xD_test;
          JP_test = [JP; NaN(1, size(JP,2))];
          [~, JP_test(2,:)] = Rob.fkine_coll(q1+qD_test);
          % Bauraumprüfung für alle Gelenkpositionen auf einmal
          [~, absdist] = check_collisionset_simplegeom_mex(Rob.collbodies_instspc, ...
            Rob.collchecks_instspc, JP_test, struct('collsearch', false));
          % Prüfe, ob alle beweglichen Kollisionsobjekte in mindestens einem
          % Bauraumkörper enthalten sind (falls Prüfung gefordert)
          mindist_all = -inf(size(JP_test,1),1);
          for i = 1:size(Rob.collbodies_instspc.link,1)
            % Indizes aller Kollisionsprüfungen mit diesem (Roboter-)Objekt i
            I = Rob.collchecks_instspc(:,1) == i; % erste Spalte für Roboter-Obj.
            if ~any(I), continue; end % Bauraum-Objekte nicht direkt prüfen. Sonst leeres Array
            % Falls mehrere Bauraum-Objekte, nehme das mit dem besten Wert
            mindist_i = min(absdist(:,I),[],2);
            % Nehme den schlechtesten Wert von allen Objekten
            mindist_all = max([mindist_i,mindist_all],[],2);
          end
          % Bauraum-Kriterium berechnen: Negativer Wert ist im Bauraum (gut),
          % positiver ist außerhalb (schlecht). Größter positiver Wert
          % maßgeblich
          h(6) = invkin_optimcrit_limits2(mindist_all(1), ... % Wert bezogen auf aktuelle Pose
            [-100.0, 0], ... % obere Grenze: Bei Schwelle zur Bauraumverletzung ist Wert inf
            [-90, -s.installspace_thresh]); % obere Grenze: z.B. ab 100mm Nähe zum Rand Kriterium aktiv
          h6_test = invkin_optimcrit_limits2(mindist_all(2), ... % Wert bezogen auf Test-Pose
            [-100.0, 0], [-90, -s.installspace_thresh]);
          % Einfacher Differenzenquotient für Kond. der IK-Jacobi-Matrix
          if ~isinf(h(6))
            h6dq = (h6_test-h(6))./qD_test';
          else % Verletzung so groß, dass Wert inf ist. Dann kein Gradient bestimmbar.
            % Indirekte Bestimmung über die Verkleinerung des (positiven) Abstands
            h6dq = (mindist_all(2)-mindist_all(1))./qD_test';
            if max(abs(h6dq)) > .1 % Normiere auf Wert 0.1 für größtes Gelenk
              h6dq = h6dq/max(abs(h6dq)) * .1; % wird weiter unten reduziert
            end
          end
        else
          % Bestimme Nullraumbewegung durch Differenzenquotient für jede
          % Gelenkkoordinate.
          JP_test = [JP; NaN(Rob.NJ, size(JP,2))];
          for kkk = 1:Rob.NJ
            q_test = q1; % ausgehend von aktueller Konfiguration
            q_test(kkk) = q_test(kkk) + 1e-6; % minimales Inkrement
            [~, JP_test(1+kkk,:)] = Rob.fkine_coll(q_test);
          end
          % Bauraumprüfung für alle Gelenkpositionen auf einmal.
          [~, absdist] = check_collisionset_simplegeom(Rob.collbodies_instspc, ...
            Rob.collchecks_instspc, JP_test, struct('collsearch', false));
          mindist_all = -inf(size(JP_test,1),1); % gleiche Rechnung wie oben
          for i = 1:size(Rob.collbodies_instspc.link,1)
            I = Rob.collchecks_instspc(:,1) == i;
            if ~any(I), continue; end
            mindist_i = min(absdist(:,I),[],2);
            mindist_all = max([mindist_i,mindist_all],[],2);
          end
          h(6) = invkin_optimcrit_limits2(mindist_all(1), ... % Wert bezogen auf aktuelle Pose
            [-100.0, 0], [-90, -s.installspace_thresh]);
          if h(6) == 0 % nichts unternehmen (im Bauraum, mit Sicherheitsabstand)
            h6dq(:) = 0;
          elseif ~isinf(h(6))
            for kkk = 1:Rob.NJ
              h6_test = invkin_optimcrit_limits2(mindist_all(1+kkk), ... % Wert bezogen auf Test-Pose dieses Gelenks
                [-100.0, 0], [-90, -s.installspace_thresh]);
              h6dq(kkk) = (h6_test-h(6))/1e-6; % Differenzenquotient bzgl. Inkrement
            end
          else % Verletzung so groß, dass Wert inf ist. Dann kein Gradient aus h bestimmbar.
            % Indirekte Bestimmung über Abstand
            for kkk = 1:Rob.NJ
              h6dq(kkk) = (mindist_all(1+kkk)-mindist_all(1))/1e-6';
            end
            if max(abs(h6dq)) > 100*eps % Normiere auf Wert 1e3 für größtes Gelenk
              h6dq = h6dq/max(abs(h6dq)) * 1e3; % wird weiter unten reduziert
            end
          end
        end
        if nargout == 4
          Stats.instspc_mindst(jj) = mindist_all(1);
        end
        v = v - wn(6)*h6dq';
      end
      if any(abs(v)>1e8),  v = v* 1e8/max(abs(v)); end
      % [SchapplerTapOrt2019], Gl. (43)
      N = (eye(Rob.NJ) - pinv(Jik)* Jik);
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
      % Reduziere das Gelenk-Inkrement so, dass das betragsgrößte
      % Winkelinkrement danach 30° hat.
      delta_q_T = delta_q_T .* 0.5/max(abs_delta_qTrev);
    end
    abs_delta_qNrev = abs(delta_q_N(sigma_PKM==0)); % nur Drehgelenke
    if any(abs_delta_qNrev > 0.05*(1-jj/n_max)) % 0.05rad=3°
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
      abs_delta_q_T_rel = abs(delta_q_T ./ delta_qlim .* K);
      if any(abs_delta_q_T_rel > maxrelstep)
        delta_q_T = delta_q_T .* maxrelstep / max(abs_delta_q_T_rel);
      end
    end
    if nsoptim && limits_set && ~isnan(maxrelstep_ns)
      abs_delta_q_N_rel = abs(delta_q_N ./ delta_qlim .* Kn);
      if any(abs_delta_q_N_rel > maxrelstep_ns)
        delta_q_N = delta_q_N .* maxrelstep_ns / max(abs_delta_q_N_rel);
      end
    end
    % Dämpfung der Nullraumbewegung. Verlangsamt die Konvergenz, reduziert
    % dafür Schwingungen zwischen delta_q_T und delta_q_N, die durch keine
    % der anderen Stabilisierungsmaßnahmen erfasst werden. Nur machen, wenn
    % auch Bewegung im Nullraum (nicht wenn q_N deaktiviert wurde)
    if nsoptim && damping_active && any(delta_q_N)
      % Korrigiere den Altwert, da bezogen auf andere Gelenkkonfig.
      delta_q_N_alt_N = N*delta_q_N_alt;
      % Benutze diskretes PT1-Filter mit T=2 (Schritte der IK) und K=1
      delta_q_N = delta_q_N_alt_N + 1/(1+2)*(1*delta_q_N-delta_q_N_alt_N);
      delta_q_N_alt = delta_q_N;
    end
    
    % Prüfe ob mit dem Inkrement eine Kollision erzeugt wird und reduziere
    % die Bewegung darauf hin. Ist nur bezugen auf die Aufgabe, da die
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
    if damping_active
      % Benutze diskretes PT1-Filter mit T=2 (Schritte der IK) und K=1
      % Zusätzlich zu obiger Filterung von delta_q_N.
      delta_q = delta_q_alt + 1/(1+2)*(1*delta_q-delta_q_alt);
    elseif all(sign(delta_q) == -sign(delta_q_alt))
      damping_active = true; % ab jetzt aktiviert lassen.
    end
    delta_q_alt = delta_q;
    
    % Gelenkwinkel-Schritt anwenden
    q2 = q1 + delta_q;
    
    % Prüfe, ob die Gelenkwinkel ihre Grenzen überschreiten und reduziere
    % die Schrittweite, falls das der Fall ist; [SchapplerTapOrt2019], Gl. (47)
    scale = 1;
    if scale_lim
      delta_ul_rel = (qmax - q2)./(qmax-q1); % Überschreitung der Maximalwerte: <0
      delta_ll_rel = (-qmin + q2)./(q1-qmin); % Unterschreitung Minimalwerte: <0
      if any([delta_ul_rel;delta_ll_rel] < 0)
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
      if nargout == 4
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
      wn(2) = wn(2) + 0.1;
    end
    % Prüfe, ob Schritt erfolgreich war (an dieser Stelle, da der 
    % Altwert von Phi noch verfügbar ist). Siehe [CorkeIK].
    Phi_neu_norm = norm(Phi_neu);
    Delta_Phi = Phi_neu_norm - norm(Phi); % "neu" - "alt";
    bestPhi = min([bestPhi;Phi_neu_norm]);
    if any(delta_q_N) && sum(wn.*h)>=sum(wn.*h_alt) && ~any(isinf(h)) && (Delta_Phi > 0 || ...
        Phi_neu_norm > bestPhi) % zusätzlich prüfen gegen langsamere Oszillationen
      % Zusätzliches Optimierungskriterium hat sich verschlechtert und
      % gleichzeitig auch die IK-Konvergenz. Das deutet auf eine
      % Konvergenz mit Oszillationen hin. Reduziere den Betrag der
      % Nullraumbewegung. Annahme: Bewegung so groß, dass keine
      % Linearisierungsfehler (außerhalb des Nullraums) zu groß.
      % Nicht bei Kriterium unendlich (separate Gradientenberechnung)
      Kn = Kn*0.8;
    end
    if Phi_iO || Delta_Phi < 0 ... % Verbesserung des Residuums
        || any(delta_q_N) && all(abs(Phi_neu)<1e-3) && Delta_Phi~=0 % Bei Nullraumbewegung auch Verschlechterung möglich, wenn noch im "guten" Bereich
      if condJ>1e4 && Delta_Phi > -Phit_tol % Singularität mit Mini-Schritten
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
      if condJ <= condlimDLS && ~nsoptim || ... % Keine Verbesserung der Konvergenz trotz guter Konditionszahl.
       rejcount > 50 % Stillstand zu lange trotz exponentieller Erhöhung von lambda.
        % Abbruch. Keine Verbesserung mit Algorithmus möglich.
        if nargout == 4
          Stats.iter = jj;
        end
        break;
      end
    end
    if nargout == 4
      Stats.Q(1+jj,:) = q1;
      Stats.PHI(1+jj,:) = Phi_voll;
      Stats.h(jj,:) = [sum(wn.*h),h'];
      Stats.condJ(jj) = condJ;
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
      if nargout == 4
        Stats.iter = jj;
      end
      if finish_in_limits && (any(q1 < qlim(:,1)) || any(q1 > qlim(:,2)))
        % Es soll eigentlich abgebrochen werden. Die Grenzen wurden aber 
        % nicht eingehalten. Mache noch mit dem Algorithmus weiter und 
        % optimiere nur noch die Grenzen (und nicht z.B. Konditionszahl)
        finish_in_limits = false; % Modus ist damit aktiviert
        nsoptim = true;
        wn = [0;1;0;0;wn(5);wn(6)]; % Nutze nur die hyperbolische Funktion des Abstands
        % Mache diese Optimierung nicht mehr zu Ende, sondern höre auf, 
        % wenn die Grenzen erreicht sind.
        break_when_in_limits = true;
        success = false; % Bereits gesetzten Wert wieder zurücknehmen
        continue
      end
      if avoid_collision_finish
        % Eigentlich soll abgebrochen werden. Prüfe nochmal auf Kollisionen
        [~, JP] = Rob.fkine_coll(q1);
        colldet = check_collisionset_simplegeom_mex(Rob.collbodies, ...
          Rob.collchecks, JP, struct('collsearch', false));
        if any(colldet)
          wn(5) = 1; % Aktiviere Kollisionsvermeidung
          wn(1:4) = 0; % Deaktiviere alle anderen Nebenbedingungen
          avoid_collision_finish = false; % Nur einmal versuchen
          success = false; % Bereits gesetzten Wert wieder zurücknehmen
          continue
        end
      end
      break;
    end
  end
  if success
    if nargout == 4
      Stats.retry_number = rr;
    end
    if ~s.retry_on_limitviol || s.retry_on_limitviol && all(q1>=qmin) && all(q1<=qmax)
      break;
    end
  end
  % Beim vorherigen Durchlauf kein Erfolg. Generiere neue Anfangswerte
  if rr == 0 && ~isnan(s.rng_seed)
    rng(s.rng_seed); % Initialisiere Zufallszahlen, falls gewünscht
  end
  if rr < size(Q0,2) % versuche eine weitere der vorgegebenen Konfigurationen
    q0 = Q0(:,rr+1);
  else % benutze eine zufällige Konfiguration
    Q0 = qmin_norm + rand(Rob.NJ,1).*(qmax_norm-qmin_norm); 
  end
end
if nargout >= 3 || nargout >= 4 && wn(5) ~= 0
  Tc_stack_PKM = Rob.fkine_coll(q1);
end
if nargout == 4 % Berechne Leistungsmerkmale für letzten Schritt
  if wn(1) ~= 0, h(1) = invkin_optimcrit_limits1(q1, qlim); end
  if wn(2) ~= 0, h(2) = invkin_optimcrit_limits2(q1, qlim, qlim_thr_h2); end
  Jik=Rob.constr3grad_q(q1, xE_soll);
  h(3) = cond(Jik);
  if wn(4) % Bestimme PKM-Jacobi für Iterationsschritt
    % Benutze die einfachen Zwangsbedingungen, da vollständige FG.
    xE_1 = xE_soll + [zeros(5,1); Phi_voll(4)];
    [~, Phi4_x_voll] = Rob.constr4grad_x(xE_1);
    [~, Phi4_q_voll] = Rob.constr4grad_q(q1);
    Jinv = -Phi4_q_voll\Phi4_x_voll; % bezogen auf 3T3R
    h(4) = cond(Jinv(Rob.I_qa,Rob.I_EE));
  end
  if wn(5) ~= 0
    [colldet,colldist] = check_collisionset_simplegeom_mex(Rob.collbodies, Rob.collchecks, ...
      Tc_stack_PKM(:,4)', struct('collsearch', true));
    h(5) = invkin_optimcrit_limits2(-min(colldist), ...
      [-100*maxcolldepth, maxcolldepth], [-80*maxcolldepth, -collobjdist_thresh]);
    if any(colldet)
      Stats.coll = true;
    end
    % Trage den Wert ein, ab dem eine Kollision vorliegt
    Stats.h_coll_thresh = invkin_optimcrit_limits2(0, ...
      [-100*maxcolldepth, maxcolldepth], [-80*maxcolldepth, -collobjdist_thresh]);
  end
  if wn(6) ~= 0 % Berechnung muss genauso sein wie oben
    [~, absdist] = check_collisionset_simplegeom_mex(Rob.collbodies_instspc, ...
      Rob.collchecks_instspc, Tc_stack_PKM(:,4)', struct('collsearch', false));
    mindist_all = -inf;
    for i = 1:size(s.collbodies_instspc.link,1)
      I = s.collchecks_instspc(:,1) == i;
      if ~any(I), continue; end
      mindist_i = min(absdist(:,I),[],2);
      mindist_all = max([mindist_i,mindist_all],[],2);
    end
    h(6) = invkin_optimcrit_limits2(mindist_all, ...
      [-100.0, s.installspace_thresh], [-90, -s.installspace_thresh]);
    Stats.instspc_mindst(Stats.iter+1) = mindist_all(1);
    % Trage den Wert ein, ab dem eine Bauraumverletzung vorliegt
    Stats.h_instspc_thresh = invkin_optimcrit_limits2(0, ...
      [-100.0, s.installspace_thresh], [-90, -s.installspace_thresh]);
  end
  Stats.h(Stats.iter+1,:) = [sum(wn.*h),h'];
  Stats.condJ(Stats.iter+1) = h(3);
end
q = q1;
if s.normalize
  q(sigma_PKM==0) = normalize_angle(q(sigma_PKM==0)); % nur Winkel normalisieren
end

