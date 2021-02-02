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
% q0 [Nx1]
%   Startkonfiguration: Alle Gelenkwinkel aller serieller Beinketten der PKM
% s
%   Struktur mit Eingabedaten. Felder, siehe Quelltext.
% 
% Ausgabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM als Lösung der IK
% Phi
%   Kinematische Zwangsbedingungen für die Lösung. Bei korrekter Berechnung
%   muss dieser Wert Null sein.
% Tc_stack_PKM 
%   Gestapelte Transformationsmatrizen der PKM . Im
%   Basis-KS. Entspricht mit Abwandlung der Anordnung wie in fkine: 
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

function [q, Phi, Tc_stack_PKM, Stats] = invkin3(Rob, xE_soll, q0, s)

%% Initialisierung
assert(isreal(xE_soll) && all(size(xE_soll) == [6 1]), ...
  'ParRob/invkin3: xE_soll muss 6x1 sein');
assert(isreal(q0) && all(size(q0) == [Rob.NJ 1]), ...
  'ParRob/invkin3: q0 muss %dx1 sein', Rob.NJ);

%% Definitionen
% Variablen zum Speichern der Zwischenergebnisse
sigma_PKM = Rob.MDH.sigma; % Marker für Dreh-/Schubgelenk
K = 1.0*ones(Rob.NJ,1);
% K(sigma_PKM==1) = 0.5; % Verstärkung für Schubgelenke kleiner

s_std = struct(...
  'K', K, ... % Verstärkung
  'Kn', 0.1*ones(Rob.NJ,1), ... % Verstärkung
  'wn', zeros(2,1), ... % Gewichtung der Nebenbedingung
  'maxstep_ns', 1e-10*ones(Rob.NJ,1), ... % Maximale Schrittweite für Nullraum zur Konvergenz
  'normalize', true, ...
  'condlimDLS', 1, ... % Grenze der Konditionszahl, ab der die Pseudo-Inverse gedämpft wird (1=immer)
  'lambda_min', 2e-4, ... % Untergrenze für Dämpfungsfaktor der Pseudo-Inversen
  'n_min', 0, ... % Minimale Anzahl Iterationen
  'n_max', 1000, ... % Maximale Anzahl Iterationen
  'rng_seed', NaN, ...  % Initialwert für Zufallszahlengenerierung
  'scale_lim', 1, ... % Herunterskalierung bei Grenzüberschreitung
  'Phit_tol', 1e-8, ... % Toleranz für translatorischen Fehler
  'Phir_tol', 1e-8,... % Toleranz für rotatorischen Fehler
  'maxrelstep', 0.1, ... % Maximale Schrittweite relativ zu Grenzen
  'maxrelstep_ns', 0.005, ... % Maximale Schrittweite der Nullraumbewegung
  'retry_limit', 100); % Anzahl der Neuversuche
if nargin < 4
  % Keine Einstellungen übergeben. Standard-Einstellungen
  s = s_std;
end
% Prüfe Felder der Einstellungs-Struktur und setze Standard-Werte, falls
% Eingabe nicht gesetzt
for f = fields(s_std)'
  if ~isfield(s, f{1})
    s.(f{1}) = s_std.(f{1});
  end
end

% Variablen aus Einstellungsstruktur holen
K = s.K; 
Kn = s.Kn; 
n_min = s.n_min;
n_max = s.n_max;
wn = s.wn;
condlimDLS = s.condlimDLS;
lambda_min = s.lambda_min;
scale_lim = s.scale_lim;
Phit_tol = s.Phit_tol;
Phir_tol = s.Phir_tol;
retry_limit = s.retry_limit;
maxrelstep = s.maxrelstep;
maxrelstep_ns = s.maxrelstep_ns;
maxstep_ns = s.maxstep_ns;
success = false;

if any(wn ~= 0) && sum(Rob.I_EE) > sum(Rob.I_EE_Task)
  % Nullraumoptimierung nur möglich, falls FG da sind. TODO: Das
  % berücksichtigt noch nicht den Fall von 3T3R-PKM in 3T0R-Aufgaben.
  nsoptim = true;
else
  % Keine zusätzlichen Optimierungskriterien
  nsoptim = false;
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
end
delta_qlim = qmax - qmin;

I_constr_t_red = Rob.I_constr_t_red;
I_constr_r_red = Rob.I_constr_r_red;
I_IK = Rob.I_constr_red;

% Zählung in Rob.NL: Starrkörper der Beinketten, Gestell und Plattform. 
% Hier werden nur die Basis-KS der Beinketten und alle bewegten Körper-KS
% der Beine angegeben.
Tc_stack_PKM = NaN((Rob.NL-1+Rob.NLEG)*3,4); % siehe fkine_legs; dort aber leicht anders
% Basis-KS. Trägt keine Information. Dient nur zum einfacheren Zugriff auf
% die Variable und zur Angleichung an Darstellung im Welt-KS.
Tc_stack_PKM(1:3,1:4) = eye(3,4);  % Basis-KS im Basis-KS.
out3_ind1 = 3; % Zeilenzähler für obige Variable (drei Zeilen stehen schon)
rejcount = 0; % Zähler für Zurückweisung des Iterationsschrittes, siehe [CorkeIK]

if nargout == 4
  Stats = struct('Q', NaN(n_max, Rob.NJ), 'PHI', NaN(n_max, 6*Rob.NLEG), ...
    'iter', n_max, 'retry_number', retry_limit, 'condJ', NaN(n_max,1), 'lambda', ...
    NaN(n_max,2), 'rejcount', NaN(n_max,1));
end
%% Iterative Lösung der IK
for rr = 0:retry_limit
  q1 = q0;
  [~,Phi_voll] = Rob.constr3(q1, xE_soll);
  Phi = Phi_voll(I_IK);
  lambda_mult = lambda_min; % Zurücksetzen der Dämpfung
  lambda = 0.0;
  rejcount = 0; % Zurücksetzen des Zählers für Fehlversuche
  for jj = 1:n_max
    % Gesamt-Jacobi bilden (reduziert um nicht betrachtete EE-Koordinaten)
    [~,Jik_voll]=Rob.constr3grad_q(q1, xE_soll);
    Jik = Jik_voll(I_IK,:);

    %% Nullstellensuche für Positions- und Orientierungsfehler
    condJ = cond(Jik);
    % (Optimierung der Aufgabe)
    % Benutze das Damped Least Squares Verfahren je nach Konditionszahl.
    % Bei Redundanz nur benutzen, falls Nullraumprojektion erfolglos.
    if condJ > condlimDLS && (~nsoptim || nsoptim && rejcount > 0)
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
        rejcount == 0 %% falls vorherige Iterationen erfolglos, keine Nullraumbewegung. Annahme: Schädlich für Konvergenz
      % Berechne Gradienten der zusätzlichen Optimierungskriterien
      v = zeros(Rob.NJ, 1);
      if wn(1) ~= 0
        [h1, hdq] = invkin_optimcrit_limits1(q1, qlim);
        v = v - wn(1)*hdq'; % [SchapplerTapOrt2019], Gl. (44)
      end
      if wn(2) ~= 0
        [h2, hdq] = invkin_optimcrit_limits2(q1, qlim);
        v = v - wn(2)*hdq'; % [SchapplerTapOrt2019], Gl. (45)
      end
      % [SchapplerTapOrt2019], Gl. (43)
      delta_q_N = (eye(Rob.NJ) - pinv(Jik)* Jik) * v;
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
    if any(abs_delta_qNrev > 0.05) % 0.05rad=3°
      % Reduziere das Gelenk-Inkrement so, dass das betragsgrößte
      % Winkelinkrement danach 3° hat.
      delta_q_N = delta_q_N .* 0.05/max(abs_delta_qNrev);
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
    
    % Inkrement der Gelenkwinkel; [SchapplerTapOrt2019], Gl. (43)
    % Verstärkungsfaktoren K und Kn oben bereits angewendet.
    delta_q = delta_q_T + delta_q_N;
    
    % Reduziere Schrittweite auf einen Maximalwert bezogen auf
    % Winkelgrenzen; [SchapplerTapOrt2019], Gl. (47)
    if limits_set && ~isnan(maxrelstep)
      % Bestimme Inkrement relativ zur Spannbreite der Grenzen
      abs_delta_q_rel = abs(delta_q ./ delta_qlim);
      if any(abs_delta_q_rel > maxrelstep)
        % Ein Element hat ein zu großes Inkrement. Normiere den
        % Inkrement-Vektor damit
        delta_q = delta_q .* maxrelstep / max(abs_delta_q_rel);
      end
    end
    
    q2 = q1 + delta_q;

    % Prüfe, ob die Gelenkwinkel ihre Grenzen überschreiten und reduziere
    % die Schrittweite, falls das der Fall ist; [SchapplerTapOrt2019], Gl. (47)
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
        q2 = q1 + scale_lim*scale*delta_q;
      end
    end

    if any(isnan(q2)) || any(isinf(q2))
      break; % ab hier kann das Ergebnis nicht mehr besser werden wegen NaN/Inf
    end

    [~,Phi_voll] = Rob.constr3(q2, xE_soll);
    % Prüfe, ob Wert klein genug ist. Bei kleinen Zahlenwerten, ist
    % numberisch teilweise keine Verbesserung möglich.
    Phi_iO = all(abs(Phi(I_constr_t_red)) < Phit_tol) && ...
             all(abs(Phi(I_constr_r_red)) < Phir_tol);
    % Prüfe, ob Schritt erfolgreich war (an dieser Stelle, da der 
    % Altwert von Phi noch verfügbar ist). Siehe [CorkeIK].
    if Phi_iO || norm(Phi_voll(I_IK)) < norm(Phi) % Erfolgreich
      % Erfolgreich. Verringere lambda bis auf Minimalwert.
      lambda_mult = max(lambda_mult/2, lambda_min);
      % Behalte Ergebnis der Iteration für weitere Iterationen.
      q1 = q2;
      % Behalte Wert für Phi nur in diesem Fall. Dadurch wird auch die Verbesserung
      % gegenüber der Iteration messbar, bei der zuletzt eine Verschlechterung eintrat.
      Phi = Phi_voll(I_IK); % Reduktion auf betrachtete FG
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
      Stats.Q(jj,:) = q1;
      Stats.PHI(jj,:) = Phi_voll;
      Stats.condJ(jj) = condJ;
      Stats.lambda(jj,:) = [lambda, lambda_mult];
      Stats.rejcount(jj) = rejcount;
    end
    % Abbruchbedingungen prüfen
    if jj >= n_min ... % Mindestzahl Iterationen erfüllt
     && Phi_iO && ... % Haupt-Bedingung ist erfüllt
     ( ~nsoptim || ...%  und keine Nebenoptimierung läuft
     nsoptim && all(abs(delta_q_N) < maxstep_ns) ) % oder die Nullraumoptimierung läuft noch
      success = true;
      if nargout == 4
        Stats.iter = jj;
      end
      break;
    end
  end
  if success
    if nargout == 4
      Stats.retry_number = rr;
    end
    break;
  end
  % Beim vorherigen Durchlauf kein Erfolg. Generiere neue Anfangswerte
  if rr == 0 && ~isnan(s.rng_seed)
    rng(s.rng_seed); % Initialisiere Zufallszahlen, falls gewünscht
  end
  q0 = qmin + rand(Rob.NJ,1).*(qmax-qmin); 
end
q = q1;
if s.normalize
  q(sigma_PKM==0) = normalize_angle(q(sigma_PKM==0)); % nur Winkel normalisieren
end
if nargout == 3
  for i = 1:Rob.NLEG
  [~, ~, Tc_stack_0i] = Rob.Leg(i).fkine(q(Rob.I1J_LEG(i):Rob.I2J_LEG(i)));
  T_0_0i = Rob.Leg(i).T_W_0;
  % Umrechnung auf PKM-Basis-KS. Nehme nur die KS, die auch einem Körper
  % zugeordnet sind. In Tc_stack_0i bei hybriden Systemen teilw. mehr.
  Tc_stack_0 = NaN(3*(Rob.Leg(i).NL),4);
  for kk = 1:Rob.Leg(i).NL
    Tc_stack_k = Tc_stack_0i(3*(kk-1)+1:kk*3,1:4);
    T_0_kk = T_0_0i * [Tc_stack_k;0 0 0 1];
    Tc_stack_0((kk-1)*3+1:kk*3,1:4) = T_0_kk(1:3,:);
  end
  % Eintragen in Ergebnis-Variable
  Tc_stack_PKM(out3_ind1+(1:3*Rob.Leg(i).NL),:) = Tc_stack_0;
  out3_ind1 = out3_ind1 + 3*Rob.Leg(i).NL;
  end
end

