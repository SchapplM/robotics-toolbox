% Inverse Kinematik für allgemeinen Roboter (Komplette Trajektorie)
% Allgemeine, stark parametrierbare Funktion zum Aufruf mit allen möglichen
% Einstellungen
% Iterative Lösung der inversen Kinematik mit inverser Jacobi-Matrix
% Zusätzlich Nutzung der differentiellen Kinematik für schnellere Konvergenz
% 
% Eingabe:
% XE
%   Trajektorie von EE-Lagen (Sollwerte)
% XDE
%   Trajektorie von EE-Geschwindigkeiten (Sollwerte)
%   (Die Orientierung wird durch Euler-Winkel-Zeitableitung dargestellt)
% XDDE
%   Trajektorie von EE-Beschleunigungen (Sollwerte)
%   Orientierung bezogen auf Euler-Winkel
% T
%   Zeitbasis der Trajektorie (Alle Zeit-Stützstellen)
% q0
%   Anfangs-Gelenkwinkel für Algorithmus
% s
%   Struktur mit Eingabedaten. Felder, siehe Quelltext.
% 
% Ausgabe:
% Q
%   Trajektorie von Gelenkpositionen (Lösung der IK)
% QD
%   Trajektorie von Gelenkgeschwindigkeiten
% QDD
%   Trajektorie von Gelenkbeschleunigungen
% Jinv_ges
%   Inverse PKM-Jacobi-Matrix für alle Bahnpunkte (spaltenweise in Zeile)
%   (Jacobi zwischen allen Gelenkgeschwindigkeiten qD und EE-geschwindigkeit xDE)
%   (Nicht: Nur Bezug zu Antriebsgeschwindigkeiten qaD)
% JinvD_ges
%   Zeitableitung von Jinv_ges
% JointPos_all
%   gestapelte Positionen aller Gelenke der PKM für alle Zeitschritte
%   (Entspricht letzter Spalte aller Transformationsmatrizen aus fkine_legs)
%   Reihenfolge: Siehe Ausgabe Tc_stack_PKM aus invkin_ser
%   * PKM-Basis
%   * Für jede Beinkette: Basis und alle bewegten Körper-KS. Ohne
%     virtuelles EE-KS
%   * Kein Plattform-KS
% 
% Siehe auch: SerRob/invkin_traj bzw. SerRob/invkin2_traj

% Quelle:
% [2] Aufzeichnungen Schappler vom 11.12.2018
% [3] Aufzeichnungen Schappler vom 06.07.2020

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-02
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [Q, QD, QDD, Phi, Jinv_ges, JinvD_ges, JointPos_all] = invkin_traj(Rob, X, XD, XDD, T, q0, s)

%% Initialisierung
s_std = struct( ...
  'I_EE', Rob.I_EE_Task, ... % FG für die IK
  'simplify_acc', false, ... % Berechnung der Beschleunigung vereinfachen
  'mode_IK', 3, ...  % 1=Seriell-IK, 2=PKM-IK, 3=beide
  'wn', zeros(5,1), ... % Gewichtung der Nebenbedingung. Standard: Ohne
  'debug', false); % Zusätzliche Ausgabe
if nargin < 7
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
debug = s.debug;
I_EE = Rob.I_EE_Task;
mode_IK = s.mode_IK;
dof_3T2R = false;

if all(s.I_EE == logical([1 1 1 1 1 0]))
  dof_3T2R = true;
  I_EE = s.I_EE;
end

if nargout == 6
  % Wenn Jacobi-Zeitableitung als Ausgabe gefordert ist, kann die
  % vollständige Formel für die Beschleunigung benutzt werden
  simplify_acc = false;
else
  % Benutze vollständige Formel entsprechend Einstellungsparameter
  simplify_acc = s.simplify_acc;
end

nt = length(T);
Q = NaN(nt, Rob.NJ);
QD = Q;
QDD = Q;
Phi = NaN(nt, length(Rob.I_constr_t_red)+length(Rob.I_constr_r_red));
% Hier werden die strukturellen FG des Roboters benutzt und nicht die
% Aufgaben-FG. Ist besonders für 3T2R relevant. Hier ist die Jacobi-Matrix
% bezogen auf die FG der Plattform ohne Bezug zur Aufgabe.
if dof_3T2R && all(Rob.I_EE == [1 1 1 1 1 1])
  % nur für aufgabenredundante 3T3R-PKM
  Jinv_ges = NaN(nt, 6*Rob.NJ);
  JinvD_ges = zeros(nt, 6*Rob.NJ);
else
  Jinv_ges = NaN(nt, sum(Rob.I_EE)*Rob.NJ);
  JinvD_ges = zeros(nt, sum(Rob.I_EE)*Rob.NJ);
end
% Gelenkkonfiguration, bei der Nebenbed. 3 (Kondition) das letzte mal
% berechnet wurde
q_wn5 = inf(Rob.NJ,1);
% Gradient von Nebenbedingung 5
h5dq = NaN(1,Rob.NJ);
% Zählung in Rob.NL: Starrkörper der Beinketten, Gestell und Plattform. 
% Hier werden nur die Basis-KS der Beinketten und alle bewegten Körper-KS
% der Beine angegeben.
JointPos_all = NaN(nt, (1+Rob.NL-2+Rob.NLEG)*3);

qk0 = q0;
qDk0 = zeros(Rob.NJ,1);
% Eingabe für Positions-IK für Korrekturschritt. Müssen konsistent zu
% ParRob/invkin2_traj sein. Auch konsistent mit SerRob/invkin2, falls von
% dort übernommen
s_pik = struct(...
  'K', ones(Rob.NJ,1), ... % Verstärkung
  'Kn', zeros(Rob.NJ,1), ... % Verstärkung ... hat keine Wirkung
  ... % keine Nullraum-Optim. bei IK-Berechnung auf Positionsebene
  'wn', zeros(3,1), ...
  'normalize', false, ... % würde Sprung erzeugen
  'n_min', 0, ... % Minimale Anzahl Iterationen
  'n_max', 1000, ... % Maximale Anzahl Iterationen
  ... % % Keine Herunterskalierung bei Grenzüberschreitung (würde Stillstand 
  ... % erzeugen; da keine Nullraumbewegung gemacht wird, kann die Grenz-
  ... % verletzung sowieso nicht verhindert werden.
  'scale_lim', 0.0, ... 
  'Phit_tol', 1e-9, ... % Toleranz für translatorischen Fehler
  'Phir_tol', 1e-9,... % Toleranz für rotatorischen Fehler
  'maxrelstep', 0.05, ... % Maximale Schrittweite relativ zu Grenzen
  'retry_limit', 0); % keine Neuversuche (würde Sprung erzeugen)


% Eingabe s_inv3 struktuieren
s_inv3 = s_pik;
s_inv3.maxstep_ns = 0*ones(Rob.NJ,1); % hat keine Wirkung
s_inv3.maxrelstep_ns = 0.005; % hat keine Wirkung
for f = fields(s_inv3)'
  if isfield(s, f{1}) && ~strcmp(f{1}, 'wn')
    s_inv3.(f{1}) = s.(f{1});
  end
end
% Eingabe s_ser struktuieren
s_ser = s_pik;
s_ser.reci = false; % Standardmäßig keine reziproken Euler-Winkel
for f = fields(s_ser)'
  if isfield(s, f{1}) && ~strcmp(f{1}, 'wn')
    s_ser.(f{1}) = s.(f{1});
  end
end
qlim = cat(1, Rob.Leg.qlim);
qDlim = cat(1, Rob.Leg.qDlim);
qDDlim = cat(1, Rob.Leg.qDDlim);
if ~all(isnan(qDlim(:)))
  limits_qD_set = true;
  qDmin = qDlim(:,1);
  qDmax = qDlim(:,2);
else
  limits_qD_set = false;
  qDmin = -inf(Rob.NJ,1);
  qDmax =  inf(Rob.NJ,1);
end
if ~all(isnan(qDDlim(:)))
  limits_qDD_set = true;
  qDDmin = qDDlim(:,1);
  qDDmax = qDDlim(:,2);
else
  limits_qDD_set = false;
  qDDmin = -inf(Rob.NJ,1);
  qDDmax =  inf(Rob.NJ,1);
end
wn = [s.wn;zeros(5-length(s.wn),1)]; % Fülle mit Nullen auf, falls altes Eingabeformat
if any(wn ~= 0)
  nsoptim = true;
else
  % Keine zusätzlichen Optimierungskriterien
  nsoptim = false;
end
% Vergleiche FG der Aufgabe und FG des Roboters
if ~dof_3T2R
  % TODO: Hier noch echte Prüfung der FG des Roboters oder detailliertere
  % Fallunterscheidungen notwendig.
  nsoptim = false;
  % Deaktiviere limits_qD_set, wenn es keinen Nullraum gibt
  limits_qD_set = false;
elseif all(Rob.I_EE == [1 1 1 1 1 0])
  % Strukturelle 3T2R-PKM: Es gibt keinen Nullraum
  limits_qD_set = false; % qD nicht beeinflussbar
end
% Variablen initialisieren (werden nicht in jedem Ausführungspfad benötigt)
if dof_3T2R && all(Rob.I_EE == [1 1 1 1 1 1])
  JD_x_inv = NaN(Rob.NJ,6); % konsistent zur obigen Initialisierung von JinvD_ges
else
  JD_x_inv = NaN(Rob.NJ,sum(Rob.I_EE));
end
Phi_q_alt = zeros(length(Rob.I_constr_t_red)+length(Rob.I_constr_r_red), Rob.NJ);
Phi_q_voll_alt = zeros(6*Rob.NLEG, Rob.NJ);
Phi_qD_voll = Phi_q_voll_alt;
Phi_x_alt = zeros(length(Rob.I_constr_t_red)+length(Rob.I_constr_r_red), sum(I_EE));
Phi_x_voll_alt = zeros(6*Rob.NLEG, 6);
Phi_xD_voll = Phi_x_voll_alt;

N = eye(Rob.NJ);
for k = 1:nt
  tic();
  x_k = X(k,:)';
  xD_k = XD(k,:)';
  xDD_k = XDD(k,:)';
  if k < nt % Schrittweite für letzten Zeitschritt angenommen wie vorletzter
    dt = T(k+1)-T(k); % Zeit bis zum nächsten Abtastpunkt
  end
  %% Gelenk-Position berechnen
  % Die Positions-IK wird nur als Korrekturschritt benutzt (Ausgleich
  % numerischer Fehler). Versuche erst die IK für alle Beinketten einzeln.
  % Annahme: Startwert und Vorwärts-Integration sind schon fast die Lösung.
  if any(mode_IK == [1 3])
    % Aufruf der Einzel-Beinketten-Funktion (etwas schneller, falls mit mex)
    [q_k, Phi_k, Tc_stack_k] = Rob.invkin_ser(x_k, qk0, s_ser);
  end
  % Falls obige IK nicht erfolgreich (aufgrund ungeklärter Ursachen),
  % versuche alternativen Algorithmus.
  if mode_IK == 2 || mode_IK == 3 && (any(abs(Phi_k(Rob.I_constr_t_red)) > s_ser.Phit_tol) || ...
      any(abs(Phi_k(Rob.I_constr_r_red)) > s_ser.Phir_tol))
    % 3T2R-Funktion. Wird hier aber nicht als 3T2R benutzt, da keine
    % Nullraumbewegung ausgeführt wird. Ist nur andere Berechnung.
    [q_k, Phi_k, Tc_stack_k] = Rob.invkin3(x_k, qk0, s_inv3);
  end
  % Abspeichern für Ausgabe.
  Q(k,:) = q_k;
  Phi(k,:) = Phi_k;
  JointPos_all(k,:) = Tc_stack_k(:,4);
  % Prüfe Erfolg der IK
  if any(abs(Phi_k(Rob.I_constr_t_red)) > s_ser.Phit_tol) || ...
     any(abs(Phi_k(Rob.I_constr_r_red)) > s_ser.Phir_tol)
    break; % Die IK kann nicht gelöst werden. Weitere Rechnung ergibt keinen Sinn.
  end
  %% Gelenk-Geschwindigkeit berechnen
  if ~dof_3T2R
    % Benutze die Ableitung der Geschwindigkeits-Zwangsbedingungen
    % (effizienter als Euler-Winkel-Zwangsbedingungen constr1...)
    Phi_q = Rob.constr4grad_q(q_k);
    Phi_x = Rob.constr4grad_x(x_k);
    J_x_inv = -Phi_q \ Phi_x;
  else % aufgabenredundante 3T3R-PKM und symmetrische und asymmetrische 3T2R-PKM
    [Phi_q,    Phi_q_voll] = Rob.constr3grad_q(q_k, x_k);
    [Phi_x_tmp,Phi_x_voll] = Rob.constr3grad_x(q_k, x_k);
    Phi_x=Phi_x_tmp(:,I_EE); % TODO: Schon in Funktion richtig machen.
    if all(Rob.I_EE == [1 1 1 1 1 1]) % Aufgabenredundanz
      % Berechne die Jacobi-Matrix basierend auf den vollständigen Zwangsbe-
      % dingungen (wird für Dynamik benutzt).
      J_x_inv = -Phi_q_voll \ Phi_x_voll;
    else % PKM mit strukturell nur 3T2R FG. Nehme die Jacobi mit reduzierten FG
      J_x_inv = -Phi_q \ Phi_x;
    end
  end
  if ~(nsoptim || limits_qD_set)
    if ~dof_3T2R || ... % beliebige PKM (3T0R, 3T1R, 3T3R)
        ~all(Rob.I_EE == [1 1 1 1 1 1]) % beliebige 3T2R-PKM
      qD_k = J_x_inv * xD_k(I_EE);
    else
      % Bei Aufgabenredundanz ist J_x_inv anders definiert
      qD_k = -Phi_q \ Phi_x * xD_k(I_EE);
    end
  else % Nullraum Optimierung
    % Korrekturterm für Linearisierungsfehler. Für PhiD_pre=0 entsteht die
    % normale inverse differentielle Kinematik. Mit dem Korrekturterm
    % bleibt die Geschwindigkeit konsistent zur Nullraumbewegung aus der
    % Beschleunigung
    PhiD_pre = Phi_q*qDk0;
    PhiD_korr = -PhiD_pre - Phi_x*xD_k(I_EE);
    qD_korr = Phi_q\PhiD_korr;
    qD_k = qDk0 + qD_korr;
    if debug % Erneuter Test
      PhiD_test = Phi_x*xD_k(I_EE) + Phi_q*qD_k;
      if any(abs(PhiD_test) > max(1e-10, max(abs(qD_k))/1e9)) % bei hohen Geschwindigkeiten ist die Abweichung größer; feine IK-Toleranz notwendig.
        error(['Korrektur der Geschwindigkeit hat nicht funktioniert (k=%d). ', ...
          'Fehler %1.1e'], k, max(abs(PhiD_test)));
      end
    end
  end
  %% Zeitableitung der Zwangsbedingungs-Gradienten und der Jacobi-Matrix
  % Zeitableitung der Zwangsbedingungs-Gradienten zuerst
  if simplify_acc % Keine explizite Berechnung der Zeitableitung (Zeitersparnis)
    if k > 1 % linksseitiger Differenzenquotient
      Phi_qD = (Phi_q - Phi_q_alt)/(T(k)-T(k-1));
      Phi_xD = (Phi_x - Phi_x_alt)/(T(k)-T(k-1));
      if dof_3T2R && all(Rob.I_EE == [1 1 1 1 1 1])
        Phi_qD_voll = (Phi_q_voll - Phi_q_voll_alt)/(T(k)-T(k-1));
        Phi_xD_voll = (Phi_x_voll - Phi_x_voll_alt)/(T(k)-T(k-1));
      end
    else
      Phi_qD = Phi_q_alt; % Mit Null initialisiert
      Phi_xD = Phi_x_alt;
      Phi_qD_voll = Phi_q_voll_alt;
      Phi_xD_voll = Phi_x_voll_alt;
    end
  else % Vollständige Berechnung
    if ~dof_3T2R
      Phi_qD = Rob.constr4gradD_q(q_k, qD_k);
      Phi_xD = Rob.constr4gradD_x(x_k, xD_k);
    else % alle 3T2R-PKM und aufgabenredundante 3T3R-PKM
      [Phi_qD,     Phi_qD_voll] = Rob.constr3gradD_q(q_k, qD_k, x_k, xD_k);
      [Phi_xD_tmp, Phi_xD_voll] = Rob.constr3gradD_x(q_k, qD_k, x_k, xD_k);
      Phi_xD=Phi_xD_tmp(:,I_EE); % TODO: Schon in Funktion richtig machen.
    end
  end
  % Danach getrennt die Zeitableitung von Jinv. Für den Differenzen-
  % quotienten genauer, wenn JD_x_inv über Phi_qD und Phi_xD gebildet wird
  % und nicht über einen eigenen Differenzenquotienten.
  if nargout >= 6
    if dof_3T2R && all(Rob.I_EE == [1 1 1 1 1 1]) % Aufgabenredundanz
      % Zeitableitung der inversen Jacobi-Matrix konsistent mit obiger
      % Form. Wird für Berechnung der Coriolis-Kräfte benutzt. Bei Kräften
      % spielt die Aufgabenredundanz keine Rolle.
      JD_x_inv = Phi_q_voll\(Phi_qD_voll*(Phi_q_voll\Phi_x_voll) - Phi_xD_voll);
    else % alle anderen
      JD_x_inv = Phi_q\(Phi_qD*(Phi_q\Phi_x) - Phi_xD);
    end
  end
  %% Gelenk-Beschleunigung berechnen
  % Direkte Berechnung aus der zweiten Ableitung der Zwangsbedingungen.
  % Siehe [3]. JD_x_inv ist nicht im Fall der Aufgabenredundanz definiert.
  qDD_k_T = -Phi_q\(Phi_qD*qD_k+Phi_xD*xD_k(I_EE)+Phi_x*xDD_k(I_EE));
  % Alternative Berechnung:
  % Die Rechnung mit Zeitableitung der inversen Jacobi funktioniert nur
  % bei vollem Rang. Bei strukturell 3T2R mit Rangverlust ist die
  % Rechnung numerisch ungünstig (Vermutung). Nutze die folgende Formel
  % daher nicht mehr. Mit dieser Formel ist die Beschleunigung für
  % 3T2R-PKM sonst bei Rangverlust nicht konsistent. Auch bei 3T0R
  % problematisch (da numerische Implementierung der beiden Formeln für
  % qDD_k_T anders ist. Gilt nur ohne AR und bei Beingelenkzahl=Anzahl EE-FG
  % qDD_k_T =  J_x_inv * xDD_k(I_EE) + JD_x_inv * xD_k(I_EE);
  if debug % Erneuter Test
    PhiDD_test3 = Phi_q*qDD_k_T + Phi_qD*qD_k + ...
      Phi_x*xDD_k(I_EE)+Phi_xD*xD_k(I_EE);
    if any(abs(PhiDD_test3) > max(1e-6, max(abs([qD_k;qDD_k_T]))/1e9)) % bei hohen Werten ist die Abweichung größer; feine IK-Toleranz notwendig.
      error(['Beschleunigung qDD_k_T erfüllt die kinematischen Bedingungen ', ...
        'nicht. Max. Fehler %1.2e'], max(abs(PhiDD_test3)));
    end
  end
  if nsoptim % Nullraumbewegung
    N = (eye(Rob.NJ) - pinv(Phi_q)* Phi_q); % Nullraum-Projektor
    % Berechne Gradienten der zusätzlichen Optimierungskriterien
    v = zeros(Rob.NJ, 1);
    if wn(1) ~= 0 % Quadratische Abweichung von Gelenkposition zur Mitte
      [~, h1dq] = invkin_optimcrit_limits1(q_k, qlim);
      v = v - wn(1)*h1dq';
    end
    if s.wn(2) ~= 0 % Hyperbolischer Abstand Gelenkposition zu Grenze
      [~, h2dq] = invkin_optimcrit_limits2(q_k, qlim);
      v = v - wn(2)*h2dq';
      if any(isinf(h2dq)), warning('h2dq Inf'); return; end
    end
    if wn(3) ~= 0 % Quadratische Gelenkgeschwindigkeiten
      [~, h3dq] = invkin_optimcrit_limits1(qD_k, qDlim);
      v = v - wn(3)*h3dq';
    end
    if wn(4) ~= 0 % Hyperbolischer Abstand Gelenkgeschwindigkeit zu Grenze
      [~, h4dq] = invkin_optimcrit_limits2(qD_k, qDlim);
      v = v - wn(4)*h4dq';
    end
    if wn(5) ~= 0 % Konditionszahl der geom. Matrix der Inv. Kin.
      if any(abs(q_k-q_wn5) > 3*pi/180) % seltenere Berechnung (Rechenzeit)
        condPhi = cond(Phi_q);
        for kkk = 1:Rob.NJ % Differenzenquotient für jede Gelenkkoordinate
          q_test = q_k; % ausgehend von aktueller Konfiguration
          q_test(kkk) = q_test(kkk) + 1e-6; % minimales Inkrement
          Phi_q_kkk = Rob.constr3grad_q(q_test, x_k);
          condPhi_kkk = cond(Phi_q_kkk);
          h5dq(kkk) = (log(condPhi_kkk)-log(condPhi))/1e-6;
        end
        q_wn5 = q_k;
      end
      v = v - wn(5)*h5dq';
    end
    qDD_N_pre = N * v;
  else
    qDD_N_pre = zeros(Rob.NJ, 1);
  end
  
  % Reduziere die Nullraumbeschleunigung weiter, falls Beschleunigungs-
  % Grenzen erreicht werden. Wird vor der Anpassung der Beschleunigung zur
  % Einhaltung der Geschwindigkeitsgrenzen gemacht, da Geschwindigkeit
  % wichtiger als Beschleunigung ist.
  if nsoptim && limits_qDD_set
    % Setze die Grenzen für qDD_N basierend auf gegebenen Grenzen für 
    % gesamte Beschleunigung und notwendige Beschleunigung qDD_T
    qDD_N_min = qDDmin - qDD_k_T;
    qDD_N_max = qDDmax - qDD_k_T;
    delta_ul_rel = (qDD_N_max - qDD_N_pre)./(qDD_N_max); % Überschreitung der Maximalwerte: <0
    delta_ll_rel = (-qDD_N_min + qDD_N_pre)./(-qDD_N_min); % Unterschreitung Minimalwerte: <0
    if any([delta_ul_rel;delta_ll_rel] < 0)
      if min(delta_ul_rel)<min(delta_ll_rel)
        % Verletzung nach oben ist die größere
        [~,I_max] = min(delta_ul_rel);
        scale = (qDD_N_max(I_max))/(qDD_N_pre(I_max));
      else
        % Verletzung nach unten ist maßgeblich
        [~,I_min] = min(delta_ll_rel);
        scale = (qDD_N_min(I_min))/(qDD_N_pre(I_min));
      end
      qDD_N_pre = scale*qDD_N_pre;
    end
  end

  if nsoptim && limits_qD_set % Nullraum-Optimierung erlaubt Begrenzung der Gelenk-Geschwindigkeit
    qDD_pre = qDD_k_T + qDD_N_pre;
    qD_pre = qD_k + qDD_pre*dt;
    deltaD_ul = (qDmax - qD_pre); % Überschreitung der Maximalwerte: <0
    deltaD_ll = (-qDmin + qD_pre); % Unterschreitung Minimalwerte: <0
    if any([deltaD_ul;deltaD_ll] < 0)
      if min(deltaD_ul)<min(deltaD_ll)
        % Verletzung nach oben ist die größere
        [~,I_worst] = min(deltaD_ul);
        qDD_lim_I = (qDmax(I_worst)-qD_k(I_worst))/dt;% [3]/(3)
      else
        % Verletzung nach unten ist maßgeblich
        [~,I_worst] = min(deltaD_ll);
        qDD_lim_I = (qDmin(I_worst)-qD_k(I_worst))/dt;
      end
      qD_pre_h = qD_pre;
      qD_pre_h(~(deltaD_ll<0|deltaD_ul<0)) = 0; % Nur Reduzierung, falls Grenze verletzt
      [~, hdqD] = invkin_optimcrit_limits1(qD_pre_h, qDlim);
      qDD_N_h = N * (-hdqD');
      % Normiere den Vektor auf den am stärksten grenzverletzenden Eintrag
      qDD_N_he = qDD_N_h/qDD_N_h(I_worst); % [3]/(5)
      % Stelle Nullraumbewegung so ein, dass schlechtester Wert gerade so
      % an der Grenze landet.
      qDD_N_korr_I = -qDD_pre(I_worst) + qDD_lim_I; % [3]/(7)
      % Erzeuge kompletten Vektor als durch Skalierung des Nullraum-Vektors
      qDD_N_korr = qDD_N_korr_I*qDD_N_he; % [3]/(8)
      qDD_N_post = qDD_N_pre+qDD_N_korr; % [3]/(6)
    else
      qDD_N_post = qDD_N_pre;
    end
  else
    qDD_N_post = qDD_N_pre;
  end
  qDD_k = qDD_k_T + qDD_N_post;
  
  % Teste die Beschleunigung (darf die Zwangsbedingungen nicht verändern)
  if debug
    % Das wäre eigentlich gar nicht notwendig, wenn die Beschleunigung
    % eine korrekte Nullraumbewegung ausführt.
    PhiDD_pre = Phi_q*qDD_k + Phi_qD*qD_k;
    PhiDD_korr = -PhiDD_pre - (Phi_x*xDD_k(I_EE)+Phi_xD*xD_k(I_EE));
    if any(abs(PhiDD_korr) > max(1e-7, max(abs(qDD_k))/1e9)) % bei hohen Beschleunigungen ist die Abweichung größer; feine IK-Toleranz notwendig.
      error(['Zeitschritt %d/%d: Beschleunigung ist nicht konsistent nach ', ...
        'Nullraumbewegung. Fehler %1.1e'], k, nt, max(abs(PhiDD_korr)));
      % Dieser Teil sollte nicht ausgeführt werden müssen (s.o.)
      qDD_korr = Phi_q\PhiDD_korr; %#ok<UNRCH>
      qDD_k = qDD_k + qDD_korr;
      % Nochmal testen
      PhiDD_test2 = Phi_q*qDD_k + Phi_qD*qD_k + ...
        Phi_x*xDD_k(I_EE)+Phi_xD*xD_k(I_EE);
      if any(abs(PhiDD_test2) > 1e-10)
        error('Korrektur der Beschleunigung hat nicht funktioniert');
      end
    end
  end
  %% Anfangswerte für Positionsberechnung in nächster Iteration
  % Berechne Geschwindigkeit aus Linearisierung für nächsten Zeitschritt
  qDk0 = qD_k + qDD_k*dt;
  % Aus Geschwindigkeit berechneter neuer Winkel für den nächsten Zeitschritt
  % Taylor-Reihe bis 2. Ordnung für Position (Siehe [2])
  qk0 = q_k + qD_k*dt + 0.5*qDD_k*dt^2;
  if any(isnan(qk0))
    break; % aufgrund von Singularität o.ä. unendlich hohe Werte
  end
  %% Ergebnisse speichern
  QD(k,:) = qD_k;
  QDD(k,:) = qDD_k;
  if nargout >= 5
    Jinv_ges(k,:) = J_x_inv(:);
  end
  if nargout >= 6
    JinvD_ges(k,:) = JD_x_inv(:);
  end
  Phi_q_alt = Phi_q;
  Phi_x_alt = Phi_x;
end
