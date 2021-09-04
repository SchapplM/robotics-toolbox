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
%   Struktur mit Eingabedaten. Felder, siehe Quelltext. Auswahl:
%   .wn [10x1] Gewichtungen der Zielfunktionen für Nullraumbewegung
%     (1) Quadratischer Abstand der Gelenkkoordinaten von ihrer Mitte
%     (2) Hyperbolischer Abstand der Gelenkkoordinaten von ihren Grenzen
%     (3) Quadratischer Abstand der Gelenkgeschwindigkeiten von ihrer Mitte
%     (4) Hyperbolischer Abstand der Gelenkgeschwindigkeiten von ihren Grenzen
%     (5) Konditionszahl der geometrischen Matrix der Inv. Kin.
%     (6) Konditionszahl der PKM-Jacobi-Matrix (Antriebe zu Plattform)
%     (7) Wie Eintrag 1, aber auf Geschwindigkeitsebene
%     (8) Wie Eintrag 2, aber auf Geschwindigkeitsebene
%     (9) Wie Eintrag 5, aber auf Geschwindigkeitsebene
%    (10) Wie Eintrag 6, aber auf Geschwindigkeitsebene
%    (11) Abstand der Kollisionskörper voneinander (hyperbolisch gewertet)
%    (12), wie 11, aber auf Geschwindigkeitsebene
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
% Stats
%   Struktur mit Detail-Ergebnissen für den Verlauf der Berechnung
% 
% Siehe auch: SerRob/invkin_traj bzw. SerRob/invkin2_traj

% Quelle:
% [2] Aufzeichnungen Schappler vom 11.12.2018
% [3] Aufzeichnungen Schappler vom 06.07.2020
% [RMG16] Reiter et al.: Inverse Kinematics in Minimum-Time Trajectory
% Planning for Kinematically Redundant Manipulators (2016)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-02
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [Q, QD, QDD, Phi, Jinv_ges, JinvD_ges, JointPos_all, Stats] = invkin_traj(Rob, X, XD, XDD, T, q0, s)

%% Initialisierung
s_std = struct( ...
  'simplify_acc', false, ... % Berechnung der Beschleunigung vereinfachen
  'mode_IK', 3, ...  % 1=Seriell-IK, 2=PKM-IK, 3=beide
  ... % Abschaltung des Optimierungskriteriums der hyperbolischen Grenzen
  ... % Kriterium standardmäßig 5% vor oberer und unterer Grenze einschalten.
  'optimcrit_limits_hyp_deact', 0.9, ...
  ... % Grenze zum Umschalten des Koordinatenraums der Nullraumbewegung
  'thresh_ns_qa', 1, ... % immer vollständigen Gelenkraum benutzen
  'wn', zeros(12,1), ... % Gewichtung der Nebenbedingung. Standard: Ohne
  'enforce_qlim', true, ... % Einhaltung der Positionsgrenzen durch Nullraumbewegung (keine Optimierung)
  'collbodies_thresh', 1.5, ... % Vergrößerung der Kollisionskörper für Aktivierung des Ausweichens
  'debug', false); % Zusätzliche Test-Berechnungen
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
taskred_rot = false;

if any(s.wn ~= 0) && sum(Rob.I_EE) > sum(Rob.I_EE_Task)
  % Nullraumoptimierung nur möglich, falls FG da sind. TODO: Das
  % berücksichtigt noch nicht den Fall von 3T3R-PKM in 3T0R-Aufgaben.
  nsoptim = true;
else
  % Keine zusätzlichen Optimierungskriterien
  nsoptim = false;
end
if Rob.I_EE(6) && ~ Rob.I_EE_Task(6)
  taskred_rot = true;
end
if all(Rob.I_EE == [1 1 1 1 1 0])
  dof_3T2R = true;
else
  dof_3T2R = false;
end

if nargout == 6
  % Wenn Jacobi-Zeitableitung als Ausgabe gefordert ist, kann die
  % vollständige Formel für die Beschleunigung benutzt werden
  simplify_acc = false;
else
  % Benutze vollständige Formel entsprechend Einstellungsparameter
  simplify_acc = s.simplify_acc;
end



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

% Eingabe s_inv3 strukturieren
s_inv3 = s_pik;
s_inv3.maxstep_ns = 0; % hat keine Wirkung
s_inv3.maxrelstep_ns = 0.005; % hat keine Wirkung
for f = fields(s_inv3)'
  if isfield(s, f{1}) && ~strcmp(f{1}, 'wn')
    s_inv3.(f{1}) = s.(f{1});
  end
end
% Eingabe s_ser strukturieren
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
if ~all(isnan(qlim(:)))
  limits_q_set = true;
  qmin = qlim(:,1);
  qmax = qlim(:,2);
else
  limits_q_set = false;
  qmin = -inf(Rob.NJ,1);
  qmax =  inf(Rob.NJ,1);
end
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
% Schwellwert in Gelenkkoordinaten für Aktivierung des Kriteriums für 
% hyperbolisch gewichteten Abstand von den Grenzen.
qlim_thr_h2 = repmat(mean(qlim,2),1,2) + repmat(qlim(:,2)-qlim(:,1),1,2).*...
  repmat([-0.5, +0.5]*s.optimcrit_limits_hyp_deact,Rob.NJ,1);
wn = [s.wn;zeros(12-length(s.wn),1)]; % Fülle mit Nullen auf, falls altes Eingabeformat

% Definitionen für die Kollisionsprüfung
collbodies_ns = Rob.collbodies;
maxcolldepth = 0;
collobjdist_thresh = 0;
if any(wn(11:12))
  % Kollisionskörper für die Kollisionserkennung z.B. 50% größer machen.
  collbodies_ns.params(collbodies_ns.type==6,1) = ... % Kapseln (Direktverbindung)
    s.collbodies_thresh*collbodies_ns.params(collbodies_ns.type==6,1);
  collbodies_ns.params(collbodies_ns.type==13,7) = ... % Kapseln (Basis-KS)
    s.collbodies_thresh*collbodies_ns.params(collbodies_ns.type==13,7);
  collbodies_ns.params(collbodies_ns.type==4|collbodies_ns.type==15,4) = ... % Kugeln
    s.collbodies_thresh*collbodies_ns.params(collbodies_ns.type==4|collbodies_ns.type==15,4);
  % Maximal mögliche Eindringtiefe bestimmen um daraus die Grenzen der
  % hyperbolischen Kollisionsfunktion zu bestimmen.
  % Ist eine etwas größere Schätzung (abhängig von relativer Größe von
  % Kugeln und Zylindern)
  maxcolldepth = 2*max([0;collbodies_ns.params(collbodies_ns.type==6,1);  ...% 1. Eintrag damit nicht leer
                        collbodies_ns.params(collbodies_ns.type==13,7); ...
                        collbodies_ns.params(collbodies_ns.type==4|collbodies_ns.type==15,4)]);
  % Vergrößere den Wert darüber hinaus. Der Wert unendlich sollte nie
  % erreicht werden (Probleme bei Gradientenbildung).
  maxcolldepth = 1.05*maxcolldepth;
  % Abstand der Objekte, ab dem die Zielfunktion anfängt (bei größeren
  % Abständen ist sie Null). Dies Wert muss kleiner sein als der, ab dem die
  % Erkennung beginnt. Unklar, ob dieser Wert immer passt. (Geht auch so).
  % Die Erkennung wird durch `collbodies_ns` bestimmt. Diese müssen also
  % eher zu groß gewählt werden. TODO: Geometrische Berechnung.
  collobjdist_thresh = 0.15 * maxcolldepth;
end

% Grenze zum Umschalten zwischen Nullraumbewegung in Antriebs- oder
% Gesamtkoordinaten. Ist die Konditionszahl schlechter, wird in Gesamt-
% koordinaten gerechnet. In Antriebskoordinaten weniger Rechenaufwand.
thresh_ns_qa = s.thresh_ns_qa;
% Vergleiche FG der Aufgabe und FG des Roboters
if ~taskred_rot % Bislang nur diese Redundanzart geprüft
  % Keine Redundanz vorliegend
  redundant = false;
else
  redundant = true;
end
% Variablen initialisieren (werden nicht in jedem Ausführungspfad benötigt)
JD_x_inv = NaN(Rob.NJ,sum(Rob.I_EE));
Phi_q_alt = zeros(length(Rob.I_constr_t_red)+length(Rob.I_constr_r_red), Rob.NJ);
Phi_q_voll_alt = zeros(6*Rob.NLEG, Rob.NJ);
Phi_qD_voll = Phi_q_voll_alt;
Phi_x_alt = zeros(length(Rob.I_constr_t_red)+length(Rob.I_constr_r_red), sum(I_EE));
Phi_x_voll_alt = zeros(6*Rob.NLEG, 6);
Phi_xD_voll = Phi_x_voll_alt;
N = eye(Rob.NJ);

nt = length(T);
Q = NaN(nt, Rob.NJ);
QD = Q;
QDD = Q;
Phi = NaN(nt, length(Rob.I_constr_t_red)+length(Rob.I_constr_r_red));
% Definition der Jacobi-Matrix.
% Hier werden die strukturellen FG des Roboters benutzt und nicht die
% Aufgaben-FG. Ist besonders für 3T2R relevant. Dort ist die Jacobi-Matrix
% bezogen auf die FG der Plattform ohne Bezug zur Aufgabe.
Jinv_ges = NaN(nt, sum(Rob.I_EE)*Rob.NJ);
JinvD_ges = zeros(nt, sum(Rob.I_EE)*Rob.NJ);
% Zählung in Rob.NL: Starrkörper der Beinketten, Gestell und Plattform. 
% Hier werden nur die Basis-KS der Beinketten und alle bewegten Körper-KS
% der Beine angegeben.
JointPos_all = NaN(nt, (1+Rob.NL-2+Rob.NLEG)*3);
qk0 = q0;
qDk0 = zeros(Rob.NJ,1);
qaD_N_pre_alt = zeros(sum(Rob.I_qa),1);
qD_N_pre_alt = zeros(Rob.NJ,1);
qaDD_N_pre1 = zeros(sum(Rob.I_qa),1);
qDD_N_pre1 = zeros(Rob.NJ,1);
Stats = struct('h', NaN(nt,1+7));
h = zeros(7,1);

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
  if ~taskred_rot && ~dof_3T2R
    % Benutze die Ableitung der Geschwindigkeits-Zwangsbedingungen
    % (effizienter als Euler-Winkel-Zwangsbedingungen constr1...)
    Phi_q = Rob.constr4grad_q(q_k);
    Phi_x = Rob.constr4grad_x(x_k);
    J_x_inv = -Phi_q \ Phi_x;
  else % aufgabenredundante 2T1R/3T1R/3T3R-PKM und symmetrische und asymmetrische 3T2R-PKM
    [Phi_q,    Phi_q_voll] = Rob.constr3grad_q(q_k, x_k);
    [Phi_x_tmp,Phi_x_voll] = Rob.constr3grad_x(q_k, x_k);
    Phi_x=Phi_x_tmp(:,I_EE); % TODO: Schon in Funktion richtig machen.
    if taskred_rot % Aufgabenredundanz
      % Berechne die Jacobi-Matrix basierend auf den vollständigen Zwangsbe-
      % dingungen (wird für Dynamik benutzt).
      % TODO: Hier wird bei 2T1R auch die volle Matrix (mit Nullen) benutzt
      % (in dem Fall unnötig große Matrix zu invertieren).
      J_x_inv = -Phi_q_voll \ Phi_x_voll(:,Rob.I_EE);
    else %dof_3T2R; PKM mit strukturell nur 3T2R FG. Nehme die Jacobi mit reduzierten FG
      J_x_inv = -Phi_q \ Phi_x;
    end
  end
  if ~(nsoptim || redundant)
    if ~taskred_rot || ... % beliebige PKM (3T0R, 3T1R, 3T3R) ohne Aufg.Red.
        dof_3T2R % beliebige 3T2R-PKM
      qD_k = J_x_inv * xD_k(I_EE);
    else
      % Bei Aufgabenredundanz ist J_x_inv anders definiert
      qD_k = -Phi_q \ Phi_x * xD_k(I_EE);
    end
  else % Nullraum-Optimierung
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
      if taskred_rot
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
    if ~taskred_rot && ~dof_3T2R
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
    if taskred_rot % Aufgabenredundanz
      % Zeitableitung der inversen Jacobi-Matrix konsistent mit obiger
      % Form. Wird für Berechnung der Coriolis-Kräfte benutzt. Bei Kräften
      % spielt die Aufgabenredundanz keine Rolle.
      JD_x_inv = Phi_q_voll\(Phi_qD_voll*(Phi_q_voll\Phi_x_voll(:,Rob.I_EE))...
        -Phi_xD_voll(:,Rob.I_EE));
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
  if nsoptim || redundant
    % Nullraum-Projektor für vollständige Gelenkkoordinaten. Muss auch für
    % Grenzkorrekturen weiter unten berechnet werden
    N = (eye(Rob.NJ) - pinv(Phi_q)* Phi_q);
    condPhi = cond(Phi_q); % Benötigt als Singularitätskennzahl
  end
  % Setze die Grenzen für qDD_N basierend auf gegebenen Grenzen für 
  % gesamte Beschleunigung und notwendige Beschleunigung qDD_T
  qDD_N_min = qDDmin - qDD_k_T;
  qDD_N_max = qDDmax - qDD_k_T;
  qDD_N_pre = zeros(Rob.NJ, 1);
  if nsoptim % Nullraumbewegung: Zwei Koordinatenräume dafür möglich (s.u.)
    h(5) = condPhi;
    Jinv_ax = J_x_inv(Rob.I_qa,:); % Jacobi-Matrix Antriebe vs Plattform
    condJ = cond(Jinv_ax);
    h(6) = condJ;
    % Bestimme Ist-Lage der Plattform (bezogen auf erste Beinkette).
    % Notwendig für die constr4-Methode für Jacobi-Matrix.
    % Rotation um z-Achse aus Zwangsbedingung 3 ablesbar. Sonst dir.Kin. erste Beinkette.
    % (wird aber aktuell nur für Debug-Modus benutzt).
    [~, Phi_r] = Rob.constr3_rot(q_k, x_k);
    x_k_ist = x_k + [zeros(5,1);Phi_r(1)]; % Rob.fkineEE_traj(q_k')'
    % Inkrement der Plattform für Prüfung der Optimierungskriterien.
    % Annahme: Nullraum-FG ist die Drehung um die z-Achse (Rotationssymm.)
    % Dadurch numerische Bestimmung der partiellen Ableitung nach 6. Koord.
    % Klein wählen, damit lineare Näherung nicht verlassen wird. Nicht zu
    % klein wählen, damit numerische Rundungsfehler nicht zu stark sind.
    xD_test = [zeros(5,1);1e-6];
    % Benutze nur für die Jacobi-Matrix die red. Koordinaten (2T1R, 3T0R)
    qD_test = J_x_inv * xD_test(Rob.I_EE); % Gelenkänderung der Nullraumbewegung
    % Führe Nullraumbewegung in Antriebskoordinaten durch. Geht nur, wenn
    % Jacobi gut konditioniert und Antriebe definiert sind.
    if condJ < thresh_ns_qa && sum(Rob.I_qa) == sum(Rob.I_EE)
      % Berechne die Nullraumbewegung im Raum der Antriebskoordinaten
      % Jacobi-Matrix bezogen auf Antriebe und Plattform
      J_ax = inv(J_x_inv(Rob.I_qa,:));
      J_ax_3T3R = zeros(6,sum(Rob.I_qa));
      J_ax_3T3R(Rob.I_EE,:) = J_ax;
      % Jacobi-Matrix zur Umrechnung von Antrieben auf Gesamtgelenke
      J_q_qa = J_x_inv * J_ax; %#ok<MINV>
      % Nullraum-Projektor bezogen auf analytische Jacobi-Matrix ohne
      % letzte redundante Koordinate
      Na = (eye(sum(Rob.I_qa)) - pinv(J_ax_3T3R(Rob.I_EE_Task,:))* J_ax_3T3R(Rob.I_EE_Task,:));
      v_qaD = zeros(sum(Rob.I_qa), 1);
      v_qaDD = zeros(sum(Rob.I_qa), 1);
      % Bestimme den Gradienten der Optimierungskriterien zuerst bezüglich
      % der redundanten EE-Koordinate und rechne dann auf die Antriebe um.
      if wn(1) ~= 0 || wn(7) ~= 0 % Quadratische Abweichung von Gelenkposition zur Mitte
        h(1) = invkin_optimcrit_limits1(q_k, qlim);
        h1_test = invkin_optimcrit_limits1(q_k+qD_test, qlim);
        h1drz = (h1_test-h(1))/xD_test(6);
        h1dqa = h1drz * J_ax(end,:);
        v_qaD = v_qaD - wn(7)*h1dqa(:);
        v_qaDD = v_qaDD - wn(1)*h1dqa(:);
      end
      if wn(2) ~= 0 || wn(8) ~= 0 % Hyperbolischer Abstand Gelenkposition zu Grenze
        [h(2), h2dq_diff] = invkin_optimcrit_limits2(q_k, qlim, qlim_thr_h2);
        % Projektion von Gesamt- in Antriebskoordinaten
        h2dqa = h2dq_diff * J_q_qa;
        % Berechnung mit Differenzenquotient ist numerisch nicht robust
        % nahe/über Grenzen (unendliche hohe Werte für Differenz)
%         h2_test = invkin_optimcrit_limits2(q_k+qD_test, qlim);
%         h2drz = (h2_test-h(2))/xD_test(6);
%         h2dqa = h2drz * J_ax(end,:);
        v_qaD = v_qaD - wn(8)*h2dqa(:);
        v_qaDD = v_qaDD - wn(2)*h2dqa(:);
      end
      if wn(3) ~= 0 % Quadratische Gelenkgeschwindigkeiten
        [h(3), h3dq_diff] = invkin_optimcrit_limits1(qD_k, qDlim);
        % Projektion von Gesamt- in Antriebskoordinaten
        h3dqa = h3dq_diff * J_q_qa;
        % Berechnung mit Differenzenquotient ist leicht ungenauer:
%         h3_test = invkin_optimcrit_limits1(qD_k+qD_test, qDlim);
%         h3drz = (h3_test-h(3))/(xD_test(6));
%         h3dqa = h3drz * J_ax(end,:);
        v_qaDD = v_qaDD - wn(3)*h3dqa(:);
      end
      if wn(4) ~= 0 % Hyperbolischer Abstand Gelenkgeschwindigkeit zu Grenze
        h(4) = invkin_optimcrit_limits2(qD_k, qDlim);
        h4_test = invkin_optimcrit_limits2(qD_k+qD_test, qDlim);
        h4drz = (h4_test-h(4))/xD_test(6);
        h4dqa =  h4drz * J_ax(end,:);
        v_qaDD = v_qaDD - wn(4)*h4dqa(:);
      end
      if wn(5) ~= 0 || wn(9) ~= 0 % Konditionszahl der geom. Matrix der Inv. Kin.
        Phi_q_test = Rob.constr3grad_q(q_k+qD_test, x_k+xD_test);
        h5_test = cond(Phi_q_test);
        h5drz = (h5_test-h(5))/xD_test(6);
        h5dqa = h5drz * J_ax(end,:);
        v_qaD = v_qaD - wn(9)*h5dqa(:);
        v_qaDD = v_qaDD - wn(5)*h5dqa(:);
      end
      if wn(6) ~= 0 || wn(10) ~= 0 % Konditionszahl der PKM-Jacobi-Matrix
        h(6) = condJ;
        % Alternative 3 für Berechnung: Inkrement für beide Gradienten bestimmen.
        % Benutze constr3, da dieser Term schon oben berechnet wurde. Weitere Alternativen siehe Debug-Teil.
        % Entspricht direkt dem Differenzenquotienten (mit Auswertung der inversen Jacobi-Matrix
        % an Test-Konfiguration q_k+eps.
        [~,Phi_q_voll_test] = Rob.constr3grad_q(q_k+qD_test, x_k+xD_test);
        [~,Phi_x_voll_test] = Rob.constr3grad_x(q_k+qD_test, x_k+xD_test);
        J_x_inv_test_v3 = -Phi_q_voll_test\Phi_x_voll_test;
        h6_test_v3 = cond(J_x_inv_test_v3(Rob.I_qa,:));       
        % Gradient bzgl. redundanter Koordinate durch Differenzenquotient
        h6drz_v3 = (h6_test_v3-h(6))/xD_test(6);

        if false && ... % Aus numerischen Gründen liefern alternative Modellierungen manchmal andere Ergebnisse (Rundungsfehler und Konditionsberechnung). TODO: Wirklich nur Numerik?
            debug && abs(h6_test_v3-h(6))>1e-10 && ... % Vergleich lohnt sich nur, wenn numerisch unterschiedlich
            h(6) < 1e2 % funktioniert schlecht bei schlechter Kondition. TODO: Schwellwert zu niedrig.
          % Alternative 1 für Berechnung: Allgemeine Zwangsbedingungen.
          % Nicht benutzen, da nicht mit constr3-Ergebnissen von oben kombinierbar.
          [~,Phi_q_voll_v4] = Rob.constr4grad_q(q_k);
          [~,Phi_x_voll_v4] = Rob.constr4grad_x(x_k_ist);
          [~,Phi_q_voll_test] = Rob.constr4grad_q(q_k+qD_test);
          [~,Phi_x_voll_test] = Rob.constr4grad_x(x_k_ist+xD_test);
          J_x_inv_test = -Phi_q_voll_test\Phi_x_voll_test(:,Rob.I_EE);
          h6_test_v1 = cond(J_x_inv_test(Rob.I_qa,:));
          % Gradient bzgl. redundanter Koordinate durch Differenzenquotient
          h6drz_v1 = (h6_test_v1-h(6))/xD_test(6);
          % Alternative 2: Differenzenquotient aus Differential der inv-
          % versen Jacobi (gleichwertig, aber unnötig kompliziert)
          PhiD_q_voll_v4 = Phi_q_voll_test-Phi_q_voll_v4;
          PhiD_x_voll_v4 = Phi_x_voll_test-Phi_x_voll_v4;
          J_x_inv_test_v2 = J_x_inv + ...
            Phi_q_voll_v4\PhiD_q_voll_v4/Phi_q_voll_v4*Phi_x_voll_v4(:,Rob.I_EE) + ...
            -Phi_q_voll_v4\PhiD_x_voll_v4(:,Rob.I_EE);
          h6_test_v2 = cond(J_x_inv_test_v2(Rob.I_qa,:));
          h6drz_v2 = (h6_test_v2-h(6))/xD_test(6);

          % Alternative 4 für Berechnung: Mit Differenzenquotient das Differential annähern.
          % (ist hier ausreichend genau).
          PhiD_q_voll_test = Phi_q_voll_test-Phi_q_voll;
          PhiD_x_voll_test = Phi_x_voll_test-Phi_x_voll;
          % Inverse Jacobi-Matrix als Inkrement. Nutze Formel für Zeitableitung
          % einer inversen Matrix (ähnlich wie bei Zeitableitungen).
          J_x_inv_test_v4 = J_x_inv + ...
            Phi_q_voll\PhiD_q_voll_test/Phi_q_voll*Phi_x_voll(:,Rob.I_EE) + ...
            -Phi_q_voll\PhiD_x_voll_test(:,Rob.I_EE);
          h6_test_v4 = cond(J_x_inv_test_v4(Rob.I_qa,:));
          h6drz_v4 = (h6_test_v4-h(6))/xD_test(6);

          % Debug-Teil: Vergleiche die vier Berechnungsalternativen.
          % Teilweise nur sehr geringer Unterschied in Jacobi-Matrix aber
          % dann größere Änderung in Konditionszahl.
          % [J_x_inv_test,J_x_inv_test_v2,J_x_inv_test_v3,J_x_inv_test_v4]-repmat(J_x_inv,1,4)
          % [h6_test_v1, h6_test_v2, h6_test_v3, h6_test_v4]-h(6)
          % [h6drz_v1,h6drz_v2,h6drz_v3,h6drz_v4];
          abserr_12 = h6drz_v1 - h6drz_v2;
          relerr_12 = abserr_12/h6drz_v1;
          if abs(abserr_12) > 1e-3 && abs(relerr_12)>1e-2
            error(['Modellierungen 1 vs 2 stimmen nicht ueberein ', ...
              '(abserr %1.1e, relerr %1.1e). condJ=%1.1e'], abserr_12, relerr_12, condJ);
          end
          abserr_34 = h6drz_v4 - h6drz_v3;
          relerr_34 = abserr_34/h6drz_v3;
          if abs(abserr_34) > 1e-3 && abs(relerr_34)>1e-2
            error(['Modellierungen 3 vs 4 stimmen nicht ueberein ', ...
              '(abserr %1.1e, relerr %1.1e). condJ=%1.1e'], abserr_34, relerr_34, condJ);
          end
          abserr_13 = h6drz_v1 - h6drz_v3;
          relerr_13 = abserr_13/h6drz_v3;
          if abs(abserr_13) > 1e-3 && abs(relerr_13)>1e-2
            error(['Modellierungen 1 vs 3 stimmen nicht ueberein ', ...
              '(abserr %1.1e, relerr %1.1e). condJ=%1.1e'], abserr_13, relerr_13, condJ);
          end
          abserr_24 = h6drz_v4 - h6drz_v2;
          relerr_24 = abserr_24/h6drz_v2;
          if abs(abserr_24) > 1e-3 && abs(relerr_24)>1e-2
            error(['Modellierungen 2 vs 4 stimmen nicht ueberein ', ...
              '(abserr %1.1e, relerr %1.1e). condJ=%1.1e'], abserr_24, relerr_24, condJ);
          end
        end
        % Projektion in Antriebskoordinaten
        h6dqa = h6drz_v3 * J_ax(end,:);
        v_qaD = v_qaD - wn(10)*h6dqa(:);
        v_qaDD = v_qaDD - wn(6)*h6dqa(:);
      end
      if wn(11) ~= 0 || wn(12) ~= 0 % Kollisionsprüfung
        % Kollisionserkennung im vergrößerten Warnbereich
        colldet = check_collisionset_simplegeom_mex(collbodies_ns, Rob.collchecks, ...
          Tc_stack_k(:,4)', struct('collsearch', true));
        if any(colldet)
          JP_test = [Tc_stack_k(:,4)'; NaN(1, size(Tc_stack_k,1))];
          [~, JP_test(2,:)] = Rob.fkine_coll(q_k+qD_test);
          % Kollisionsprüfung für alle Gelenkpositionen auf einmal. Prüfe
          % nur die Fälle, bei denen die vergrößerten Objekte bereits eine
          % Kollision angezeigt haben.
          [~, colldist_test] = check_collisionset_simplegeom_mex( ...
            Rob.collbodies, Rob.collchecks(colldet,:), JP_test, struct('collsearch', false));
          % Kollisions-Kriterium berechnen
          h(7) = invkin_optimcrit_limits2(-min(colldist_test(1,:)), ... % zurückgegebene Distanz ist zuerst negativ
            [-100*maxcolldepth, maxcolldepth], [-80*maxcolldepth, -collobjdist_thresh]);
          h7_test = invkin_optimcrit_limits2(-min(colldist_test(2,:)), ... % zurückgegebene Distanz ist zuerst negativ
            [-100*maxcolldepth, maxcolldepth], [-80*maxcolldepth, -collobjdist_thresh]);
          % Gradient bzgl. redundanter Koordinate durch Differenzenquotient
          h7drz = (h7_test-h(7))/xD_test(6);
          h7dqa = h7drz * J_ax(end,:);
          v_qaD = v_qaD - wn(10)*h7dqa(:);
          v_qaDD = v_qaDD - wn(6)*h7dqa(:);
        else
          h(7) = 0;
        end
      end
      % Begrenze die Werte für die Gradienten (können direkt an Grenzen
      % oder Singularitäten extrem werden). Dann Numerik-Fehler und keine
      % saubere Nullraumbewegung mehr möglich.
      if any(abs(v_qaD)>1e8),  v_qaD  = v_qaD* 1e8/max(abs(v_qaD));  end
      if any(abs(v_qaDD)>1e8), v_qaDD = v_qaDD*1e8/max(abs(v_qaDD)); end
      % Berechne die Nullraumbewegung im Gelenkraum aus den Gradienten
      qaD_N_pre = Na * v_qaD;
      qaDD_N_pre1 = Na*(qaD_N_pre-qaD_N_pre_alt)/dt;
      % Speichere den Altwert für den Differenzenquotienten
      qaD_N_pre_alt = qaD_N_pre;
      qDD_N_pre = J_q_qa * (Na * v_qaDD + qaDD_N_pre1);
      % Skaliere Wert herunter, damit Größenordnung ähnlich ist wie mit
      % zweiter Methode (vollständige Gelenkkoordinaten). TODO: Genaue
      % Formel noch unklar. So erstmal halb geraten und passt nicht ganz.
      qDD_N_pre = qDD_N_pre / norm(J_q_qa);
    end
    % Berechne Nullraumbewegung in vollständigen Gelenkkoordinaten.
    % Robuster, aber auch rechenaufwändiger. Daher nur benutzen, wenn
    % Kondition der Antriebe schlecht ist. Kein if-else, damit debug geht.
    if (condJ >= thresh_ns_qa || sum(Rob.I_qa) ~= sum(Rob.I_EE) || debug) && ...
        condPhi < 1e10 % numerisch nicht für singuläre PKM sinnvoll
      % Berechne Gradienten der zusätzlichen Optimierungskriterien
      % (bezogen auf vollständige Koordinaten)
      v_qD = zeros(Rob.NJ, 1);
      v_qDD = zeros(Rob.NJ, 1);
      if wn(1) ~= 0 || wn(7) ~= 0 % Quadratische Abweichung von Gelenkposition zur Mitte
        [h(1), h1dq] = invkin_optimcrit_limits1(q_k, qlim);
        v_qD = v_qD - wn(7)*h1dq(:);
        v_qDD = v_qDD - wn(1)*h1dq(:);
      end
      if wn(2) ~= 0 || wn(8) ~= 0 % Hyperbolischer Abstand Gelenkposition zu Grenze
        [h(2), h2dq] = invkin_optimcrit_limits2(q_k, qlim, qlim_thr_h2);
        v_qD = v_qD - wn(8)*h2dq(:);
        v_qDD = v_qDD - wn(2)*h2dq(:);
      end
      if wn(3) ~= 0 % Quadratische Gelenkgeschwindigkeiten
        [h(3), h3dq] = invkin_optimcrit_limits1(qD_k, qDlim);
        v_qDD = v_qDD - wn(3)*h3dq(:);
      end
      if wn(4) ~= 0 % Hyperbolischer Abstand Gelenkgeschwindigkeit zu Grenze
        [h(4), h4dq] = invkin_optimcrit_limits2(qD_k, qDlim);
        v_qDD = v_qDD - wn(4)*h4dq(:);
      end
      if wn(5) ~= 0 || wn(9) ~= 0 % Konditionszahl der geom. Matrix der Inv. Kin.
        Phi_q_test = Rob.constr3grad_q(q_k+qD_test, x_k+xD_test);
        h5_test = cond(Phi_q_test);
        if abs(h5_test-h(5)) < 1e-12
          h5dq = zeros(1,Rob.NJ); % Bei isotropen PKM kein Gradient möglich (aber Rundungsabweichungen)
        else
          h5dq = (h5_test-h(5))./(qD_test');
        end
        v_qD = v_qD - wn(9)*h5dq(:);
        v_qDD = v_qDD - wn(5)*h5dq(:);
      end
      if wn(6) ~= 0 || wn(10) ~= 0 % Konditionszahl der PKM-Jacobi-Matrix
        h(6) = condJ;
        % Siehe gleiche Berechnung oben.
        [~,Phi_q_voll_test] = Rob.constr3grad_q(q_k+qD_test,x_k+xD_test);
        [~,Phi_x_voll_test] = Rob.constr3grad_x(q_k+qD_test,x_k+xD_test);
        % Daraus mit Differenzenquotient das Differential annähern.
        % (ist hier ausreichend genau).
        % Alternative 1: Jacobi-Matrix direkt berechnen
        J_x_inv_test = -Phi_q_voll_test\Phi_x_voll_test(:,Rob.I_EE);
        h6_test_v1 = cond(J_x_inv_test(Rob.I_qa,:));
        
        if debug && abs(h6_test_v1-h(6)) > 1e-12 && h(6) < 1e8 && ...% Näherung nur bei akzeptabler Kondition vergleichbar
            cond(Phi_q_voll) < 500 % IK sollte nicht singulär sein
          % Teste zwei alternative Berechnungen (siehe oben bei Antriebskoord.)
          % Alternative 2: Über Jacobi-Matrix-Inkrement
          PhiD_q_voll_test = Phi_q_voll_test-Phi_q_voll;
          PhiD_x_voll_test = Phi_x_voll_test-Phi_x_voll;
          % Inverse Jacobi-Matrix als Inkrement. Nutze Formel für Zeitableitung
          % einer inversen Matrix (ähnlich wie bei Zeitableitungen).
          J_x_inv_test = J_x_inv + ...
            Phi_q_voll\PhiD_q_voll_test/Phi_q_voll*Phi_x_voll(:,Rob.I_EE) + ...
            -Phi_q_voll\PhiD_x_voll_test(:,Rob.I_EE);
          h6_test_v2 = cond(J_x_inv_test(Rob.I_qa,:));
          abserr_12 = h6_test_v1 - h6_test_v2;
          relerr_12 = abserr_12/(h6_test_v1-h(6));
          if abs(abserr_12) > 1e-3 * (1+log10(condJ)^2) && ...
             abs(relerr_12)>1e-2 * (1+log10(condJ)^2) % höhere Toleranz bei schlechter Kondition
            error(['Modellierungen 1 vs 2 stimmen nicht ueberein ', ...
              '(abserr %1.1e, relerr %1.1e). condJ=%1.1e'], abserr_12, relerr_12, condJ);
          end
        end
        if abs(h6_test_v1-h(6)) < 1e-12
          h6dq = zeros(1,Rob.NJ); % Bei isotropen PKM kein Gradient möglich (aber Rundungsabweichungen)
        else
          h6dq = (h6_test_v1-h(6))./(qD_test');
        end
        v_qD = v_qD - wn(10)*h6dq(:);
        v_qDD = v_qDD - wn(6)*h6dq(:);
      end
      if wn(11) ~= 0 || wn(12) ~= 0 % Kollisionsprüfung
        % Kollisionserkennung im vergrößerten Warnbereich
        colldet = check_collisionset_simplegeom_mex(collbodies_ns, Rob.collchecks, ...
          Tc_stack_k(:,4)', struct('collsearch', true));
        if any(colldet)
          JP_test = [Tc_stack_k(:,4)'; NaN(1, size(Tc_stack_k,1))];
          [~, JP_test(2,:)] = Rob.fkine_coll(q_k+qD_test);
          % Kollisionsprüfung für alle Gelenkpositionen auf einmal. Prüfe
          % nur die Fälle, bei denen die vergrößerten Objekte bereits eine
          % Kollision angezeigt haben.
          [~, colldist_test] = check_collisionset_simplegeom_mex( ...
            Rob.collbodies, Rob.collchecks(colldet,:), JP_test, struct('collsearch', false));
          % Kollisions-Kriterium berechnen
          h(7) = invkin_optimcrit_limits2(-min(colldist_test(1,:)), ... % zurückgegebene Distanz ist zuerst negativ
            [-100*maxcolldepth, maxcolldepth], [-80*maxcolldepth, -collobjdist_thresh]);
          h7_test = invkin_optimcrit_limits2(-min(colldist_test(2,:)), ... % zurückgegebene Distanz ist zuerst negativ
            [-100*maxcolldepth, maxcolldepth], [-80*maxcolldepth, -collobjdist_thresh]);
          h7dq = (h7_test-h(7))./(qD_test');
          v_qD = v_qD - wn(12)*h7dq(:);
          v_qDD = v_qDD - wn(11)*h7dq(:);
        else
          h(7) = 0;
        end
      end
      % Begrenze die Werte für die Gradienten (können direkt an Grenzen
      % oder Singularitäten extrem werden). Dann Numerik-Fehler und keine
      % saubere Nullraumbewegung mehr möglich.
      if any(abs(v_qD)>1e6),  v_qD  = v_qD* 1e6/max(abs(v_qD));  end
      if any(abs(v_qDD)>1e6), v_qDD = v_qDD*1e6/max(abs(v_qDD)); end
      % Berechne die Nullraumbewegung im Gelenkraum aus den Gradienten
      if condJ >= thresh_ns_qa || sum(Rob.I_qa) ~= sum(Rob.I_EE) || debug
        qD_N_pre = N * v_qD;
        qDD_N_pre1 = N*(qD_N_pre-qD_N_pre_alt)/dt;
        % Speichere den Altwert für den Differenzenquotienten
        qD_N_pre_alt = qD_N_pre;
        % Nullraum-Beschleunigung nach der Methode mit vollst. Gelenkraum
        qDD_N_pre_voll = qDD_N_pre1 + N * v_qDD;
      end
      if debug && condJ < thresh_ns_qa && sum(Rob.I_qa) == sum(Rob.I_EE) && ...
          any(abs(qDD_N_pre)>1e-6) && any(abs(qDD_N_pre_voll)>1e-6) % bei sehr kleinen Zahlen numerische Probleme für Test
        % Prüfe, ob beide Methoden (q vs qa) das gleiche Ergebnis geben.
        % Die Richtung ist gleich, aber nicht der Betrag. Vermutlich wegen
        % unterschiedlicher Vektorräume, in die projiziert wird.
        % TODO: Aktuell führen die D-Terme teilweise zu unterschiedlichen
        % Bewegungsrichtungen im Nullraum. Sollte nicht der Fall sein.
        test_qDD_N_ratio = qDD_N_pre./qDD_N_pre_voll;
        test_qDD_N_ratio(abs(qDD_N_pre_voll) < 1e-6) = NaN; % Test numerisch nicht sinnvoll
        % TODO: Die Richtung kann auseinanderlaufen, noch kein guter Test
        % möglich. Liegt an unterschiedlichen Größen der Gradienten.
        if all(abs(qDD_N_pre_voll)>1e-6) && any(test_qDD_N_ratio < 0) && ...
            sum(wn~=0)==1 % Bei einem einzigen Kriterium sollte es gehen.
          error('Die Richtung der Nullraumbewegung ist nicht gleich in beiden Koordinatenräumen');
        end
        % Prüfe, ob das Verhältnis der Nullraumbewegungen für alle Gelenke
        % gleich ist. Betrachte dafür nur Gelenke, die ungleich Null sind.
        % Ist eine Beschl. fast Null und eine andere sehr hoch, ist die
        % numerische Prüfung nicht genau genug.
        % Zunächst Prüfung bezogen auf vollständige Gelenk-Koordinaten
        [~,I_test_qDD_N_ratio_maxabs] = max(abs(test_qDD_N_ratio));
        I_nonzero = abs(qDD_N_pre_voll) > 1e-3 * max(abs(qDD_N_pre_voll));
        test_qDD_N_ratio_rel = 1-test_qDD_N_ratio/...
          test_qDD_N_ratio(I_test_qDD_N_ratio_maxabs);
        if ~any(isnan(test_qDD_N_ratio)) && ... % bei exakt h=0 kommt sonst Fehler
          abs(diff(minmax2(test_qDD_N_ratio_rel(I_nonzero)'))) > 5e-2 % 5% genau (für kleine Zahlenwerte)
          disp(test_qDD_N_ratio_rel);
          error(['Die Nullraumbewegung aus Antriebs- oder vollständigen ', ...
            'Koordinaten ist nicht gleichförmig (bzgl. vollständige)']);
        end
        % Gleiche Prüfung bezogen auf Antriebe
        test_qaDD_N_ratio = qaDD_N_pre1 ./ qDD_N_pre1(Rob.I_qa);
        test_qaDD_N_ratio_rel = 1-test_qaDD_N_ratio/test_qaDD_N_ratio(1);
        if ~any(isnan(test_qaDD_N_ratio)) && ... % bei exakt h=0 kommt sonst Fehler
          ...  % Teste auf 1% genau. Lasse mehr Fehler zu, wenn schlecht konditioniert.
          abs(diff(minmax2(test_qaDD_N_ratio_rel'))) > (1+condJ/1e3)*1e-2
          error(['Die Nullraumbewegung aus Antriebs- oder vollständigen ', ...
            'Koordinaten ist nicht gleichförmig (bzgl. Antriebe)']);
        end
        if cond(Phi_q) < 1e10
          % Die Alternative Berechnung lohnt sich nur zu testen, wenn die
          % IK-Jacobi nicht singulär ist (teilw. bei 3T1R der Fall)
          xD_N_test1 = J_ax * qDD_N_pre_voll(Rob.I_qa); %#ok<MINV>
        else
          xD_N_test1 = NaN(sum(Rob.I_EE),1);
        end
        xD_N_test1_3T3R = zeros(6,1); xD_N_test1_3T3R(Rob.I_EE) = xD_N_test1;
        xD_N_test2 = J_ax * qDD_N_pre(Rob.I_qa); %#ok<MINV>
        xD_N_test2_3T3R = zeros(6,1); xD_N_test2_3T3R(Rob.I_EE) = xD_N_test2;
        xD_nonns_abs = abs([xD_N_test1_3T3R(Rob.I_EE_Task);xD_N_test2_3T3R(Rob.I_EE_Task)]);
        xD_nonns_rel = abs([xD_N_test1_3T3R(Rob.I_EE_Task)/max(abs(xD_N_test1_3T3R)); ...
                        xD_N_test2(Rob.I_EE_Task)/max(abs(xD_N_test2))]);
        if any(xD_nonns_abs>1e-6*(1+condJ/1e3) & xD_nonns_rel > 1e-9*(1+condJ/1e3))
          error(['Nullraumbewegung ist nicht korrekt. Fehler für xD: ', ...
            'abs %1.1e, rel %1.1e'], max(xD_nonns_abs), max(xD_nonns_rel));
        end
      end
      if condJ >= thresh_ns_qa || sum(Rob.I_qa) ~= sum(Rob.I_EE)
        % Benutze die Methode mit vollständigem Gelenkraum (umständliches
        % if-else wegen debug-Abfrage).
        qDD_N_pre = qDD_N_pre_voll;
      end
    end
  end
  
  % Reduziere die Nullraumbeschleunigung weiter, falls Beschleunigungs-
  % Grenzen erreicht werden. Sollte eigentlich nur hier gemacht werden,
  % wird aber zur Verbesserung der Robustheit auch zusätzlich noch unten
  % gemacht. Hat unten zur Folge, dass Verletzung von Positions- und
  % Geschwindigkeitsgrenzen nicht mit allen Mitteln verhindert werden
  if redundant && limits_qDD_set
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
      % Reduktion der Beschleunigung auf Altwert für gewünschte Nullraum-
      % geschwindigkeit übertragen. Entspricht Anti-Windup für Integrator.
      qaD_N_pre_alt = qaD_N_pre_alt - (1-scale)*qaDD_N_pre1*dt;
      qD_N_pre_alt = qD_N_pre_alt - (1-scale)*qDD_N_pre1*dt;
    end
  end
  
  if redundant && limits_qD_set && ...% Nullraum-Optimierung erlaubt Begrenzung der Gelenk-Geschwindigkeit
      condPhi < 1e10 % numerisch nicht für singuläre PKM sinnvoll
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
      % qD_pre_h(~(deltaD_ll<0|deltaD_ul<0)) = 0; % Nur Reduzierung, falls Grenze verletzt
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
      
      % Die Nullraumbewegung zur Vermeidung der Geschwindigkeitsgrenzen
      % kann fehlschlagen, wenn die fragliche Geschwindigkeitskomponente
      % nicht im Nullraum beeinflussbar ist. Daher nochmals Begrenzung der
      % neuen Beschleunigung im Nullraum. 
      delta_ul_rel = (qDD_N_max - qDD_N_post)./(qDD_N_max); % Überschreitung der Maximalwerte: <0
      delta_ll_rel = (-qDD_N_min + qDD_N_post)./(-qDD_N_min); % Unterschreitung Minimalwerte: <0
      if any([delta_ul_rel;delta_ll_rel] < 0)
        if min(delta_ul_rel)<min(delta_ll_rel)
          % Verletzung nach oben ist die größere
          [~,I_max] = min(delta_ul_rel);
          scale = (qDD_N_max(I_max))/(qDD_N_post(I_max));
        else
          % Verletzung nach unten ist maßgeblich
          [~,I_min] = min(delta_ll_rel);
          scale = (qDD_N_min(I_min))/(qDD_N_post(I_min));
        end
        qDD_N_post = scale*qDD_N_post;
      end
    else
      qDD_N_post = qDD_N_pre;
    end
  else
    qDD_N_post = qDD_N_pre;
  end

  % Berechne maximale Nullraum-Beschleunigung bis zum Erreichen der
  % Positionsgrenzen. Reduziere, falls notwendig. Berechnung nach Betrachtung
  % der Geschwindigkeits- und Beschl.-Grenzen, da Position wichtiger ist.
  if redundant && limits_q_set && s.enforce_qlim && ... % Nullraum-Optimierung erlaubt Begrenzung der Gelenk-Position
      condPhi < 1e10 % numerisch nicht für singuläre PKM sinnvoll
    qDD_pre2 = qDD_k_T+qDD_N_post;
    % Daraus berechnete Position und Geschwindigkeit im nächsten Zeitschritt
    qD_pre2 = qD_k + qDD_pre2*dt;
    q_pre2 = q_k + qD_pre2*dt + 0.5*qDD_pre2*dt^2;
    % Prüfe, ob Grenzen damit absehbar verletzt werden
    delta_ul = (qmax - q_pre2); % Überschreitung der Maximalwerte: <0
    delta_ll = (-qmin + q_pre2); % Unterschreitung Minimalwerte: <0
    if any([delta_ul;delta_ll] < 0)
      if min(delta_ul)<min(delta_ll)
        % Verletzung nach oben ist die größere
        [~,I_worst] = min(delta_ul);
        qDD_lim_I = 2/dt^2*(qmax(I_worst)-q_k(I_worst)-qD_pre2(I_worst)*dt);
      else
        % Verletzung nach unten ist maßgeblich
        [~,I_worst] = min(delta_ll);
        qDD_lim_I = 2/dt^2*(qmin(I_worst)-q_k(I_worst)-qD_pre2(I_worst)*dt);
      end
      q_pre_h = q_pre2;
      [~, hdq] = invkin_optimcrit_limits1(q_pre_h, qlim);
      % Dieser Beschleunigungsvektor liegt im Nullraum der Jacobi-Matrix
      % (erfüllt also noch die Soll-Beschleunigung des Endeffektors).
      % Der Vektor führt zu einer Reduzierung der Geschwindigkeit von den
      % Grenzen weg
      qDD_N_h = N * (-hdq');
      % Normiere den Vektor auf den am stärksten grenzverletzenden Eintrag
      qDD_N_he = qDD_N_h/qDD_N_h(I_worst); % [3]/(5)
      % Stelle Nullraumbewegung so ein, dass schlechtester Wert gerade so
      % an der Grenze landet.
      qDD_N_korr_I = -qDD_pre2(I_worst) + qDD_lim_I; % [3]/(7)
      % Erzeuge kompletten Vektor als durch Skalierung des Nullraum-Vektors
      qDD_N_korr = qDD_N_korr_I*qDD_N_he; % [3]/(8)
      qDD_N_post2 = qDD_N_post+qDD_N_korr; % [3]/(6)
      if false % Debug
        Iutest = q_k + qD_k*dt + 0.5*(qDD_k_T+qDD_N_post2)*dt^2 > qmax + 1e-6;
        Iltest = q_k + qD_k*dt + 0.5*(qDD_k_T+qDD_N_post2)*dt^2 < qmin - 1e-6;
        if min(delta_ul)<min(delta_ll) && Iutest(I_worst)
          error('Fehler bei Korrektur der Verletzung der Positions-Obergrenze');
        end
        if min(delta_ul)>min(delta_ll) && Iltest(I_worst)
          error('Fehler bei Korrektur der Verletzung der Positions-Untergrenze');
        end
        if any(Iutest|Iltest)
          warning(['Beim Versuch die Verletzung der Positions-Grenze fuer ', ...
            'Gelenk %d zu vermeiden, wurde eine andere Grenze verletzt'], I_worst);
        end
      end
      % Die Nullraumbewegung zur Vermeidung der Positionsgrenzen
      % kann fehlschlagen, wenn die fragliche Gelenkwinkelkomponente
      % nicht im Nullraum beeinflussbar ist. Daher nochmals Begrenzung der
      % neuen Beschleunigung im Nullraum. 
      delta_ul_rel = (qDD_N_max - qDD_N_post2)./(qDD_N_max); % Überschreitung der Maximalwerte: <0
      delta_ll_rel = (-qDD_N_min + qDD_N_post2)./(-qDD_N_min); % Unterschreitung Minimalwerte: <0
      if any([delta_ul_rel;delta_ll_rel] < 0)
        if min(delta_ul_rel)<min(delta_ll_rel)
          % Verletzung nach oben ist die größere
          [~,I_max] = min(delta_ul_rel);
          scale = (qDD_N_max(I_max))/(qDD_N_post2(I_max));
        else
          % Verletzung nach unten ist maßgeblich
          [~,I_min] = min(delta_ll_rel);
          scale = (qDD_N_min(I_min))/(qDD_N_post2(I_min));
        end
        qDD_N_post2 = scale*qDD_N_post2;
      end
    else
      % Keine Verletzung der Geschwindigkeitsgrenzen. Lasse
      % Beschleunigung so wie sie ist
      qDD_N_post2 = qDD_N_post;  
    end
  else
    qDD_N_post2 = qDD_N_post;
  end

  qDD_k = qDD_k_T + qDD_N_post2;
  % Teste die Beschleunigung (darf die Zwangsbedingungen nicht verändern)
  if debug
    % Das wäre eigentlich gar nicht notwendig, wenn die Beschleunigung
    % eine korrekte Nullraumbewegung ausführt.
    PhiDD_pre = Phi_q*qDD_k + Phi_qD*qD_k;
    PhiDD_korr = -PhiDD_pre - (Phi_x*xDD_k(I_EE)+Phi_xD*xD_k(I_EE));
    if any(abs(PhiDD_korr) > max(1e-6, max(abs(qDD_k))/1e6)) % bei hohen Beschleunigungen ist die Abweichung größer; feine IK-Toleranz notwendig.
      if cond(Phi_q) > 1e8
        % Dieser Teil sollte nicht ausgeführt werden müssen. Bei schlechter
        % Konditionierung der Zwangsbedingungs-Gradienten notwendig.
        % Nur beim Debuggen machen, da derartig schlecht konditionierte
        % Roboter sowieso nicht simuliert werden sollten.
        qDD_korr = Phi_q\PhiDD_korr;
        qDD_k = qDD_k + qDD_korr;
        % Nochmal testen
        PhiDD_test2 = Phi_q*qDD_k + Phi_qD*qD_k + ...
          Phi_x*xDD_k(I_EE)+Phi_xD*xD_k(I_EE);
        if any(abs(PhiDD_test2) > 1e-10)
          error('Korrektur der Beschleunigung hat nicht funktioniert');
        end
      else
        error(['Zeitschritt %d/%d: Beschleunigung ist nicht konsistent nach ', ...
        'Nullraumbewegung. Fehler %1.1e'], k, nt, max(abs(PhiDD_korr)));
      end
    end
  end
  % TODO: Konsistente Reihenfolge in wn und h.
  Stats.h(k,:) = [sum(wn([1:6,11]).*h),h(1:7)'];
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
