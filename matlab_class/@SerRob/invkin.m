% Inverse Kinematik für allgemeinen Roboter
% Allgemeine, stark parametrierbare Funktion zum Aufruf mit allen möglichen
% Einstellungen
% Iterative Lösung der inversen Kinematik mit inverser Jacobi-Matrix
% 
% Eingabe:
% xE_soll
%   EE-Lage (Sollwert)
% q0
%   Anfangs-Gelenkwinkel für Algorithmus
% s
%   Struktur mit Eingabedaten. Felder, siehe Quelltext.
% 
% Ausgabe:
% q
%   Lösung der IK
% Phi
%   Erfüllung der Zielfunktion (Fehler) beim Ergebnis q
%   Die kinematischen Zwangsbedingungen geben den Positions- und
%   Orientierungsfehler am Ende der inversen Kinematik an.
% Q
%   Alle Zwischenergebnisse der Iterationen der Gelenkwinkel

% Quelle:
% [1] Aufzeichnungen Schappler vom 3.8.2018

function [q, Phi, Q] = invkin(Rob, xE_soll, q0, s)

% Wähle die Indizes der Schubgelenke in den Minimalkoordinaten aus
sigmaJ = Rob.MDH.sigma(Rob.MDH.mu>=1); % Marker für Dreh-/Schubgelenk (in den Minimalkoordinaten)
if length(sigmaJ) < Rob.NQJ
  error('Marker für Gelenktypen der in Minimalkoordinaten enthaltenen Gelenke passt nicht');
end
% Standard-Einstellung der Verstärkungs-Faktoren: Die Schubgelenke müssen
% langsamer konvergieren als die Drehgelenke, da die Rotation die Position
% beeinflusst, aber nicht umgekehrt.
K_def = 0.1*ones(Rob.NQJ,1);
K_def(sigmaJ==1) = K_def(sigmaJ==1); % Verstärkung für Schubgelenke kleiner

s_std = struct( ...
             'I_EE', Rob.I_EE_Task, ... % FG für die IK
             'K', K_def, ... % Verstärkung
             'Kn', 1e-2*ones(Rob.NQJ,1), ... % Verstärkung
             'wn', 0, ... % Gewichtung der Nebenbedingung
             'n_min', 0, ... % Minimale Anzahl Iterationen
             'n_max', 1000, ... % Maximale Anzahl Iterationen
             'Phit_tol', 1e-10, ... % Toleranz für translatorischen Fehler
             'Phir_tol', 1e-10, ... % Toleranz für rotatorischen Fehler
             'constr_m', 2, ... % Nr. der Methode für die Zwangsbedingungen
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
constr_m = s.constr_m;
Phit_tol = s.Phit_tol;
Phir_tol = s.Phir_tol;
retry_limit = s.retry_limit;

qmin = Rob.qlim(:,1);
qmax = Rob.qlim(:,2);

if wn ~= 0
  nsoptim = true;
else
  % Keine zusätzlichen Optimierungskriterien
  nsoptim = false;
end

% Indizes für kinematische Zwangsbedingungen festlegen
n_Phi_t = sum(s.I_EE(1:3));
if constr_m == 1
  I_IK2 = [1 2 3 4 5 6];
else
  I_IK2 = [1 2 3 6 5 4];
end
I_IK = I_IK2(s.I_EE);
% Damit der Roboter einen Nullraum für Nebenoptimierungen hat, muss er min.
% 7FG für 6FG-Aufgaben und 6FG für 5FG-Aufgaben haben.
if nsoptim && Rob.NQJ <= length(I_IK)
  nsoptim = false;
end

success = false;

for rr = 1:retry_limit
  if nargout == 3
    Q = NaN(n_max, Rob.NQJ);
    Q(1,:) = q0;
  end
  % Variablen zum Speichern der Zwischenergebnisse
  q1 = q0;
  for jj = 2:n_max

    dxq=Rob.constr1grad_tq(q1); % Variante 1 = Variante 2
    if constr_m == 1
      dpq=Rob.constr1grad_rq(q1, xE_soll);
    else
      dpq=Rob.constr2grad_rq(q1, xE_soll, true);
    end
    Jdk_voll = [dxq; dpq];

    if constr_m == 1
      Phi_voll = Rob.constr1(q1, xE_soll);
    else
      Phi_voll = Rob.constr2(q1, xE_soll, true);
    end

    %% Reduktion auf betrachtete FG
    Jdk = Jdk_voll(I_IK,:);
    Phi = Phi_voll(I_IK);
    %% Nullstellensuche für Positions- und Orientierungsfehler
    % (Optimierung der Aufgabe)
    % Normale Invertierung der Jacobi-Matrix der seriellen Kette
    delta_q_T = Jdk \ (-Phi);
    %% Optimierung der Nebenbedingungen (Nullraum)
    delta_q_N = zeros(size(delta_q_T));
    if nsoptim && jj < n_max-10 % die letzten Iterationen sind zum Ausgleich des Positionsfehlers (ohne Nullraum)
      % Berechne Gradienten der zusätzlichen Optimierungskriterien
      v = zeros(Rob.NQJ, 1);
      if wn(1) ~= 0
        [~, hdq] = Rob.optimcrit_limits1(q1);
        % [1], Gl. (25)
        v = v - hdq';
      end
      % [1], Gl. (24)
      delta_q_N = (eye(Rob.NQJ) - pinv(Jdk)* Jdk) * v;
    end

    % [1], Gl. (23)
    delta_q = K.*delta_q_T + Kn.*delta_q_N;
    q2 = q1 + delta_q;

    if any(isnan(q2)) || any(isinf(q2))
      break; % ab hier kann das Ergebnis nicht mehr besser werden wegen NaN/Inf
    end

    q1 = q2;
    q1(sigmaJ==0) = normalize_angle(q1(sigmaJ==0)); % nur Winkel normalisieren

    if jj > n_min ... % Mindestzahl Iterationen erfüllt
        && all(abs(Phi(1:n_Phi_t)) < Phit_tol) && all(abs(Phi(n_Phi_t+1:end)) < Phir_tol) && ... % Haupt-Bedingung ist erfüllt
        ( ~nsoptim || ... %  und keine Nebenoptimierung läuft
        nsoptim && all(abs(delta_q_N) < 1e-10) ) % oder die Nullraumoptimierung läuft noch
      success = true;
      break;
    end
    if nargout == 3
      Q(jj,:) = q1;
    end
  end
  if success
    break;
  end
  % Beim vorherigen Durchlauf kein Erfolg. Generiere neue Anfangswerte
  q0 = qmin + rand(Rob.NQJ,1).*(qmax-qmin);
end
q = q1;
