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

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07/2019-06
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [q, Phi, Tc_stack_PKM] = invkin3(Rob, xE_soll, q0, s)

%% Initialisierung
assert(isreal(xE_soll) && all(size(xE_soll) == [6 1]), ...
  'ParRob/invkin3: xE_soll muss 6x1 sein');
assert(isreal(q0) && all(size(q0) == [Rob.NJ 1]), ...
  'ParRob/invkin3: q0 muss %dx1 sein', Rob.NJ);

%% Definitionen
% Variablen zum Speichern der Zwischenergebnisse
sigma_PKM = Rob.MDH.sigma; % Marker für Dreh-/Schubgelenk
K = 0.6*ones(Rob.NJ,1);
K(sigma_PKM==1) = K(sigma_PKM==1) / 5; % Verstärkung für Schubgelenke kleiner

s_std = struct('I_EE', Rob.I_EE, ... % FG für die IK
               'K', K, ... % Verstärkung
               'Kn', 0.4*ones(Rob.NJ,1), ... % Verstärkung
               'wn', zeros(2,1), ... % Gewichtung der Nebenbedingung
               'maxstep_ns', 1e-10*ones(Rob.NJ,1), ... % Maximale Schrittweite für Nullraum zur Konvergenz
               'normalize', true, ...
               'n_min', 0, ... % Minimale Anzahl Iterationen
               'n_max', 1000, ... % Maximale Anzahl Iterationen
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
scale_lim = s.scale_lim;
Phit_tol = s.Phit_tol;
Phir_tol = s.Phir_tol;
retry_limit = s.retry_limit;
maxrelstep = s.maxrelstep;
maxrelstep_ns = s.maxrelstep_ns;
maxstep_ns = s.maxstep_ns;
success = false;

if any(wn ~= 0)
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
%% Iterative Lösung der IK
for rr = 0:retry_limit
  q1 = q0;
  [~,Phi_voll] = Rob.constr3(q1, xE_soll);
  Phi = Phi_voll(I_IK);
  for jj = 1:n_max
    % Gesamt-Jacobi bilden (reduziert um nicht betrachtete EE-Koordinaten)
    [~,Jik_voll]=Rob.constr3grad_q(q1, xE_soll);
    Jik = Jik_voll(I_IK,:);

    %% Nullstellensuche für Positions- und Orientierungsfehler
    % (Optimierung der Aufgabe)
    delta_q_T = Jik \ (-Phi);
    %% Optimierung der Nebenbedingungen (Nullraum)
    delta_q_N = zeros(size(delta_q_T));
    if nsoptim && jj < n_max-10 % die letzten Iterationen sind zum Ausgleich des Positionsfehlers (ohne Nullraum)
      % Berechne Gradienten der zusätzlichen Optimierungskriterien
      v = zeros(Rob.NJ, 1);
      if wn(1) ~= 0
        [h1, hdq] = invkin_optimcrit_limits1(q1, qlim);
        v = v - wn(1)*hdq'; % [1], Gl. (25)
      end
      if wn(2) ~= 0
        [h2, hdq] = invkin_optimcrit_limits2(q1, qlim);
        v = v - wn(2)*hdq';
      end
      % [1], Gl. (24)
      delta_q_N = (eye(Rob.NJ) - pinv(Jik)* Jik) * v;
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
    if limits_set && ~isnan(maxrelstep_ns)
      abs_delta_q_N_rel = abs(delta_q_N ./ delta_qlim .* Kn);
      if any(abs_delta_q_N_rel > maxrelstep_ns)
        delta_q_N = delta_q_N .* maxrelstep_ns / max(abs_delta_q_N_rel);
      end
    end
    
    % Inkrement der Gelenkwinkel
    delta_q = K.*delta_q_T + Kn.*delta_q_N;
    
    % Reduziere Schrittweite auf einen Maximalwert bezogen auf
    % Winkelgrenzen
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
    % die Schrittweite, falls das der Fall ist
    delta_ul_rel = (qmax - q2)./(qmax-q1); % Überschreitung der Maximalwerte: <0
    delta_ll_rel = (-qmin + q2)./(q1-qmin); % Unterschreitung Minimalwerte: <0
    if scale_lim && any([delta_ul_rel;delta_ll_rel] < 0)
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

    if any(isnan(q2)) || any(isinf(q2))
      break; % ab hier kann das Ergebnis nicht mehr besser werden wegen NaN/Inf
    end

    q1 = q2;
    [~,Phi_voll] = Rob.constr3(q1, xE_soll);
    Phi = Phi_voll(I_IK);

    if jj >= n_min ... % Mindestzahl Iterationen erfüllt
      && max(abs(Phi(I_constr_t_red))) < Phit_tol && max(abs(Phi(I_constr_r_red))) < Phir_tol && ... % Haupt-Bedingung ist erfüllt
      ( ~nsoptim || ...%  und keine Nebenoptimierung läuft
      nsoptim && all(abs(delta_q_N) < maxstep_ns) ) % oder die Nullraumoptimierung läuft noch
     success = true; 
     break;
    end
  end
  if success
    break;
  end
  q0 = qmin + rand(Rob.NJ,1).*(qmax-qmin); 
end
if s.normalize
  q1(sigma_PKM==0) = normalize_angle(q1(sigma_PKM==0)); % nur Winkel normalisieren
end
q = q1;

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

