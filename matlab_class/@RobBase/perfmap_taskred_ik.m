% Erzeuge eine Leistungs-Karte der inversen Kinematik bei Aufgabenredundanz
% 
% Eingabe:
% R
%   Roboter-Klasse
% X_tref [NT x 6]
%   Endeffektor-Trajektorie (Zeilen: Zeit; Spalten: 6 EE-Koordinaten)
% IL_ref [N x 1]
%   Indizes der N Eckpunkte in den Zeitwerten aus X_tref
% s_in
%   Struktur mir Einstellungswerten (siehe Quelltext)
% 
% Ausgabe:
% H_all [NI x NP x NK+4]
%   Diskretisierung aller `NK` IK-Zielfunktionen, die in der Einzelpunkt-IK
%   benutzt werden können. Für jedes Kriterium wird eine Rasterung erstellt
%   mit Auflösung NI x NP. Zusätzlich werden die physikalischen Werte der
%   Kriterien gespeichert: Kollisionstiefe, Bauraumüberschreitung,
%   Kondition IK-Jacobi, Kondition Jacobi (siehe invkin-Funktionen)
% Q_all [NP x NJ x NI]
%   Gelenkwinkel für die Konfiguration des Roboters zu jeder der Punkte auf
%   der gerasterten Karte aus H_all.
% s_ref [NI x 1]
%   Stützstellen des Trajektorienverlaufs für das zu erstellende Bild
%   Normalisierte Bahnkoordinate der Trajektorie, Auswahl von Zeit-Stütz-
%   stellen mit äquidistantem Wegverlauf. Anzahl `NI` richtet sich nach den
%   Einstellungen
% s_tref [NT x 1]
%   Normalisierte Stützstellen der eingegebenen Zeit-Trajektorie X_tref
%   Jeder Eckpunkt aus IL_ref entspricht einer ganzen Zahl.
% phiz_range [NP x 1]
%   Stützstellen der EE-Rotation (phi_z) für zu erstellendes Bild
%   (Anzahl NP richtet sich nach der Auflösung des Bildes aus Einstellung)
% 
% Quelle: Schappler, M. and Ortmaier, T.: Singularity Avoidance of
% Task-Redundant Robots in Pointing Tasks: On Nullspace Projection and
% Cardan Angles as Orientation Coordinates, ICINCO 2021.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [H_all, Q_all, s_ref, s_tref, phiz_range] = perfmap_taskred_ik(R, X_tref, IL_ref, s_in)

%% Initialisierung der Einstellungen
s = struct( ...
  'q0', rand(R.NJ,1), ... % IK-Anfangswerte für ersten Punkt
  'I_EE_full', R.I_EE, ...
  'I_EE_red', R.I_EE_Task, ...
  'discretization_type', 'trajectory', ...
  'optimcrit_limits_hyp_deact', 0.9, ...% Hyperbolische Funktion nur nahe an Grenzen
  'map_unit', 'normalized', ...
  'map_phistart', 0, ... % Mittelwert, an dem die Suchläufe gespiegelt werden
  'maplim_phi', [-pi, pi], ... % Grenzen des Wertebereichs
  'mapres_thresh_eepos', 3e-3, ... % 3mm max. Abstand zwischen zwei Stützstellen in s_ref
  'mapres_thresh_eerot', 3*pi/180, ... % 3deg ...
  'mapres_thresh_pathcoordres', 0.05, ...% min 20 values between key points
  'mapres_redcoord_dist_deg', 3, ... % Auflösung der redundanten Koordinate; 3deg
  'settings_ik', struct(''), ... % Einstellungs-Struktur für den IK-Aufruf (Modifikation der Optimierungskriterien)
  'verbose', false);
if nargin == 4
  for f = fields(s_in)'
    if isfield(s, f{1})
      s.(f{1}) = s_in.(f{1});
    else % Fall soll eigentlich nicht vorkommen. Daher Prüfung als zweites
      error('Feld %s aus s_in kann nicht übergeben werden', f{1});
    end
  end
end
if isnan(s.map_phistart) || s.map_phistart > s.maplim_phi(2) || ...
    s.map_phistart < s.maplim_phi(1) % Abfangen ungültiger Eingaben
  warning(['Feld map_phistart aus Eingabe ungültig (%1.1f). Grenzen: ', ...
    '%1.1f und %1.1f'], s.map_phistart, s.maplim_phi(1), s.maplim_phi(2));
  s.map_phistart = mean(s.maplim_phi);
end
if all(s.I_EE_full == s.I_EE_red)
  warning('Aufgaben-FG identisch mit Roboter-FG. Keine Redundanz');
end
if R.Type == 0
  qlim = R.qlim;
else
  qlim = cat(1, R.Leg.qlim);
end
%% Trajektorie normalisieren
% Erzeuge eine konsistente Eingabe (falls die Trajektorie gekürzt wurde,
% sich aber die Indizes auf eine längere Trajektorie beziehen)
IL = IL_ref(IL_ref<=size(X_tref,1));
if isempty(IL)
  IL = 1; % Ungültige Eingabe. Kein Fortschritt über Eckpunkte normalisierbar.
end
if IL(end) < size(X_tref,1)
  % Indizes stimmen nicht mit der Trajektorie überein. Erzeuge letzten
  % Eckpunkt so, dass Normalisierung bis zum Ende der Trajektorie erfolgt.
  IL = [IL; size(X_tref,1)];
end
% Eckpunkte bestimmen
XL = X_tref(IL, :);
% Get normalized path coordinate
s_tref = NaN(size(X_tref,1),1);
% Get progress related to key points of the trajectory. Assumption
% monotonous increase/decrease between key points. Do not consider entry 6
% for phi_z (changes due to origin of data from task redundancy IK).
for i = 1:size(XL,1)-1
  I1 = IL(i);
  I2 = IL(i+1);
  p_all = ( X_tref(I1:I2,1:5)-repmat(XL(i,1:5),I2-I1+1,1) ) ./ ...
           repmat(XL(i+1,1:5)-XL(i,1:5),I2-I1+1,1);
  % Deaktiviere Koordinaten, die sich nur innerhalb der Rechenungenauigkeit
  % verändern. Aus diesen soll keine Bahnkoordinate berechnet werden.
  p_all(:,abs(XL(i+1,1:5)-XL(i,1:5)) < 1e-10) = NaN;
  p = mean(p_all,2, 'omitnan');
  % Berechne die Bahnkoordinate aus dem normalisierten Fortschritt.
  s_tref(I1:I2) = (i-1)+p;
end
I_nonmon = find([false;diff(s_tref) < 0]);
% Nehme den nächsten vorherigen Punkt, falls ein Abstand so gering ist,
% dass die Bahnkoordinate nicht mehr monoton steigt
for i = I_nonmon(:)'
  s_tref(i) = s_tref(i-1); % der vorherige muss i.O. belegt sein.
end
if any(diff(s_tref) < 0)
  warning(['Ermittelte Bahnkoordinate ist nicht monoton steigend. ', ...
    'Trajektorie ungeeignet für Aufteilung in Eckpunkte.']);
  s_tref = (1:size(X_tref,1))'/size(X_tref,1);
end
if size(XL,1)==1 % kein Fortschritt bestimmbar
  s_tref = (1:size(X_tref,1))'/size(X_tref,1);
  XL = X_tref([1 end],:);
end
if any(s_tref > size(XL,1)-1 +1e-10)
  warning(['Normalisierung der Bahnkoordinate nicht erfolgreich. max(s)=', ...
    '%1.1f bei %d Punkten'], max(s_tref), size(XL,1));
end
% Output of the normalized progress of the trajectory input
s_ref = s_tref;
% Remove detailed data and limit to resolution from settings (e.g. 1mm/1deg)
last_x = inf(1,6);
last_s = inf;
I_remove = false(size(X_tref,1),1);
for i = 1:size(X_tref,1)
  if norm(X_tref(i,1:3)-last_x(1:3)) < s.mapres_thresh_eepos && ...
     norm(X_tref(i,4:6)-last_x(4:6)) < s.mapres_thresh_eerot && ...
     abs(s_ref(i)-last_s) < s.mapres_thresh_pathcoordres
    I_remove(i) = true; % diesen Punkt löschen
  else
    last_x = X_tref(i,:);
    last_s = s_ref(i);
  end
end
X_ref = X_tref(~I_remove,:);
s_ref = s_ref(~I_remove);

%% Initialisierung weiterer Größen
% Create range of values for the redundant coordinate.
phiz_range = unique([s.map_phistart:-s.mapres_redcoord_dist_deg*pi/180:s.maplim_phi(1), ...
                   s.map_phistart: s.mapres_redcoord_dist_deg*pi/180:s.maplim_phi(2)])';
if s.verbose
  fprintf(['The performance map contains %d trajectory samples and %d ', ...
    'values for the redundant coordinates. %d evaluations in total\n'], ...
    size(X_ref,1), length(phiz_range(:)), size(X_ref,1)*length(phiz_range(:)));
  fprintf('Start discretization of the inverse kinematics\n');
end
% IK-Grundeinstellungen
s_ik = struct('Phit_tol', 1e-12, 'Phir_tol', 1e-12, ... % sehr genau rechnen
  'scale_lim', 0.7, ...
  'retry_limit', 0);
s_ep = s_ik; % Einstellungen für die Eckpunkte-IK
s_ep.n_max = 5000; % Mehr Versuche (Abstände zwischen Punkten größer als bei Traj.-IK)
% s_ep.maxrelstep_ns = 0.05; % Große Werte, Größere Nullraumbewegung pro Zeitschritt
s_ep.retry_on_limitviol = true;
s_ep.retry_limit = 100; % Neuversuche erlauben (bei Einzelpunkt i.O.)
s_ep.normalize = false;
% Keine Einhaltung der Grenzen erzwingen, da dies eher
% Konfigurationswechsel bringt. Diese erzeugen dann Artefakte im Bild.
s_ep.finish_in_limits = false;
s_ep.scale_lim = 0;
% Einstellungen für Dummy-Berechnung ohne Änderung der Gelenkwinkel.
% Dient nur zur Berechnung der Optimierungskriterien.
s_ep_dummy = s_ep;
s_ep_dummy.finish_in_limits = false; % Muss deaktiviert sein. Sonst ...
s_ep_dummy.retry_on_limitviol = false; % ... Veränderung von wn in IK-Aufruf
s_ep_dummy.n_max = 1;
s_ep_dummy.retry_limit = 0;
% Hierdurch werden die Kriterien berechnet (konsistent mit SerRob/invkin2
% und ParRob/invkin4)
s_ep_dummy.wn = ones(R.idx_ik_length.wnpos,1); 
s_ep_dummy.K = zeros(R.NJ,1); % hierdurch keine Bewegung und damit ...
s_ep_dummy.Kn = zeros(R.NJ,1); % ... sofortiger Abbruch
s_ep_dummy.optimcrit_limits_hyp_deact = s.optimcrit_limits_hyp_deact;
s_ep_dummy.Phit_tol = 1; % Deaktiviere Einhaltung einer IK-Toleranz. Es findet ...
s_ep_dummy.Phir_tol = 1; % ... sowieso keine Bewegung statt.
% Spezifische Einstellungen für einige Optimierungskriterien übernehmen.
% Betrifft solche, die den Wert der Kriterien beeinflussen.
for f = fields(s.settings_ik)'
  if any(strcmp(f{1}, {'cond_thresh_ikjac', 'optimcrit_limits_hyp_deact', ...
      'collbodies_thresh', 'installspace_thresh'})) || ...
      R.Type == 2 && any(strcmp(f{1}, {'cond_thresh_jac'}))
    s_ep_dummy.(f{1}) = s.settings_ik.(f{1});
  end
end
% Einstellung für IK bei globaler Diskretisierung über Trajektorie
s_ep_glbdscr = s_ep; % leave as is (temporarily permit limit violations; no scaling until limit)
s_ep_glbdscr.retry_limit = 10;
s_ep_glbdscr.normalize = false; % no normalization (due to joint limits)
s_ep_glbdscr = rmfield(s_ep_glbdscr, 'finish_in_limits'); % does not work without redundancy
s_traj_glbdscr = struct('simplify_acc', true);
H_all = NaN(size(X_ref,1), length(phiz_range(:)), R.idx_ik_length.hnpos+4);
Q_all = NaN(length(phiz_range(:)), R.NJ, size(X_ref,1));

% Startwert prüfen. Roboter dafür auf 3T3R einstellen
if R.Type == 0,   R.I_EE_Task = s.I_EE_full;
else,             R.update_EE_FG(s.I_EE_full,s.I_EE_full); end
if R.Type == 0
  [q0, Phi, ~, ~] = R.invkin2(R.x2tr([X_ref(1,1:5)';s.map_phistart]), s.q0, s_ik);
else
  [q0, Phi, ~, ~] = R.invkin4([X_ref(1,1:5)';s.map_phistart], s.q0, s_ik);
end
if any(abs(q0-s.q0)>1e-3) || any(abs(Phi)>1e-5)
  warning('initial value q0 for performance map does not match parameter map_phistart');
end
%% Rasterung durchführen
for ii_sign = 1:2 % move redundant coordinate in positive and negative direction
  if s.verbose
    fprintf('Start computation for sign %+d\n', (-1)^ii_sign);
  end
  if ii_sign == 1
    I_ii = find(phiz_range >= s.map_phistart);
  else
    I_ii = flipud(find(phiz_range <= s.map_phistart));
  end
  if isempty(I_ii)
    continue % Bei Eingabe einer NaN-Trajektorie oder unpassenden Grenzen zum Startwert
  end
  H_all_ii = NaN(size(X_ref,1), length(I_ii), R.idx_ik_length.hnpos+4);
  Q_all_ii = NaN(length(I_ii), R.NJ, size(X_ref,1));
  t_lastmessage = tic();
  t1 = tic();
  phiz_range_ii = phiz_range(I_ii);
  for i = 1:size(X_ref,1) % loop trajectory samples
    t1_i = tic();
    % Get joint configurations using a virtual trajectory
    if i > 1 && all(~isnan(Q_all_ii(1, :, i-1)))
      q0_j = Q_all_ii(1, :, i-1)';
    else
      q0_j = s.q0;
    end
    if strcmp(s.discretization_type, 'trajectory')
      if R.Type == 0
        R.I_EE_Task = s.I_EE_full;
      else
        R.update_EE_FG(s.I_EE_full,s.I_EE_full); % Roboter auf 3T3R einstellen
      end
      X_i_traj = [repmat(X_ref(i,1:5), length(phiz_range_ii), 1), phiz_range_ii(:)];
      XD_i_traj = [zeros(length(phiz_range_ii), 5), [0;diff(phiz_range_ii(:))]];
      XDD_i_traj = zeros(length(phiz_range_ii), 6);
      T_i_traj = (0:1:length(phiz_range_ii)-1)';
      if R.Type == 0 %#ok<IFBDUP> % Seriell
        [Q_i,~,~,Phi_i] = R.invkin2_traj(X_i_traj, XD_i_traj, XDD_i_traj, T_i_traj, q0_j, s_traj_glbdscr); 
      else % Parallel
        [Q_i,~,~,Phi_i] = R.invkin2_traj(X_i_traj, XD_i_traj, XDD_i_traj, T_i_traj, q0_j, s_traj_glbdscr); 
      end
      kk_add_count = 0;
      for kk = 1:size(Q_i,1)  % Trajektorie für jeden Punkt berechnen. Bei Abbruch Rest neu berechnen
        if any(isnan(Q_i(kk,:))) || any(abs(Phi_i(kk,:))>1e-6)
          if kk > 1 && ~all(isnan(Q_i(kk-1,:)))
            q0_kk = Q_i(kk-1,:)'; % nehme vom vorherigen (oben/unten zu Anfangswert)
          elseif i > 1 && ~all(isnan(Q_all_ii(kk, :, i-1)))
            q0_kk = Q_all_ii(kk, :, i-1)'; % nehme von links daneben, falls verfügbar
          else
            q0_kk = s.q0;
          end
          if R.Type == 0 %#ok<IFBDUP> % Seriell
            [Q_i_add,~,~,Phi_i_add] = R.invkin2_traj(X_i_traj(kk:end,:), ...
              [zeros(1,6);XD_i_traj(kk+1:end,:)], XDD_i_traj(kk:end,:), ...
              T_i_traj(kk:end,:), q0_kk, s_traj_glbdscr); 
          else % Parallel
            [Q_i_add,~,~,Phi_i_add] = R.invkin2_traj(X_i_traj(kk:end,:), ...
              [zeros(1,6);XD_i_traj(kk+1:end,:)], XDD_i_traj(kk:end,:), ...
              T_i_traj(kk:end,:), q0_kk, s_traj_glbdscr); 
          end
          Q_i(kk:end,:) = Q_i_add;  % alle weiteren ab NaN mit neuen Gelenkwinkeln belegen und weiter prüfen
          Phi_i(kk:end,:) = Phi_i_add;
          kk_add_count = kk_add_count + 1;
        end
      end
%       if kk_add_count > 0
%         fprintf(['Analyse der Rotations-Trajektorie für Punkt %d/%d abgeschlossen. ', ...
%           'Insgesamt %d Punkte wurden neu berechnet.\n'], i, size(X_tref,1), kk_add_count);
%       end
      R.update_EE_FG(s.I_EE_full,s.I_EE_red); % Roboter auf 3T2R einstellen
      for j = 1:length(phiz_range_ii)
        x_j = [X_ref(i,1:5), phiz_range_ii(j)]';
        q_j = Q_i(j,:)';
        if ~all(abs(Phi_i(j,:)) < 1e-8), continue; end
        % IK benutzen, um Zielfunktionswerte zu bestimmen (ohne Neuberechnung)
        if R.Type == 0
          [q_dummy, ~, ~,Stats_dummy] = R.invkin2(R.x2tr(x_j), q_j, s_ep_dummy);
        else
          [q_dummy, ~, ~,Stats_dummy] = R.invkin4(x_j, q_j, s_ep_dummy);
        end
        if any(abs(q_j - q_dummy) > 1e-8) % darf nicht sein (Logik-Fehler)
          error('IK-Ergebnis hat sich bei Test verändert');
        end
        Q_all_ii(j, :, i) = q_j;
        % Speichere die Optimierungskriterien für Nullraumbewegungen
        H_all_ii(i,j,1:end-4) = Stats_dummy.h(Stats_dummy.iter+1,2:(R.idx_ik_length.hnpos+1));
        % Speichere Abstand zu Kollision und Bauraumgrenze separat.
        % Positive Zahlen sind eine Überschreitung bzw. Eindringen (schlecht).
        H_all_ii(i,j,end-3) = Stats_dummy.maxcolldepth(Stats_dummy.iter+1,1);
        H_all_ii(i,j,end-2) = Stats_dummy.instspc_mindst(Stats_dummy.iter+1,1);
        % Speichere die Konditionszahl der Jacobi-Matrix separat. Zwei
        % verschiedene: IK-Jacobi und analytische Jacobi
        H_all_ii(i,j,end-1:end) = Stats_dummy.condJ(Stats_dummy.iter+1,:);
        % Bauraumverletzung und Kollision nochmal gesondert eintragen
        if R.Type == 0, idxshift = 0; % Seriell
        else, idxshift = 1; end % PKM
        if H_all_ii(i,j,(4+idxshift):end-2) > Stats_dummy.h_coll_thresh
          H_all_ii(i,j,(4+idxshift):end-2) = inf; % Kollision liegt vor
        end
        if H_all_ii(i,j,(5+idxshift):end-2) > Stats_dummy.h_instspc_thresh
          H_all_ii(i,j,(5+idxshift):end-2) = inf; % Bauraumverletzung liegt vor
        end
      end
    elseif strcmp(s.discretization_type, 'position-level')
    % Get joint configurations using position-level IK (less efficient)
    % Here enforcing the joint limits is easier, but not necessary to
    % determine the condition numbers
      for j = 1:length(phiz_range_ii)
        if j == 1
          % assure joint limits by retrying, only they match
          s_ep_glbdscr.retry_on_limitviol = true;
        else
          % Not able to retry on position limits violation. Otherwise,
          % platform angles greater than 180° can not be checked. For this
          % the initial value has to be the next point
          s_ep_glbdscr.retry_on_limitviol = false;
        end
        R.update_EE_FG(s.I_EE_full,s.I_EE_full); % Roboter auf 3T3R einstellen
        % IK benutzen, um Zielfunktionswerte zu bestimmen (ohne Neuberechnung)
        x_j = [X_ref(i,1:5), phiz_range_ii(j)]';
        if j > 1
          delta_x = [zeros(5,1);phiz_range_ii(j)-phiz_range_ii(j-1)];
          if R.Type == 0
            Ja_PosDiscr = RS.jacobia(Q_all_ii(j-1, :, i)');
            q0_j = Q_all_ii(j-1, :, i)' + Ja_PosDiscr\delta_x;  % Ja_PosDiscr wurde/muss invertiert werden
          else
            [~,Phi_q] = R.constr4grad_q(Q_all_ii(j-1, :, i)');
            [~,Phi_x] = R.constr4grad_x([X_ref(i,1:5),phiz_range_ii(j-1)]');
            Jtilde_inv_x = -Phi_q\Phi_x; % Full coordinate Jacobian (equ. 17 in paper)
            q0_j = Q_all_ii(j-1, :, i)' + Jtilde_inv_x*delta_x;
          end
        else
          if i > 1
            q0_j = []; % will be selected below
          else
            q0_j = q0;
          end
        end
        if any(isnan(q0_j))
          % The previous pose was not computed successfully. Take the
          % second last and so on
          q0_j = q0; % overwrite this later
          for jjj = j-1:-1:1 % look back until the first platform rotation
            if ~any(isnan(Q_all_ii(jjj, :, i)))
              q0_j = Q_all_ii(jjj, :, i)';
              break;
            end
          end
        end
        % create list of initial values for the IK and compute IK
        q0_list = q0_j';
        if j > 1 && ~any(isnan(Q_all_ii(j-1, :, i)))
          q0_list = [q0_list; Q_all_ii(j-1, :, i)];  %#ok<AGROW>
        end
        if i > 1 && j > 1 && ~any(isnan(Q_all_ii(j-1, :, i-1)))
          q0_list = [q0_list; Q_all_ii(j-1, :, i-1)];  %#ok<AGROW>
        end
        if i > 1 && ~any(isnan(Q_all_ii(j, :, i-1)))
          q0_list = [q0_list; Q_all_ii(j, :, i-1)];  %#ok<AGROW>
        end
        if i == 1 && j == 1
          % Add random poses to be able to start with the best pose
          qlim_noninf = qlim;
          qlim_noninf(isinf(qlim_noninf)) = pi*sign(qlim(isinf(qlim)));
          q0_list = [q0_list; repmat(qlim_noninf(:,1)',200,1)+rand(200, R.NJ).* ...
            repmat(qlim_noninf(:,2)' - qlim_noninf(:,1)',200,1)];  %#ok<AGROW>
        end
        % In case of second run overwrite everything and directly take
        % results for phi=0. Otherwise there may be a discontinuity
        if ii_sign == 2 && j == 1 % first value corresponds to 0
          q0_list = Q_all(length(phiz_range_ii),:,i);
        end
        if any(isnan(q0_list(:))), error('An Initial value is NaN'); end % this makes a random new initial seed, which is not desired here
        q_j_list = NaN(size(q0_list));
        for k = 1:size(q0_list,1)
          [q_k, Phi] = R.invkin2(x_j, q0_list(k,:)', s_ep_glbdscr);
          if any(abs(Phi) > 1e-8)
            % Try other IK method
            [q_k, Phi] = R.invkin4(x_j, q0_list(k,:)', s_ep_glbdscr);
            % Mark this as not working, if other method fails as well
            if any(abs(Phi) > 1e-8)
              q_j_list(k,:) = NaN;
              continue
            end
          end
          % normalize joint angles (to stay near the limits' center)
          % Do not do this to see what happens after a full rotation
          % q_k(RP.MDH.sigma==0) = normalizeAngle(q_k(RP.MDH.sigma==0), ...
          %   mean(qlim(RP.MDH.sigma==0,:),2)); % normalize to center of limits
          q_j_list(k,:) = q_k;
        end
        if all(isnan(q_j_list(:)))
          % warning('IK did not find a solution for phi_z=%1.1fdeg', 180/pi*x_j(6));
          continue % this IK configuration did not work.
        end
        % Compute performance criteria
        R.update_EE_FG(s.I_EE_full,s.I_EE_red); % Roboter auf 3T2R einstellen
        h_list = NaN(size(q_j_list,1),R.idx_ik_length.hnpos+4);
        for k = 1:size(q_j_list,1)
          % IK benutzen, um Zielfunktionswerte zu bestimmen (ohne Neuberechnung)
          [q_dummy, ~,~,Stats_dummy] = R.invkin4(x_j, q_j_list(k,:)', s_ep_dummy);
          if any(abs(q_j_list(k,:)' - q_dummy) > 1e-8)
            error('IK-Ergebnis hat sich bei Test verändert');
          end
          h_list(k,:) = [Stats_dummy.h(Stats_dummy.iter+1,2:end), ...
            Stats_dummy.maxcolldepth(Stats_dummy.iter+1,1), ...
            Stats_dummy.instspc_mindst(Stats_dummy.iter+1,1), ...
            Stats_dummy.condJ(Stats_dummy.iter+1,:)];
        end
        % select the joint angles that are nearest to the previous pose
        if ~(i==1 && j == 1) % not possible for first sample
          [~,Ibest_k] = min(sum((q_j_list-repmat(q0_list(1,:),size(q_j_list,1),1)).^2,2));
        else
          % Alternative: Pick the best joint configuration (away from limits)
          % This may require reconfigurations within the plane phiz-s.
          % Therefore only use for the first sample.
          [~, Ibest_k] = min(h_list(:,2)+h_list(:,1)); % squared and hyperbolic limit. Use best
        end
        if any(isnan(q_j_list(Ibest_k,:))), error('Unexpected NaN'); end
        Q_all_ii(j, :, i) = q_j_list(Ibest_k,:);
        H_all_ii(i,j,:) = h_list(Ibest_k,:);
      end
    else
      error('Mode not defined');
    end
    if s.verbose && (toc(t_lastmessage)-toc(t1_i) > 20 || i == 1)
      fprintf(['Duration for trajectory sample number %d/%d: %1.1fs. Remaining %d ', ...
        'samples (estimated %1.1fmin)\n'], i, size(X_ref,1), toc(t1_i), ...
        (size(X_ref,1)-i), toc(t1_i)*(size(X_ref,1)-i)/60);
      t_lastmessage = tic();
    end
  end
  % Save data for the sign value ii
  if ii_sign == 1 % positive sign
    % take all the result data
    I_phi = 1:length(phiz_range_ii);
    % store in second half of result variables
    I_all = I_ii; % length(phiz_range_ii):(2*length(phiz_range_ii)-1);
  else
    % Beginning of the interval. The computation starts with somewhere in
    % the middle, but the lowest value has to be saved as the first one.
    % Therefore `fliplr`. Remove the middle entry because it is already
    % included in the positive values.
    I_phi = fliplr(2:length(phiz_range_ii));
    I_all = 1:length(phiz_range_ii)-1;
  end
  assert(all(phiz_range(I_all)==phiz_range_ii(I_phi)), 'Logik-Fehler');
  for k = 1:size(Q_all_ii,3)
    Q_all(I_all,:,k) = Q_all_ii(I_phi,:,k);
  end
  for k = 1:size(H_all_ii,3)
    H_all(:,I_all,k) = H_all_ii(:,I_phi,k);
  end
end
% Debug: Nachrechnen der direkten Kinematik: Stimmen die Werte im Plot?
% Funktioniert nur, wenn IK-Fehlschlag mit NaN eingetragen wird
if false
  for i = 1:size(Q_all,1) %#ok<UNRCH>
    phi_i = phiz_range(i);
    X_traj = [X_ref(:,1:5),repmat(phi_i,size(Q_all,3),1)];
    Q_traj = squeeze(Q_all(i,:,:))';
    X_EE = R.fkineEE2_traj(Q_traj);
    phiz_test = angleDiff(X_traj(:,6),X_EE(:,6));
    assert(all(abs(phiz_test) < 1e-6), 'Gelenkkoordinaten stimmen nicht zu phiz-Werten');
  end
end

if s.verbose
  fprintf('Finished discretization of the trajectory. Total %1.1fmin)\n', toc(t1)/60);
end
if length(unique(phiz_range))~=length(phiz_range)
  error('Something went wrong when assembling phiz_range');
end
