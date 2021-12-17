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
%   Gestapelte Transformationsmatrizen der PKM. Im Basis-KS.
%   Entspricht mit Abwandlung der Anordnung wie in fkine: 
%   * PKM-Basis
%   * Für jede Beinkette: Basis und alle bewegten Körper-KS. Ohne
%     virtuelles EE-KS
%   * Kein Plattform-KS

% Quelle:
% [2] Aufzeichnungen Schappler vom 11.12.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [q, Phi, Tc_stack_PKM, Stats] = invkin4(R, xE_soll, q0, s_in)

%% Standardwerte für vom Benutzer setzbare Einstellungen
s_user = struct(...
              'K', ones(R.NJ,1), ... % Verstärkung Aufgabenbewegung
             'Kn', ones(R.NJ,1), ... % Verstärkung Nullraumbewegung
             'wn', zeros(R.idx_ik_length.wnpos,1), ... % Gewichtung der Nebenbedingung
           'xlim', R.xlim, ... % Begrenzung der Endeffektor-Koordinaten
     'maxstep_ns', 1e-10, ... % Maximale Schrittweite für Nullraum zur Konvergenz (Abbruchbedingung)
      'normalize', false, ... % Normalisieren auf +/- 180°
     'condlimDLS', 1, ... % Grenze der Konditionszahl, ab der die Pseudo-Inverse gedämpft wird (1=immer)
     'lambda_min', 2e-4, ... % Untergrenze für Dämpfungsfaktor der Pseudo-Inversen
          'n_min', 0, ... % Minimale Anzahl Iterationen
          'n_max', 1000, ... % Maximale Anzahl Iterationen
       'rng_seed', NaN, ... % Initialwert für Zufallszahlengenerierung
      'scale_lim', 0, ... % Herunterskalierung bei Grenzüberschreitung
     'scale_coll', 0.0, ... % Herunterskalieren bei angehender Kollision
'collbodies_thresh', 1.5, ... % Vergrößerung der Kollisionskörper für Aktivierung des Ausweichens
'collision_thresh', NaN, ... % absoluter Schwellwert für die Aktivierung der Kollisions-Kennzahl (hyperbolisch)
'installspace_thresh', 0.100, ... % Ab dieser Nähe zur Bauraumgrenze Nullraumbewegung zur Einhaltung des Bauraums
       'Phit_tol', 1e-8, ... % Toleranz für translatorischen Fehler
       'Phir_tol', 1e-8, ... % Toleranz für rotatorischen Fehler
     'maxrelstep', 0.1, ...% Maximale Schrittweite relativ zu Grenzen 
  'maxrelstep_ns', 0.005,...% Maximale Schrittweite der Nullraumbewegung
'finish_in_limits', false, ...% Führe am Ende eine Nullraumoptimierung zur Wiederherstellung der Grenzen durch
'avoid_collision_finish', false,... % Nullraumbewegung am Ende zur Vermeidung von Kollisionen
  ... % Bei hyperbolischen Grenzen kann z.B. mit Wert 0.9 erreicht werden, 
  ... % dass in den mittleren 90% der Gelenkwinkelspannweite das Kriterium 
  ... % deaktiviert wird (Stetigkeit durch Spline). Deaktivierung mit NaN.
  'optimcrit_limits_hyp_deact', NaN, ... 
'cond_thresh_ikjac', 1, ... % Schwellwert zur Aktivierung der IK-Jacobi-Optimierung
'cond_thresh_jac', 1, ... % Schwellwert zur Aktivierung der PKM-Jacobi-Optimierung
  'retry_on_limitviol', false, ... % Bei Grenzverletzung neu versuchen mit true
    'retry_limit', 100, ...
    'debug', false);
%% Standard-Einstellungen mit Eingaben überschreiben
% Alle Standard-Einstellungen aus s_user mit in s_in übergebenen Einstellungen
% überschreiben.
if nargin == 4 && ~isempty(s_in)
  for f = fields(s_in)'
    if isfield(s_user, f{1})
      s_user.(f{1}) = s_in.(f{1});
    else
      error('Feld "%s" aus s_in kann nicht übergeben werden', f{1});
    end
  end
end
if sum(R.I_EE) <= sum(R.I_EE_Task)
  % Setze Gewichtung der Nullraum-Zielfunktionen auf Null. Es gibt keinen
  % Nullraum. Muss hier gemacht werden. Sonst Logik-Fehler in Funktion.
  s_user.wn(:) = 0;
end
% Deaktiviere Kollisionsvermeidung, wenn keine Körper definiert sind.
if s_user.wn(R.idx_ikpos_wn.coll_hyp) && (isempty(R.collchecks) || isempty(R.collbodies))
  s_user.wn(R.idx_ikpos_wn.coll_hyp) = 0;
  s_user.scale_coll = 0;
  s_user.avoid_collision_finish = false;
end
%% Eingabe-Struktur mit PKM-Parametern zusammenstellen
% Diese Reihenfolge ermöglicht für Kompilierung geforderte gleichbleibende 
% Feldreihenfolge in Eingabevariablen
Leg_pkin_gen = zeros(R.NLEG,length(R.Leg(1).pkin_gen));
Leg_T_N_E_vec = zeros(6,R.NLEG);% 1:3 Euler-Winkel, 4:6 Position
Leg_T_0_W_vec = zeros(6,R.NLEG);% 1:3 Euler-Winkel, 4:6 Position
Leg_phi_W_0 = zeros(3,R.NLEG);
Leg_qlim = zeros(6,2*R.NLEG);
for i = 1:R.NLEG
  Leg_pkin_gen(i,:) = R.Leg(i).pkin_gen;
  T_N_E = R.Leg(i).T_N_E;
  Leg_T_N_E_vec(1:3,i) = r2eulxyz(T_N_E(1:3,1:3));
  Leg_T_N_E_vec(4:6,i) = T_N_E(1:3,4);
  T_0_W = R.Leg(i).T_0_W;
  Leg_T_0_W_vec(1:3,i) = r2eulxyz(T_0_W(1:3,1:3));
  Leg_T_0_W_vec(4:6,i) = T_0_W(1:3,4);
  Leg_phi_W_0(:,i) = R.Leg(i).phi_W_0;
  Leg_qlim(1:R.Leg(i).NJ,(1+2*(i-1)):(2+2*(i-1))) = R.Leg(i).qlim;
end
s = struct(...
      'I_EE_Task', R.I_EE_Task,...
          'sigma', R.MDH.sigma,...
              'K', s_user.K, ...
             'Kn', s_user.Kn, ...
             'wn', s_user.wn, ...
           'xlim', s_user.xlim, ...
     'maxstep_ns', s_user.maxstep_ns, ...
      'normalize', s_user.normalize, ... 
     'condlimDLS', s_user.condlimDLS, ...
     'lambda_min', s_user.lambda_min, ...
          'n_min', s_user.n_min, ...
          'n_max', s_user.n_max, ...
       'rng_seed', s_user.rng_seed, ... 
      'scale_lim', s_user.scale_lim, ...
     'scale_coll', s_user.scale_coll, ...
       'Phit_tol', s_user.Phit_tol, ... 
       'Phir_tol', s_user.Phir_tol, ... 
     'maxrelstep', s_user.maxrelstep, ...
  'maxrelstep_ns', s_user.maxrelstep_ns,...
'finish_in_limits', s_user.finish_in_limits,...
'avoid_collision_finish', s_user.avoid_collision_finish, ...
'optimcrit_limits_hyp_deact', s_user.optimcrit_limits_hyp_deact, ...
'cond_thresh_ikjac', s_user.cond_thresh_ikjac, ...
'cond_thresh_jac', s_user.cond_thresh_jac, ...
'retry_on_limitviol', s_user.retry_on_limitviol, ...
    'retry_limit', s_user.retry_limit,...
     'collbodies', R.collbodies, ... % Liste der Kollisionskörper
'collbodies_thresh', s_user.collbodies_thresh,...
'collision_thresh', s_user.collision_thresh, ...
     'collchecks', R.collchecks, ... % Liste der zu prüfenden Kollisionsfälle
'installspace_thresh', s_user.installspace_thresh, ...
'collbodies_instspc', R.collbodies_instspc, ... % Ersatzkörper zur Bauraumprüfung
'collchecks_instspc', R.collchecks_instspc, ... % Prüfliste für Bauraum
          'debug', s_user.debug,...
     'I_constr_t', R.I_constr_t,...
     'I_constr_r', R.I_constr_r,...
 'I_constr_t_red', R.I_constr_t_red,...
 'I_constr_r_red', R.I_constr_r_red,...
   'I_constr_red', R.I_constr_red,....
           'I_qa', R.I_qa,....
      'r_P_B_all', R.r_P_B_all, ...
    'phi_P_B_all', R.phi_P_B_all, ...
    'phiconv_W_E', R.phiconv_W_E, ...
          'T_P_E', R.T_P_E, ...
  'Leg_I_EE_Task', cat(1,R.Leg.I_EE_Task), ...
   'Leg_pkin_gen', Leg_pkin_gen, ...
  'Leg_T_N_E_vec', Leg_T_N_E_vec, ...
  'Leg_T_0_W_vec', Leg_T_0_W_vec, ...
    'Leg_phi_W_0', Leg_phi_W_0, ...
'Leg_phiconv_W_0', cat(1,R.Leg.phiconv_W_0), ...
       'Leg_qlim', Leg_qlim);
%% Funktionsaufruf. 
% Entspricht kinematics/pkm_invkin3.m.template
if nargout == 3
  [q, Phi, Tc_stack_PKM] = R.invkin3fcnhdl(xE_soll, q0, s);
elseif nargout <= 2
  [q, Phi] = R.invkin3fcnhdl(xE_soll, q0, s);
else
  [q, Phi, Tc_stack_PKM, Stats] = R.invkin3fcnhdl(xE_soll, q0, s);
end