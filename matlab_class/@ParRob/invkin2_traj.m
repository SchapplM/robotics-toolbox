% Inverse Kinematik für allgemeinen Roboter (Komplette Trajektorie)
% Allgemeine, stark parametrierbare Funktion zum Aufruf mit allen möglichen
% Einstellungen
% Iterative Lösung der inversen Kinematik mit inverser Jacobi-Matrix
% Zusätzlich Nutzung der differentiellen Kinematik für schnellere Konvergenz
%
% Eingabe:
% XE [N x 6]
%   Trajektorie von EE-Lagen (Sollwerte)
% XDE [N x 6]
%   Trajektorie von EE-Geschwindigkeiten (Sollwerte)
%   (Die Orientierung wird durch Euler-Winkel-Zeitableitung dargestellt)
% XDDE [N x 6]
%   Trajektorie von EE-Beschleunigungen (Sollwerte)
%   Orientierung bezogen auf Euler-Winkel
% T [N x 1]
%   Zeitbasis der Trajektorie (Alle Zeit-Stützstellen)
% q0 [NJ x 1]
%   Anfangs-Gelenkwinkel für Algorithmus
% s_in
%   Struktur mit Eingabedaten für PKM-Trajektorien-IK. Kann leer gelassen
%   werden. Dann Standardwerte. Felder und Standardwerte siehe Quelltext.
% s_ser_in
%   Struktur mit Einstellungen für IK-Algorithmus der einzelnen Beinketten
%   der PKM. Entspricht Eingabe von SerRob/invkin2
%
% Ausgabe:
% Q [N x NJ]
%   Trajektorie von Gelenkpositionen (Lösung der IK)
% QD [N x NJ]
%   Trajektorie von Gelenkgeschwindigkeiten
% QDD [N x NJ]
%   Trajektorie von Gelenkbeschleunigungen
% Phi [N x NCONSTR]
%   Werte der kinematischen Zwangsbedingungen für alle Bahnpunkte
% Jinv_ges [N x NumEl(J)]
%   Inverse PKM-Jacobi-Matrix für alle Bahnpunkte (spaltenweise in Zeile)
%   (Jacobi zwischen allen Gelenkgeschwindigkeiten qD und EE-geschwindigkeit xDE)
%   (Nicht: Nur Bezug zu Antriebsgeschwindigkeiten qaD)
% JinvD_ges [N x NumEl(J)]
%   Zeitableitung von Jinv_ges
% JointPos_all
%   gestapelte Positionen aller Gelenke der PKM für alle Zeitschritte
%   (Entspricht letzter Spalte aller Transformationsmatrizen aus fkine_legs)
% Stats
%   Struktur mit Detail-Ergebnissen für den Verlauf der Berechnung
%
% Siehe auch: SerRob/invkin_traj

% Quelle:
% [2] Aufzeichnungen Schappler vom 11.12.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-04
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [Q, QD, QDD, Phi, Jinv_ges, JinvD_ges, JointPos_all, Stats] = invkin2_traj(R, X, XD, XDD, T, q0, s_in)

%% Eingabe-Struktur mit PKM-Parametern zusammenstellen
Leg_I_EE_Task = true(R.NLEG,6);
Leg_pkin_gen = zeros(R.NLEG,length(R.Leg(1).pkin_gen));
Leg_T_N_E_vec = zeros(6,R.NLEG);% 1:3 Euler-Winkel, 4:6 Position
Leg_T_0_W_vec = zeros(6,R.NLEG);% 1:3 Euler-Winkel, 4:6 Position
Leg_phi_W_0 = zeros(3,R.NLEG);
Leg_phiconv_W_0 = uint8(zeros(R.NLEG,1));
phiconv_W_E = uint8(R.phiconv_W_E);
Leg_sigmaJ = zeros(R.Leg(1).NJ,R.NLEG);
Leg_qlim = zeros(6,2*R.NLEG);
Leg_qDlim = zeros(6,2*R.NLEG);
Leg_qDDlim = zeros(6,2*R.NLEG);
Leg_phiconv_W_E = uint8(zeros(R.NLEG,1));
for i = 1:R.NLEG
  Leg_I_EE_Task(i,:) = R.Leg(i).I_EE_Task;
  Leg_pkin_gen(i,:) = R.Leg(i).pkin_gen';
  T_N_E = R.Leg(i).T_N_E;
  Leg_T_N_E_vec(1:3,i) = r2eulxyz(T_N_E(1:3,1:3));
  Leg_T_N_E_vec(4:6,i) = T_N_E(1:3,4);
  T_0_W = R.Leg(i).T_0_W;
  Leg_T_0_W_vec(1:3,i) = r2eulxyz(T_0_W(1:3,1:3));
  Leg_T_0_W_vec(4:6,i) = T_0_W(1:3,4);
  Leg_phi_W_0(:,i) = R.Leg(i).phi_W_0;
  Leg_phiconv_W_0(i) = R.Leg(i).phiconv_W_0;
  Leg_sigmaJ(:,i) = R.Leg(i).MDH.sigma(R.Leg(i).MDH.mu>=1);
  Leg_qlim(1:R.Leg(i).NJ,(1+2*(i-1)):(2+2*(i-1))) = R.Leg(i).qlim;
  Leg_qDlim(1:R.Leg(i).NJ,(1+2*(i-1)):(2+2*(i-1))) = R.Leg(i).qDlim;
  Leg_qDDlim(1:R.Leg(i).NJ,(1+2*(i-1)):(2+2*(i-1))) = R.Leg(i).qDDlim;
  Leg_phiconv_W_E(i) = R.Leg(i).phiconv_W_E;
end

s = struct('I_EE', R.I_EE,...
      'I_EE_Task', R.I_EE_Task,...
          'sigma', R.MDH.sigma,...
   'simplify_acc', false,...
        'mode_IK', 3,... % 1=Seriell-IK, 2=Parallel-IK, 3=Beide
          'debug', false,...
             'wn', zeros(R.idx_ik_length.wntraj,1),...
 ... % Schwelle der Zielfunktion zum Abbruch der Berechnung
 'abort_thresh_h', NaN(R.idx_ik_length.hntraj,1), ...
           'xlim', R.xlim, ...
          'xDlim', R.xDlim, ...
         'xDDlim', R.xDDlim, ...
 ... % Interpolation der Grenzen für die redundante Koordinate
 'xlim6_interp', zeros(3,0), ...
       'q_poserr', R.update_q_poserr(), ...
 ... % Standardmäßig keine Begrenzung der Nullraumgeschwindigkeit
 ... %  bei Rastpunkten o.ä. Mit dieser Option einstellbar:
 'nullspace_maxvel_interp', zeros(2,0), ...
   'thresh_ns_qa', 1, ... % ab dieser Konditionszahl NR-Bewegung in Gesamt-Koordinaten. Standardmäßig immer.
'ik_solution_min_norm', true, ... % IK-Lösung mit minimaler Norm der Gelenk-Beschleunigung
 'optimcrit_limits_hyp_deact', 0.9, ... % innerhalb der mittleren 90% der Gelenkspannweite hyperbolische Grenzfunktion ausschalten
'cond_thresh_ikjac', 1, ... % Schwellwert zur Aktivierung der IK-Jacobi-Optimierung
'cond_thresh_jac', 1, ... % Schwellwert zur Aktivierung der PKM-Jacobi-Optimierung
     'collbodies', R.collbodies, ... % Liste der Kollisionskörper
'collbodies_thresh', 1.5, ... % Vergrößerung der Kollisionskörper für Aktivierung des Ausweichens
'collision_thresh', NaN, ... % absoluter Schwellwert für die Aktivierung der Kollisions-Kennzahl (hyperbolisch)
     'collchecks', R.collchecks, ... % Liste der zu prüfenden Kollisionsfälle
'installspace_thresh', 0.100, ... % Ab dieser Nähe zur Bauraumgrenze Nullraumbewegung zur Einhaltung des Bauraums
'collbodies_instspc', R.collbodies_instspc, ... % Ersatzkörper zur Bauraumprüfung
'collchecks_instspc', R.collchecks_instspc, ... % Prüfliste für Bauraum
 'I_constr_t_red', R.I_constr_t_red,...
 'I_constr_r_red', R.I_constr_r_red,...
   'I1constr_red', R.I1constr_red,...
   'I2constr_red', R.I2constr_red,...
     'I_constr_t', R.I_constr_t,...
     'I_constr_r', R.I_constr_r,...
   'I_constr_red', R.I_constr_red,...
           'I_qa', R.I_qa,...
      'r_P_B_all', R.r_P_B_all, ...
    'phi_P_B_all', R.phi_P_B_all, ...
    'phiconv_W_E', phiconv_W_E, ...
          'T_P_E', R.T_P_E, ...
  'Leg_I_EE_Task', Leg_I_EE_Task, ...
   'Leg_pkin_gen', Leg_pkin_gen, ...
  'Leg_T_N_E_vec', Leg_T_N_E_vec, ...
  'Leg_T_0_W_vec', Leg_T_0_W_vec, ...
    'Leg_phi_W_0', Leg_phi_W_0, ...
'Leg_phiconv_W_0', Leg_phiconv_W_0, ...
     'Leg_sigmaJ', Leg_sigmaJ, ...
       'Leg_qlim', Leg_qlim, ...
      'Leg_qDlim', Leg_qDlim, ...
     'Leg_qDDlim', Leg_qDDlim, ...
   'enforce_qlim', true, ... % Standardmäßig Positions-Grenzen um jeden Preis versuchen einzuhalten
  'enforce_qDlim', true, ... % Das gleiche für Geschwindigkeits-Grenzen
  'enforce_xlim', true, ... % Einhaltung der Winkelgrenzen für die Plattform (bei Nullraumbewegung)
  'enforce_xDlim', true, ... % Einhaltung der Geschwindigkeitsgrenzen für die Plattform (bei Nullraumbewegung)
'Leg_phiconv_W_E', Leg_phiconv_W_E);
%% Eingabestruktur für IK-Einstellungen
% Einstellungen für IK. Siehe SerRob/invkin2. Standard-Werte müssen damit
% und mit ParRob/invkin_traj konsistent sein.
s_ser = struct( ...
  'reci', false, ... % Standardmäßig keine reziproken Euler-Winkel
  'K', ones(R.Leg(1).NQJ,1), ... % Verstärkung
  'maxrelstep', 0.05, ... % Maximale auf Grenzen bezogene Schrittweite
  'normalize', false, ... % Kein Normalisieren auf +/- 180° (erzeugt Sprung)
  'condlimDLS', 1, ... % Grenze der Konditionszahl, ab der die Pseudo-Inverse gedämpft wird (1=immer)
  'lambda_min', 2e-4, ... % Untergrenze für Dämpfungsfaktor der Pseudo-Inversen
  'n_min', 0, ... % Minimale Anzahl Iterationen
  'n_max', 1000, ... % Maximale Anzahl Iterationen
  'rng_seed', NaN, ... % Initialwert für Zufallszahlengenerierung
  'Phit_tol', 1e-8, ... % Toleranz für translatorischen Fehler
  'Phir_tol', 1e-8); % Toleranz für rotatorischen Fehler
%% Standard-Einstellungen mit Eingaben überschreiben
% Alle Standard-Einstellungen mit in s_in übergebenen Einstellungen
% überschreiben. Diese Reihenfolge ermöglicht für Kompilierung
% geforderte gleichbleibende Feldreihenfolge in Eingabevariablen
if nargin >= 7 && ~isempty(s_in)
  for f = fields(s_in)'
    if ~isfield(s, f{1})
      if isfield(s_ser, f{1})
        s_ser.(f{1}) = s_in.(f{1});
      else
        error('Feld "%s" kann nicht übergeben werden', f{1});
      end
    else
      s.(f{1}) = s_in.(f{1});
    end
  end
end
if length(s_ser.K) == R.NJ
  s_ser.K = s_ser.K(1:R.Leg(1).NQJ);
end
% Eingaben nochmal prüfen
I_nonnan = all(~isnan(s.xlim),2);
assert(all(s.xlim(I_nonnan,1)<s.xlim(I_nonnan,2)), ...
  'Reihenfolge der Grenzen für xlim ist falsch');
%% Funktionsaufruf. 
% Entspricht robot_invkin_eulangresidual.m.template
[Q, QD, QDD, Phi, Jinv_ges, JinvD_ges, JointPos_all, Stats] = R.invkintrajfcnhdl(X, XD, XDD, T, q0, s, s_ser);
end