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
%
% Siehe auch: SerRob/invkin_traj

% TODO: Nullraumbewegung mit Nebenbedingung
% TODO: Erfolg der IK prüfen

% Quelle:
% [2] Aufzeichnungen Schappler vom 11.12.2018


function [Q, QD, QDD, Phi, Jinv_ges, JinvD_ges] = invkin2_traj(R, X, XD, XDD, T, q0, s_in, s_ser_in)

%% Eingabe-Struktur mit PKM-Parametern zusammenstellen
Leg_I_EE_Task = true(R.NLEG,6);
Leg_pkin_gen = zeros(R.NLEG,length(R.Leg(1).pkin_gen));
Leg_T_N_E_vec = zeros(6,R.NLEG);% 1:3 Euler-Winkel, 4:6 Position
Leg_T_0_W_vec = zeros(6,R.NLEG);% 1:3 Euler-Winkel, 4:6 Position
Leg_I_EElink = uint8(zeros(R.NLEG,1));
Leg_phi_W_0 = zeros(3,R.NLEG);
Leg_phiconv_W_0 = uint8(zeros(R.NLEG,1));
Leg_NQJ = zeros(R.NLEG,1);
phiconv_W_E = uint8(R.phiconv_W_E);
Leg_sigmaJ = zeros(R.Leg(1).NJ,R.NLEG);
Leg_qlim = zeros(6,2*R.NLEG);
Leg_phiconv_W_E = uint8(zeros(R.NLEG,1));
for i = 1:R.NLEG
  Leg_I_EE_Task(i,:) = R.Leg(i).I_EE_Task;
  Leg_pkin_gen(i,:) = R.Leg(i).pkin_gen;
  Leg_I_EElink(i,:) = uint8(R.Leg(i).I_EElink);
  T_N_E = R.Leg(i).T_N_E;
  Leg_T_N_E_vec(1:3,i) = r2eulxyz(T_N_E(1:3,1:3));
  Leg_T_N_E_vec(4:6,i) = T_N_E(1:3,4);
  T_0_W = R.Leg(i).T_0_W;
  Leg_T_0_W_vec(1:3,i) = r2eulxyz(T_0_W(1:3,1:3));
  Leg_T_0_W_vec(4:6,i) = T_0_W(1:3,4);
  Leg_phi_W_0(:,i) = R.Leg(i).phi_W_0;
  Leg_phiconv_W_0(i) = R.Leg(i).phiconv_W_0;
  Leg_NQJ(i) = R.Leg(i).NJ;
  Leg_sigmaJ(:,i) = R.Leg(i).MDH.sigma(R.Leg(i).MDH.mu>=1);
  Leg_qlim(1:R.Leg(i).NJ,(1+2*(i-1)):(2+2*(i-1))) = R.Leg(i).qlim;
  Leg_phiconv_W_E(i) = R.Leg(i).phiconv_W_E;
end

s = struct('I_EE', R.I_EE,...
      'I_EE_Task', R.I_EE_Task,...
   'simplify_acc', false,...
        'mode_IK', 2,...
          'debug', false,...
  'I_constr_t_red', R.I_constr_t_red,...
 'I_constr_r_red', R.I_constr_r_red,...
   'I1constr_red', R.I1constr_red,...
    'I2constr_red', R.I2constr_red,...
    'I_constr_t', R.I_constr_t,...
    'I_constr_r', R.I_constr_r,...
       'I_qa', R.I_qa,...
  'r_P_B_all', R.r_P_B_all, ...
  'phi_P_B_all', R.phi_P_B_all, ...
  'NLEG', R.NLEG, ...
    'NJ', R.NJ, ...
  'phiconv_W_E', phiconv_W_E, ...
  'T_P_E', R.T_P_E, ...
  'I1J_LEG', R.I1J_LEG, ...
  'I2J_LEG', R.I2J_LEG, ...
  'Leg_I_EE_Task', Leg_I_EE_Task, ...
  'Leg_pkin_gen', Leg_pkin_gen, ...
  'Leg_T_N_E_vec', Leg_T_N_E_vec, ...
  'Leg_T_0_W_vec', Leg_T_0_W_vec, ...
  'Leg_I_EElink', Leg_I_EElink, ...
  'Leg_phi_W_0', Leg_phi_W_0, ...
  'Leg_phiconv_W_0', Leg_phiconv_W_0, ...
  'Leg_NQJ', Leg_NQJ,...
  'Leg_sigmaJ', Leg_sigmaJ, ...
  'Leg_qlim', Leg_qlim, ...
  'Leg_phiconv_W_E', Leg_phiconv_W_E);
%% Eingabestruktur für IK-Einstellungen
% Einstellungen für IK
K_def = 0.5*ones(R.Leg(1).NQJ,1);
s_ser = struct('reci', true, ...
  'K', K_def, ... % Verstärkung
  'Kn', 1e-2*ones(R.Leg(1).NQJ,1), ... % Verstärkung
  'wn', zeros(2,1), ... % Gewichtung der Nebenbedingung
  'scale_lim', 0.0, ... % Herunterskalierung bei Grenzüberschreitung
  'maxrelstep', 0.05, ... % Maximale auf Grenzen bezogene Schrittweite
  'normalize', true, ... % Normalisieren auf +/- 180°
  'n_min', 0, ... % Minimale Anzahl Iterationen
  'n_max', 1000, ... % Maximale Anzahl Iterationen
  'rng_seed', NaN, ... Initialwert für Zufallszahlengenerierung
  'Phit_tol', 1e-8, ... % Toleranz für translatorischen Fehler
  'Phir_tol', 1e-8, ... % Toleranz für rotatorischen Fehler
  'retry_limit', 100); % Anzahl der Neuversuche);
%% Standard-Einstellungen mit Eingaben überschreiben
% Alle Standard-Einstellungen mit in s_in übergebenen Einstellungen
% überschreiben. Diese Reihenfolge ermöglicht für Kompilierung
% geforderte gleichbleibende Feldreihenfolge in Eingabevariablen
if nargin >= 7 && ~isempty(s_in)
  for f = fields(s_in)'
    if ~isfield(s, f{1})
      error('Feld "%s" kann nicht übergeben werden', f{1});
    else
      s.(f{1}) = s_in.(f{1});
    end
  end
end
if nargin == 8
  for ff = fields(s_ser_in)'
    if ~isfield(s_ser, ff{1})
      error('Feld "%s" kann nicht übergeben werden', ff{1});
    else
      s_ser.(ff{1}) = s_ser_in.(ff{1});
    end
  end
end
%% Funktionsaufruf. 
% Entspricht robot_invkin_eulangresidual.m.template
[Q, QD, QDD, Phi, Jinv_ges, JinvD_ges] = R.invkintrajfcnhdl(X, XD, XDD, T, q0, s, s_ser);
end