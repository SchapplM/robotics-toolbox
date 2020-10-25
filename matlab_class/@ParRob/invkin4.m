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

% TODO: Nullraumbewegung mit Nebenbedingung
% TODO: Erfolg der IK prüfen

% Quelle:
% [2] Aufzeichnungen Schappler vom 11.12.2018


function [q, Phi, Tc_stack_PKM] = invkin4(R, xE_soll, q0, s_in)

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
Leg_qlim = zeros(6,2*R.NLEG);
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
  Leg_qlim(1:R.Leg(i).NJ,(1+2*(i-1)):(2+2*(i-1))) = R.Leg(i).qlim;
end
sigma_PKM = R.MDH.sigma; % Marker für Dreh-/Schubgelenk
K = 0.6*ones(R.NJ,1);
K(sigma_PKM==1) = K(sigma_PKM==1) / 5;

s = struct('I_EE', R.I_EE,...
      'I_EE_Task', R.I_EE_Task,...
          'sigma', R.MDH.sigma,...
              'K', K, ... % Verstärkung
             'Kn', 0.4*ones(R.NJ,1), ... % Verstärkung
             'wn', zeros(2,1), ... % Gewichtung der Nebenbedingung
     'maxstep_ns', 1e-10, ... % Maximale Schrittweite für Nullraum zur Konvergenz
      'normalize', false, ... % Normalisieren auf +/- 180°
          'n_min', 0, ... % Minimale Anzahl Iterationen
          'n_max', 1000, ... % Maximale Anzahl Iterationen
      'scale_lim', 1, ... Initialwert für Zufallszahlengenerierung
       'Phit_tol', 1e-8, ... % Toleranz für translatorischen Fehler
       'Phir_tol', 1e-8, ... % Toleranz für rotatorischen Fehler
     'maxrelstep', 0.1, ...% Maximale Schrittweite relativ zu Grenzen 
  'maxrelstep_ns', 0.005,...% Maximale Schrittweite der Nullraumbewegung
    'retry_limit', 100,...
             'NL', R.NL,...
        'I1J_LEG', R.I1J_LEG,...
        'I2J_LEG', R.I2J_LEG,...
 'I_constr_t_red', R.I_constr_t_red,...
 'I_constr_r_red', R.I_constr_r_red,...
   'I_constr_red', R.I_constr_red,....
      'r_P_B_all', R.r_P_B_all, ...
    'phi_P_B_all', R.phi_P_B_all, ...
    'phiconv_W_E', phiconv_W_E, ...
          'T_P_E', R.T_P_E, ...
  'Leg_I_EE_Task', Leg_I_EE_Task, ...
   'Leg_pkin_gen', Leg_pkin_gen, ...
  'Leg_T_N_E_vec', Leg_T_N_E_vec, ...
  'Leg_T_0_W_vec', Leg_T_0_W_vec, ...
  'Leg_I_EElink', Leg_I_EElink, ...
  'Leg_phi_W_0', Leg_phi_W_0, ...
  'Leg_phiconv_W_0', Leg_phiconv_W_0, ...
  'Leg_NQJ', Leg_NQJ,...
  'Leg_qlim', Leg_qlim);

%% Standard-Einstellungen mit Eingaben überschreiben
% Alle Standard-Einstellungen mit in s_in übergebenen Einstellungen
% überschreiben. Diese Reihenfolge ermöglicht für Kompilierung
% geforderte gleichbleibende Feldreihenfolge in Eingabevariablen
if nargin >= 3 && ~isempty(s_in)
  for f = fields(s_in)'
    if isfield(s, f{1})
      s.(f{1}) = s_in.(f{1});
    else
      error('Feld "%s" aus s_in kann nicht übergeben werden', f{1});
    end
  end
end
%% Funktionsaufruf. 
% Entspricht robot_invkin_eulangresidual.m.template
if nargout == 3
  [q, Phi, Tc_stack_PKM] = R.invkin3fcnhdl(xE_soll, q0, s);
else
  [q, Phi] = R.invkin3fcnhdl(xE_soll, q0, s);
end
