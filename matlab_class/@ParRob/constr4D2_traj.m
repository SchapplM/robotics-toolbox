% Zeitableitung der kinematischen Zwangsbedingungen einer Trajektorie
% Variante 4:
% * Bezogen auf Winkelgeschwindigkeit des Koppelpunktes Bi
%   (effektiv werden die Geschw.-ZB nach den Gelenk-Geschw. abgeleitet)
% 
% Eingabe:
% Q:
%   Gelenkkoordinaten (Trajektorie)
% QD:
%   Gelenkgeschwindigkeiten (Trajektorie)
% XE
%   Trajektorie von EE-Lagen
% XDE
%   Trajektorie von EE-Geschwindigkeiten
%   (Die Orientierung wird durch Euler-Winkel-Zeitableitung dargestellt)
% s:
%   Struktur mit Einstellungen für diese Funktion. Enthält die Parameter
%   des Roboters, die sonst in der ParRob-Klasse gespeichert sind.
% 
% Ausgabe:
% PhiD
%   Ableitung der kinematischen Zwangsbedingungen nach der Zeit
%   für alle Zeitschritte der Trajektorie der Eingabedaten
%   Keine reduzierten Einträge. Gebe immer alle möglichen Zwangsbedingungen an.
% 
% Diese Datei ist identisch mit: ParRob/constr4D.m mit dem Zusatz für Trajektorien
% Hier Aufruf einer kompilierbaren Matlab-Funktion aus Vorlage:
% kinematics/pkm_constr4D_traj.m.template

% %VERSIONINFO%
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function PHID = constr4D2_traj(R, Q, QD, X, XD)

assert(size(Q,2) == R.NJ, 'Q muss Trajektorie von Gelenkwinkeln sein');
assert(size(QD,2) == R.NJ, 'QD muss Trajektorie von Gelenkwinkeln sein');
assert(size(X,2) == 6, 'X muss Trajektorie von EE-Posen sein');
assert(size(XD,2) == 6, 'XD muss Trajektorie von EE-Geschwindigkeiten sein');

%% Eingabe-Struktur mit PKM-Parametern zusammenstellen
Leg_I_EE_Task = true(R.NLEG,6);
Leg_pkin_gen = zeros(R.NLEG,length(R.Leg(1).pkin_gen));
Leg_T_N_E_vec = zeros(6,R.NLEG);% 1:3 Euler-Winkel, 4:6 Position
Leg_I_EElink = uint8(zeros(R.NLEG,1));
Leg_phi_W_0 = zeros(3,R.NLEG);
Leg_phiconv_W_0 = uint8(zeros(R.NLEG,1));
Leg_NQJ = zeros(R.NLEG,1);
for i = 1:R.NLEG
  Leg_I_EE_Task(i,:) = R.Leg(i).I_EE_Task;
  Leg_pkin_gen(i,:) = R.Leg(i).pkin_gen';
  Leg_I_EElink(i,:) = uint8(R.Leg(i).I_EElink);
  T_N_E = R.Leg(i).T_N_E;
  Leg_T_N_E_vec(1:3,i) = r2eulxyz(T_N_E(1:3,1:3));
  Leg_T_N_E_vec(4:6,i) = T_N_E(1:3,4);
  Leg_phi_W_0(:,i) = R.Leg(i).phi_W_0;
  Leg_phiconv_W_0(i) = R.Leg(i).phiconv_W_0;
  Leg_NQJ(i) = R.Leg(i).NJ;
end
s = struct( ...
  'I_constr_t', R.I_constr_t, ...
  'I_constr_r', R.I_constr_r, ...
  'I_constr_r_red', R.I_constr_r_red,...
  'r_P_B_all', R.r_P_B_all,...
  'T_P_E', R.T_P_E, ...
  'Leg_pkin_gen', Leg_pkin_gen,...
  'Leg_T_N_E_vec', Leg_T_N_E_vec,...
  'Leg_I_EE_Task', Leg_I_EE_Task,...
  'Leg_phi_W_0', Leg_phi_W_0,...
  'Leg_phiconv_W_0', Leg_phiconv_W_0);
%% Aufruf der Funktion
% Eigenständig funktionierende Datei kann kompiliert werden und ist damit
% wesentlich schneller als die Implementierung mit der Matlab-Klasse.
PHID = R.constr4Dtrajfcnhdl(Q, QD, X, XD, s);
