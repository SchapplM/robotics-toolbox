% Testskript zur Implementierung der Limitierung der Rotation um die
% Z-Achse des Endeffektors
%   - zunächst für 3T2R und später auch für 2T2R (Translation entlang Z)
%  

close all
clear
clc
% Unterdrücke Warnung für Schlechte Konditionierung der Jacobi-Matrix
warning('off', 'MATLAB:rankDeficientMatrix');
warning('off', 'Coder:MATLAB:rankDeficientMatrix'); % gleiche Warnung aus mex-Datei
if isempty(which('serroblib_path_init.m'))
  warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end

%% Definitionen, Benutzereingaben
% Robotermodell entweder aus PKM-Bibliothek oder nur aus
% Seriell-Roboter-Bibliothek laden. Stellt keinen Unterschied dar.
use_parrob = false;
debug_plot = false;
rob_path = fileparts(which('robotics_toolbox_path_init.m'));
respath = fullfile(rob_path, 'examples_tests', 'results');

I_EE_3T3R = logical([1 1 1 1 1 1]);
I_EE_3T2R = logical([1 1 1 1 1 0]);

%% Klasse für PKM erstellen (basierend auf serieller Beinkette)
if isempty(which('serroblib_path_init.m'))
  warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end
if ~use_parrob
  % Typ des seriellen Roboters auswählen (S6RRPRRRV3 = UPS)
  SName='S6RRPRRR14V3';
  % Instanz der Roboterklasse erstellen
  RS = serroblib_create_robot_class(SName);
  serroblib_update_template_functions({SName});
  RS.fill_fcn_handles(true, true);
%  serroblib_create_template_functions({SName}, false, false);
%   matlabfcn2mex({[RS.mdlname, '_invkin_eulangresidual']});
  % RS.mex_dep(true)
  RP = ParRob('P6RRPRRR14V3G1P4A1');
  RP.create_symmetric_robot(6, RS, 0.5, 0.2);
  % Schubgelenke sind aktuiert
  I_qa = false(36,1);
  I_qa(3:6:36) = true;
  RP.update_actuation(I_qa);
  % Benutze PKM-Bibliothek für gespeicherte Funktionen
  if ~isempty(which('parroblib_path_init.m'))
    parroblib_addtopath({'P6RRPRRR14V3G1P4A1'});
  end
end

%% Alternativ: Klasse für PKM erstellen (basierend auf PKM-Bibliothek)
if use_parrob
  if isempty(which('parroblib_path_init.m'))
    warning('Repo mit parallelen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
    return
  end
  RP = parroblib_create_robot_class('P6RRPRRR14V3G1P4A1', 0.5, 0.2);
  parroblib_update_template_functions({'P6RRPRRR14V3G1P4A1'});
end
RP.fill_fcn_handles(true, true);
% parroblib_create_template_functions({RP.mdlname}, false, false);
% matlabfcn2mex({[RP.mdlname(1:end-6), '_invkin']});
% matlabfcn2mex({[RP.mdlname(1:end-6), '_invkin3']});
% matlabfcn2mex({[RP.mdlname(1:end-6), '_invkin_traj']});
%% Plattform-Konfiguration verändern
% Mit einer Kreisförmigen Plattformkoppelpunktanordnung ist die PKM
% singulär (Jacobi der direkten Kinematik). Daher paarweise Anordnung
RP.align_platform_coupling(4, [0.2;0.1]);

%% Grenzen für die Gelenkpositionen setzen
% Dadurch wird die Schrittweite bei der inversen Kinematik begrenzt (auf 5%
% der Spannbreite der Gelenkgrenzen) und die Konfiguration klappt nicht um.
for i = 1:RP.NLEG
  % Begrenze die Winkel der Kugel- und Kardangelenke auf +/- 360°
  RP.Leg(i).qlim = repmat([-2*pi, 2*pi], RP.Leg(i).NQJ, 1);
  RP.Leg(i).qDlim = repmat([-4*pi, 4*pi], RP.Leg(i).NQJ, 1);
  RP.Leg(i).qDDlim = RP.Leg(i).qDlim/0.1; % geteilt durch Anstiegszeit
  % Begrenze die Länge der Schubgelenke
  RP.Leg(i).qlim(3,:) = [0.1, 1.5];
  RP.Leg(i).qDlim(3,:) = [-0.5, 0.5];
  RP.Leg(i).qDDlim(3,:) = RP.Leg(i).qDlim(3,:)/0.1; % geteilt durch Anstiegszeit für Trajektorie
end
qlim_pkm = cat(1, RP.Leg.qlim);
%% Startpose bestimmen
% Mittelstellung im Arbeitsraum
X = [ [0.15;0.05;0.5]; [10;-10;5]*pi/180 ];
% Startwerte für numerische IK
q0 = qlim_pkm(:,1)+rand(36,1).*(qlim_pkm(:,2)-qlim_pkm(:,1));
q0(RP.I_qa) = 0.5; % mit Schubaktor größer Null anfangen (damit Konfiguration nicht umklappt)
% Inverse Kinematik auf zwei Arten berechnen
[~, Phi] = RP.invkin1(X, q0);
if any(abs(Phi) > 1e-8)
  error('Inverse Kinematik konnte in Startpose nicht berechnet werden (Beine gemeinsam)');
end
% IK berechnen. Dabei solange probieren, bis Schubgelenke nicht umklappen
[q, Phis] = RP.invkin_ser(X, q0, struct('retry_on_limitviol',true));
if any(abs(Phis) > 1e-6)
  error('Inverse Kinematik (für jedes Bein einzeln) konnte in Startpose nicht berechnet werden');
end
if any(q(RP.I_qa) < 0)
  error('Start-Konfiguration ist umgeklappt mit Methode Seriell. Darf nicht passieren');
end
qs = q;
%% Wähle willkürlichen Winkel-Offset für Beinketten-Koppel-KS
% Gelenkkoordinaten sollen möglichst um die Null sein, um die
% Winkel-Normalisierung zu vereinfachen
% Benutze die zusätzliche Transformation am Endeffektor der einzelnen
T_0_E = RP.x2t(X);
disp('Gelenkwinkel der Kardangelenke vor Verschiebung der Nullpunkte:');
disp(qs([1:6:RP.NJ;2:6:RP.NJ])*180/pi);
disp('Gelenkwinkel der Kugelgelenke vor Verschiebung der Nullpunkte:');
disp(qs([4:6:RP.NJ;5:6:RP.NJ;6:6:RP.NJ])*180/pi);
for i = 1:6
  % Initialisierung (für Mehrfach-Durchführungen)
  RP.Leg(i).update_EE([], zeros(3,1));
  % Gelenkwinkel dieser Beinkette vorbereiten
  qs_i = qs(RP.I1J_LEG(i):RP.I2J_LEG(i));
  % Kugel-Gelenk vorher */150/*. Jetzt */30/*
  % q_spher_fix = qs_i(4:6) - [0;120*pi/180;0];
  q_spher_fix = [0;30;60]*pi/180;
  
  [~,T_0_Pi] = RP.Leg(i).fkineEE(qs_i);
  % Letzte drei Winkel virtuell auf vorgegebenen Wert setzen
  qs_i_test = [qs_i(1:3); q_spher_fix];
  [~,T_0_Pi_test] = RP.Leg(i).fkineEE(qs_i_test);
  % Differenz-Rotation zwischen beiden Fällen als Zusatz-Rotation N-E setzen
  R_N_E = T_0_Pi_test(1:3,1:3)' * T_0_Pi(1:3,1:3);
  RP.Leg(i).update_EE([], r2eul(R_N_E, RP.Leg(i).phiconv_N_E));
  % Start-Winkel anpassen
  qs(RP.I1J_LEG(i)+3:RP.I2J_LEG(i)) = q_spher_fix;
  % % Test: 
  % test = T_0_Pi_test(1:3,1:3)*R_N_E - T_0_E(1:3,1:3);
  % % Kinematik neu berechnen
  % [~,T_0_Pi_test2] = RP.Leg(i).fkineEE(qs_i_test);
  % Gelenkwinkel dieser Beinkette vorbereiten
  qs_i = qs(RP.I1J_LEG(i):RP.I2J_LEG(i));
  % Ändere Beinketten-Basis-KS so, dass auch die Winkel des Kardan-Gelenks
  % Null sind.
  % TODO: Zelle nach Definition des Roboters nur einmal ausführbar
  % Kardan-Gelenk vorher 90/150. Jetzt 30/-30
  % q_univ_fix = qs_i(1:2) - 0*[pi/2; pi];
  q_univ_fix = [30;-30]*pi/180;
  R_0_0i1 = RP.Leg(i).T_W_0(1:3,1:3);
  [~,T_0_Pi] = RP.Leg(i).fkineEE(qs_i);
  % Erste zwei Winkel virtuell Null setzen
  qs_i_test = [q_univ_fix; qs_i(3:6)];
  T_0i_Pi_test = RP.Leg(i).fkineEE(qs_i_test);
  % Differenz-Rotation zwischen beiden Fällen zusätzlich zur Rotations 0i-0
  % hinzufügen
  R_0i1_0i2 = R_0_0i1' * T_0_Pi(1:3,1:3) * T_0i_Pi_test(1:3,1:3)';
  % test = R_0_0i1 * R_0i1_0i2 * T_0i_Pi_test(1:3,1:3) - T_0_Pi(1:3,1:3)
  R_0_0i_neu = R_0_0i1 * R_0i1_0i2;
  RP.Leg(i).update_base([], r2eul(R_0_0i_neu, RP.Leg(i).phiconv_W_0));
  % Start-Winkel anpassen
  qs(RP.I1J_LEG(i):RP.I1J_LEG(i)+1) = q_univ_fix;
end

% Winkel erneut normalisieren
qs(RP.MDH.sigma==0) = normalize_angle(qs(RP.MDH.sigma==0));
disp('Gelenkwinkel der Kardangelenke nach Verschiebung der Nullpunkte:');
disp(qs([1:6:RP.NJ;2:6:RP.NJ])*180/pi);
disp('Gelenkwinkel der Kugelgelenke nach Verschiebung der Nullpunkte:');
disp(qs([4:6:RP.NJ;5:6:RP.NJ;6:6:RP.NJ])*180/pi);

%% Zwangsbedingungen in Startpose testen
RP.update_EE_FG(I_EE_3T3R, I_EE_3T3R);
Phi1=RP.constr1(qs, X);
Phit1=RP.constr1_trans(qs, X);
Phir1=RP.constr1_rot(qs, X);
if any(abs(Phi1) > 1e-6)
  error('ZB in Startpose ungleich Null');
end

%% Roboter in Startpose plotten
if debug_plot
  change_current_figure(1); clf; hold on; grid on; % Bild als Kinematik-Skizze
  xlabel('x in m');ylabel('y in m');zlabel('z in m'); view(3);
  s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
  RP.plot( qs, X, s_plot );
  title('6UPS in Startkonfiguration als Kinematik-Skizze');

  change_current_figure(2); clf; hold on; grid on; % Bild der Entwurfsparameter
  xlabel('x in m');ylabel('y in m');zlabel('z in m'); view(3);
  s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0, 'mode', 4);
  RP.plot( qs, X, s_plot );
  title('6UPS in Startkonfiguration mit Ersatzkörpern');
end

%% Gelenkwinkelgrenzen festlegen (für Optimierung
for i = 1:RP.NLEG
  q_i = qs(RP.I1J_LEG(i):RP.I2J_LEG(i));
  qlim_i = repmat(q_i,1,2);
  % Grenzen um die aktuelle Konfiguration herum wählen
  I1 = RP.Leg(i).MDH.sigma==1;
  % Schubachsen: Maximaler Hub +-500mm
  qlim_i(I1,:) = repmat(q_i(I1),1,2) + repmat([-0.5, 0.5], sum(I1),1);
  I0 = RP.Leg(i).MDH.sigma==0;
  % Kippwinkel der Kardan- und Kugelgelenke max. 60° in jede Richtung
  qlim_i(I0,:) = repmat(q_i(I0),1,2) + repmat([-pi/3, pi/3], sum(I0),1);
  RP.Leg(i).qlim = qlim_i;
end
% qlim für gesamte PKM festlegen
qlim = NaN(RP.NJ,2);
J1 = 1;
for i = 1:RP.NLEG
  J2 = J1+RP.Leg(i).NQJ-1;
  qlim(J1:J2,:) = RP.Leg(i).qlim;
  J1 = J2+1;
end

%% Initialisierung Teil 2
RP.update_EE_FG(I_EE_3T3R, I_EE_3T2R);
% RP.fill_fcn_handles(true,true);

%% Jacobi-Matrizen auswerten

[G_q_red,G_q_voll] = RP.constr3grad_q(qs, X);
[~,G_x_voll] = RP.constr3grad_x(qs, X); % TODO G_x_red ist noch nicht voll implementiert

% Testen der Komponentenaufteilung
G_q = G_q_voll(RP.I_constr_red,:);
G_x = G_x_voll(RP.I_constr_red,:);
if any(G_q_red(:)-G_q(:))
  error('Aufteilung der ZB-Komponenten stimmt nicht zwischen constr3grad_q/constr3grad_x/ParRob');
end

% Aufteilung der Ableitung nach den Gelenken in Gelenkklassen 
% * aktiv/unabhängig (a),
% * passiv+schnitt/abhängig (d)
G_a = G_q(:,RP.I_qa);
G_d = G_q(:,RP.I_qd);
% Jacobi-Matrix zur Berechnung der abhängigen Gelenke und EE-Koordinaten
G_dx = [G_d, G_x];

fprintf('%s: Rang der vollständigen Jacobi der inversen Kinematik: %d/%d\n', ...
  RP.mdlname, rank(G_q), RP.NJ);
fprintf('%s: Rang der vollständigen Jacobi der direkten Kinematik: %d/%d\n', ...
  RP.mdlname, rank(G_dx), sum(RP.I_EE_Task)+sum(RP.I_qd));
fprintf('%s: Rang der Jacobi der aktiven Gelenke: %d/%d\n', ...
  RP.mdlname, rank(G_a), sum(RP.I_EE_Task));


%% Trajektorienpunkte vorgeben
% X0 = [ [0;0;0.5]; [0;0;0]*pi/180 ];
% % Trajektorie mit beliebigen Bewegungen der Plattform
% XL = [X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
%       X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.3]]; ...
%       X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
%       X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.3, 0.0]]; ...
%       X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
%       X0'+1*[[ 0.0, 0.0, 0.0], [0.3, 0.0, 0.0]]; ...
%       X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
%       X0'+1*[[ 0.2,-0.1, 0.3], [0.3, 0.2, 0.1]]; ...
%       X0'+1*[[-0.1, 0.2,-0.1], [0.5,-0.2,-0.2]]; ...
%       X0'+1*[[ 0.2, 0.3, 0.2], [0.2, 0.1, 0.3]]];
% XL = [XL; XL(1,:)]; % Rückfahrt zurück zum Startpunkt.
d1 = 0.25;
X0 = [ [0;0;0.7]; [0;0;0]*pi/180 ];
% Trajektorie mit beliebigen Bewegungen der Plattform
XL = [X0'+1*[[ 0.0, 0.0, 0],   [0.0, 0.0, 0]*pi/180]; ...
      X0'+1*[[ 0.0, 0.0, -d1], [0.0, 0.0, 30.0]*pi/180]; ...
      X0'+1*[[ -d1, 0.0, -d1], [0.0, 0.0, 50.0]*pi/180]; ...
      X0'+1*[[ -d1, -d1, -d1], [0.0, 0.0, 60.0]*pi/180]; ...
      X0'+1*[[ d1, -d1, -d1],  [0.0, 0.0, -30.0]*pi/180]; ...
      X0'+1*[[ d1, d1, -d1],   [0.0, 0.0, -50.0]*pi/180]; ...
      X0'+1*[[ 0.0, d1, -d1],  [0.0, 0.0, -60.0]*pi/180]; ...
      X0'+1*[[ 0.0, d1, 0],    [0.0, 0.0, 0]*pi/180]];
% XL = [XL; XL(1,:)]; % Rückfahrt zurück zum Startpunkt.

[X_t,XD_t,XDD_t,t] = traj_trapez2_multipoint(XL, 1, 0.1, 0.01, 1e-3, 1e-1);

% Inverse Kinematik berechnen
q0 = qs; % Lösung der IK von oben als Startwert
t0 = tic();
% IK-Einstellungen: Sehr lockere Toleranzen, damit es schneller geht
s = struct('Phit_tol', 1e-3, 'Phir_tol', 1*pi/180);
RP.update_EE_FG(I_EE_3T3R, I_EE_3T3R);
[q1, Phi_num1] = RP.invkin1(X_t(1,:)', q0, s);
if any(abs(Phi_num1) > 1e-2)
  warning('IK konvergiert nicht');
end

if debug_plot
  % Plot des Roboters im Startpunkt
  change_current_figure(3); clf; hold on; grid on; % Bild als Kinematik-Skizze
  xlabel('x in m');ylabel('y in m');zlabel('z in m'); view(3);
  s_plot = struct( 'ks_legs', [], 'straight', 0);
  RP.plot( q1, X_t(1,:)', s_plot );
  hold on;
  plot3(XL(:,1), XL(:,2), XL(:,3), 'r-', 'LineWidth', 2);
end
% % Plot der KSe der Punkte im Startpunkt
% change_current_figure(4); clf; hold on; grid on; % Bild als Kinematik-Skizze
% xlabel('x in m');ylabel('y in m');zlabel('z in m'); view(3);
% s_plot_KS = struct( 'ks', [8, RS.NJ+2], 'straight', 0, 'mode', 2);
% % RP.plot( q1, X_t(1,:)', s_plot );
% hold on;
% for kk = 1:size(XL,1)
%   [q_kk, Phi_num_kk] = RS.invkin2(RS.x2tr(XL(kk,:)'), [0;0;0;0;0;0], s);
%   hold on;
%   RS.plot( q_kk, s_plot_KS);
% end
% % XL_new_plot = XL;
% % XL_new_plot(:,1) = XL_new_plot(:,1)+0.5;
% % plot3(XL_new_plot(:,1), XL_new_plot(:,2), XL_new_plot(:,3), 'r-', 'LineWidth', 1);
% title('Eckpunkte mit Koordinatensystemen zeichnen');

% Mit Einzelpunkt-IK Punkte 3T3R anfahren und plotten
if debug_plot
  s_plot = struct( 'ks_legs', [], 'straight', 0);
  for kk = 1:size(XL,1)
    [q_traj_ep, Phi_traj_ep] = RP.invkin1(XL(kk,:)', q0, s);
    if any(abs(Phi_num1) > 1e-2)
      warning('IK konvergiert nicht');
    end
    change_current_figure(100+kk); clf; hold on; grid on;
    xlabel('x in m');ylabel('y in m');zlabel('z in m'); view(3);
    RP.plot( q_traj_ep, XL(kk,:)', s_plot );
    hold on;
    plot3(XL(:,1), XL(:,2), XL(:,3), 'r-', 'LineWidth', 1);
    hold on;
  end
end
%% Einzelpunkt-IK | Initialisierung und Berechnung
fprintf('Starte Berechnungen für Trajektorie als Einzelpunkte\n');
RP.update_EE_FG(I_EE_3T3R, I_EE_3T2R);
Phirt_tol = 1e-9; % 1e-12 ist manchmal numerisch nicht erreichbar
n_max = 2500;
amount_optcrit = 1+3; % 1(3T2R) + 2(limred hyp + limred quadr + limred quadrhyp)
h7_ep = NaN(1+n_max, amount_optcrit, size(XL,1)); % h6 kann auch ausgelesen werden, wenn wn(6) nicht aktiv ist
h8_ep = h7_ep;
phiz_ep = h7_ep;
iter_ep = NaN(size(XL,1), amount_optcrit);
Q_ep = NaN(1+n_max, 36, size(XL,1), amount_optcrit);
limred_crit_active = zeros(1+n_max, amount_optcrit, size(XL,1));
warning_iter_count = 0;
warning_phi_count = 0;
warnplot_needed_vector = NaN;
warnplot_needed_vector_counter = 1;
warnplot_needed_state = 0;
phiz_oob         = zeros(amount_optcrit-1,size(XL,1));  % oob: out-of-bounds bzgl. xlim
phiz_oob_vector  = NaN;
phiz_oob_vector_counter = 1;
phiz_oob_state = 0;
phiz_oob_thresh  = zeros(amount_optcrit-1,size(XL,1));  % zwischen xlim und 0.8*xlim
phiz_oob_thresh_vector  = NaN;
phiz_oob_thresh_vector_counter = 1;
phiz_oob_thresh_state = 0;

% Start-Gelenkwinkel
q0_ep = q1;

% Structs definieren
s_ep_wn = zeros(RP.idx_ik_length.wnpos,amount_optcrit);  % 3T2R mit wn(:) = 0
s_ep_wn(RP.idx_ikpos_wn.qlim_hyp) = 1; % Abstoßung von Gelenkwinkelgrenzen (Hyp.) aktivieren
% limred mit hyperbolischem Ansatz
s_ep_wn(RP.idx_ikpos_wn.xlim_par,2) = 0;
s_ep_wn(RP.idx_ikpos_wn.xlim_hyp,2) = 1;
% limred mit quadratischem Ansatz
s_ep_wn(RP.idx_ikpos_wn.xlim_par,3) = 1;
s_ep_wn(RP.idx_ikpos_wn.xlim_hyp,3) = 0;
% limred mit quadratischem und hyperbolischem Ansatz
s_ep_wn(RP.idx_ikpos_wn.xlim_par,4) = 1;
s_ep_wn(RP.idx_ikpos_wn.xlim_hyp,4) = 1;
Iwnxlim = [RP.idx_ikpos_wn.xlim_par, RP.idx_ikpos_wn.xlim_hyp];
opt1 = s_ep_wn(Iwnxlim,2);   % Sicherung für späteren Plot
opt2 = s_ep_wn(Iwnxlim,3);   % Sicherung für späteren Plot
opt3 = s_ep_wn(Iwnxlim,4);   % Sicherung für späteren Plot
s_ep = struct( ...
  'n_min', 0, 'n_max', n_max, 'Phit_tol', Phirt_tol, 'Phir_tol', Phirt_tol, ...
  'scale_lim', 0, 'wn', zeros(RP.idx_ik_length.wnpos,1), 'retry_limit', 0);
s_ep.xlim   = [NaN(5,2); [-45 45]*pi/180]; % in [rad] übergeben

for i = 1:size(XL,1)
  % Inverse Kinematik für die Auswertung mit Stats
  for k = 1:amount_optcrit
    s_ep.wn = s_ep_wn(:,k); % k verschiedene Optimierungskriterien
    [q_IK_ep, phi_IK_ep, ~, Stats_IK_ep] = RP.invkin3(XL(i,:)', q0_ep, s_ep);
    h7_ep(:,k,i)   = Stats_IK_ep.h(:,1+RP.idx_ikpos_hn.xlim_par);
    h8_ep(:,k,i)   = Stats_IK_ep.h(:,1+RP.idx_ikpos_hn.xlim_hyp);
    phiz_ep(:,k,i) = Stats_IK_ep.PHI(:,4);
    iter_ep(i,k)   = Stats_IK_ep.iter;
    Q_ep(:,:,i,k)  = Stats_IK_ep.Q(:,:);
    % Testweise Aufruf der kompilierten Funktion
    [q_IK_ep2, phi_IK_ep2, ~, Stats_IK_ep2] = RP.invkin4(XL(i,:)', q0_ep, s_ep);
    test_q_v3v4 = q_IK_ep-q_IK_ep2;
    assert(all(abs(normalizeAngle(test_q_v3v4,0)) < 1e-6), 'Ergebnis von invkin3 und invkin4 stimmt nicht überein');

    % Test-Berechnung ob wn(6) bzw. wn(7) aktiv ist (phi<1e-3, siehe invkin)
    for nn = 1:(Stats_IK_ep.iter+1)
      Stats_Phi_temp = Stats_IK_ep.PHI(nn,:);
      if all(abs(Stats_Phi_temp(RP.I_constr_red))<1e-3)  % vierten Eintrag ignorieren
        limred_crit_active(nn,k,i) = 1;
      end
    end

    % Warnung, wenn aktuelle IK ungültig -> Fehlerausgabe am Ende
    if (any(abs(phi_IK_ep) > Phirt_tol)) && (k == 4)  % nur bei vollständiger Optimierung interessant
      warning('Inverse Kinematik bei Punkt %d für Optimierungsmethode %d fehlerhaft!', i, k); 
      warning_phi_count = warning_phi_count + 1;
      warnplot_needed_state = 1;
    end

    if k > 1
      if phiz_ep(Stats_IK_ep.iter+1,k,i) <= s_ep.xlim(6,1) || phiz_ep(Stats_IK_ep.iter+1,k,i) >= s_ep.xlim(6,2)
        phiz_oob(k-1,i) = 1;  % k-1, damit Vektor nicht mit 3T3R befüllt wird
        phiz_oob_state = 1;
      else
        if phiz_ep(Stats_IK_ep.iter+1,k,i) <= s_ep.xlim(6,1)*0.8 || phiz_ep(Stats_IK_ep.iter+1,k,i) >= s_ep.xlim(6,2)*0.8
          phiz_oob_thresh(k-1,i) = 1;
          phiz_oob_thresh_state = 1;
        end 
      end
    end
  end
  
  % Auswertung 1 - Teil 2: Endergebnis aus der IK für Optimierungsmethode liegt NICHT innerhalb von xlim
  if phiz_oob_state == 1
    for k_it = 1:amount_optcrit-1
      if  phiz_oob(k_it,i) == 1
        phiz_oob_vector(phiz_oob_vector_counter) = i; % aktuellen Punkt anhängen
        phiz_oob_vector_counter = phiz_oob_vector_counter + 1;
        break;  % sobald eine Stelle gefunden wurde, kann ausgebrochen werden
        % es wird ohnehin ein Plot für alle Opt.Krit eines Punktes erstellt
      end
    end
    phiz_oob_state = 0; % zurücksetzen, damit es für andere Punkte erkannt werden kann
  end
  % Auswertung 2: Endergebnis aus der IK für Optimierungsmethode liegt NICHT innerhalb von xlim*0.8
  if phiz_oob_thresh_state == 1
    for k_it = 1:amount_optcrit-1
      if  phiz_oob_thresh(k_it,i) == 1
        phiz_oob_thresh_vector(phiz_oob_thresh_vector_counter) = i;
        phiz_oob_thresh_vector_counter = phiz_oob_thresh_vector_counter + 1;
        break;
      end
    end
    phiz_oob_thresh_state = 0; % zurücksetzen, damit es für andere Punkte erkannt werden kann
  end
  % Vektor für Warnungen zusammenstellen
  if warnplot_needed_state == 1
    warnplot_needed_vector(warnplot_needed_vector_counter) = i; % aktuellen Punkt anhängen
    warnplot_needed_vector_counter = warnplot_needed_vector_counter + 1;
    warnplot_needed_state = 0;  % zurücksetzen
  end
  fprintf('IK für Punkt %d erfolgreich berechnet\n', i);
end

% Ausgabe der Statistik
fprintf('\nInverse Kinematik für Einzelpunkte abgeschlossen!\n');
if warning_phi_count ~= 0
  error('Fehlerhafte IK-Berechnung von %d Einzelpunkten! Siehe Konsole für betroffene Punkte und prüfe Ursache.', warning_phi_count);
else
  fprintf('\nStatistik: \n');
  fprintf('Warnungen bezüglich erreichter max. Iteration bei den Punkten: ');
  if isnan(warnplot_needed_vector)
    warnplot_needed_vector = 0;
  end
  for nnn = 1:size(warnplot_needed_vector,2)
    fprintf(' %d', warnplot_needed_vector(nnn));
  end
  fprintf('\n');
  
  if ~isnan(phiz_oob_vector(1))
    warning('Fehlerhafte IK mit limred (letztes phiz liegt außerhalb von xlim = [%d %d] bei den Punkten: ', s_ep.xlim(6,1)*180/pi, s_ep.xlim(6,2)*180/pi);
    for nnn = 1:size(phiz_oob_vector,2)
      fprintf(' %d', phiz_oob_vector(nnn));
    end
    fprintf('\n');
  else
    fprintf('KEINE fehlerhafte IK mit limred -> letztes phiz aller Punkte liegt innerhalb von xlim = [%d %d])', s_ep.xlim(6,1)*180/pi, s_ep.xlim(6,2)*180/pi);
    fprintf('\n');
  end
  
  if ~isnan(phiz_oob_thresh_vector(1))
    fprintf('Letztes phiz zwischen Schwellwert und xlim bei den Punkten: ');
    for nnn = 1:size(phiz_oob_thresh_vector,2)
      fprintf(' %d', phiz_oob_thresh_vector(nnn));
    end
    fprintf('\n\n');
  else
    fprintf('KEIN phiz liegt zwischen Schwellwert und xlim.');
    fprintf('\n\n');
  end
  
end

%% Einzelpunkt-IK | Plot Sets
plot_task = 2;  % 1:Warnungen, 2: Auswertung1(außerhalb xlim), 3: Auswertung2(zwischen Schwellwert und xlim), 4:selbstgewählte Punkte
% STANDARD = 2

switch plot_task
  case 1
    plot_vector = warnplot_needed_vector;
    fprintf('Starte Plot für Warnungen! Standard ist Plot der Auswertung1, bitte notfalls ändern!\n');
    noplot = 0;
  case 2
    if isnan(phiz_oob_vector(1))
      fprintf('Kein Plot für Einzelpunkte nötig, da die IK mit limred erfolgreich berechnet wurde!\n');
      fprintf('Bei Bedarf können Warnungen oder bestimmte Punkte geplottet werden. Siehe Code.\n');
      noplot = 1;
    else
      plot_vector = phiz_oob_vector;
      fprintf('Starte Standard-Plot für fehlerhafte IK mit limred (phiz außerhalb von xlim)!\n');
      noplot = 0;
    end
  case 3
    plot_vector = phiz_oob_thresh_vector;
    fprintf('Starte Plot für Punkte, bei welchen das letzte phiz zwischen Schwellwert und xlim liegt. Standard ist Plot der Auswertung1, bitte notfalls ändern!\n');
    noplot = 0;
  case 4
    plot_vector = [1:size(XL,1)];
%     plot_vector = [1];
    fprintf('Starte Plot für selbstgewählte Punkte! Standard ist Plot der Auswertung1, bitte notfalls ändern!\n');
    noplot = 0;
end

if noplot ~= 1
  for ii = 1:size(plot_vector,2)
    pkt = plot_vector(1,ii);
    f3 = change_current_figure(100+ii);clf;
    ylable_first_plot_incolumn = ones(4,1);   % 4 = Anzahl der Zeilen
    for kk = 1:amount_optcrit
      index_phiz = 1:(iter_ep(pkt,kk)+1);
      
      % Subplots Erste Zeile: Verlauf von phiz -------------------------------------
      subplot(4,4,kk); hold on;
      plot(index_phiz, phiz_ep(1:index_phiz(end),kk,pkt)*180/pi, 'm', 'LineWidth',1);
      xlim_vek = repmat([s_ep.xlim(6,1) s_ep.xlim(6,2)]*180/pi,index_phiz(end),1);
      plot(index_phiz, xlim_vek(:,1), 'r--', 'LineWidth',1);
      plot(index_phiz, xlim_vek(:,2), 'r--', 'LineWidth',1);
      plot(index_phiz, xlim_vek(:,1)*0.8, 'b--', 'LineWidth',1);
      plot(index_phiz, xlim_vek(:,2)*0.8, 'b--', 'LineWidth',1);
      axe = gca;
      axe.XLim = [index_phiz(1) index_phiz(end)];
      if ylable_first_plot_incolumn(1) == true
        ylabel('phi_z in °');
        ylable_first_plot_incolumn(1) = false;
      end
      xlabel('Iteration lfd. Nr.'); grid on;
      switch kk
        case 1
          title('OHNE limred: wn=[0 0 0 0 0 0 0], aber mit wn(2) = 1');
        case 2
          title(sprintf('Optimierungsmethode 1: wn=[0 0 0 0 0 %d %d]', opt1(1), opt1(2)));
        case 3
          title(sprintf('Optimierungsmethode 2: wn=[0 0 0 0 0 %d %d]', opt2(1), opt2(2)));
        case 4
          title(sprintf('Optimierungsmethode 3: wn=[0 0 0 0 0 %d %d]', opt3(1), opt3(2)));
      end
      
      % Subplots Zweite Zeile: Verlauf von h(7) -------------------------------------
      if s_ep_wn(7,kk) ~= 0
        subplot(4,4,amount_optcrit+kk); hold on;
        plot(index_phiz, h7_ep(1:index_phiz(end),kk,pkt), 'm', 'LineWidth', 1');
        axe = gca;
        axe.XLim = [index_phiz(1) index_phiz(end)];
        if ylable_first_plot_incolumn(2) == true
          ylabel('Optimierungskriterium h(7)');
          ylable_first_plot_incolumn(2) = false;
        end
        xlabel('Iteration lfd. Nr.'); grid on;
      end
      
      % Subplots Dritte Zeile: Verlauf von h(8) -------------------------------------
      if s_ep_wn(8,kk) ~= 0
        subplot(4,4,amount_optcrit*2+kk); hold on;
        plot(index_phiz, h8_ep(1:index_phiz(end),kk,pkt), 'm', 'LineWidth', 1');
        % wenn h(6) = inf, dann grünen Asterisk plotten
        for nn = 1:index_phiz(end)
          if isinf(h8_ep(nn,kk,pkt))
            plot(nn, 0,'*g', 'DisplayName','h(8) = inf');
          end
        end
        hold on;
        grid on;
        axe = gca;
        axe.XLim = [index_phiz(1) index_phiz(end)];
        if ylable_first_plot_incolumn(3) == true
          ylabel('Optimierungskriterium h(8)');
          ylable_first_plot_incolumn(3) = false;
        end
        xlabel('Iteration lfd. Nr.'); grid on;
      end
      
      % Subplots Vierte Zeile: Verlauf von limred_crit_active -------------------------------------
      if s_ep_wn(7,kk) ~= 0 || s_ep_wn(8,kk) ~= 0
        subplot(4,4,amount_optcrit*3+kk); hold on;
        for nn = 1:index_phiz(end)
          if limred_crit_active(nn,kk,pkt) == 1
            plot(nn, 1,'om', 'DisplayName','limred aktiv');
          elseif limred_crit_active(nn,kk,pkt) == 0
            plot(nn, 0,'or', 'DisplayName','limred inaktiv');
          end
        end
        hold on;
        axe = gca;
        axe.XLim = [index_phiz(1) index_phiz(end)];
        axe.YLim = [-1 2];
        ylabel('Zustand limred: 0/1'); grid on;
        xlabel('Iteration lfd. Nr.');
      end
      
%       % Subplots Fünfte Zeile: Verlauf der Gelenkwinkel -------------------------------------
%       for kkk = 1:RS.NQJ
%         subplot(5,18,amount_optcrit*RS.NQJ*4+RS.NQJ*(kk-1)+kkk);hold on;
%         plot(index_phiz, Q_ep(1:index_phiz(end),kkk,pkt,kk)/RS.qunitmult_eng_sci(kkk));
%         plot([0;index_phiz(end)], RS.qlim(kkk,1)*[1;1]/RS.qunitmult_eng_sci(kkk), 'r--');
%         plot([0;index_phiz(end)], RS.qlim(kkk,2)*[1;1]/RS.qunitmult_eng_sci(kkk), 'r--');
%         xlabel('lfd. Nr.');
%         ylabel(sprintf('q_%d / %s', k, RS.qunit_eng{kkk}));
%         grid on;
%       end
    end
    txt = ['Punkt Nr. ',num2str(pkt)];
    sgtitle(txt);
    
    filename = ['Verlaufsplot_' int2str(ii) '.png'];
    saveas(f3,filename)
  end
end

%% Einzelpunkt-IK | Plot Gelenkwinkel

wanted_pkt = 7;     % eintragen, welcher Punkt untersucht werden soll
wanted_method = 4;  % eintragen, welche Optimierungsmethode untersucht werden soll

index_phiz = 1:(iter_ep(wanted_pkt,wanted_method)+1);
leg_count = 1;
joint_count = 0;
for kkk = 1:RP.NJ
  joint_count = joint_count + 1;
  if joint_count == 7
    joint_count = 1;
    leg_count = leg_count + 1;
  end
  subplot(RP.NLEG,RS.NQJ,kkk);hold on;
  plot(index_phiz, Q_ep(1:index_phiz(end),kkk,wanted_pkt,wanted_method)/RP.Leg(leg_count).qunitmult_eng_sci(joint_count));
  plot([0;index_phiz(end)], RP.Leg(leg_count).qlim(joint_count,1)*[1;1]/RP.Leg(leg_count).qunitmult_eng_sci(joint_count), 'r--');
  plot([0;index_phiz(end)], RP.Leg(leg_count).qlim(joint_count,2)*[1;1]/RP.Leg(leg_count).qunitmult_eng_sci(joint_count), 'r--');
  ylabel(sprintf('q_{%d}', kkk));
  if any(kkk == [31 32 33 34 35 36])
    xlabel('lfd. Nr. Iter');
  end
  grid on;
end
txt = ['Gelenkwinkelverläufe für Punkt ', num2str(wanted_pkt), ' und Opt.Methode ', num2str(wanted_method)];
sgtitle(txt);


%% Trajektorien-IK | Initialisierung und Berechnung
fprintf('\n\nStarte Berechnungen für gesamte Trajektorie\n');
T = t;
RP.update_EE_FG(I_EE_3T3R, I_EE_3T2R);
Phirt_tol = 1e-12;
n_max = 2500;
Namen_Methoden = cell(1,6);
% Namen_Methoden = cell(1,1);
% Initialisierung
phiz_traj_diff_k = NaN(size(T,1),size(Namen_Methoden,2));
phiz_traj_diff_k_pre = phiz_traj_diff_k;
h9_k = NaN(size(T,1),size(Namen_Methoden,2));
h10_k = h9_k;
h11_k = h9_k;
phizD_traj =  NaN(size(t,1),1);
Q_k_ges   = NaN(size(t,1),RP.NJ,size(Namen_Methoden,2));
QD_k_ges  = Q_k_ges;
QDD_k_ges = Q_k_ges;
limred_fail_flag_traj = NaN(1,1);  % ohne 3T2R-Referenz
limred_fail_counter = 1;
traj_plot_needed = 0;
wn_limred_save = NaN(5,length(Namen_Methoden)); % erstes Element bezieht sich auf Größe wn(13:17)

for k = 6:length(Namen_Methoden)
  s_traj_wn = zeros(RP.idx_ik_length.wntraj,1); % wn(:)=0 -> keine Opt.Krit
  s_traj    = struct('n_min', 50, 'n_max', n_max, 'Phit_tol', Phirt_tol, 'Phir_tol', Phirt_tol, ...
    'reci', true, 'wn', s_traj_wn, 'enforce_qlim', false);
  s_traj.xlim   = [NaN(5,2); [-45 45]*pi/180]; % in [rad] übergeben
  s_traj.xDlim  = [NaN(5,2); [-0.21 0.21]];    % 0.21rad/s = 2rpm laut unitconversion.io/de/rpm-zu-rads-konvertierung
  s_qhyp = 0; % Schalter/Wert für Abstand von Gelenkwinkelgrenzen (Hyp.)
  s_traj.wn(RP.idx_iktraj_wnP.qDlim_par) = 0.5;
  switch k
    % Zum Debuggen kann vollständiges Kriterium an Anfang gestellt und Rest
    % auskommentiert werden -> Schleifen auf 1:1 und Namen_Methoden = cell(1,1);
%     case 1 % vollständiges Optimierungskriterium immer ans Ende stellen
%       name_method = sprintf('3T2R-IK mit wn(13:17)=1');
%       s_traj.wn(RP.idx_iktraj_wnP.qlim_hyp) = s_qhyp;    % Abstand von Gelenkwinkelgrenzen (Hyp.) aktiv
%       s_traj.wn(13:17) = 1;
    case 1
      name_method = sprintf('3T2R-IK mit wn(:)=0');
    case 2
      name_method = sprintf('3T2R-IK mit wn(2)=1');
      s_traj.wn(RP.idx_iktraj_wnP.qlim_hyp) = s_qhyp;     % Abstand von Gelenkwinkelgrenzen (Hyp.) aktiv
    case 3     
      name_method = sprintf('3T2R-IK mit wn(15:16)=1');
      s_traj.wn(RP.idx_iktraj_wnP.qlim_hyp) = s_qhyp;     % Abstand von Gelenkwinkelgrenzen (Hyp.) aktiv
      % quadr. für qD und qDD
      s_traj.wn(RP.idx_iktraj_wnP.xlim_par) = 1;
      s_traj.wn(RP.idx_iktraj_wnD.xlim_par) = 1;
    case 4
      name_method = sprintf('3T2R-IK mit wn(17:18)=1');
      s_traj.wn(RP.idx_iktraj_wnP.qlim_hyp) = s_qhyp;    % Abstand von Gelenkwinkelgrenzen (Hyp.) aktiv
       % hyp. für qD und qDD
      s_traj.wn(RP.idx_iktraj_wnP.xlim_hyp) = 1;
      s_traj.wn(RP.idx_iktraj_wnD.xlim_hyp) = 1;
    case 5
      name_method = sprintf('3T2R-IK mit wn(19)=1');
      s_traj.wn(RP.idx_iktraj_wnP.qlim_hyp) = s_qhyp;    % Abstand von Gelenkwinkelgrenzen (Hyp.) aktiv
      s_traj.wn(RP.idx_iktraj_wnP.xDlim_par) = 1;        % quadr. für qDD
    case 6 % vollständiges Optimierungskriterium immer ans Ende stellen
      name_method = sprintf('3T2R-IK mit wn(15:19)=1');
%       s_traj.wn(RP.idx_iktraj_wnP.qlim_hyp) = s_qhyp;    % Abstand von Gelenkwinkelgrenzen (Hyp.) aktiv
      s_traj.wn(RP.idx_iktraj_wnP.xlim_par) = 1; % P-Regler quadratisch
      s_traj.wn(RP.idx_iktraj_wnD.xlim_par) = 0.5; % D-Regler quadratisch
      s_traj.wn(RP.idx_iktraj_wnP.xlim_hyp) = 0.5; % P-Regler hyperbolisch
      s_traj.wn(RP.idx_iktraj_wnD.xlim_hyp) = 0.05; % D-Regler hyperbolisch
      s_traj.wn(RP.idx_iktraj_wnP.xDlim_par) = 0.1; % Dämpfung xlim quadratisch
  end
  wn_limred_save(:,k) = s_traj.wn(15:19);
    
  % IK berechnen
  [q0_traj, phi_IK_ep] = RP.invkin3(XL(i,:)', q1, s_ep);
  [Q_k, QD_k, QDD_k, Phi_k, ~, ~, ~, Stats_Traj_k] = RP.invkin_traj(X_t,XD_t,XDD_t,t,q0_traj,s_traj);
  Q_k_ges(:,:,k) = Q_k;
  QD_k_ges(:,:,k) = QD_k;
  QDD_k_ges(:,:,k) = QDD_k;
  fprintf('\nBerechnung der Trajektorie mit "%s" beendet.\n', name_method);
  
  if max(abs(Phi_k(:))) > max(s_traj.Phit_tol,s_traj.Phir_tol)
   warning('Fehler in Trajektorie zu groß. IK nicht berechenbar');
  end
  
  % Testweise Aufruf der kompilierten Funktion
  [Q_k2, QD_k2, QDD_k2, Phi_k2, ~, ~, ~, Stats_Traj_k2] = RP.invkin2_traj( ...
    X_t,XD_t,XDD_t,t,q0_traj,s_traj);
%   test_q_traj = Q_k2-Q_k;
%   assert(all(abs(normalizeAngle(test_q_traj(:),0)) < 1e-2), ['Ergebnis von ', ...
%     'invkin_traj und invkin2_traj stimmt nicht überein']);


  % Actual platform trajectory
  [X_ist_k, XD_ist_k, ~] = RP.fkineEE2_traj(Q_k, QD_k, QDD_k);
  % Save original vector
  X_ist_k_pre = X_ist_k;
  % Get platform pose from integration to avoid restriction to +/- pi
  X_ist_k_int = repmat(X_ist_k(1,:), length(t), 1) + cumtrapz(t, XD_ist_k);
  % Normalize platform angles from direct calculation using angles from
  % integration as center. Gives exact solution without limitation to +/-pi
  X_ist_k(:,4:6) = normalizeAngle(X_ist_k(:,4:6), X_ist_k_int(:,4:6));
    
%   I_err = abs(Phi_k) > max(s_traj.Phit_tol, s_traj.Phir_tol) | isnan(Phi_k);
%   % Prüfe, ob die Trajektorie vorzeitig abbricht. Das ist kein Fehler, da
%   % bei ungünstiger Parametrierung des IK-Algorithmus keine Lösung
%   % gefunden werden kann. Bei guter Parametrierung ist dann eine Lösung
%   % möglich.
%   n_iO = n; % Anzahl der i.O. Trajektorienpunkte
%   if any(I_err(:))
%     I1 = find(sum(I_err,2),1);
%     n_iO = I1-1; % Bis einen Punkt vor dem Fehler war es noch i.O.
%     warning(['Fehler in Trajektorie zu groß. Zuerst bei Zeitschritt %d/%d ', ...
%       '(t=%1.3fs). Traj.-IK nicht vollständig berechenbar'], I1, size(Q_k,1), t(I1));
%   end

  % Auswertung für den Plot: phiz
  phiz_traj_diff_k(:,k)     = -X_t(:,6) + X_ist_k(:,6);
  phiz_traj_diff_k_pre(:,k) = -X_t(:,6) + X_ist_k_pre(:,6);
  if k > 1
    if phiz_traj_diff_k(size(t,1),k) <= s_traj.xlim(6,1) || phiz_traj_diff_k(size(t,1),k) >= s_traj.xlim(6,2)
      limred_fail_flag_traj(1,limred_fail_counter) = k;
      limred_fail_counter = limred_fail_counter + 1;
      traj_plot_needed = 1;
    end
  end
  % Auswertung für den Plot: h(9) bis h(11)
  h9_k(:,k) = Stats_Traj_k.h(:,10);
  h10_k(:,k) = Stats_Traj_k.h(:,11);
  h11_k(:,k) = Stats_Traj_k.h(:,12);
  % Auswertung für den Plot: phizD
  phizD_traj(:,k) = Stats_Traj_k.phi_zD(:,1);
end
fprintf('\nBerechnung und Auswertung der Trajektorien-IK beendet.\n');

if ~isnan(limred_fail_flag_traj(1,1))
  warning('Bei der Trajektorienberechnung befindet sich der Endpunkt bei einer Optimierungsmethode NICHT innerhalb der Grenzen xlim:');
  fprintf('Optimierungsmethoden: ');
  for nnn = 1:size(limred_fail_flag_traj,2)
    fprintf('%d ', limred_fail_flag_traj(1,nnn));
  end
  if any(limred_fail_flag_traj == length(Namen_Methoden))  % length(Namen_Methoden)=vollständiges Optimierungskriterium
    warning('\nOptimierungskriterium %d ist fehlerhaft. Unbedingt Plot zu Gelenkwinkeln überprüfen!', length(Namen_Methoden));
    warning('Nachfolgend der Plot der Trajektorie bzgl. allen Optimierungsmethoden.');
  else
    fprintf('\n\nNachfolgend der Plot zur Trajektorie mit allen Optimierungsmethoden.\n');
    fprintf('Sollte die vollständige Optimierung mit allen Kriterien erfolgreich sein, kann das Ergebnis dennoch als positiv bewertet werden!\n');
  end
else
  fprintf('\n\nKein Plot zur Trajektorie nötig, da alle Optimierungskriterien die Endpunkte innerhalb der Grenzen positioniert haben.\n');
end

%% Trajektorien-IK | Plot
T = t;
if traj_plot_needed
  f4 = change_current_figure(300+1);clf;
  sgtitle('Verlauf von phiz, h(8:10) und phizD');
  index_traj = 1:size(T,1);
  ylable_first_plot_incolumn = ones(5,1);   % 5 = Anzahl der Zeilen
  
  for kk = 1:size(Namen_Methoden,2)
    % Subplots Erste Zeile: Verlauf von phiz -------------------------------------
    subplot(5,size(Namen_Methoden,2),kk); hold on;
    plot(T, phiz_traj_diff_k(:,kk)*180/pi, 'm', 'LineWidth',1);
%     plot(T, phiz_traj_diff_k(:,kk), 'm', 'LineWidth',1);
%     plot(T, phiz_traj_diff_k_pre(:,kk)*180/pi, 'k--', 'LineWidth',1);
    xlim_vek = repmat([s_traj.xlim(6,1) s_traj.xlim(6,2)]*180/pi,index_traj(end),1);
    plot(T, xlim_vek(:,1), 'r--', 'LineWidth',1);
    plot(T, xlim_vek(:,2), 'r--', 'LineWidth',1);
    xlim_vek_thresh = repmat([s_traj.xlim(6,1) s_traj.xlim(6,2)]*180/pi*0.8,index_traj(end),1);
    plot(T, xlim_vek_thresh(:,1), 'b--', 'LineWidth',1);
    plot(T, xlim_vek_thresh(:,2), 'b--', 'LineWidth',1);
    axe = gca;
    axe.XLim = [0 T(end)];
    switch kk
      case 1
        title(sprintf('Optimierungsmethode 1: wn(:)=0'));
      case 2
        title(sprintf('Optimierungsmethode 2: wn(2)=1'));
      case 3
        title(sprintf('Optimierungsmethode 3: wn=[%d %d %d %d %d]', wn_limred_save(1,kk), wn_limred_save(2,kk), wn_limred_save(3,kk), wn_limred_save(4,kk), wn_limred_save(5,kk)));
      case 4
        title(sprintf('Optimierungsmethode 4: wn=[%d %d %d %d %d]', wn_limred_save(1,kk), wn_limred_save(2,kk), wn_limred_save(3,kk), wn_limred_save(4,kk), wn_limred_save(5,kk)));
      case 5
        title(sprintf('Optimierungsmethode 5: wn=[%d %d %d %d %d]', wn_limred_save(1,kk), wn_limred_save(2,kk), wn_limred_save(3,kk), wn_limred_save(4,kk), wn_limred_save(5,kk)));
      case 6
        title(sprintf('Optimierungsmethode 5: wn=[%d %d %d %d %d]', wn_limred_save(1,kk), wn_limred_save(2,kk), wn_limred_save(3,kk), wn_limred_save(4,kk), wn_limred_save(5,kk)));
    end
    if ylable_first_plot_incolumn(1) == true
      ylabel('Verlauf von phiz');
      ylable_first_plot_incolumn(1) = false;
    end
    xlabel('Zeit in s'); grid on;
    
    % Subplots Zweite Zeile: Verlauf von h(9) -------------------------------------
    if wn_limred_save(1,kk) ~= 0 || wn_limred_save(2,kk) ~= 0
      subplot(5,size(Namen_Methoden,2),size(Namen_Methoden,2)+kk); hold on;
      plot(T, h9_k(:,kk), 'm', 'LineWidth',1);
      axe = gca;
      axe.XLim = [0 T(end)];
      hold on;
      if ylable_first_plot_incolumn(2) == true
        ylabel('Optim.krit h(9)');
        ylable_first_plot_incolumn(2) = false;
      end
      xlabel('Zeit in s'); grid on;
    end
    
    % Subplots Dritte Zeile: Verlauf von h(10) -------------------------------------
    if wn_limred_save(3,kk) ~= 0 || wn_limred_save(4,kk) ~= 0
      subplot(5,size(Namen_Methoden,2),size(Namen_Methoden,2)*2+kk); hold on;
      plot(T, h10_k(:,kk), 'm', 'LineWidth',1);
      for nn = 1:index_traj(end)
        if isinf(h10_k(nn,kk))
          plot(T(nn), 0,'*g', 'LineWidth',1);
        end
      end
      hold on; grid on;
      axe = gca;
      axe.XLim = [0 T(end)];
      xlabel('Zeit in s');
      if ylable_first_plot_incolumn(3) == true
        ylabel('Optim.krit h(10)');
        ylable_first_plot_incolumn(3) = false;
      end
    end
    
    % Subplots Vierte Zeile: Verlauf von h(11) -------------------------------------
    if wn_limred_save(5,kk) ~= 0
      subplot(5,size(Namen_Methoden,2),size(Namen_Methoden,2)*3+kk); hold on;
      plot(T, h11_k(:,kk), 'm', 'LineWidth',1);
      axe = gca;
      axe.XLim = [0 T(end)];
      hold on;
      if ylable_first_plot_incolumn(4) == true
        ylabel('Optim.krit h(11)');
        ylable_first_plot_incolumn(4) = false;
      end
      xlabel('Zeit in s'); grid on;
    end
    
    % Subplots Fünfte Zeile: Verlauf von phizD -------------------------------------
    subplot(5,size(Namen_Methoden,2),size(Namen_Methoden,2)*4+kk); hold on;
    plot(T, phizD_traj(:,kk), 'm', 'LineWidth',1);
    axe = gca;
    axe.XLim = [0 T(end)];
    hold on;
    if ylable_first_plot_incolumn(5) == true
      ylabel('Geschwindigkeit von phi_z in °/s');
      ylable_first_plot_incolumn(5) = false;
    end
    xlabel('Zeit in s'); grid on;
  end
  linkxaxes
  fprintf('\nPlot für Trajektorie beendet\n');
end
 
