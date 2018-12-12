% Roboterklasse für 3RRR-PKM testen
% 
% Ablauf:
% * Beispiel-Parameter und Roboter definieren
% * Beispiel zu Jacobi-Matrizen
% * Beispieltrajektorie kartesisch: Inverse Kinematik und Visualisierung
% 
% Beispielsystem: Beinlängen 0.6/0.6; Basis-Kreis 1.0, Plattform-Kreis 0.3

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-12
% (C) Institut für mechatronische Systeme, Universität Hannover

clear
clc
rob_path = fileparts(which('robotics_toolbox_path_init.m'));
respath = fullfile(rob_path, 'examples_tests', 'results');
%% Init
if isempty(which('serroblib_path_init.m'))
  warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
end

% Typ des seriellen Roboters auswählen (Drei Drehgelenke)
SName='S3RRR1';

% Instanz der Roboterklasse erstellen
RS = serroblib_create_robot_class(SName);
RS.fill_fcn_handles(true);
RS.mex_dep()

% Parameter setzen
[beta_mdh, b_mdh, alpha_mdh, a_mdh, theta_mdh, d_mdh, qoffset_mdh] = ...
  S3RRR1_pkin2mdhparam(zeros(length(RS.pkin),1));
d_mdh(:) = 0;
a_mdh(2) = 0.6;
a_mdh(3) = 0.6;
pkin = S3RRR1_mdhparam2pkin(beta_mdh, b_mdh, alpha_mdh, a_mdh, theta_mdh, d_mdh, qoffset_mdh);
RS.update_mdh(pkin);

%% Klasse für PKM erstellen
RP = ParRob('P3RRR1');
RP = RP.create_symmetric_robot(3, RS, 1, 0.3);
RP = RP.initialize();
RP.I_EE = logical([1 1 0 0 0 1]); % Für IK der PKM
RS.I_EE = logical([1 1 0 0 0 1]); % Für IK der Beinketten mit invkin_ser

% Basis und EE-KS anders definieren, um diese Eigenschaften zu berücksichtigen
RP.update_base([0.1;0.1;0], [45;30;45]*pi/180);
RP.update_EE(  [0;0;0.1],   [0;0;45 ]*pi/180);
%% Startpose bestimmen
% Mittelstellung im Arbeitsraum
X = [ [0.0;0.0;0.1]; [0;0;0]*pi/180 ];

% Inverse Kinematik berechnen
% [~, Phi] = RP.invkin1(X, q0);
% if any(abs(Phi) > 1e-10)
%   error('Inverse Kinematik konnte in Startpose nicht berechnet werden');
% end

[qs, Phis] = RP.invkin_ser(X, rand(RP.NJ,1));
q=qs;
if any(abs(Phis) > 1e-6)
  error('Inverse Kinematik (für jedes Bein einzeln) konnte in Startpose nicht berechnet werden');
end

%% Zwangsbedingungen in Startpose testen
Phi1=RP.constr1(q, X);
Phit1=RP.constr1_trans(q, X);
Phir1=RP.constr1_rot(q, X);
if any(abs(Phi1) > 1e-6)
  error('ZB in Startpose ungleich Null');
end

%% Jacobi-Matrizen auswerten

[G_q,Gv_q]  = RP.constr1grad_q(q, X);
[G_x, Gv_x] = RP.constr1grad_x(q, X);

% Aufteilung der Ableitung nach den Gelenken in Gelenkklassen 
% * aktiv/unabhängig (a),
% * passiv+schnitt/abhängig (d)
G_a = G_q(:,RP.I_qa);
G_d = G_q(:,RP.I_qd);
Gv_a = Gv_q(:,RP.I_qa);
Gv_d = Gv_q(:,RP.I_qd);  
% Jacobi-Matrix zur Berechnung der abhängigen Gelenke und EE-Koordinaten
G_dx = [G_d, G_x];
Gv_dx = [Gv_d, Gv_x];

fprintf('%s: Rang der vollständigen Jacobi der inversen Kinematik: %d/%d\n', ...
  RP.Name, rank(G_q), RP.NJ);
fprintf('%s: Rang der vollständigen Jacobi der direkten Kinematik: %d/%d\n', ...
  RP.Name, rank(G_dx), sum(RP.I_EE)+sum(RP.I_qd));
fprintf('%s: Rang der Jacobi der aktiven Gelenke: %d/%d\n', ...
  RP.Name, rank(G_a), sum(RP.I_EE));

%% Beispieltrajektorie definieren

% Dreieck-Trajektorie
XL = [X'+1*[ 0.0, 0.0, zeros(1,3), 0.0]; ... % Start in Null-Lage
      X'+1*[ 0.1, 0.0, zeros(1,3), 0.2]; ...
      X'+1*[ 0.0, 0.3, zeros(1,3),-0.2]; ...
      X'+1*[ 0.1,-0.1, zeros(1,3), 0.1]];
XL = [XL; XL(1,:)]; % Rückfahrt zurück zum Startpunkt.
[X_t,XD_t,XDD_t,t] = traj_trapez2_multipoint(XL, 1, 0.1, 0.01, 1e-3, 1e-1);

%% Roboter in Startpose mit Beispieltrajektorie plotten
figure(1);clf;
hold on;grid on;
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
view(3);
s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
RP.plot( q, X, s_plot );
plot3(X_t(:,1), X_t(:,2), X_t(:,3), 'r--');

%% Beispieltrajektorie berechnen (inverse Kinematik)

% Inverse Kinematik berechnen
Phi_t = NaN(length(t), RP.NJ);
Q_t = NaN(length(t), RP.NJ);
q0 = q; % Lösung der IK von oben als Startwert

t0 = tic();t1=t0;
% IK-Einstellungen: Sehr lockere Toleranzen, damit es schneller geht
s = struct('constr_m', 1, 'Phit_tol', 1e-6, 'Phir_tol', 1e-4);

% Start der Berechnung
for i = 1:length(t)
  xE_soll = X_t(i,:)';
  % Erster IK-Schritt aus differentieller Kinematik
  if i > 1
    Phi_q = RP.constr1grad_q(q0, xE_soll);
    Phi_x = RP.constr1grad_x(q0, xE_soll);
    delta_x = XD_t(i,:)'*(t(i)-t(i-1));
    delta_q = -Phi_q \ Phi_x * delta_x(RP.I_EE);
    q0k = q0+delta_q;
  end

  % IK berechnen
  [q1, Phi_num1] = RP.invkin_ser(xE_soll, q0, s);
  if any(abs(Phi_num1) > 2e-4)
    % Trajektorie wahrscheinlich außerhalb des Arbeitsraums
    [q2, Phi_num2] = RP.invkin1(xE_soll, q0);
    % [qa_3, qp_3, qc3, ev] = bsp3Rrr_invkin(xE_soll, pkin);
    if any(abs(Phi_num1) > 2e-4)
      warning('i=%d: IK konvergiert nicht', i);
      return
    end
  end

  % Ergebnisse speichern
  Q_t(i,:) = q1;
  Phi_t(i,:) = Phi_num1;
  % Ausgabe der Rechenzeit
  if toc(t1) > 10
    % Schätze die noch benötigte Zeit
    tr_est = toc(t0)/i*(length(t)-i-1);
    fprintf('%1.1fs nach Start. %1.1f%% (%d/%d) Punkte berechnet. Restzeit: %1.0fmin\n', ...
      toc(t0), 100*i/length(t), i, length(t), tr_est/60);
    t1 = tic(); % Zeit seit letzter Meldung zurücksetzen
  end
end

save(fullfile(respath, 'ParRob_class_example_3RRR_traj.mat'));

%% Zeitverlauf der Trajektorie plotten
figure(4);clf;
subplot(3,2,sprc2no(3,2,1,1)); hold on;
plot(t, X_t(:,RP.I_EE));set(gca, 'ColorOrderIndex', 1)
plot([0;t(end)],[X(RP.I_EE)';X(RP.I_EE)'],'o--')
legend({'$x$', '$y$', '$\varphi$'}, 'interpreter', 'latex')
grid on;
ylabel('x_E');
subplot(3,2,sprc2no(3,2,2,1));
plot(t, XD_t(:,RP.I_EE));
grid on;
ylabel('xD_E');
subplot(3,2,sprc2no(3,2,3,1));
plot(t, XDD_t(:,RP.I_EE));
grid on;
ylabel('xDD_E');
subplot(3,2,sprc2no(3,2,1,2));
plot(t, Q_t);
grid on;
ylabel('Q');
subplot(3,2,sprc2no(3,2,2,2));
plot(t, Phi_t(:,1:6));
grid on;
ylabel('\Phi_{trans}');
subplot(3,2,sprc2no(3,2,3,2));
plot(t, Phi_t(:,7:9));
grid on;
ylabel('\Phi_{rot}');


%% Animation des bewegten Roboters
s_anim = struct( 'gif_name', fullfile(respath, 'ParRob_class_example_3RRR.gif'));
s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
figure(5);clf;hold all;
view(3);
axis auto
hold on;grid on;
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
plot3(X_t(:,1), X_t(:,2), X_t(:,3), 'r--');
RP.anim( Q_t(1:20:end,:), X_t(1:20:end,:), s_anim, s_plot);