% Roboterklasse für 6UPS-PKM testen
% 
% Ablauf:
% * Beispiel-Parameter und Roboter definieren
% * Beispiel zu Jacobi-Matrizen
% * Beispieltrajektorie kartesisch: Inverse Kinematik und Visualisierung
% 
% Beispielsystem: Basis-Kreis 0.5, Plattform-Kreis 0.2

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-12
% (C) Institut für mechatronische Systeme, Universität Hannover

clear
clc
rob_path = fileparts(which('robotics_toolbox_path_init.m'));
respath = fullfile(rob_path, 'examples_tests', 'results');
%% Init
if isempty(which('serroblib_path_init.m'))
  warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end

% Typ des seriellen Roboters auswählen (RRPRRR = UPS)
SName='S6RRPRRR14';

% Instanz der Roboterklasse erstellen
RS = serroblib_create_robot_class(SName);
RS.fill_fcn_handles(true);
RS.mex_dep()

% Parameter setzen
[beta_mdh, b_mdh, alpha_mdh, a_mdh, theta_mdh, d_mdh, qoffset_mdh] = ...
  S6RRPRRR14_pkin2mdhparam(zeros(length(RS.pkin),1));
d_mdh(4) = 0.2;
alpha_mdh(2) =  pi/2;
alpha_mdh(3) = -pi/2;
pkin = S6RRPRRR14_mdhparam2pkin(beta_mdh, b_mdh, alpha_mdh, a_mdh, theta_mdh, d_mdh, qoffset_mdh);
RS.update_mdh(pkin);

%% Klasse für PKM erstellen

RP = ParRob('P6RRPRRR14');
RP = RP.create_symmetric_robot(5, RS, 0.5, 0.2);
% Erste Achse zusätzlich drehen: Die verwendete serielle Kette dreht um die
% z-Achse, die automatische Anordnung der seriellen Ketten in
% create_symmetric_robot dreht die Basis-Koppel-KS auch um die z-Achse.
% Die Reihenfolge dieser Drehungen muss daher vertauscht werden
for i = 1:5
  phi_W_0i = RP.Leg(i).phi_W_0;
  phi_W_0i(1) = -pi/2;
  phi_W_0i(2) = -phi_W_0i(3); % Durch rotx(-pi/2) wird y zu -z
  phi_W_0i(3) = 0; % Keine Drehung mehr um mitgedrehte z-Achse
  % Basis-KS der seriellen Beinkette aktualisieren. Nehme ZYX-Euler-Winkel
  % (Nr. 11)
  RP.Leg(i).update_base([], phi_W_0i, uint8(11));
end
RP = RP.initialize();

% Schubgelenke sind aktuiert
I_qa = false(30,1);
I_qa(3:6:30) = true;
RP.update_actuation(I_qa);

%% Startpose bestimmen
% Mittelstellung im Arbeitsraum
X = [ [0.0;0.0;0.5]; [0;0;0]*pi/180 ];
q0 = rand(30,1); % Startwerte für numerische IK
q0(I_qa) = 0; % mit Schubaktor bei Null anfangen (damit Konfiguration nicht umklappt)

% Inverse Kinematik auf zwei Arten berechnen
[~, Phi] = RP.invkin3(X, q0);
if any(abs(Phi) > 1e-8)
  error('Inverse Kinematik konnte in Startpose nicht berechnet werden');
end

[qs, Phis] = RP.invkin_ser(X, rand(30,1));
q=qs;
if any(abs(Phis) > 1e-6)
  error('Inverse Kinematik (für jedes Bein einzeln) konnte in Startpose nicht berechnet werden');
end

%% Zwangsbedingungen in Startpose testen
Phi1=RP.constr3(q, X);
Phit1=RP.constr3_trans(q, X);
Phir1=RP.constr3_rot(q, X);
if any(abs(Phi1) > 1e-6)
  error('ZB in Startpose ungleich Null');
end

%% Roboter in Startpose plotten
figure(1);clf;
hold on;grid on;
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
view(3);
s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
RP.plot( q, X, s_plot );


%% Jacobi-Matrizen auswerten

[G_q,Gv_q]  = RP.constr3grad_q(q, X);
[G_x, Gv_x] = RP.constr3grad_x(q, X);

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
  RP.mdlname, rank(G_q), RP.NJ);
fprintf('%s: Rang der vollständigen Jacobi der direkten Kinematik: %d/%d\n', ...
  RP.mdlname, rank(G_dx), sum(RP.I_EE)+sum(RP.I_qd));
fprintf('%s: Rang der Jacobi der aktiven Gelenke: %d/%d\n', ...
  RP.mdlname, rank(G_a), sum(RP.I_EE));

% return
%% Beispieltrajektorie berechnen und zeichnen

% Dreieck-Trajektorie
XL = [X'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
      X'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.3]]; ...
      X'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
      X'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.3, 0.0]]; ...
      X'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
      X'+1*[[ 0.0, 0.0, 0.0], [0.3, 0.0, 0.0]]; ...
      X'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
      X'+1*[[ 0.2,-0.1, 0.3], [0.3, 0.2, 0.1]]; ...
      X'+1*[[-0.1, 0.2,-0.1], [0.5,-0.2,-0.2]]; ...
      X'+1*[[ 0.2, 0.3, 0.2], [0.2, 0.1, 0.3]]];
XL = [XL; XL(1,:)]; % Rückfahrt zurück zum Startpunkt.
[X_t,XD_t,XDD_t,t] = traj_trapez2_multipoint(XL, 1, 0.1, 0.01, 1e-3, 1e-1);
% Inverse Kinematik berechnen
Phi_t = NaN(length(t), 30);
Q_t = NaN(length(t), 30);
q0 = q; % Lösung der IK von oben als Startwert
t0 = tic();t1=t0;
% IK-Einstellungen: Sehr lockere Toleranzen, damit es schneller geht
s = struct('Phit_tol', 1e-3, 'Phir_tol', 1*pi/180);

fprintf('Inverse Kinematik für Trajektorie berechnen: %d Bahnpunkte\n', length(t));
% Start der Berechnung
for i = 1:length(t)
  xE_soll = X_t(i,:)';
  % Erster IK-Schritt aus differentieller Kinematik
  if i > 1
    Phi_q = RP.constr3grad_q(q0, xE_soll);
    Phi_x = RP.constr3grad_x(q0, xE_soll);
    delta_x = XD_t(i,:)'*(t(i)-t(i-1));
    delta_q = -Phi_q \ Phi_x * delta_x;
    q0k = q0+delta_q;
  else
    q0k = q0;
  end
  % IK berechnen
  [q1, Phi_num1] = RP.invkin3(xE_soll, q0k, s);
  if any(abs(Phi_num1) > 1e-2)
    warning('IK konvergiert nicht');
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
  q0 = q1;
end

save(fullfile(respath, 'ParRob_class_example_5UPS_traj.mat'));

%% Zeitverlauf der Trajektorie plotten
figure(4);clf;
subplot(3,2,sprc2no(3,2,1,1)); hold on;
plot(t, X_t);set(gca, 'ColorOrderIndex', 1)
plot([0;t(end)],[X';X'],'o--')
legend({'$x$', '$y$', '$\varphi$'}, 'interpreter', 'latex')
grid on;
ylabel('x_E');
subplot(3,2,sprc2no(3,2,2,1));
plot(t, XD_t);
grid on;
ylabel('xD_E');
subplot(3,2,sprc2no(3,2,3,1));
plot(t, XDD_t);
grid on;
ylabel('xDD_E');
subplot(3,2,sprc2no(3,2,1,2));
plot(t, Q_t);
grid on;
ylabel('Q');
subplot(3,2,sprc2no(3,2,2,2)); hold on;
plot(t, Phi_t(:,sort([1:6:30, 2:6:30, 3:6:30])));
plot(t([1 end]), s.Phit_tol*[1;1], 'r--');
plot(t([1 end]),-s.Phit_tol*[1;1], 'r--');
grid on;
ylabel('\Phi_{trans}');
subplot(3,2,sprc2no(3,2,3,2)); hold on;
plot(t, Phi_t(:,sort([4:6:30, 5:6:30, 6:6:30])));
plot(t([1 end]), s.Phir_tol*[1;1], 'r--');
plot(t([1 end]),-s.Phir_tol*[1;1], 'r--');
grid on;
ylabel('\Phi_{rot}');

figure(1);clf;
subplot(2,1,1); hold on;
plot(t, Phi_t(:,sort([1:6:30, 2:6:30, 3:6:30])));
plot(t([1 end]), s.Phit_tol*[1;1], 'r--');
plot(t([1 end]),-s.Phit_tol*[1;1], 'r--');
grid on;
ylabel('\Phi_{trans} [m]');
subplot(2,1,2); hold on;
plot(t, Phi_t(:,sort([4:6:30, 5:6:30, 6:6:30])));
plot(t([1 end]), s.Phir_tol*[1;1], 'r--');
plot(t([1 end]),-s.Phir_tol*[1;1], 'r--');
grid on;
ylabel('\Phi_{rot} [rad]');
xlabel('Zeit [s]');
set(gcf, 'Color', 'w');

%figure_save_path = fullfile(fileparts(which('matlab_tools_path_init.m')), 'examples_tests', 'plot_examples');
%mkdirs(figure_save_path);
%export_fig(fullfile(figure_save_path, 'example_out_1.pdf'));

%% Animation des bewegten Roboters
s_anim = struct( 'gif_name', fullfile(respath, 'ParRob_class_example_5UPS.gif'));
s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
figure(5);clf;hold all;
view(3);
axis auto
hold on;grid on;
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
RP.anim( Q_t(1:20:end,:), X_t(1:20:end,:), s_anim, s_plot);
fprintf('Animation der Bewegung gespeichert: %s\n', fullfile(respath, 'ParRob_class_example_5UPS.gif'));
fprintf('Test für 5UPS beendet\n');