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
RP = RP.create_symmetric_robot(6, RS, 0.5, 0.2);
% Erste Achse zusätzlich drehen: Die verwendete serielle Kette dreht um die
% z-Achse, die automatische Anordnung der seriellen Ketten in
% create_symmetric_robot dreht die Basis-Koppel-KS auch um die z-Achse.
% Die Reihenfolge dieser Drehungen muss daher vertauscht werden
for i = 1:6
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
I_qa = false(36,1);
I_qa(3:6:36) = true;
RP.update_actuation(I_qa);

%% Startpose bestimmen
% Mittelstellung im Arbeitsraum
X = [ [0.0;0.0;0.5]; [0;0;0]*pi/180 ];
q0 = rand(36,1); % Startwerte für numerische IK
q0(I_qa) = 0; % mit Schubaktor bei Null anfangen (damit Konfiguration nicht umklappt)

% Inverse Kinematik auf zwei Arten berechnen
[~, Phi] = RP.invkin1(X, q0);
if any(abs(Phi) > 1e-8)
  error('Inverse Kinematik konnte in Startpose nicht berechnet werden');
end

[qs, Phis] = RP.invkin_ser(X, rand(36,1));
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

%% Roboter in Startpose plotten
figure(1);clf;
hold on;grid on;
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
view(3);
s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
RP.plot( q, X, s_plot );


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
q0 = q; % Lösung der IK von oben als Startwert
t0 = tic();
% IK-Einstellungen: Sehr lockere Toleranzen, damit es schneller geht
s = struct('Phit_tol', 1e-3, 'Phir_tol', 1*pi/180);
[q1, Phi_num1] = RP.invkin1(X_t(1,:)', q0, s);
if any(abs(Phi_num1) > 1e-2)
  warning('IK konvergiert nicht');
end
fprintf('Inverse Kinematik für Trajektorie berechnen: %d Bahnpunkte\n', length(t));
[Q_t, ~, ~, Phi_t] = RP.invkin_traj(X_t, XD_t, XDD_t, t, q1, s);
fprintf('%1.1fs nach Start. %d Punkte berechnet.\n', ...
  toc(t0), length(t));
save(fullfile(respath, 'ParRob_class_example_6UPS_traj.mat'));

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
plot(t, Phi_t(:,sort([1:6:36, 2:6:36, 3:6:36])));
plot(t([1 end]), s.Phit_tol*[1;1], 'r--');
plot(t([1 end]),-s.Phit_tol*[1;1], 'r--');
grid on;
ylabel('\Phi_{trans}');
subplot(3,2,sprc2no(3,2,3,2)); hold on;
plot(t, Phi_t(:,sort([4:6:36, 5:6:36, 6:6:36])));
plot(t([1 end]), s.Phir_tol*[1;1], 'r--');
plot(t([1 end]),-s.Phir_tol*[1;1], 'r--');
grid on;
ylabel('\Phi_{rot}');

%% Animation des bewegten Roboters
s_anim = struct( 'gif_name', fullfile(respath, 'ParRob_class_example_6UPS.gif'));
s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
figure(5);clf;hold all;
view(3);
axis auto
hold on;grid on;
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
RP.anim( Q_t(1:20:end,:), X_t(1:20:end,:), s_anim, s_plot);
fprintf('Animation der Bewegung gespeichert: %s\n', fullfile(respath, 'ParRob_class_example_6UPS.gif'));
fprintf('Test für 6UPS beendet\n');