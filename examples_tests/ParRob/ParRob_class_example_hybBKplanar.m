% Teste planare PKM mit seriell-hybriden Beinketten

% André Brünger, Masterarbeit bei Moritz Schappler, 2019-04
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-12
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

clear
clc
rob_path = fileparts(which('robotics_toolbox_path_init.m'));
respath = fullfile(rob_path, 'examples_tests', 'results');
%% Init
if isempty(which('hybroblib_path_init.m'))
  warning('Repo mit seriell-hybriden Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end

%% Klasse für Beinkette erstellen
% Instanz der Roboterklasse für die Beinkette erstellen
RS = hybroblib_create_robot_class('hybBKplanar', '', 'hybBKplanarBsp1');
% Längen gegenüber gespeicherten Parametern verkleinern (KS dann besser
% sichtbar)
RS.update_mdh(RS.pkin/4);

RS.fill_fcn_handles(true, true);
% Mex-Dateien immer neu erstellen (noch kein Update-Mechanismus)
hybroblib_create_template_functions({'hybBKplanar', ''}, false, true)
% RS.mex_dep()

RS.I_EE = logical([1 1 0 0 0 1]); % Für IK der Beinketten mit invkin_ser
%% Klasse für PKM erstellen
RP = ParRob('P2FB2');
RP.create_symmetric_robot(2, RS, 1, 0.5);
RP.Leg(1).phi_W_0 = [0;pi;0]; % Drehen, damit PKM symmetrisch ist
RP.Leg(1).update_base();
RP.Leg(1).update_EE([0;0;0], [pi;0;-pi]); % Basis-Drehung ausgleichen
RP.Leg(2).phi_W_0 = [0;0;0];
RP.Leg(2).update_base();
RP.Leg(2).update_EE([0;0;0], [0;0;0]);

RP.update_actuation(logical([1 1 0 0 1 0]'));% Index der aktiven (und damit unabhängigen Gelenke)
RP.update_EE_FG(logical([1 1 0 0 0 1])); % Für IK der PKM: planare Bewegung

% Basis und EE-KS anders definieren, um diese Eigenschaften zu berücksichtigen
RP.update_EE(  [0;0;0],   [0;0;0]*pi/180);
RP.fill_fcn_handles(true, true);
%% Alle Funktionen einmal ausführen
xE = rand(6,1);
q0 = [180; 90; 0; 180; 90; 0]*pi/180;

RP.fkine_legs(q0);
RP.fkine_platform(xE); 
RP.fkine(q0, xE);

RP.constr1(q0, xE);
RP.constr1_trans(q0, xE);
RP.constr1_rot(q0, xE);
RP.constr1grad_q(q0, xE);
RP.constr1grad_x(q0, xE);

%% Startpose bestimmen
% Mittelstellung im Arbeitsraum
X = [ [0.0;1;0]; [0;0;0]*pi/180 ];

try
  [q, Phis] = RP.invkin_ser(X, q0, struct('retry_limit',1));
  if any(abs(Phis) > 1e-6)
    error('Inverse Kinematik (für jedes Bein einzeln) konnte in Startpose nicht berechnet werden');
  end
catch
  q = [180; 90 ; 0; 180; 90 ; 0]*pi/180;
end
%% Zwangsbedingungen in Startpose testen
Phi1=RP.constr1(q, X);
Phit1=RP.constr1_trans(q, X);
Phir1=RP.constr1_rot(q, X);
if any(abs(Phi1) > 1e-6)
  warning('ZB in Startpose ungleich Null');
end

%% Jacobi-Matrizen auswerten
[G_q, Gv_q]  = RP.constr1grad_q(q, X);
[G_x, Gv_x] = RP.constr1grad_x(q, X);
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
  RP.mdlname, rank(G_dx), sum(RP.I_EE)+sum(RP.I_qd));
fprintf('%s: Rang der Jacobi der aktiven Gelenke: %d/%d\n', ...
  RP.mdlname, rank(G_a), sum(RP.I_EE));

%% Beispieltrajektorie definieren
% Polygon-Trajektorie
XL = [X'+[0.25*[ 0.0, 0.0, 0], [zeros(1,2), 0.0]]; ... % Start in Null-Lage
      X'+[0.25*[ 1.0, 0.0, 0], [zeros(1,2), 0.0]]; ...
      X'+[0.25*[ 0.0, -1.0, 0], [zeros(1,2), 0.0]]; ...
      X'+[0.25*[ -0.5,-0.5, 0], [zeros(1,2), -0.5]]];
XL = [XL; XL(1,:)]; % Rückfahrt zurück zum Startpunkt.
[X_t,XD_t,XDD_t,t] = traj_trapez2_multipoint(XL, 1, 0.1, 0.01, 1e-3, 1e-1);

%% Roboter in Startpose mit Beispieltrajektorie plotten
figure(1);clf;
hold on;grid on;
xlabel('x in m');ylabel('y in m');zlabel('z in m');
view(3);
% Alle (Körper-)KS des Roboters einzeichnen
s_plot = struct('ks_legs', 1:RP.I2L_LEG(end), 'straight', 0);
RP.plot( q, X, s_plot );
view([0 90])
xlabel('x in m');ylabel('y in m');zlabel('z in m');

%% Beispieltrajektorie berechnen (inverse Kinematik)
fprintf('Inverse Kinematik für Trajektorie berechnen: %d Bahnpunkte\n', length(t));
q0 = q; % Lösung der IK von oben als Startwert
t0 = tic();
% IK-Einstellungen: Sehr lockere Toleranzen, damit es schneller geht
s = struct('Phit_tol', 1e-3, 'Phir_tol', 1*pi/180, 'retry_limit', 1);
% Startpunkt prüfen, Anfangswert verbessern
[q1, Phi_num1] = RP.invkin1(X_t(1,:)', q0, s);
if any(abs(Phi_num1) > 1e-2)
  warning('IK konvergiert nicht');
end
[Q_t, ~, ~, Phi_t] = RP.invkin_traj(X_t, XD_t, XDD_t, t, q1, s);
if any(any(abs(Phi_t(:,RP.I_constr_t_red)) > s.Phit_tol)) || ...
   any(any(abs(Phi_t(:,RP.I_constr_r_red)) > s.Phir_tol))
   warning('Fehler in Trajektorie zu groß. IK nicht berechenbar');
  I=any((abs(Phi_t) > 1e-2)')';
  II = find(I);
  I1 = II(1);
  [q_test, Phi_test] = RP.invkin_ser(X_t(I1,:)', q0, s);
end
fprintf('%1.1fs nach Start. %d Punkte berechnet.\n', toc(t0), length(t));
save(fullfile(respath, 'ParRob_class_example_hybBKplanar_traj.mat'));

%% Zeitverlauf der Trajektorie plotten
figure(4);clf;
subplot(3,2,sprc2no(3,2,1,1)); hold on;
plot(t, X_t(:,RP.I_EE));set(gca, 'ColorOrderIndex', 1)
plot([0;t(end)],[X(RP.I_EE)';X(RP.I_EE)'],'o--')
legend({'$x$', '$y$', '$\varphi$','Anfangswerte'}, 'interpreter', 'latex')
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
subplot(3,2,sprc2no(3,2,2,2)); hold on;
plot(t, Phi_t(:,[1,2,4,5]));
plot(t([1 end]), s.Phit_tol*[1;1], 'r--');
plot(t([1 end]),-s.Phit_tol*[1;1], 'r--');
grid on;
ylabel('\Phi_{trans}');
subplot(3,2,sprc2no(3,2,3,2)); hold on;
plot(t, Phi_t(:,[3,6]));
plot(t([1 end]), s.Phir_tol*[1;1], 'r--');
plot(t([1 end]),-s.Phir_tol*[1;1], 'r--');
grid on;
ylabel('\Phi_{rot}');
linkxaxes
%% Bild zu Zeitschritt der Bewegung erstellen
% Damit sind übereinanderliegene KS teilweise besser sichtbar
figure(5);clf;
hold on;grid on;
xlabel('x in m');ylabel('y in m');zlabel('z in m');
view(3);
s_plot = struct('ks_legs', 1:RP.I2L_LEG(end), 'straight', 0);
RP.plot( Q_t(200,:)', X_t(200,:)', s_plot );
view([0 90])
xlabel('x in m');ylabel('y in m');zlabel('z in m');

%% Animation des bewegten Roboters
s_anim = struct( 'gif_name', fullfile(respath, 'ParRob_class_example_hybBKplanar.gif'));
s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I2L_LEG], 'straight', 0);
fhdl=figure(6);clf;hold all;
set(fhdl,'units','normalized','outerposition',[0 0 1 1]); % Vollbild
view([0,90]);
axis auto
hold on;grid on;
xlabel('x in m');ylabel('y in m');zlabel('z in m');
plot3(X_t(:,1), X_t(:,2), X_t(:,3), 'r--');
RP.anim( Q_t(1:20:size(Q_t,1),:), X_t(1:20:size(X_t,1),:), s_anim, s_plot);
fprintf('Animation der Bewegung gespeichert: %s\n', fullfile(respath, 'ParRob_class_example_hybBKplanar.gif'));
fprintf('Test für hybBKplanar beendet\n');