% Roboterklasse für 3RPR-PKM testen
% 
% Ablauf:
% * Beispiel-Parameter und Roboter definieren
% * Beispiel zu Jacobi-Matrizen
% * Beispieltrajektorie kartesisch: Inverse Kinematik und Visualisierung
% 
% Beispielsystem: Basis-Kreis 1.0, Plattform-Kreis 0.3 (Radius)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-12
% (C) Institut für mechatronische Systeme, Universität Hannover

clear
clc
rob_path = fileparts(which('robotics_toolbox_path_init.m'));
respath = fullfile(rob_path, 'examples_tests', 'results');
warning on;
%% Init
if isempty(which('serroblib_path_init.m'))
  warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end

% Typ des seriellen Roboters auswählen (Dreh+Schub+Dreh)
SName='S3RPR1';

% Instanz der Roboterklasse erstellen
RS = serroblib_create_robot_class(SName);
RS.fill_fcn_handles(true);
RS.mex_dep()

% Parameter setzen
[beta_mdh, b_mdh, alpha_mdh, a_mdh, theta_mdh, d_mdh, qoffset_mdh] = ...
  S3RPR1_pkin2mdhparam(zeros(length(RS.pkin),1));
d_mdh(:) = 0;
a_mdh(2) = 0;
a_mdh(3) = 0;
pkin = S3RPR1_mdhparam2pkin(beta_mdh, b_mdh, alpha_mdh, a_mdh, theta_mdh, d_mdh, qoffset_mdh);
RS.update_mdh(pkin);

%% Klasse für PKM erstellen
RS.I_EE = logical([1 1 0 0 0 1]); % Für IK der Beinketten mit invkin_ser
if ~isempty(which('parroblib_path_init.m'))
  parroblib_addtopath({'P3RPR1G1P1A2'});
end
RP = ParRob('P3RPR1G1P1A2');
RP.create_symmetric_robot(3, RS, 1, 0.3);
RP.initialize();
RP.update_EE_FG(logical([1 1 0 0 0 1])); % Für IK der PKM
% Schubgelenke sind aktuiert
I_qa = false(RP.NJ,1);
I_qa(2:3:RP.NJ) = true;
RP.update_actuation(I_qa);

RP.fill_fcn_handles(true, true); % mex-Funktionen für PKM benutzen. Bei Bedarf erstellen
%% Startpose bestimmen
% Mittelstellung im Arbeitsraum
X = [ [0.2;0.1;0.0]; [0;0;-20]*pi/180 ];

% Inverse Kinematik berechnen
[~, Phi] = RP.invkin1(X, rand(RP.NJ,1));
if any(abs(Phi) > 1e-7)
  error('Inverse Kinematik (für Gesamt-PKM) konnte in Startpose nicht berechnet werden');
end

[q, Phi] = RP.invkin_ser(X, rand(RP.NJ,1));
if any(abs(Phi) > 1e-6)
  warning('Inverse Kinematik (für jedes Bein einzeln) konnte in Startpose nicht berechnet werden');
end

%% Zwangsbedingungen in Startpose testen
Phi1=RP.constr1(q, X);
Phit1=RP.constr1_trans(q, X);
Phir1=RP.constr1_rot(q, X);
if any(abs(Phi1) > 1e-6)
  warning('ZB in Startpose ungleich Null');
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
  RP.mdlname, rank(G_q), RP.NJ);
fprintf('%s: Rang der vollständigen Jacobi der direkten Kinematik: %d/%d\n', ...
  RP.mdlname, rank(G_dx), sum(RP.I_EE)+sum(RP.I_qd));
fprintf('%s: Rang der Jacobi der aktiven Gelenke: %d/%d\n', ...
  RP.mdlname, rank(G_a), sum(RP.I_EE));

Jinv_num_voll = -inv(G_q) * G_x;
Jinv_num = Jinv_num_voll(RP.I_qa,:);
fprintf('%s: Rang der inversen PKM-Jacobi: %d/%d (Kondition %1.1e)\n', ...
  RP.mdlname, rank(Jinv_num, 1e-6), sum(RP.I_qa), cond(Jinv_num));

% Inverse Jacobi-Matrix aus symbolischer Berechnung (mit Funktion aus HybrDyn)
if ~isempty(which('parroblib_path_init.m'))
  Jinv_sym = RP.jacobi_qa_x(q, X);
  test_Jinv = Jinv_sym - Jinv_num;
  if max(abs(test_Jinv(:))) > 1e-10
    error('Inverse Jacobi-Matrix stimmt nicht zwischen numerischer und symbolischer Berechnung überein');
  else
    fprintf('Die inverse Jacobi-Matrix stimmt zwischen symbolischer und numerischer Berechnung überein\n');
  end
end

%% Beispieltrajektorie definieren

% Dreieck-Trajektorie
XL = [X'+1*[ 0.0, 0.0, zeros(1,3), 0.0]; ... % Start in Null-Lage
      X'+1*[ 0.2, 0.0, zeros(1,3), 0.4]; ...
      X'+1*[ 0.3, 0.2, zeros(1,3),-0.4]; ...
      X'+1*[ 0.2,-0.2, zeros(1,3), 0.3]];
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
fprintf('Inverse Kinematik für Trajektorie berechnen: %d Bahnpunkte\n', length(t));
% Inverse Kinematik berechnen
q0 = q; % Lösung der IK von oben als Startwert

t0 = tic();
% IK-Einstellungen: Feine Toleranz. Geht mit kompilierter Funktion schnell.
s = struct('Phit_tol', 1e-7, 'Phir_tol', 1e-6);
[q1, Phi_num1] = RP.invkin1(X_t(1,:)', q0, s);
if any(abs(Phi_num1) > 1e-2)
  warning('IK konvergiert nicht');
end
[Q_t, ~, ~, Phi_t] = RP.invkin2_traj(X_t, XD_t, XDD_t, t, q1, s);
if any(any(abs(Phi_t(:,RP.I_constr_t_red)) > s.Phit_tol)) || ...
   any(any(abs(Phi_t(:,RP.I_constr_r_red)) > s.Phir_tol))
   error('Fehler in Trajektorie zu groß. IK nicht berechenbar');
end
fprintf('%1.1fs nach Start. %d Punkte berechnet.\n', ...
  toc(t0), length(t));

save(fullfile(respath, 'ParRob_class_example_3RPR_traj.mat'));

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
subplot(3,2,sprc2no(3,2,2,2)); hold on;
plot(t, Phi_t(:,[1,2,4,5,7,8]));
plot(t([1 end]), s.Phit_tol*[1;1], 'r--');
plot(t([1 end]),-s.Phit_tol*[1;1], 'r--');
grid on;
ylabel('\Phi_{trans}');
subplot(3,2,sprc2no(3,2,3,2)); hold on;
plot(t, Phi_t(:,[3,6,9]));
plot(t([1 end]), s.Phir_tol*[1;1], 'r--');
plot(t([1 end]),-s.Phir_tol*[1;1], 'r--');
grid on;
ylabel('\Phi_{rot}');

%% Animation des bewegten Roboters
s_anim = struct( 'gif_name', fullfile(respath, 'ParRob_class_example_3RPR.gif'));
s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
figure(5);clf;hold all;
view(3);
axis auto
hold on;grid on;
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
plot3(X_t(:,1), X_t(:,2), X_t(:,3), 'r--');
RP.anim( Q_t(1:20:end,:), X_t(1:20:end,:), s_anim, s_plot);
fprintf('Animation der Bewegung gespeichert: %s\n', fullfile(respath, 'ParRob_class_example_3RPR.gif'));
fprintf('Test für 3RPR beendet\n');