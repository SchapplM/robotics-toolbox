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

%% Definitionen, Benutzereingaben
% Robotermodell entweder aus PKM-Bibliothek oder nur aus
% Seriell-Roboter-Bibliothek laden. Stellt keinen Unterschied dar.
use_parrob = false;

rob_path = fileparts(which('robotics_toolbox_path_init.m'));
respath = fullfile(rob_path, 'examples_tests', 'results');

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
  RS.fill_fcn_handles(true, true);
  % RS.mex_dep(true)
  RP = ParRob('P6RRPRRR14V3A1');
  RP = RP.create_symmetric_robot(6, RS, 0.5, 0.2);
  RP.initialize();
  % Schubgelenke sind aktuiert
  I_qa = false(36,1);
  I_qa(3:6:36) = true;
  RP.update_actuation(I_qa);
  % Benutze PKM-Bibliothek für gespeicherte Funktionen
  if ~isempty(which('parroblib_path_init.m'))
    parroblib_addtopath({'P6RRPRRR14V3A1'});
  end
  RP.fill_fcn_handles();
end
%% Alternativ: Klasse für PKM erstellen (basierend auf PKM-Bibliothek)
if use_parrob
  if isempty(which('parroblib_path_init.m'))
    warning('Repo mit parallelen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
    return
  end
  RP = parroblib_create_robot_class('P6RRPRRR14V3A1', 0.5, 0.2);
end

%% Startpose bestimmen
% Mittelstellung im Arbeitsraum
X = [ [0.15;0.05;0.5]; [10;-10;5]*pi/180 ];
q0 = rand(36,1); % Startwerte für numerische IK
q0(RP.I_qa) = 0.2; % mit Schubaktor größer Null anfangen (damit Konfiguration nicht umklappt)

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

G_q = RP.constr1grad_q(q, X);
G_x = RP.constr1grad_x(q, X);

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

% Inverse Jacobi-Matrix aus symbolischer Berechnung (mit Funktion aus HybrDyn)
if ~isempty(which('parroblib_path_init.m'))
  Jinv_sym = RP.jacobi_qa_x(q, X);
  Jinv_num_voll = -inv(G_q) * G_x;
  Jinv_num = Jinv_num_voll(RP.I_qa,:);
  test_Jinv = Jinv_sym - Jinv_num;
  if max(abs(test_Jinv(:))) > 1e-10
    error('Inverse Jacobi-Matrix stimmt nicht zwischen numerischer und symbolischer Berechnung überein');
  else
    fprintf('Die inverse Jacobi-Matrix stimmt zwischen symbolischer und numerischer Berechnung überein\n');
  end
end
%% Beispieltrajektorie berechnen und zeichnen
X0 = [ [0;0;0.5]; [0;0;0]*pi/180 ];
% Trajektorie mit beliebigen Bewegungen der Plattform
XL = [X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.3]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.3, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [0.3, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.2,-0.1, 0.3], [0.3, 0.2, 0.1]]; ...
      X0'+1*[[-0.1, 0.2,-0.1], [0.5,-0.2,-0.2]]; ...
      X0'+1*[[ 0.2, 0.3, 0.2], [0.2, 0.1, 0.3]]];
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
if any(any(abs(Phi_t(:,RP.I_constr_t_red)) > s.Phit_tol)) || ...
   any(any(abs(Phi_t(:,RP.I_constr_r_red)) > s.Phir_tol))
   error('Fehler in Trajektorie zu groß. IK nicht berechenbar');
end
fprintf('%1.1fs nach Start. %d Punkte berechnet.\n', ...
  toc(t0), length(t));
save(fullfile(respath, 'ParRob_class_example_6UPS_traj.mat'));

%% Zeitverlauf der Trajektorie plotten
figure(4);clf;
subplot(3,2,sprc2no(3,2,1,1)); hold on;
plot(t, X_t);set(gca, 'ColorOrderIndex', 1)
plot([0;t(end)],[X0';X0'],'o--')
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