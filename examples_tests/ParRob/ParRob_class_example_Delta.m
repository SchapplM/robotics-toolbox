% Beispiel-Skript für Delta-Roboter
% * Erstellung des Modells
% * Betrachtung der Freiheitsgrade anhand der PKM-Jacobi-Matrix
% * Inverse Kinematik für kartesische Trajektorie
% 
% TODO: Bei der IK klappt eine Achse manchmal um und die Bewegung sieht
% komisch aus. Das liegt an der Normalisierung des Winkels in
% Zwischenschritten der IK

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-03
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc

rob_path = fileparts(which('robotics_toolbox_path_init.m'));
resdir = fullfile(rob_path, 'examples_tests', 'results');
if isempty(which('serroblib_path_init.m'))
  warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end

%% Allgemeine Einstellungen für Roboter-Klassen
% FG für Gesamt-PKM: Alle FG sind belegt, da Rotations-FG auf Null gehalten
% werden müssen (in der inversen Kinematik) und nicht "frei" sind
% Der EE-Marker kann trotzdem wie folgt gesetzt werden:
I_EE = logical([1 1 1 0 0 0]);
% FG für die Beinketten: Position und Orientierung der Koppel-KS muss
% vollständig berechnet werden, damit die IK gelöst wird
I_EE_Leg = logical([1 1 1 1 1 1]);
%% Initialisierung der Klasse für die serielle Beinkette
% Funktionen für Beinkette initialisieren
% Typ des seriellen Roboters auswählen (S5RRPRR2 = RUU = Delta-Bein)
SName='S5RRRRR2';

% Instanz der Roboterklasse erstellen
RS = serroblib_create_robot_class(SName);
RS.fill_fcn_handles(true, true); % Benutze kompilierte Funktionen
% RS.mex_dep();

% Parameter setzen
RS.I_EE = I_EE_Leg;
% Beinlängen setzen
RS.update_mdh([0.3; 0.5]);

RS.qlim = repmat([-2*pi, 2*pi], RS.NQJ, 1);
%% Klasse für PKM erstellen
RP = ParRob('Delta');
RP.create_symmetric_robot(3, RS, 0.2, 0.15);
RP.initialize();

X0 = [0.1;0.05;-0.7; pi; 0; 0];
% EE-FG setzen: Technische EE-FG der PKM, Aufgaben-EE-FG, FG aller
% Beinketten (muss nochmal gesetzt werden)
RP.update_EE_FG(I_EE, I_EE, repmat(logical(I_EE_Leg),3,1));

% Basis-Orientierung der Beinketten nachbearbeiten: So, dass die
% Basis-Koppelgelenke tangential auf Kreis liegen
for i = 1:3
  phi_W_0_neu = RP.Leg(i).phi_W_0 - [0;0;pi/2];
  R_tmp = eulxyz2r(phi_W_0_neu) * rotx(-pi/2);
  RP.Leg(i).update_base([], r2eulxyz(R_tmp));
end

% Endeffektor der Beinketten drehen, damit Orientierung mit Koppel-KS
% übereinstimmen kann
for i = 1:3
  phi_z = -RP.Leg(i).phi_W_0(2);
  RP.Leg(i).update_EE(zeros(3,1), [pi/2;0;phi_z])
  RP.phi_P_B_all(:,i) = [0;0;0];% Bei Delta-Roboter muss phi_P_B_all alle null setzen.
end

% Endeffektor weiter nach unten, damit besser sichtbar. z-Achse nach unten gedreht
RP.update_EE([0;0;-0.2], [pi;0;0]);

%% Zwangsbedingungen in Startpose testen
% Anfangs-Pose so, dass Bein bereits im Arbeitsraum mit richtiger
% Orientierung liegt.
q0_leg = [30; 90; 0; 0; -60]*pi/180;
q0 = repmat(q0_leg, 3, 1);

RP.constr1(q0, X0)
RP.constr1_trans(q0, X0)
RP.constr1_rot(q0, X0)
RP.constr1grad_q(q0, X0)
RP.constr1grad_x(q0, X0)

% IK mit zwei Methoden testen. Sehr genau berechnen, damit die Prüfung
% rotatorischer FG im nächsten Abschnitt auch sehr genau ist.
[q, Phi_test1] = RP.invkin_ser(X0, q0, struct('Phit_tol', 1e-13, 'Phir_tol', 1e-13));
% Orientierung korrekt einstellen
[q_test, Phi_test2] = RP.invkin1(X0, q0);

Phi1=RP.constr1(q, X0);
if any(abs(Phi1) > 1e-6) || any(isnan(Phi1))
  error('ZB in Startpose ungleich Null');
  q = q0;
end

%% Teste Jacobi-Matrizen
Phi1=RP.constr1(q, X0);
[~,G_q]  = RP.constr1grad_q(q, X0);
[~,G_x] = RP.constr1grad_x(q, X0);
G_q(abs(G_q)<1e-10) = 0;
G_x(abs(G_x)<1e-10) = 0;
% Aufteilung der Ableitung nach den Gelenken in Gelenkklassen 
% * aktiv/unabhängig (a),
% * passiv+schnitt/abhängig (d)
G_a = G_q(:,RP.I_qa);
G_d = G_q(:,RP.I_qd);
% Jacobi-Matrix zur Berechnung der abhängigen Gelenke und EE-Koordinaten
G_dx = [G_d, G_x];

% Gebe Rang der Einzel-Matrizen aus
fprintf('%s: Rang der vollständigen Jacobi der inversen Kinematik: %d/%d\n', ...
  RP.mdlname, rank(G_q), RP.NJ);
fprintf('%s: Rang der vollständigen Jacobi der direkten Kinematik: %d/%d\n', ...
  RP.mdlname, rank(G_dx), sum(RP.I_EE)+sum(RP.I_qd));
fprintf('%s: Rang der Jacobi der aktiven Gelenke: %d/%d\n', ...
  RP.mdlname, rank(G_a), sum(RP.I_EE));

% Berechne PKM-Jacobi
G_dx_inv = pinv(G_dx);
J_voll = G_dx \ G_a;
fprintf('%s: Rang der PKM-Jacobi (aktiv -> passiv+EE): %d/%d\n', ...
  RP.mdlname, rank(J_voll), 6);

J_d = J_voll(1:sum(RP.I_qd),:);
J_x = J_voll(sum(RP.I_qd)+1:end,:);

% Teste EE-FG mit Beispiel-Geschwindigkeit der Antriebe. Berechne daraus
% die Plattform-Geschwindigkeiten. Nicht belegte FG dürfen keine Geschw.
% haben.
qaD = 100*rand(3,1);
qdDxD = G_dx \ G_a * qaD;
x_test = qdDxD(sum(RP.I_qd)+1:end);
if any(abs(x_test(~RP.I_EE)) > 1e-10) % Genauigkeit hier ist abhängig von Zwangsbed.
  error('Rotationskoordinaten werden laut Jacobi-Matrix  durch die Antriebe bewegt')
end



%% Trajektorie bestimmen
% Start in Grundstellung
k=1; XE = X0';
% Fahrt in Startstellung
k=k+1; XE(k,:) = XE(k-1,:) + [ -0.1,0.1,0.15, 0,0,0];
% Beginn Würfel
d1=0.1;
h1=0.1;
k=k+1; XE(k,:) = XE(k-1,:) + [ d1,0,0, 0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,-d1,0  0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [-d1,0,0, 0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0,-h1, 0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0, h1, 0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,d1,0,  0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0,-h1, 0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [ d1,0,0, 0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,-d1,0  0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0, h1, 0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0,-h1, 0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [-d1,0,0, 0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,d1,0,  0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0, h1, 0,0,0];
[X,XD,XDD,T] = traj_trapez2_multipoint(XE, 1, 1e-1, 1e-2, 1e-3, 0.25);

%% Roboter in Startpose mit Beispieltrajektorie plotten
figure(1);clf;
hold on;grid on;
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
view(3);
s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], ...
                 'ks_platform', RP.NLEG+[1,2], ...
                 'straight', 0);
RP.plot( q, X0, s_plot );
plot3(X(:,1), X(:,2), X(:,3), 'r--');

%% Gelenkkräfte und inverse Kinematik berechnen
% Benutze feine Toleranz für Traj.-IK. Ansonsten werden durch debug=true
% Fehlermeldungen aufgrund numerischer Fehler ausgelöst.
iksettings = struct('n_max', 100, 'Phit_tol', 1e-9, 'Phir_tol', 1e-9, 'debug', true);
[Q, QD, QDD, Phi] = RP.invkin_traj(X,XD,XDD,T,q0,iksettings);
for i = 1:length(T)
  if max(abs( Phi(i,:) )) > 1e-4
    warning('IK stimmt nicht. Wahrscheinliche Ursache: Ist innerhalb von n_max nicht konvergiert');
    return
  end
end

%% Zeitverlauf der Trajektorie plotten
figure(4);clf;
subplot(3,2,sprc2no(3,2,1,1)); hold on;
plot(T, X(:,RP.I_EE));set(gca, 'ColorOrderIndex', 1)
% plot([0;T(end)],[X(RP.I_EE)';X(RP.I_EE)'],'o--')
% legend({'$x$', '$y$', '$\varphi$'}, 'interpreter', 'latex')
grid on;
ylabel('x_E');
subplot(3,2,sprc2no(3,2,2,1));
plot(T, XD(:,RP.I_EE));
grid on;
ylabel('xD_E');
subplot(3,2,sprc2no(3,2,3,1));
plot(T, XDD(:,RP.I_EE));
grid on;
ylabel('xDD_E');
subplot(3,2,sprc2no(3,2,1,2));
plot(T, Q);
grid on;
ylabel('Q');
subplot(3,2,sprc2no(3,2,2,2)); hold on;
plot(T, Phi(:,RP.I_constr_t_red));
plot(T([1 end]), iksettings.Phit_tol*[1;1], 'r--');
plot(T([1 end]),-iksettings.Phit_tol*[1;1], 'r--');
grid on;
ylabel('\Phi_{trans}');
subplot(3,2,sprc2no(3,2,3,2)); hold on;
plot(T, Phi(:,RP.I_constr_r_red));
plot(T([1 end]), iksettings.Phir_tol*[1;1], 'r--');
plot(T([1 end]),-iksettings.Phir_tol*[1;1], 'r--');
grid on;
ylabel('\Phi_{rot}');

%% Animation
mkdirs(resdir);
s_anim = struct( 'mp4_name', fullfile(resdir, 'delta_box_traj.mp4'));
figure(9);clf;
set(9,'units','normalized','outerposition',[0 0 1 1]); % Vollbild
hold on;
plot3(X(:,1), X(:,2), X(:,3));
grid on;
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
view(3);
title('Animation der kartesischen Trajektorie');
RP.anim( Q(1:20:length(T),:), X(1:20:length(T),:), s_anim, s_plot);