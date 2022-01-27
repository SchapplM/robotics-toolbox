% Beispiel für 3T2R Parallelroboter
% PKM mit zwei Beinkette mit 5 Gelenk-FG
% [Gogu2008] Seite 161 (Isoglide-T3R2)
% 
% Quelle:
% [Gogu2008] Gogu, Grigore:Structural Synthesis of Parallel Robots, Part 1:
% Methodology (2008), Springer

% Li, Junnan (MA bei Moritz Schappler)
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc


%% PKM initialisieren
% SerRob-Klassen für Beinketten generieren
RS1 = serroblib_create_robot_class('S5PRRRR2');%yz
RS1.fill_fcn_handles(false);
RS2 = serroblib_create_robot_class('S6PRRRRR6');
RS2.fill_fcn_handles(false);
serroblib_update_template_functions({RS1.mdlname, RS2.mdlname});

% ParRob-Klasse für PKM erstellen
RP = ParRob('Isoglide5_3T2R');
% Einzelne Beinketten setzen (andere Reihenfolge als in [Gogu2008]
% (dort Kette B und D mit 5FG; A,C,E mit 6FG)
RP.Leg = copy(RS1);
RP.Leg(2) = copy(RS1);
RP.Leg(3) = copy(RS2);
RP.Leg(4) = copy(RS2);
RP.Leg(5) = copy(RS2);
RP.NLEG = 5;
% PKM-Parameter definieren
RP.align_base_coupling(1, 1.0);
RP.align_platform_coupling(1, 0.3);
RP.initialize();

%Basis KS einstellen
phiconv_W_0 = uint8(2);
RP.Leg(1).update_base( 3*RP.r_P_B_all(:,1)+[0;0.5;0], [0;-pi/2;0], phiconv_W_0);
% Die Basis-Orientierung der zweiten Beinkette muss identisch zur ersten
% sein. Ansonsten hat der Mechanismus nicht die Mobilität 3T2R.
RP.Leg(2).update_base( 3*RP.r_P_B_all(:,1)+[0;0.5;0.5], [0;-pi/2;0], phiconv_W_0);
RP.Leg(3).update_base( 3*RP.r_P_B_all(:,3), [0;0;0], phiconv_W_0);
RP.Leg(4).update_base( 3*RP.r_P_B_all(:,4), [0;0;0], phiconv_W_0);
RP.Leg(5).update_base( 3*RP.r_P_B_all(:,5), [pi/2;0;0], phiconv_W_0);
RP.align_platform_coupling(1, 0.3);
RP.initialize();

% Kinematik-Parameter der Beinketten neu belegen
pkin_all = [0,0.75,0.65,0.1,0.1,0.25,0,0,0,0,0,0,0,0;
            0,0.75,0.65,0.1,0.1,0.25,0,0,0,0,0,0,0,0;
            0,0.65,0.75,0,0,0,0,0,0.1,0,0,0,0,0;
            0,0.65,0.75,0,0,0,0,0,0.1,0,0,0,0,0;
            0,0.65,0.75,0,0,0,0,0,0.1,0,0,0,0,0]';%14x5
for i = 1:RP.NLEG
  RP.Leg(i).update_mdh(pkin_all(1:size(RP.Leg(i).pkin),i));
end

% EE-KS mit Null-Rotation vorbelegen
for i = 1:RP.NLEG
  RP.Leg(i).update_EE(zeros(3,1),zeros(3,1));
  RP.phi_P_B_all(:,i) = [0;0;0];% Bei 3T2R Roboter muss man phi_P_B_all aktuell null setzen
end

% EE-FG der PKM setzen (für inverse Kinematik)
I_EE = logical([1 1 1 1 1 0]);
I_EE_Task = logical([1 1 1 1 1 0]);

% Schubgelenke als aktuiert setzen
I_qa_Typ1 = zeros(1,5);
I_qa_Typ1(1) = 1;
I_qa_Typ2 = zeros(1,6);
I_qa_Typ2(1) = 1;
I_qa = [I_qa_Typ1,I_qa_Typ1,I_qa_Typ2,I_qa_Typ2,I_qa_Typ2];
RP.update_actuation(I_qa);

% EE-KS drehen, falls die z-Achse der Beinkette in die falsche Richtung
% zeigt
for j = 1:2 % muss nur für 5FG-Beinketten gemacht werden
  [~, T_W] = RP.Leg(j).fkine(zeros(RP.I2J_LEG(j)-RP.I1J_LEG(j)+1,1));
  T_W_EE = T_W(:,:,end);
  R_W_EE = T_W_EE(1:3,1:3);
  eul_WE = r2eul(R_W_EE',RP.phiconv_W_E);
  RP.Leg(j).update_EE([], eul_WE);
end
I_EE_Legs = [RP.Leg(1).I_EE;RP.Leg(2).I_EE;RP.Leg(3).I_EE;RP.Leg(4).I_EE;RP.Leg(5).I_EE];
RP.update_EE_FG(I_EE,I_EE_Task,I_EE_Legs);
%% PKM testen
% Definition der Test-Pose. Wähle ungleich Null, damit es keine
% Singularität ist (gibt dann Rangverlust)
X_E = [ [0.05;0.1;1]; [1;2;0]*pi/180 ];
q0 = rand(RP.NJ,1); % Anfangswert für IK
q0(RP.I_qa) = 0.2; % Schubachsen mit positivem Anfangswert

% IK für Roboter berechnen
[q,phi] = RP.invkin_ser(X_E, q0);
if any(isnan([phi;q]))
  warning('Inverse Kinematik nicht lösbar');
  q(isnan(q)) = q0(isnan(q));
end
figure(1);clf;
hold on;grid on;
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
view(3);
s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I2L_LEG], 'straight', 0);
RP.plot( q, X_E, s_plot );
return
% Entferne den Eintrag der Z-Komponente der ZB für die zweite Beinkette
% Dieser Eintrag ist nicht relevant für die Kinematik
% TODO: Erneute Anpassung der Zwangsbedingungs-Funktionen. Entfernung
% des folgenden Eintrags ist problemlos möglich
% Der Eintrag Phi2_rz ist die Zeile, die den Rang nicht erhöht
% (overconstraint?)
% RP.I_constr_red(9) = []; % TODO: Entspricht "degree of overconstraint" aus [Gogu2008], S. 163?

% Jacobi-Matrizen berechnen
[~,G_q_voll] = RP.constr3grad_q(q, X_E); % vollständige Gradientenmatrix 30x28
[~,G_x_voll] = RP.constr3grad_x(q, X_E); % vollständige Matrix 30x6
G_q_voll(abs(G_q_voll)<1e-8) = 0; % Damit das Debuggen einfacher ist
G_x_voll(abs(G_x_voll)<1e-8) = 0;
G_q = G_q_voll(RP.I_constr_red,:); % Wähle nur die relevanten ZB aus
G_x = G_x_voll(RP.I_constr_red,:); % (die auch durch Gelenke beeinflussbar sind)
G_eta = G_x_voll(RP.I_constr_red,RP.I_EE_Task); % Gradient für Aufgabenkoordinaten eta

% Aufteilung der Ableitung nach den Gelenken in Gelenkklassen 
G_a = G_q(:,RP.I_qa); % aktiv/unabhängig (a),
G_d = G_q(:,RP.I_qd); % passiv+schnitt/abhängig (d)
% Jacobi-Matrix zur Berechnung der abhängigen Gelenke und EE-Koordinaten
G_dx = [G_d, G_x];
G_deta = [G_d, G_eta]; % das gleiche mit Aufgabenkoordinaten
J_voll1 = G_dx \ G_a; % vollständige Jacobi-Matrix bezogen auf x-Koordinaten
J_voll2 = G_deta \ G_a; % ... bezogen auf eta-Koordinaten
Jinv_voll1 = G_q \ G_x; % vollständige inverse Jacobi-Matrix in x-Koord
Jinv_voll2 = G_q \ G_eta; % ... -eta-Koord.
J_qa_eta = Jinv_voll2(RP.I_qa,:); % Inverse Jacobi-Matrix des Roboters (eta-Koord)
J_eta_qa = J_voll2(sum(RP.I_qd)+1:end,:); % Jacobi-Matrix (wird mit qaD multi.)

% Prüfe inverse Jacobi-Matrix gegen nicht-invertierte
matrix_test = J_eta_qa*J_qa_eta - eye(5);
if any(abs(matrix_test(:)) > 1e-10)
  error('Jacobi-Matrix und ihre Inverse passen nicht zueinander');
end

% FG test
qaD = 100*rand(sum(RP.I_qa),1);
qdDxD = J_voll1 * qaD;
xD_test = qdDxD(sum(RP.I_qd)+1:end);
if any(abs(xD_test(~RP.I_EE)) > 1e-4) % Genauigkeit hier ist abhängig von Zwangsbed.
  fprintf('Falsche Koordinaten werden laut Jacobi-Matrix  durch die Antriebe bewegt\n')
end

% Gebe Rang der Einzel-Matrizen aus
fprintf('%s: Rang der vollständigen Jacobi der inversen Kinematik (%dx%d): %d/%d\n', ...
  RP.mdlname, size(G_q,1), size(G_q,2), rank(G_q), RP.NJ);
fprintf('%s: Rang der vollständigen Jacobi der direkten Kinematik (%dx%d): %d/%d\n', ...
  RP.mdlname, size(G_dx,1), size(G_dx,2), rank(G_dx), sum(RP.I_EE)+sum(RP.I_qd));
fprintf('%s: Rang der Jacobi der aktiven Gelenke (%dx%d): %d/%d\n', ...
  RP.mdlname, size(G_a,1), size(G_a,2), rank(G_a), sum(RP.I_EE));
fprintf('%s: Rang der PKM-Jacobi (aktiv -> EE) (%dx%d): %d/%d\n', ...
  RP.mdlname, size(J_eta_qa,1), size(J_eta_qa,2), rank(J_eta_qa), 5);

% Prüfe Rang der rotatorischen Zwangsbedingungen der ersten beiden
% Beinketten.
% TODO: Welchem Kriterium aus [Gogu2008] entspricht das
if rank(G_q_voll(10:12,:)) ~= 2 || rank(G_q_voll(7:12,:)) ~= 5
  error('Die zweite Beinkette ist nicht so angeordnet, dass sie zusammen mit der ersten Rotations-Mobilität zwei hat.');
end
%% PKM zeichnen
figure(1);clf;
hold on;grid on;
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
view(3);
s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I2L_LEG], 'straight', 0);
RP.plot( q, X_E, s_plot );

%%  Trajektorie bestimmen
% Start in Grundstellung
k=1; XE = X_E';
% Fahrt in Startstellung
% k=k+1; XE(k,:) = XE(k-1,:) + [ -0.01,0.01,0.015, 0,0,0];
% Beginn Würfel
d1=0.1;
h1=0.2;
r1=10*pi/180;
% k=k+1; XE(k,:) = XE(k-1,:) + [0,d1,0, 0,0,0];
% k=k+1; XE(k,:) = XE(k-1,:) + [d1,0,0  0,0,0];
% k=k+1; XE(k,:) = XE(k-1,:) + [0,-d1,0,  0,0,0];
% k=k+1; XE(k,:) = XE(k-1,:) + [-d1,0,0, 0,0,0];
% k=k+1; XE(k,:) = XE(k-1,:) + [0,0,h1, 0,0,0];
% k=k+1; XE(k,:) = XE(k-1,:) + [0,0,-h1,  0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0,0  r1,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0, 0, 0,-r1,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0,0, 0,r1,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0,0, -r1,0,0];
[X,XD,XDD,T] = traj_trapez2_multipoint(XE, 1, 2e-1, 1e-1, 5e-3, 0);

%% Roboter in Startpose mit Beispieltrajektorie plotten
figure(2);clf;
hold on;grid on;
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
view(3);
s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], ...
                 'ks_platform', RP.NLEG+[1,2], ...
                 'straight', 0);
RP.plot( q, X_E, s_plot );
plot3(X(:,1), X(:,2), X(:,3), 'r--');

%% Inverse Kinematik berechnen
iksettings = struct('n_max', 500, 'Phit_tol', 1e-6, 'Phir_tol', 1e-6, 'debug', true, ...
  'retry_limit', 1, 'mode_IK', 2);
warning off
[Q, QD, QDD, Phi] = RP.invkin_traj(X,XD,XDD,T,q,iksettings);
warning on
for i = 1:length(T)
  if max(abs( Phi(i,:) )) > 1e-4 || any(isnan( Phi(i,:) ))
    warning('IK stimmt nicht bei i=%d. Wahrscheinliche Ursache: Ist innerhalb von n_max nicht konvergiert', i);
    return
  end
end

%% Tatsächliche EE-Koordinaten mit vollständiger direkter Kinematik bestimmen
% Dient zur Prüfung, ob die inverse Kinematik wirklich stimmt und welchen
% Wert die freie EE-Koordinate x6 einnimmt
n = length(T);
X_ist = NaN(n,6*RP.NLEG);
i_BiKS = 1;
II_BiKS = NaN(RP.NLEG,1);
for i = 1:RP.NLEG
  II_BiKS(i) = i_BiKS+RP.Leg(i).NL+1;
  i_BiKS = II_BiKS(i);
end
for i = 1:n
  Tc_ges = RP.fkine(Q(i,:)', NaN(6,1));
  % Schnitt-KS aller Beinketten bestimmen
  T_Bi = Tc_ges(:,:,II_BiKS);
  for j = 1:RP.NLEG
    R_0_Bj = T_Bi(1:3,1:3,j);
    r_0_0_Bj = T_Bi(1:3,4,j);
    r_P_P_Bj = RP.r_P_B_all(:,j);
    r_0_Bj_P = -R_0_Bj*r_P_P_Bj;
    r_0_Ej = r_0_0_Bj + r_0_Bj_P;
    R_0_Ej = R_0_Bj * RP.T_P_E(1:3,1:3);
    if j == 1
      r_0_E_Legs = r_0_Ej;
      R_0_E_Legs = R_0_Ej;
    else
      test_pos = r_0_E_Legs-r_0_Ej; % Test, ob Koppelpunkt-Position stimmt
      test_ori = R_0_E_Legs(:)-R_0_Ej(:); % Test, ob EE-Rotation stimmt
      if any(abs(test_pos)>2e-6) || any(abs(test_ori)> 1e-3) % muss größer als IK-Toleranz sein
        error('i=%d: EE aus Beinkette %d stimmt nicht mit Beinkette 1 überein', i, j);
      end
    end
    T_E_Leg_j = rt2tr(R_0_Ej, r_0_Ej);
    X_ist(i,6*(j-1)+1:6*j) = RP.t2x(T_E_Leg_j);
  end
end

%% Verlauf Ist- und Soll-Koordinaten der Plattform
legtxt = {};
for i = 1:RP.NLEG
  legtxt={legtxt{:}, sprintf('xE from Leg %d', i)};
end
legtxt={legtxt{:}, 'xE desired'};
fhdl=figure(6);clf;
set(fhdl, 'Name', 'Test_LegFkine', 'NumberTitle', 'off');
for i = 1:6 % Alle 6 EE-Koordinaten (x)
  subplot(3,2,i); hold on
  linhdl1=plot(T, X_ist(:,i:6:end));
  set(gca, 'ColorOrderIndex', 1);
  linhdl3=plot(T, X(:,i), '--');
  if i < 4, unit = 'm';
  else, unit = 'rad';
  end
  ylabel(sprintf('x_%d in %s', i, unit));
  grid on
  if i == 5
    legend(legtxt);
  end
end
linkxaxes

%% Zeitverlauf der Trajektorie plotten
fhdl=figure(4);clf;
set(fhdl, 'Name', 'PKM_Traj_Overview', 'NumberTitle', 'off');
subplot(3,2,sprc2no(3,2,1,1)); hold on;
plot(T, X(:,RP.I_EE));set(gca, 'ColorOrderIndex', 1)
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
rob_path = fileparts(which('robotics_toolbox_path_init.m'));
resdir = fullfile(rob_path, 'examples_tests', 'results');
mkdirs(resdir);
s_anim = struct('gif_name', fullfile(resdir, 'ParRob_class_example_Gogu_3T2R_animation.gif'));
figure(9);clf;
hold on;
plot3(X(:,1), X(:,2), X(:,3));
grid on;
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
view(3);
title('Animation der kartesischen Trajektorie');
RP.anim( Q(1:20:length(T),:), X(1:20:length(T),:), s_anim, s_plot);