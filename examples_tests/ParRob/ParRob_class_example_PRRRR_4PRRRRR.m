% Beispiel für 3T2R Parallelroboter
% PKM mit einer Beinkette mit 5 Gelenk-FG
% [Gogu2008] Seite 161 (abgewandelte Form)
% 
% Quelle:
% [Gogu2008] Gogu, Grigore:Structural Synthesis of Parallel Robots, Part 1:
% Methodology (2008), Springer

% Li, Junnan (MA bei Moritz Schappler)
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc
use_mex = true;

%% PKM initialisieren
% SerRob-Klassen für Beinketten generieren
RS1 = serroblib_create_robot_class('S5PRRRR2');%yz
RS1.fill_fcn_handles(use_mex, true);
RS2 = serroblib_create_robot_class('S6PRRRRR6');
RS2.fill_fcn_handles(use_mex, true);
serroblib_update_template_functions({RS1.mdlname, RS2.mdlname});

% ParRob-Klasse für PKM erstellen
RP = ParRob('Isoglide5_3T2R_mod');
% Einzelne Beinketten setzen (anders als in [Gogu2008])
% Dort Beinkette D auch 5FG; hier 6FG.
RP.Leg = copy(RS1);
RP.Leg(2) = copy(RS2);
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
RP.Leg(1).update_base( 3*RP.r_P_B_all(:,1), [0;-pi/2;0], phiconv_W_0);
RP.Leg(2).update_base( 3*RP.r_P_B_all(:,2), [pi/2;0;0], phiconv_W_0);
RP.Leg(3).update_base( 3*RP.r_P_B_all(:,3), [0;0;0], phiconv_W_0);
RP.Leg(4).update_base( 3*RP.r_P_B_all(:,4), [0;0;0], phiconv_W_0);
RP.Leg(5).update_base( 3*RP.r_P_B_all(:,5), [0;0;0], phiconv_W_0);
RP.align_platform_coupling(1, 0.3);
RP.initialize();

% Kinematik-Parameter der Beinketten neu belegen (ungefähr wie in
% [Gogu2008])
pkin1 = zeros(length(RP.Leg(1).pkin), 1);
pkin1(strcmp(RP.Leg(1).pkin_names,'a2')) = 0.45;
pkin1(strcmp(RP.Leg(1).pkin_names,'a3')) = 0.45;
pkin1(strcmp(RP.Leg(1).pkin_names,'a4')) = 0.45;
pkin1(strcmp(RP.Leg(1).pkin_names,'d3')) = 0.10;
pkin1(strcmp(RP.Leg(1).pkin_names,'d4')) = 0.10;
pkin1(strcmp(RP.Leg(1).pkin_names,'d5')) = 0.25;
pkin2 = zeros(length(RP.Leg(2).pkin), 1);
pkin2(strcmp(RP.Leg(2).pkin_names,'a3')) = 0.65;
pkin2(strcmp(RP.Leg(2).pkin_names,'a4')) = 0.75;
pkin2(strcmp(RP.Leg(2).pkin_names,'d2')) = 0.10;
RP.Leg(1).update_mdh(pkin1);
for i = 2:RP.NLEG
  RP.Leg(i).update_mdh(pkin2);
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
I_qa = [I_qa_Typ1,I_qa_Typ2,I_qa_Typ2,I_qa_Typ2,I_qa_Typ2];
RP.update_actuation(I_qa);
%% Prüfen, ob EE-FG der Beinkette korrekt sind
% LJN: beurteilen ob eingegebene I_EE möglich
I_EE_Max = ones(1,6);
for j = 1:1%RP.NLEG
  [T_O, T_W] = RP.Leg(j).fkine(zeros(RP.I2J_LEG(j)-RP.I1J_LEG(j)+1,1));
  T_W_O = T_W(:,:,1);
  T_W_EE = T_W(:,:,end);
  R_W_O = T_W_O(1:3,1:3);
  R_W_EE = T_W_EE(1:3,1:3);
  R_O_E = R_W_O' * R_W_EE;
  eul_OE = r2eulxyz(R_O_E');
  eul_WE = r2eulxyz(R_W_EE');
  RP.Leg(j).update_EE([], eul_WE);
  I_EE_Max = I_EE_Max & RP.Leg(j).I_EE;
end
if (I_EE & I_EE_Max) ~= I_EE
  error('unmögliche EE_FG')
end

RP.Leg(1).I_EE = I_EE;
RP.update_EE_FG(I_EE,I_EE_Task);

%% PKM testen
% Definition der Test-Pose
X_E = [ [0.1;0.2;1]; [10;5;0]*pi/180 ];

% IK für Roboter berechnen
for i = 1:10
  q0 = rand(RP.NJ,1); % Anfangswert für IK
  q0(RP.I_qa) = 0.2; % Schubachsen mit positivem Anfangswert
  [q,phi] = RP.invkin_ser(X_E, q0);
  if any(isnan(q))
    warning('IK konnte in Versuch %d nicht berechnet werden', i);
  else
    break
  end
end

% Jacobi-Matrizen berechnen
[G_q_red, G_q] = RP.constr3grad_q(q, X_E);
assert(all(size(G_q_red)==[29,29]), 'Ausgabe 1 von constr3grad_q muss 29x29 sein');
assert(all(size(G_q)==[30,29]), 'Ausgabe 2 von constr3grad_q muss 30x29 sein');
[G_x_red, G_x] = RP.constr3grad_x(q, X_E);
assert(all(size(G_x_red)==[29,5]), 'Ausgabe 1 von constr3grad_x muss 29x6 sein');
assert(all(size(G_x)==[30,6]), 'Ausgabe 2 von constr3grad_x muss 30x6 sein');
G_q(abs(G_q)<1e-12) = 0;
G_x(abs(G_x)<1e-12) = 0;
% Aufteilung der Ableitung nach den Gelenken in Gelenkklassen 
% * aktiv/unabhängig (a),
% * passiv+schnitt/abhängig (d)
G_a = G_q(:,RP.I_qa);
G_d = G_q(:,RP.I_qd);
% Jacobi-Matrix zur Berechnung der abhängigen Gelenke und EE-Koordinaten
G_dx = [G_d, G_x];
J_voll = G_dx \ G_a;

% FG test
qaD = 100*rand(sum(RP.I_qa),1);
qdDxD = J_voll * qaD;
xD_test = qdDxD(sum(RP.I_qd)+1:end);

if any(abs(xD_test(~RP.I_EE)) > 1e-4) % Genauigkeit hier ist abhängig von Zwangsbed.
  fprintf('Falsche Koordinaten werden laut Jacobi-Matrix  durch die Antriebe bewegt\n')
end

% Gebe Rang der Einzel-Matrizen aus
fprintf('%s: Rang der vollständigen Jacobi der inversen Kinematik: %d/%d\n', ...
  RP.mdlname, rank(G_q), RP.NJ);
fprintf('%s: Rang der vollständigen Jacobi der direkten Kinematik: %d/%d\n', ...
  RP.mdlname, rank(G_dx), sum(RP.I_EE)+sum(RP.I_qd));
fprintf('%s: Rang der Jacobi der aktiven Gelenke: %d/%d\n', ...
  RP.mdlname, rank(G_a), sum(RP.I_EE));
fprintf('%s: Rang der PKM-Jacobi (aktiv -> passiv+EE): %d/%d\n', ...
  RP.mdlname, rank(J_voll), 5);
%% PKM zeichnen
figure(1);clf;
hold on;grid on;
xlabel('x in m');ylabel('y in m');zlabel('z in m');
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
r1=15*pi/180;
k=k+1; XE(k,:) = XE(k-1,:) + [0,d1,0, 0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [d1,0,0  0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,-d1,0,  0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [-d1,0,0, 0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0,h1, 0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0,-h1,  0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0,0  r1,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0, 0, 0,-r1,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0,0, 0,r1,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0,0, -r1,0,0];
% XE(:,6) = NaN;
[X,XD,XDD,T] = traj_trapez2_multipoint(XE, 1, 1e-1, 1e-2, 1e-3, 0.25);

%% Roboter in Startpose mit Beispieltrajektorie plotten
figure(2);clf;
hold on;grid on;
xlabel('x in m');ylabel('y in m');zlabel('z in m');
view(3);
s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], ...
                 'ks_platform', RP.NLEG+[1,2], ...
                 'straight', 0);
RP.plot( q, X_E, s_plot );
plot3(X(:,1), X(:,2), X(:,3), 'r--');

%% Inverse Kinematik berechnen
iksettings = struct('n_max', 100, 'Phit_tol', 1e-6, 'Phir_tol', 1e-6, 'debug', false, ...
  'retry_limit', 1);
warning off
[Q, QD, QDD, Phi] = RP.invkin_traj(X,XD,XDD,T,q,iksettings);
warning on
for i = 1:length(T)
  if max(abs( Phi(i,:) )) > 1e-4 || any(isnan( Phi(i,:) ))
    warning('IK stimmt nicht. Wahrscheinliche Ursache: Ist innerhalb von n_max nicht konvergiert');
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
    T_E_Leg_j = rt2tr(R_0_Bj, r_0_Ej);
    X_ist(i,6*(j-1)+1:6*j) = RP.t2x(T_E_Leg_j);
  end
end

%% Verlauf Ist- und Soll-Koordinaten der Plattform
legtxt = {};
for i = 1:RP.NLEG
  legtxt={legtxt{:}, sprintf('xE from Leg %d', i)};
end
legtxt={legtxt{:}, 'xE desired'};
fhdl = figure(6);clf;
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
fhdl = figure(4);clf;
set(fhdl, 'Name', 'PKM_Traj_Overview', 'NumberTitle', 'off');
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
rob_path = fileparts(which('robotics_toolbox_path_init.m'));
resdir = fullfile(rob_path, 'examples_tests', 'results');
mkdirs(resdir);
s_anim = struct( 'gif_name', fullfile(resdir, 'ParRob_PRRRR_4PRRRRR.gif'));
figure(9);clf;
hold on;
plot3(X(:,1), X(:,2), X(:,3));
grid on;
xlabel('x in m');
ylabel('y in m');
zlabel('z in m');
view(3);
title('Animation der kartesischen Trajektorie');
RP.anim( Q(1:20:length(T),:), X(1:20:length(T),:), s_anim, s_plot);