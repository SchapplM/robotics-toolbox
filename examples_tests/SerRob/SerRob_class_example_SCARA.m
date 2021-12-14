% Beispielskript für die Benutzung der Roboterklasse SerRob anhand eines
% SCARA-Beispielroboters mit Modellparametern von Kuka KR5 SCARA R350 Z200
% 
% Ablauf:
% * Berechne Gelenkwinkeltrajektorie
% * Zeichne und visualiere die Trajektorie
% * Berechne neue kartesische Trajektorie
% * Berechne die inverse Kinematik für die kartesische Trajektorie
% * Zeichne und visualiere die zweite Trajektorie

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-11
% (C) Institut für mechatronische Systeme, Universität Hannover

clear
clc

%% Init
if isempty(which('serroblib_path_init.m'))
  warning('Repo mit Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
end

% Typ des seriellen Roboters auswählen (SCARA)
SName='S4RRPR1';
% Modellparameter auswählen (hinterlegt aus Datenblatt)
RName='S4RRPR1_KUKA1';
% Instanz der Roboterklasse erstellen
serroblib_update_template_functions({SName}, false);
RS = serroblib_create_robot_class(SName, RName);
% Test-Einstellungen generieren
TSS = RS.gen_testsettings(true, false);
RS.update_base([0.5; 0.5; 0]); % Basis-Transformation (zur freien Positionierung des Roboters)
RS.I_EE = logical([1 1 1 0 0 1]); % nur 4FG in InvKin betrachten
%% Alle Funktionen einmal aufrufen
q0 = rand(4,1);

qD0 = TSS.QD(10,:)';
qDD0 = TSS.QDD(10,:)';
T_E = RS.fkineEE(q0);
xE = [T_E(1:3,4); r2eul(T_E(1:3,1:3), RS.phiconv_W_E)];

% Rufe Kinematik-Funktionen auf
RS.fkine(q0);
RS.fkineEE(q0);
RS.jacobiR(q0);
RS.jacobig(q0);
RS.jacobit(q0);
RS.jacobiw(q0);
RS.jtraf(q0);

% Rufe Dynamik-Funktionen mit Inertialparametern als Eingang auf
RS.DynPar.mode = 2;
T2 = RS.ekin(q0,qD0);
U2 = RS.epot(q0);
g2 = RS.gravload(q0);
M2 = RS.inertia(q0);
c2 = RS.corvec(q0,qD0);
C2 = RS.cormat(q0,qD0);
t2 = RS.invdyn(q0,qD0,qDD0);

% Rufe Dynamik-Funktionen mit Inertialparametervektor als Eingang auf
RS.DynPar.mode = 3;
[T3,Treg]=RS.ekin(q0,qD0);
[U3,Ureg3]=RS.epot(q0);
[g3,greg3]=RS.gravload(q0);
[M3,Mreg3]=RS.inertia(q0);
[c3,creg3]=RS.corvec(q0,qD0);
[C3,Creg3]=RS.cormat(q0,qD0);
[t3,treg3]=RS.invdyn(q0,qD0,qDD0);

% Rufe Dynamik-Funktionen mit Minimalparametervektor als Eingang auf
RS.DynPar.mode = 4;
[T4,Treg4]=RS.ekin(q0,qD0);
[U4,Ureg4]=RS.epot(q0);
[g4,greg4]=RS.gravload(q0);
[M4,Mreg4]=RS.inertia(q0);
[c4,creg4]=RS.corvec(q0,qD0);
[C4,Creg4]=RS.cormat(q0,qD0);
[t4,treg4]=RS.invdyn(q0,qD0,qDD0);

% Rufe IK-Funktionen auf
RS.constr1(q0, RS.x2tr(xE));
RS.constr1grad(q0, RS.x2tr(xE));
RS.constr2(q0, RS.x2tr(xE));
RS.constr2grad(q0, RS.x2tr(xE));
RS.invkin(RS.x2tr(xE), q0+0.1*ones(RS.NJ,1), struct('constr_m', 1)); % IK mit Methode 1 testen, sonst Warnung

% Funktionen kompilieren
RS.fill_fcn_handles(false);
RS.mex_dep();

%% Gelenkwinkel-Trajektorie berechnen
% Für jedes Gelenk 
k=1; QE = RS.qref';
for i = 1:RS.NJ
  qi = RS.qref;
  qi(i) = RS.qlim(i,1);
  k=k+1; QE(k,:) = qi;
  qi(i) = RS.qlim(i,2);
  k=k+1; QE(k,:) = qi;
  qi = RS.qref;
  k=k+1; QE(k,:) = qi;
end

[Q,QD,QDD,T] = traj_trapez2_multipoint(QE, max(abs(RS.qDlim)'), 1e-1, 1e-2, 1e-3, 0.25);

% Kartesische Trajektorie berechnen
X = NaN(size(Q,1),6);
for k = 1:size(Q,1)
  T_0_Ek = RS.fkineEE(Q(k,:)');
  X(k,:) = RS.t2x(T_0_Ek);
end
% Gelenkkräfte berechnen
TAU = RS.invdyn_traj(Q, QD, QDD);

%% Roboter in Grundstellung plotten (mit im Gelenkraum entworfener Trajektorie)

s_plot = struct( 'ks', [1:RS.NJ, RS.NJ+2], 'straight', 0);
figure(2);clf;
hold on;
grid on;
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
view(3);
RS.plot( RS.qref, s_plot );
title(sprintf('Grundstellung: %s', RS.descr));
plot3(X(:,1)+RS.r_W_0(1), X(:,2)+RS.r_W_0(2), X(:,3)+RS.r_W_0(3));

%% Trajektorie aus Gelenkraum visualisieren

figure(3);clf;
for k = 1:RS.NQJ
  subplot(4,4,sprc2no(4,4,1,k));hold on;
  plot(T, Q(:,k)/RS.qunitmult_eng_sci(k));
  plot([0;T(end)], RS.qlim(k,1)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
  plot([0;T(end)], RS.qlim(k,2)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
  xlabel('t [s]');
  ylabel(sprintf('q_%d / %s', k, RS.qunit_eng{k}));
  grid on;
  title(sprintf('Zeitverlauf Gelenkgrößen Achse %d',k));
  subplot(4,4,sprc2no(4,4,2,k));hold on;
  plot(T, QD(:,k)/RS.qunitmult_eng_sci(k));
  plot([0;T(end)], RS.qDlim(k,1)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
  plot([0;T(end)], RS.qDlim(k,2)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
  xlabel('t [s]');
  ylabel(sprintf('qD_%d / %s/s', k, RS.qunit_eng{k}));
  grid on;
  subplot(4,4,sprc2no(4,4,3,k));hold on;
  plot(T, QDD(:,k)/RS.qunitmult_eng_sci(k));
  xlabel('t [s]');
  ylabel(sprintf('qDD_%d / %s/s^2', k, RS.qunit_eng{k}));
  grid on;
  subplot(4,4,sprc2no(4,4,4,k));hold on;
  plot(T, TAU(:,k));
  xlabel('t [s]');
  ylabel(sprintf('\\tau_%d / %s', k, RS.tauunit_sci{k}));
  grid on;
end
linkxaxes

s_anim = struct( 'gif_name', '');
figure(4);clf;
hold on;
plot3(X(:,1)+RS.r_W_0(1), X(:,2)+RS.r_W_0(2), X(:,3)+RS.r_W_0(3));
grid on;
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
view(3);
title('Animation der Gelenktrajektorie');
RS.anim( Q(1:50:end,:), [], s_anim, s_plot);

%% Kartesische Trajektorie
% Würfel-Trajektorie erstellen
q0 = RS.qref+[30;-30;0;0]*pi/180;
T_E = RS.fkineEE(q0);
x0 = [T_E(1:3,4); r2eul(T_E(1:3,1:3), RS.phiconv_W_E)];
% Start in Grundstellung
k=1; XE = x0';
% Fahrt in Startstellung
k=k+1; XE(k,:) = XE(k-1,:) + [ -0.2,0.1,0, 0,0,0];
% Beginn Würfel
d1=0.15;
k=k+1; XE(k,:) = XE(k-1,:) + [ d1,0,0, 0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,-d1,0  0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [-d1,0,0, 0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0, -0.1, 0,0,pi/4];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0, 0.1, 0,0,-pi/4];
k=k+1; XE(k,:) = XE(k-1,:) + [0,d1,0,  0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0, -0.1, 0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [ d1,0,0, 0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,-d1,0  0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0, 0.1, 0,0,-pi/4];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0, -0.1, 0,0,pi/4];
k=k+1; XE(k,:) = XE(k-1,:) + [-d1,0,0, 0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,d1,0,  0,0,0];
k=k+1; XE(k,:) = XE(k-1,:) + [0,0, 0.1, 0,0,0];

[X,XD,XDD,T] = traj_trapez2_multipoint(XE, 1, 1e-1, 1e-2, 1e-3, 0.25);

% Gelenkkräfte und inverse Kinematik berechnen
[Q, QD, QDD] = RS.invkin_traj(X,XD,XDD,T,q0,struct('n_max', 50, 'Phit_tol', 1e-4, 'Phir_tol', 1e-4 , 'constr_m', 1, 'wn', 1));
TAU = RS.invdyn_traj(Q, QD, QDD);
% IK-Ergebnis testen
for i = 1:length(T)
  if max(abs( RS.constr1(Q(i,:)', RS.x2tr(X(i,:)')) )) > 1e-4
    error('IK stimmt nicht');
  end
end
%% Roboter in Grundstellung plotten (mit im Arbeitsraum entworfener Trajektorie)

s_plot = struct( 'ks', [1:RS.NJ, RS.NJ+2], 'straight', 0);
figure(6);clf;
hold on;
grid on;
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
view(3);
RS.plot( q0, s_plot );
title(sprintf('Grundstellung: %s', RS.descr));
plot3(X(:,1)+RS.r_W_0(1), X(:,2)+RS.r_W_0(2), X(:,3)+RS.r_W_0(3));

%% Trajektorie aus Gelenkraum visualisieren

figure(7);clf;
for k = 1:RS.NQJ
  subplot(4,4,sprc2no(4,4,1,k));hold on;
  plot(T, Q(:,k)/RS.qunitmult_eng_sci(k));
  plot([0;T(end)], RS.qlim(k,1)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
  plot([0;T(end)], RS.qlim(k,2)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
  xlabel('t [s]');
  ylabel(sprintf('q_%d / %s', k, RS.qunit_eng{k}));
  grid on;
  title(sprintf('Zeitverlauf Gelenkgrößen Achse %d',k));
  subplot(4,4,sprc2no(4,4,2,k));hold on;
  plot(T, QD(:,k)/RS.qunitmult_eng_sci(k));
  plot([0;T(end)], RS.qDlim(k,1)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
  plot([0;T(end)], RS.qDlim(k,2)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
  xlabel('t [s]');
  ylabel(sprintf('qD_%d / %s/s', k, RS.qunit_eng{k}));
  grid on;
  subplot(4,4,sprc2no(4,4,3,k));hold on;
  plot(T, QDD(:,k)/RS.qunitmult_eng_sci(k));
  xlabel('t [s]');
  ylabel(sprintf('qDD_%d / %s/s^2', k, RS.qunit_eng{k}));
  grid on;
  subplot(4,4,sprc2no(4,4,4,k));hold on;
  plot(T, TAU(:,k));
  xlabel('t [s]');
  ylabel(sprintf('\\tau_%d / %s', k, RS.tauunit_sci{k}));
  grid on;
end
linkxaxes

% Vergleich Soll- und Ist- Trajektorie
figure(8);clf;
for k = 1:6
  subplot(3,6,sprc2no(3,6,1,k));hold on;
  plot(T, X(:,k));
  % plot(T, XI(:,k), '--');
  grid on;
  if k < 4
    ylabel(sprintf('%s_E [m]', char(119+k)));
  else
    ylabel(sprintf('\\phi_{E,1} [rad]'));
  end
  subplot(3,6,sprc2no(3,6,2,k));hold on;
  plot(T, XD(:,k));
  grid on;
  subplot(3,6,sprc2no(3,6,3,k));hold on;
  plot(T, XDD(:,k));
  grid on;
end

s_anim = struct( 'gif_name', '');
figure(9);clf;
hold on;
plot3(X(:,1)+RS.r_W_0(1), X(:,2)+RS.r_W_0(2), X(:,3)+RS.r_W_0(3));
grid on;
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
view(3);
title('Animation der kartesischen Trajektorie');
RS.anim( Q(1:50:end,:), [], s_anim, s_plot);