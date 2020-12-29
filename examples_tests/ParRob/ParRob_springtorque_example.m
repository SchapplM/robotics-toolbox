% Beispiel für in Gelenken einer PKM angebrachte Federn. Aufruf aller
% Funktionen und Testen auf Plausibilität und Funktionalität
% Beispiel-Roboter: 6UPS
% Hinweis zu Kommentaren: Die Begriffe "Kraft" und "Moment" werden teil-
% weise verallgemeinert (je nach Gelenktyp) benutzt (je nach Zusammenhang)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc
usr_robotfigure = false;
usr_define_springtorque = false;
%% Definiere Roboter
RP = parroblib_create_robot_class('P6RRPRRR14V3G1P4A1', 0.5, 0.2);
RP.fill_fcn_handles(true, true); % kompilierte Funktionen
for i = 1:RP.NLEG % Gelenkgrenzen setzen (für IK)
  RP.Leg(i).qlim = repmat([-2*pi, 2*pi], RP.Leg(i).NQJ, 1);
  RP.Leg(i).qlim(3,:) = [0.4, 0.7];
end
qlim_pkm = cat(1, RP.Leg.qlim);
% Startpose: Leicht außermittig im Arbeitsraum
X0 = [ [0.15;0.05;0.5]; [10;-10;5]*pi/180 ];
q0 = qlim_pkm(:,1)+rand(36,1).*(qlim_pkm(:,2)-qlim_pkm(:,1));
q0(RP.I_qa) = 0.5; % gegen Umklappen der Schubgelenke
[q, Phi] = RP.invkin_ser(X0, q0);
if any(abs(Phi) > 1e-6)
  error('Inverse Kinematik konnte in Startpose nicht berechnet werden');
end
if usr_robotfigure
  figure(1); clf; hold on; grid on; %#ok<UNRCH> % Bild als Kinematik-Skizze
  xlabel('x in m');ylabel('y in m');zlabel('z in m'); view(3);
  s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
  RP.plot( q, X0, s_plot );
  title('6UPS in Startkonfiguration');
end
%% Gelenksteifigkeit setzen
% Annahme einer Drefeder in den Gelenken. Dadurch Änderung der Schnittkräfte.
for k = 1:RP.NLEG
  I_k = RP.I1J_LEG(k):RP.I2J_LEG(k);
  % Ruhelage der Feder ist die Startkonfiguration
  RP.Leg(k).DesPar.joint_stiffness_qref = q0(I_k);
  % Federsteifigkeit auf moderaten Wert setzen
  RP.Leg(k).DesPar.joint_stiffness = ones(length(I_k),1)*100;
end

%% Trajektorie definieren
% Einfache Fahrt nach oben mit gleicher Orientierung (zum Testen der
% Trajektorien-Funktionen)
XL = [X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.3]]];
[X_t,XD_t,XDD_t,T] = traj_trapez2_multipoint(XL, 1, 0.1, 0.01, 1e-3, 1e-1);
[Q_t, QD_t, QDD_t, Phi_t, Jinv_t] = RP.invkin2_traj(X_t, XD_t, XDD_t, T, q);
if any(abs(Phi_t(:)) > 1e-6), error('Trajektorien-IK nicht erfolgreich'); end
%% Parameter definieren
mges_PKM = rand(size(RP.DynPar.mges));
rSges_PKM = rand(size(RP.DynPar.rSges));
ISges_PKM = rand(size(RP.DynPar.Icges));
mges_PKM(end,:) = 0; % Masselose Plattform (für Plausibilitäts-Test der Gravitation)
rSges_PKM(end,:) = 0;
ISges_PKM(end,:) = 0;
RP.update_dynpar1 (mges_PKM, rSges_PKM, ISges_PKM);
% Inertial-Parameter als Dynamik-Parameter benutzen (notwendig für
% Regressorform der Schnittkräfte).
RP.DynPar.mode = 3;
for i = 1:RP.NLEG, RP.Leg(i).DynPar.mode = 3; end
% Wähle einen beliebigen Punkt der Trajektorie zum Testen
i = randi([1,length(T)]);
q = Q_t(i,:)';
qD = QD_t(i,:)';
qDD = QDD_t(i,:)';
xP = X_t(i,:)';
xDP = XD_t(i,:)';
xDDP = XDD_t(i,:)';
JinvP = reshape(Jinv_t(i,:), RP.NJ, sum(RP.I_EE));

% Berechne das Gelenkmoment der Federn
tau_add = RP.springtorque(q);
if usr_define_springtorque
  % Setze ein benutzerdefiniertes Gelenkmoment zum Testen ein (zur
  % vereinfachten Plausibilitätsprüfung der Ergebnisse)
  tau_add(:) = 0; %#ok<UNRCH>
  tau_add(1) = 1;
  % Feder-Ruhelage so einstellen, dass genau das vorgegebene Moment
  % herauskommt (nur für die ausgewählte Pose, nicht für ganze Trajektorie)
  for k = 1:RP.NLEG
    I_k = RP.I1J_LEG(k):RP.I2J_LEG(k);
    RP.Leg(k).DesPar.joint_stiffness_qref = q(I_k) - tau_add(I_k)./RP.Leg(k).DesPar.joint_stiffness;
  end
  tau_add2 = RP.springtorque(q);
  test_tau_add = tau_add - tau_add2;
  assert(all(abs(test_tau_add) < 1e-10), 'Rückwärts-Rechnung der geforderten Feder-Ruhelage stimmt nicht');
end
% Federmoment für ganze Trajektorie bestimmen
tau_add_traj = RP.springtorque_traj(Q_t);
% Antriebskräfte für die Kompensation der inversen Dynamik
tauA = RP.invdyn2_actjoint(q, qD, qDD, xP, xDP, xDDP, JinvP);
%% Test: Plattformkraft aus Gelenkfeder
% Direkter Aufruf gegen Regressorform. Beide müssen das gleiche Ergebnis
% haben. Diese Funktion wird von anderen (später getesteten) Funktionen
% aufgerufen. Wird bspw. auch bei der Vorwärtsdynamik benutzt
% (Plattform-Bewegung)
t1 = tic();
taured_Fs = RP.jointtorque_platform(q, xP, tau_add);
T_jointtorque_platform = toc(t1);
t1 = tic();
[taured_Fs2, taured_Fs_reg] = RP.jointtorque_platform(q, xP, tau_add);
T_jointtorque_platform_reg = toc(t1);
test1_taured_Fs_reg = taured_Fs_reg*tau_add - taured_Fs;
assert(all(abs(test1_taured_Fs_reg) < 1e-6), 'jointtorque_platform stimmt nicht');
test2_taured_Fs_reg = taured_Fs2 - taured_Fs;
assert(all(abs(test2_taured_Fs_reg) < 1e-6), 'jointtorque_platform stimmt nicht');
fprintf(['Getestet: jointtorque_platform direkt (%1.1fms) gegen Regressor', ...
  'form (%1.1fms)\n'], 1e3*T_jointtorque_platform, 1e3*T_jointtorque_platform_reg);

% Das gleiche mit Eingabe der Jacobi-Matrix
Phi_q = RP.constr4grad_q(q);
Phi_x = RP.constr4grad_x(xP, true);
Jinv_num_voll = -Phi_q \ Phi_x;
Jinv_num = Jinv_num_voll(RP.I_qa,:);
t1 = tic();
taured_Fs3 = RP.jointtorque_platform(q, xP, tau_add);
T_jointtorque_platform = toc(t1);
t1 = tic();
[taured_Fs4, taured_Fs_reg2] = RP.jointtorque_platform(q, xP, tau_add);
T_jointtorque_platform_reg = toc(t1);
test3_taured_Fs_reg = taured_Fs_reg2*tau_add - taured_Fs3;
test4_taured_Fs_reg = taured_Fs3 - taured_Fs4;
test5_taured_Fs_reg = taured_Fs2 - taured_Fs4;
assert(all(abs(test3_taured_Fs_reg) < 1e-6), 'jointtorque_platform stimmt nicht (Jacobi-Eingabe)');
assert(all(abs(test4_taured_Fs_reg) < 1e-6), 'jointtorque_platform stimmt nicht (Jacobi-Eingabe)');
assert(all(abs(test5_taured_Fs_reg) < 1e-6), 'jointtorque_platform stimmt nicht (Jacobi-Eingabe)');
fprintf(['Getestet: jointtorque_platform direkt (%1.1fms) gegen Regressor', ...
  'form (%1.1fms) (mit Jacobi-Eingabe)\n'], 1e3*T_jointtorque_platform, ...
  1e3*T_jointtorque_platform_reg);

% Aufruf mit Trajektorie
t1 = tic();
[Fx_traj,Fx_traj_reg] = RP.jointtorque_platform_traj(Q_t, X_t, tau_add_traj, Jinv_t);
T_jointtorque_platform_traj = toc(t1);
t1 = tic();
Fx_traj2 = RP.jointtorque2_platform_traj(Fx_traj_reg, tau_add_traj);
T_jointtorque2_platform_traj = toc(t1);
test_Fx_traj = Fx_traj - Fx_traj2;
assert(all(abs(test_Fx_traj(:)) < 1e-6), 'jointtorque2_platform_traj stimmt nicht');
fprintf(['Getestet: jointtorque_platform_traj direkt (%1.1fms) gegen Regressor', ...
  'form mit jointtorque2_platform_traj (%1.1fms)\n'], 1e3*T_jointtorque_platform_traj, ...
  1e3*T_jointtorque2_platform_traj);
%% Test: Antriebskraft aus Gelenkfeder
% Direkter Aufruf gegen Regressorform. Berechnet die zur Kompensation der
% Gelenkfeder notwendigen Antriebskräfte. Verschiedene Berechnungsmethoden
% müssen zum gleichen Ergebnis führen.
% Berechnung für einzelne Pose des Roboters
t1 = tic();
Fa = RP.jointtorque_actjoint(q, xP, tau_add, Jinv_num_voll);
T_jointtorque_actjoint = toc(t1);
t1 = tic();
[Fa2, Fa_reg] = RP.jointtorque_actjoint(q, xP, tau_add, Jinv_num_voll);
T_jointtorque_actjoint_reg = toc(t1);
test1_Fa_reg = Fa_reg*tau_add - Fa;
assert(all(abs(test1_Fa_reg) < 1e-6), 'jointtorque_actjoint stimmt nicht');
test2_Fa_reg = Fa2 - Fa;
assert(all(abs(test2_Fa_reg) < 1e-6), 'jointtorque_actjoint stimmt nicht');
fprintf(['Getestet: jointtorque_actjoint direkt (%1.1fms) gegen Regressor', ...
  'form (%1.1fms)\n'], 1e3*T_jointtorque_actjoint, 1e3*T_jointtorque_actjoint_reg);

% Berechnung für Trajektorie
t1 = tic();
[Fa_traj] = RP.jointtorque_actjoint_traj(Q_t, X_t, tau_add_traj, Jinv_t);
T_jointtorque_actjoint_traj = toc(t1);
t1 = tic();
[Fa_traj2,Fa_traj_reg] = RP.jointtorque_actjoint_traj(Q_t, X_t, tau_add_traj, Jinv_t);
T_jointtorque_actjoint_traj_reg = toc(t1);
t1 = tic();
Fa_traj3 = RP.jointtorque2_actjoint_traj(Fa_traj_reg, tau_add_traj);
T_jointtorque2_actjoint_traj = toc(t1);
test_Fa_traj = Fa_traj - Fa_traj2;
test_Fa_traj2 = Fa_traj - Fa_traj3;
assert(all(abs(test_Fa_traj(:)) < 1e-6), 'jointtorque_actjoint_traj stimmt nicht bei ein vs zwei Ausgaben');
assert(all(abs(test_Fa_traj2(:)) < 1e-6), 'jointtorque2_actjoint_traj stimmt nicht gegen jointtorque_actjoint_traj');

fprintf(['Getestet: jointtorque_actjoint_traj mit einer Ausgabe (%1.1fms) ', ...
  'gegen jointtorque_actjoint_traj (%1.1fms) und jointtorque2_actjoint_traj ', ...
  '(%1.1fms)\n'], 1e3*T_jointtorque_actjoint_traj, 1e3*T_jointtorque_actjoint_traj_reg, ...
  1e3*T_jointtorque2_actjoint_traj);

%% Test: Schnittkraft aus Inverser Dynamik
% Berechne die Schnittkräfte aus der inversen Dynamik ohne Berücksichtigung
% der Gelenkfedern. Verschiedene Implementierungen müssen das gleiche
% Ergebnis haben. Prüfe zusätzlich auf Plausibilität durch Abgleich mit
% Antriebskraft.
t1 = tic();
[w_B, w_all_linkframe, w_all_baseframe] = RP.internforce(q, qD, qDD, tauA);
T_internforce = toc(t1);
% Prüfe die Komponenten der Schnittkräfte/-momente, die den
% Antriebsgelenken entsprechen. Muss identisch mit Antriebskraft sein.
tauA2 = NaN(length(tauA),1);
for i = 1:RP.NLEG
  iActJoint = RP.I_qa(RP.I1J_LEG(i):RP.I2J_LEG(i));
  ColActJoint = 1+find(iActJoint); % Jede Spalte entspricht einem Gelenk. Basis ist erste Spalte.
  RowActJoint = 3+3*(RP.Leg(i).MDH.sigma(iActJoint)==0); % Zeile 3 (fz) für Schubgelenke oder Zeile 6 (mz) für Drehgelenk
  tauA2(i) = w_all_linkframe(RowActJoint, ColActJoint, i);
end
test_tauA = tauA - tauA2;
assert(all(abs(test_tauA) < 1e-10), 'Schnittkraft stimmt nicht gegen Antriebskraft in internforce');
% Berechne die Regressorform der Schnittkräfte und vergleiche mit
% vorherigem
t1 = tic();
[w_B_reg, w_all_linkframe_reg, w_all_baseframe_reg] = RP.internforce_regmat(q, qD, qDD, xP, xDP, xDDP, JinvP);
T_internforce_regmat = toc(t1);
% Testen
w_B_fromreg = reshape(w_B_reg*RP.DynPar.ipv_n1s, 6, RP.NLEG);
test_w_B = w_B - w_B_fromreg;
assert(all(abs(test_w_B(:))<1e-10), 'Erste Ausgabe von internforce stimmt nicht gegen internforce_regmat');
t1 = tic();
w_all_linkframe_fromreg = RP.internforce3(w_all_linkframe_reg);
T_internforce3 = toc(t1);
test_w_all_linkframe = w_all_linkframe_fromreg - w_all_linkframe;
assert(all(abs(test_w_all_linkframe(:))<1e-10), 'Zweite Ausgabe von internforce stimmt nicht gegen internforce_regmat/internforce3');
w_all_baseframe_fromreg = RP.internforce3(w_all_baseframe_reg);
test_w_all_baseframe = w_all_baseframe_fromreg - w_all_baseframe;
assert(all(abs(test_w_all_baseframe(:))<1e-10), 'Dritte Ausgabe von internforce stimmt nicht gegen internforce_regmat/internforce3');

fprintf(['Getestet: internforce (%1.1fms) gegen internforce_regmat ', ...
  '(%1.1fms) und internforce3 (%1.1fms)\n'], 1e3*T_internforce, ...
  1e3*T_internforce_regmat, 1e3*T_internforce3);
%% Test: Schnittkraft aus Gravitation, Plausibilität
% Betrachte den Vektor der Gravitationsmomente anstelle des Vektors der
% Federmomente. Beides entspricht einem Potential und der inneren Dynamik
% der PKM. Bei Gravitation ist aufgrund der Implementierung das erwartete
% Ergebnis bekannt.
% Voraussetzung: Plattform ist masselos (siehe Parameter oben).

% Vektor der Gravitationsmomente der Beinketten der PKM ohne Betrachtung
% der PKM-Kopplung
tau_Grav = NaN(RP.NJ,1);
for i = 1:RP.NLEG
  tau_Grav(RP.I1J_LEG(i):RP.I2J_LEG(i)) = RP.Leg(i).gravload(q(RP.I1J_LEG(i):RP.I2J_LEG(i)));
end
% Antriebskräfte der PKM zur Kompensation der Gravitationskräfte
tauA_Grav = RP.invdyn2_actjoint(q, 0*qD, 0*qDD, xP,0*xDP,0*xDDP,JinvP);
% Schnittkräfte zur Kompensation des Vektors der Gelenkmomente aus den
% Beinketten-Gravitationsmomenten (inkl dafür notwendige Antriebskraft).
[w_B_grav, w_all_linkframe_grav, w_all_baseframe_grav] = RP.internforce(q, 0*qD, 0*qDD, tauA_Grav, tau_Grav);
% Schnittkräfte aufgrund der Gravitation (und damit verbundener Antriebs-
% kräfte). Hier ist die PKM im statischen Gleichgewicht.
[w_B_grav2, w_all_linkframe_grav2, w_all_baseframe_grav2] = RP.internforce(q, 0*qD, 0*qDD, tauA_Grav);
% Prüfe, in wie weit die beiden Ergebnisse übereinstimmen
test_w_B_grav = w_B_grav-w_B_grav2;
% test_w_all_linkframe_grav = w_all_linkframe_grav-w_all_linkframe_grav2;
% test_w_all_baseframe_grav = w_all_baseframe_grav-w_all_baseframe_grav2;
tau_Grav2 = NaN(length(tau_Grav),1); tau_Grav3 = tau_Grav2;
% Entferne Einträge der Schnittkraft-Matrix, die keine Aussage über die
% Gravitation zulassen (Komponenten, die nicht in Gelenkrichtung gehen)
for i = 1:RP.NLEG
  for j = 1:RP.Leg(i).NJ
    Row_j = 3+3*(RP.Leg(i).MDH.sigma(j)==0);
    Col_j = 1+j;
%     test_w_all_linkframe_grav(1:6~=Row_j, Col_j, i) = NaN;
%     test_w_all_baseframe_grav(1:6~=Row_j, Col_j, i) = NaN;
    % Extrahiere die Einträge, die zu den Gelenken gehören zum Abgleich.
    tau_Grav2(RP.I1J_LEG(i)+j-1) = w_all_linkframe_grav(Row_j, Col_j, i);
    tau_Grav3(RP.I1J_LEG(i)+j-1) = w_all_linkframe_grav2(Row_j, Col_j, i);
  end
  % Keine Aussage über Basis-Reaktionskräfte der Beinkette möglich.
%   test_w_all_linkframe_grav(:,1,i) = NaN;
%   test_w_all_baseframe_grav(:,1,i) = NaN;
end
% Die Kräfte an den Plattform-Koppelgelenken müssen gleich sein
assert(all(abs(test_w_B_grav(:))<1e-10), 'Erste Ausgabe von internforce (mit Gravitation) stimmt nicht');
% Der Vergleich der Schnittkräfte mit den beiden Berechnungen ist nicht
% möglich. Es kann nur der eingegebene Vektor der Gelenkmomente abgebildet
% werden. Bei der ersten Rechnung wird der Gelenkmoment-Vektor nicht als
% interne Dynamik aufgefasst. Variablen für folgenden Test daher
% auskommentiert.
% assert(all(abs(test_w_all_linkframe_grav(:))<1e-10|isnan(test_w_all_linkframe_grav(:))), ...
%   'Zweite Ausgabe von internforce (mit Gravitation) stimmt nicht');
% assert(all(abs(test_w_all_baseframe_grav(:))<1e-10|isnan(test_w_all_baseframe_grav(:))), ...
%   'Dritte Ausgabe von internforce (mit Gravitation) stimmt nicht');
% Normale Berechnung der inversen Dynamik. Passive Gelenke haben keine
% Schnittmomente (in Richtung der Gelenkkoordinate), aktive Gelenke haben
% die Schnittkraft, die der Antriebskraft entspricht (in Gelenkkoordinate)
assert(all(abs(tau_Grav3(~RP.I_qa)) < 1e-10), 'Passive Gelenke haben keine Schnittkraft von Null');
assert(all(abs(tau_Grav3( RP.I_qa)-tauA_Grav) < 1e-10), 'Schnittkraft der aktiven Gelenke entspricht nicht der Antriebskraft');
% Berechnung für Gelenkmoment-Vektor. Schnittkraft in passiven Gelenken
% muss jetzt dem vorgegebenen Vektor entsprechen. Bei aktiven Gelenken
% zusätzlich die vorgegebene Antriebskraft zur Kompensation des Effekts.
assert(all(abs(tau_Grav2(~RP.I_qa) - tau_Grav(~RP.I_qa)) < 1e-10), ...
  'Vorgegebene Schnittmomente lassen sich nicht für passive Gelenke rekonstruieren');
assert(all(abs(tau_Grav2(RP.I_qa) + tauA_Grav - tau_Grav(RP.I_qa)) < 1e-10), ...
  'Vorgegebene Schnittmomente lassen sich nicht für aktive Gelenke rekonstruieren');
fprintf('Getestet: Plausibilität von internforce mit Gravitationsmomenten\n');

%% Test: Schnittkraft aus Gelenkfeder
% Verschiedene Implementierungen für die Berechnung der Schnittkraft aus
% den Gelenkmomenten der Gelenkfeder. Prinzipiell ähnliche Vorgehensweise
% wie im vorherigen Beispiel für die Gravitationsmomente.
t1 = tic();
[w_B_spring, w_all_linkframe_spring, w_all_baseframe_spring] = RP.internforce(q, qD, qDD, Fa, tau_add);
T_internforce = toc(t1);
% Extrahiere die Gelenkmomente der Beinkette aus den Schnittkräften. Die
% Einträge müssen dem vorgegebenen Vektor der externen Momente sowie der zu
% dessen Kompensation notwendigen Antriebskraft entsprechen.
tau_add2 = NaN(length(tau_add),1);
for i = 1:RP.NLEG
  for j = 1:RP.Leg(i).NJ
    Row_j = 3+3*(RP.Leg(i).MDH.sigma(j)==0);
    Col_j = 1+j;
    tau_add2(RP.I1J_LEG(i)+j-1) = w_all_linkframe_spring(Row_j, Col_j, i);
  end
end
test_tau_add = NaN(length(tau_add),1);
test_tau_add(~RP.I_qa) = tau_add(~RP.I_qa) - tau_add2(~RP.I_qa); % passiv
test_tau_add(RP.I_qa) = tau_add(RP.I_qa) - (tau_add2(RP.I_qa)+Fa); % aktiv
assert(all(abs(test_tau_add) < 1e-10), 'Schnittkraft stimmt nicht gegen Gelenkmoment in internforce');

% Gleiche Berechnung nur mit Regressorform. Ergebnis muss gleich sein.
t1 = tic();
[w_B_reg_spring, w_all_linkframe_reg_spring, w_all_baseframe_reg_spring] = RP.internforce_regmat(q, qD, qDD, xP, xDP, xDDP, JinvP, ones(RP.NJ,1));
T_internforce_regmat = toc(t1);
w_B_spring_fromreg = reshape(w_B_reg_spring*tau_add, 6, RP.NLEG);
test_w_B_spring = w_B_spring - w_B_spring_fromreg;
t1 = tic();
w_all_linkframe_spring_fromreg = RP.internforce3(w_all_linkframe_reg_spring, tau_add);
T_internforce3 = toc(t1);
test_w_all_linkframe_spring = w_all_linkframe_spring_fromreg - w_all_linkframe_spring;
w_all_baseframe_spring_fromreg = RP.internforce3(w_all_baseframe_reg_spring, tau_add);
test_w_all_baseframe_spring = w_all_baseframe_spring_fromreg - w_all_baseframe_spring;
% Erneute Extraktion der Gelenkmomente aus den Schnittkräften (s.o.)
tau_add3 = NaN(length(tau_add),1);
for i = 1:RP.NLEG
  for j = 1:RP.Leg(i).NJ
    Row_j = 3+3*(RP.Leg(i).MDH.sigma(j)==0);
    Col_j = 1+j;
    tau_add3(RP.I1J_LEG(i)+j-1) = w_all_linkframe_spring_fromreg(Row_j, Col_j, i);
  end
end
% Erneute Prüfung, ob Gelenkmomente in Schnittkraft enthalten.
test_tau_add2 = NaN(length(tau_add),1);
test_tau_add2(~RP.I_qa) = tau_add(~RP.I_qa) - tau_add3(~RP.I_qa);
test_tau_add2(RP.I_qa) = tau_add(RP.I_qa) - (tau_add3(RP.I_qa)+Fa);
assert(all(abs(test_tau_add2) < 1e-10), 'Schnittkraft stimmt nicht gegen Gelenkmoment in internforce_regmat');
% Prüfe, ob Regressorform und direkte Berechnung zu gleichen Ergebnis führen.
assert(all(abs(test_w_B_spring(:))<1e-10), 'Erste Ausgabe von internforce (mit Feder) stimmt nicht gegen internforce_regmat');
assert(all(abs(test_w_all_linkframe_spring(:))<1e-10), 'Zweite Ausgabe von internforce (mit Feder) stimmt nicht gegen internforce_regmat/internforce3');
assert(all(abs(test_w_all_baseframe_spring(:))<1e-10), 'Dritte Ausgabe von internforce (mit Feder) stimmt nicht gegen internforce_regmat/internforce3');
fprintf(['Getestet: internforce (%1.1fms) gegen internforce_regmat ', ...
  '(%1.1fms) und internforce3 (%1.1fms)\n'], 1e3*T_internforce, ...
  1e3*T_internforce_regmat, 1e3*T_internforce3);

%% Test: Schnittkraft aus Gelenkfeder mit Trajektorien-Funktion
% Gleiche Tests wie oben, nur mit Trajektorien-Funktionen. Einmal im
% Körper-KS, einmal im Basis-KS. Letzteres erfordert eine zusätzliche
% Berechnung in der Funktion (daher separater Test)
t1 = tic();
[FLegl_t] = RP.internforce_traj(Q_t, 0*Q_t, 0*Q_t, Fa_traj, tau_add_traj);
T_internforce_traj = toc(t1);
t1 = tic();
[Regmatl_traj] = RP.internforce_regmat_traj(Q_t, 0*Q_t, 0*Q_t, ...
  X_t, 0*X_t, 0*X_t, Jinv_t, ones(size(tau_add_traj)));
T_internforce_regmat_traj = toc(t1);
t1 = tic();
FLegl_t2 = RP.internforce3_traj(Regmatl_traj, tau_add_traj);
T_internforce3_traj = toc(t1);
test_FLegl = FLegl_t-FLegl_t2;
assert(all(abs(test_FLegl(:)) < 1e-10), 'internforce_regmat_traj stimmt nicht gegen internforce3_traj für die Gelenkelastizität');
fprintf(['Getestet: internforce_traj (%1.2fs) gegen internforce_regmat_traj ', ...
  '(%1.2fs) und internforce3_traj (%1.2fs) (Körper-KS)\n'], T_internforce_traj, ...
  T_internforce_regmat_traj, T_internforce3_traj);

t1 = tic();
[~, FLeg0_t] = RP.internforce_traj(Q_t, 0*Q_t, 0*Q_t, Fa_traj, tau_add_traj);
T_internforce_traj = toc(t1);
t1 = tic();
[~, Regmat0_traj] = RP.internforce_regmat_traj(Q_t, 0*Q_t, 0*Q_t, ...
  X_t, 0*X_t, 0*X_t, Jinv_t, ones(size(tau_add_traj)));
T_internforce_regmat_traj = toc(t1);
t1 = tic();
FLeg0_t2 = RP.internforce3_traj(Regmat0_traj, tau_add_traj);
T_internforce3_traj = toc(t1);
test_FLeg0 = FLeg0_t-FLeg0_t2;
assert(all(abs(test_FLeg0(:)) < 1e-10), 'internforce_regmat_traj stimmt nicht gegen internforce3_traj für die Gelenkelastizität (Basis-KS)');
fprintf(['Getestet: internforce_traj (%1.2fs) gegen internforce_regmat_traj ', ...
  '(%1.2fs) und internforce3_traj (%1.2fs) (Basis-KS)\n'], T_internforce_traj, ...
  T_internforce_regmat_traj, T_internforce3_traj);