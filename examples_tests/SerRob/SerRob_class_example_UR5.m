% Beispiel zur Erstellung der SerRob-Klasse für den Universal Robots UR5
% Inhalt:
% * Generierung der Roboter-Kinematik (beispielhaft für Hinzufügen eines
%   neuen Roboters bzw. einer neuen Modell-Variante)
% * Umrechnung zwischen Standard und modifizierter DH Notation (SDH/MDH)
% * Plausibilisierung mit Zeichnung des Roboters
% * Inverse Kinematik für Beispiel-Trajektorie
%   (mit 3T3R- und 3T2R-Algorithmus. Beide haben hier fast identisches
%   Ergebnis, da die Nullraumbewegung nur mit dem letzten Gelenk passiert)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-09
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc
usr_addtodb = false; % Hinzufügen zur Datenbank. Muss nur einmalig gemacht werden.

rob_path = fileparts(which('robotics_toolbox_path_init.m'));
resdir = fullfile(rob_path, 'examples_tests', 'results');
if isempty(which('serroblib_path_init.m'))
  warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end
%% Struktur erstellen und in Seriell-Roboter-Datenbank einfügen
% Falls die kinematische Kette noch nicht in der Datenbank ist, wird sie
% hiermit eingefügt. Wandle DH in MDH um.
% https://www.universal-robots.com/articles/ur/parameters-for-calculations-of-kinematics-and-dynamics/
if usr_addtodb
  N=6; %#ok<UNRCH>
  MDH_struct_num = struct('beta',  NaN(N,1), 'b', NaN(N,1), ...
    'alpha', NaN(N,1), 'a', NaN(N,1), ...
    'theta', NaN(N,1), 'd', NaN(N,1), ...
    'sigma', NaN(N,1), 'offset', NaN(N,1));
  % b/beta sind Null (keine Baumstruktur)
  MDH_struct_num.beta = zeros(N,1);
  MDH_struct_num.b = zeros(N,1);
  % MDH alpha_{i} entspricht SDH alpha_{i-1}
  MDH_struct_num.alpha = [0; pi/2; 0; 0; pi/2; pi/2]; % letztes ist -pi/2 -> Drehe a6 und EE-KS
  % MDH a_{i} entspricht SDH a_{i-1}
  MDH_struct_num.a = [0; 0; NaN; NaN; 0; 0];
  % MDH=SDH
  MDH_struct_num.theta = [0; 0; 0; 0; 0; 0];
  % MDH=SDH
  MDH_struct_num.d = [NaN; 0; 0; NaN; NaN; NaN];
  MDH_struct_num.sigma = [0; 0; 0; 0; 0; 0]; % nur Drehgelenke
  MDH_struct_num.offset = zeros(N,1);
  MDH_struct_idx = serroblib_mdh_numparam2indexstruct(MDH_struct_num);

  Name_DB = serroblib_add_robot(MDH_struct_idx,'111111');
  serroblib_gen_bitarrays(6);
  % Nach erstmaligem Hinzufügen: generate_variant_pkin_conv_fcns.m
  % Optional: Code für diese Modellvariante des allgemeinen
  % Industrieroboters erzeugen.
  % serroblib_generate_mapleinput({Name_DB});
  % serroblib_generate_code({Name_DB});
end
%% CAD-Modell zeichnen

% Typ des seriellen Roboters auswählen (UR-Typ)
SName='S6RRRRRR10V4';
% Modellparameter auswählen (hinterlegt aus Datenblatt)
RName='S6RRRRRR10V4_UR5';
% Vorlagen-Funktionen neu erstellen (nur notwendig bei Code-Änderung)
% serroblib_create_template_functions({SName}, false);
serroblib_update_template_functions({SName}, false);
% Instanz der Roboterklasse erstellen
RS = serroblib_create_robot_class(SName, RName);
RS.fill_fcn_handles(true, true); % kompilierte Funktionen benutzen

% Alles neu kompilieren (nur, falls Vorlagen geändert wurden seit dem
% letzten Kompilieren)
% RS.mex_dep(true)

% Definiere eine EE-Transformation. Es muss eine Drehung zwischen letzter
% Gelenkachse und der EE-z-Achse geben. Ansonsten funktioniert die
% Aufgabenredundanz in der inversen Kinematik nicht
RS.update_EE([0;0;-0.0], [pi;0;0]);
%% Transformation zwischen MDH und SDH KS
% Diese Transformation kann benutzt werden, um das STL-Modell richtig ein-
% zustellen (siehe S6RRRRRR10V4_UR5_init_CAD) oder zum Vergleich zu SDH.
q0 = pi/180*[0; -90; 60; -60; -90; 0];
Tc_mdh = RS.fkine(q0);
% Umrechnung von MDH auf SDH. alpha und a sind um eins verschoben (s.o.)
a_sdh = [RS.MDH.a(2:RS.NJ);0];
alpha_sdh = [RS.MDH.alpha(2:RS.NJ);0];
% Standard-DH Koordinatentransformation
Tc_sdh = NaN(4,4,RS.NJ+1);
Tc_sdh(:,:,1) = eye(4);
for i = 1:RS.NJ
  Tc_sdh(:,:,i+1) = Tc_sdh(:,:,i) * trotz(q0(i))*transl([0;0;RS.MDH.d(i)])* ...
                              trotx(alpha_sdh(i))*transl([a_sdh(i);0;0]);
end
T_mdh_sdh = NaN(4,4,RS.NJ+1);
for i = 1:RS.NL
  T_mdh_sdh(:,:,i) = invtr(Tc_mdh(:,:,i)) * Tc_sdh(:,:,i);
end
%% Strichmodell plotten
figure(1);clf;
s_plot = struct( 'ks', [1:RS.NJ, RS.NJ+2], 'mode', 1);
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
view(3);
RS.plot(q0, s_plot);
sgtitle('UR 5 Strichmodell');
%% CAD-Modell plotten
figure(2);clf;
s_plot = struct( 'ks', [1:RS.NJ, RS.NJ+2], 'mode', 2);
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
view(3);
RS.plot( q0, s_plot );
sgtitle('UR 5 CAD-Modell');
%% Trajektorie definieren
% Würfel-Trajektorie erstellen
q1 = q0+[0;5;35;-40;0;0]*pi/180; % damit Arbeitsraum günstiger
% Debug: Prüfen
% figure(3);clf;hold on; grid on;
% xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
% view(3);
% RS.plot( q1, s_plot );

fprintf('Konditionszahl der Jacobi in IK-Anfangswert: %1.2f\n', cond(RS.jacobig(q1)));
T_E = RS.fkineEE(q1);
x0 = [T_E(1:3,4); r2eul(T_E(1:3,1:3), RS.phiconv_W_E)];
% Start in Grundstellung
k=1; XE = x0';
% Fahrt in Startstellung
% k=k+1; XE(k,:) = XE(k-1,:) + [ -0.2,0.1,0, 0,0,0];
% Beginne Quader-Trajektorie
d1=0.25;
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
[X,XD,XDD,T,IL] = traj_trapez2_multipoint(XE, 0.5, 1e-1, 1e-2, 5e-3, 0.1);
% In vorheriges Bild einzeichnen
change_current_figure(3);
plot3(X(:,1), X(:,2), X(:,3), 'k-');
%% Inverse Kinematik der Eckpunkte berechnen
for i = 1:size(XE,1)
  % Mit 3T3R-IK
  [q_3T3R, Phi_3T3R] = RS.invkin2(RS.x2tr(XE(i,:)'), q1);
  % Mit 3T2R-IK
  s_ik = struct('I_EE', logical([1 1 1 1 1 0]));
  s_ik.wn = zeros(RS.idx_ik_length.wnpos, 1);
  s_ik.wn(RS.idx_ikpos_wn.qlim_hyp) = 1;
  [q_3T2R, Phi_3T2R] = RS.invkin2(RS.x2tr(XE(i,:)'), q1, s_ik);
  if any(abs([Phi_3T3R;Phi_3T2R]) > 1e-10)
    error('IK für Eckpunkt %d funktioniert mit einer Methode nicht', i);
  end
  % Zeige, dass das Ergebnis mit geänderter Redundanter Koordinate
  % identisch ist
  XE_i_test = XE(i,:)'; XE_i_test(6) = rand(1,1);
  q_3T2R_test = RS.invkin2(RS.x2tr(XE_i_test), q1, s_ik);
  if any(abs(q_3T2R_test-q_3T2R) > 1e-10)
    error('3T2R-IK für Eckpunkt %d hat anderes Ergebnis bei anderem x6. Darf nicht sein.', i);
  end
end
fprintf('IK für alle %d Eckpunkte der Trajektorie berechnet.\n', size(XE,1));
%% Inverse Kinematik der Trajektorie berechnen
fprintf('Berechne IK für Trajektorie mit %d Punkten.\n', length(T));
% Berechne die IK mit vollständigen Rotations-FG (wie in X vorgegeben)
t1=tic();
[Q_3T3R, QD_3T3R, ~, PHI_3T3R] = RS.invkin2_traj(X,XD,XDD,T,q1);
fprintf('Dauer der IK-Berechnung (3T3R): %1.1fs (%1.2fms pro Bahnpunkt)\n', ...
  toc(t1), 1e3*toc(t1)/length(T));

% Stelle die IK auf Aufgabenredundanz ein
t1=tic();
s_ik = struct('I_EE', logical([1 1 1 1 1 0]), 'retry_limit', 0); 
s_ik.wn = zeros(RS.idx_ik_length.wntraj, 1);
s_ik.wn(RS.idx_ikpos_wn.qlim_hyp) = 1; % Hyperbolischer Abstand der Gelenkpositionen von ihren Grenzen
[Q, QD, ~, PHI_3T2R] = RS.invkin2_traj(X,XD,XDD,T,q1,s_ik);
fprintf('Dauer der IK-Berechnung: %1.1fs (%1.2fms pro Bahnpunkt)\n', ...
  toc(t1), 1e3*toc(t1)/length(T));
% Prüfe, ob IK erfolgreich
Jcond_all = NaN(length(T),2);
for i = 1:length(T)
  if max(abs( PHI_3T3R(i,:) )) > 1e-4 || any(isnan(PHI_3T3R(i,:))) ||...
     max(abs( PHI_3T2R(i,:) )) > 1e-4 || any(isnan(PHI_3T2R(i,:)))
    [~,iE_next] = min(abs(T(IL)-T(i)));
    warning(['IK stimmt nicht. Fehler bei Zeitschritt %d (t=%1.3fs). Nach Eckpunkt %d. ', ...
      'Trajektorie nicht durchführbar?'], i, T(i), iE_next);
    figure(3);clf;
    s_plot = struct( 'ks', [1:RS.NJ, RS.NJ+2], 'mode', 1);
    hold on; grid on;
    plot3(X(:,1), X(:,2), X(:,3));
    xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
    view(3);
    RS.plot(Q_3T3R(i-1,:)', s_plot);
    title('Letzter funktionierender Trajektorienpunkt');
    % Folgender Fehler wird auskommentiert, falls neue Traj. ausprobiert wird.
    error('Fehler in Trajektorien IK. Das muss funktionieren');
    break %#ok<UNRCH>
  end
  Jcond_all(i,1) = cond(RS.jacobig(Q_3T3R(i,:)'));
  Jcond_all(i,2) = cond(RS.jacobig(Q(i,:)'));
end

%% Zeitverlauf der Trajektorie
figure(7);clf;
for k = 1:RS.NQJ
  subplot(2,3,k);hold on;
  hdl1=plot(T, Q_3T3R(:,k)/RS.qunitmult_eng_sci(k));
  hdl2=plot(T, Q(:,k)/RS.qunitmult_eng_sci(k));
  plot([0;T(end)], RS.qlim(k,1)*[1;1]/RS.qunitmult_eng_sci(k), 'r-');
  plot([0;T(end)], RS.qlim(k,2)*[1;1]/RS.qunitmult_eng_sci(k), 'r-');
  xlabel('t in s');
  ylabel(sprintf('q_%d in %s', k, RS.qunit_eng{k}));
  grid on;
end
legend([hdl1;hdl2], {'3T3R', '3T2R'});
sgtitle(sprintf('Zeitverlauf der Gelenkwinkel'));
linkxaxes
figure(8);clf;
for k = 1:RS.NQJ
  subplot(2,3,k);hold on;
  hdl1=plot(T, QD_3T3R(:,k)/RS.qunitmult_eng_sci(k));
  hdl2=plot(T, QD(:,k)/RS.qunitmult_eng_sci(k));
  plot([0;T(end)], RS.qDlim(k,1)*[1;1]/RS.qunitmult_eng_sci(k), 'r-');
  plot([0;T(end)], RS.qDlim(k,2)*[1;1]/RS.qunitmult_eng_sci(k), 'r-');
  xlabel('t in s');
  ylabel(sprintf('qD_%d in %s', k, RS.qunit_eng{k}));
  grid on;
end
legend([hdl1;hdl2], {'3T3R', '3T2R'});
sgtitle(sprintf('Zeitverlauf der Gelenkgeschwindigkeit'));
linkxaxes
figure(9);clf;
plot(T, Jcond_all);
grid on
sgtitle(sprintf('Zeitverlauf Konditionszahl Jacobi'));

%% Test
% Zeige, dass das Ergebnis identisch ist, wenn die aufgabenredundante
% Koordinate beliebig geändert wird
X_test = X; X_test(:,6) = rand(length(T),1);
XD_test = XD; XD_test(:,6) = rand(length(T),1);
XDD_test = XDD; XDD_test(:,6) = rand(length(T),1);
[Q_test, ~, ~, PHI] = RS.invkin2_traj(X_test,XD_test,XDD_test,T,q1,s_ik);
test_Q_AR = Q_test - Q;
if any(abs(test_Q_AR(:))>1e-10)
  error('IK hat anderes Ergebnis bei Änderung der redundanten Koordinate. Darf nicht sein.');
end

%% Animation
s_anim = struct( 'mp4_name', fullfile(resdir,'UR5_Traj.mp4')); % als mp4 speichern
s_plot = struct( 'ks', [1:RS.NJ, RS.NJ+2], 'mode', 2); % alle KS zeichnen, als Strich-Modell.
fhdl=figure(10);clf;
set(fhdl,'units','normalized','outerposition',[0 0 1 1], 'color','w'); % Vollbild
hold on;
plot3(X(:,1), X(:,2), X(:,3));
grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
view(3);
title('Animation der kartesischen Trajektorie');
RS.anim( Q, [], s_anim, s_plot);
fprintf('Animation der Bewegung nach %s gespeichert.\n', resdir);
