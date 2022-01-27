% Asym. 3T2R PKM (UPS-Struktur) mit passiver Führungskette:
% PUU-Leading, RUU-Leading und UPU-Leading
% Ergebnis des Beispielskripts:
% * Nachweis, dass diese Art von PKM kinematisch modellierbar ist
% 
% Quellen:
% [STO19] Schappler, M. et al.: Modeling Parallel Robot Kinematics for 3T2R
% and 3T3R Tasks using Reciprocal Sets of Euler Angles, MDPI Robotics KaRD2

% MA Bejaoui (Bejaoui2020_M963; Betreuer: Moritz Schappler), 2020-04
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

if isempty(which('serroblib_path_init.m'))
  warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end
if isempty(which('parroblib_path_init.m'))
  warning('Repo mit parallelen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end

for robnr = 1:3
  if robnr == 1
    RS1 = serroblib_create_robot_class('S5RRRRR12V1');% passive Fuehrungsbeinkette US , Denavit-Hartenberg falsch
    RS2 = serroblib_create_robot_class('S6PRRRRR6'); % Folgebeinketten (P)US
    Name = '6UPS/US';
    % ParRob-Klasse fuer PKM erstellen
    RP = ParRob('OneUS_FivePUS');
    % Beinketten definieren
    RP.NLEG = 6;
    RP.Leg = copy(RS1); % Fuehrungsbeinkette RRS
    pkin_RS1 = zeros(length(RS1.pkin),1);
    pkin_RS1(strcmp(RS1.pkin_names, 'a2')) = 0.5;
    pkin_RS1(strcmp(RS1.pkin_names, 'a3')) = 1.5;
    pkin_RS1(strcmp(RS1.pkin_names, 'alpha2')) = pi/2;
    RP.Leg(1).update_mdh(pkin_RS1);
    for ii=2:RP.NLEG % Folgebeinketten 4 PUS
      RP.Leg(ii) = copy(RS2);
    end
    pkin_RS2 = zeros(length(RS2.pkin),1);
    pkin_RS2(strcmp(RS2.pkin_names, 'a4')) = 1.0;
    pkin_RS2(strcmp(RS2.pkin_names, 'alpha3')) = pi/2;
    for ii=2:RP.NLEG % Folgebeinketten 4 UPS( vielleicht Verallgemeinerung fuer spaeter)
      RP.Leg(ii).update_mdh(pkin_RS2(1:size(RP.Leg(ii).pkin),1));
    end
    RP.align_base_coupling(8, [0.5;0.3;0.2]);  % Wahl ist wichtig fuer Loesbarkeit der IK
    RP.align_platform_coupling(4, [0.2;0.1]); % Wahl ist wichtig fuer Loesbarkeit der IK
    RP.initialize();
    I_qa_Typ1 = zeros(1,5); % keine Aktuierung fuer passives Fuehrungsbein
    I_qa_Typ2 = zeros(1,6); % Folgebeinketten
    I_qa_Typ2(1) = 1;  % das erstes Gelenk (P) ist aktuiert
    I_qa = [I_qa_Typ1,repmat(I_qa_Typ2,1,5)];
  elseif robnr == 2
    RS1 = serroblib_create_robot_class('S5RRRRR2');% passive Fuehrungsbeinkette RUU
    RS2 = serroblib_create_robot_class('S6RRPRRR14V3'); % Folgebeinketten U(P)S
    Name = '6UPS/RUU';
    % ParRob-Klasse fuer PKM erstellen
    RP = ParRob('OneRUU_FiveUPS');
    % Beinketten definieren
    RP.NLEG = 6;
    RP.Leg = copy(RS1); % Fuehrungsbeinkette RUU
    pkin_RS1 = zeros(length(RS1.pkin),1);
    pkin_RS1(strcmp(RS1.pkin_names, 'a2')) = 0.5;
    pkin_RS1(strcmp(RS1.pkin_names, 'a4')) = 1.0;
    RP.Leg(1).update_mdh(pkin_RS1(1:size(RP.Leg(1).pkin),1));
    % (Beinkette 2 hat keine Kinematik-Parameter)
    for ii=2:RP.NLEG % Folgebeinketten 5 UPS
      RP.Leg(ii) = copy(RS2);
    end
    
    RP.Leg(1).update_base( [0.5;0;0], r2eulxyz(rotx(pi/2)*roty(0))); % Fuehrungsbeinkette UPU
    RP.Leg(2).update_base( [-2;0;0], r2eulxyz(rotx(0)*rotz(0))); % Folgebeinkette UPS
    RP.Leg(3).update_base( [-3;2;0], r2eulxyz(rotx(0)*rotz((0)))); % Folgebeinketten UPS
    RP.Leg(4).update_base( [3;1;0], r2eulxyz(rotx(0)*rotz(0))); % Folgebeinkette UPS
    RP.Leg(5).update_base( [3;-1;0], r2eulxyz(rotx(0)*rotz(0))); % Folgebeinkette UPS
    RP.Leg(6).update_base( [-3;-2;0], r2eulxyz(rotx(0)*rotz(0))); % Folgebeinkette UPS
    % Reihenfolge wie im Bild
    RP.r_P_B_all(:,1) = rotz(0)*[0.5;0;0]; %schon
    RP.r_P_B_all(:,2) = rotz(0)*[-2;0;0]; %schon
    RP.r_P_B_all(:,3) = rotz(0)*[-1;0.7;0];
    RP.r_P_B_all(:,6) = rotz(0)*[-1;-0.7;0];
    RP.r_P_B_all(:,4) = rotz(0)*[2;1.5;0]; %schon
    RP.r_P_B_all(:,5) = rotz(0)*[2;-1.5;0]; %schon
    RP.initialize();
    for ii = 1:RP.NLEG
      RP.Leg(ii).update_EE(zeros(3,1),zeros(3,1));
    end
    I_qa_Typ1 = zeros(1,5); % keine Aktuierung fuer passives Fuerhungsbein 
    I_qa_Typ2 = zeros(1,6); % Folgebeinketten
    I_qa_Typ2(3) = 1;  % das 3te Gelenk P ist aktuiert
    I_qa = [I_qa_Typ1,repmat(I_qa_Typ2,1,5)];
  elseif robnr == 3
    RS1 = serroblib_create_robot_class('S5RRRRR12V1');% passive Fuehrungsbeinkette US , Denavit-Hartenberg falsch
    RS2 = serroblib_create_robot_class('S6RRPRRR14V3'); % Folgebeinketten U(P)S
    Name = '6UPS/US';
    % ParRob-Klasse fuer PKM erstellen
    RP = ParRob('OneUS_FiveUPS');
    % Beinketten definieren
    RP.NLEG = 6;
    RP.Leg = copy(RS1); % Fuehrungsbeinkette RRS
    pkin_RS1 = zeros(length(RS1.pkin),1);
    pkin_RS1(strcmp(RS1.pkin_names, 'a2')) = 0.5;
    pkin_RS1(strcmp(RS1.pkin_names, 'a3')) = 1.5;
    pkin_RS1(strcmp(RS1.pkin_names, 'alpha2')) = pi/2;
    RP.Leg(1).update_mdh(pkin_RS1(1:size(RP.Leg(1).pkin),1));
    for ii=2:RP.NLEG % Folgebeinketten 5 UPS
      RP.Leg(ii) = copy(RS2);
    end
    RP.align_base_coupling(8, [0.5;0.3;0.2]);  % Wahl ist wichtig fuer Loesbarkeit der IK
    RP.align_platform_coupling(4, [0.2;0.1]); % Wahl ist wichtig fuer Loesbarkeit der IK
    RP.initialize(); 

    I_qa_Typ1 = zeros(1,5); % keine Aktuierung fuer passives Fuehrungsbein
    I_qa_Typ2 = zeros(1,6); % Folgebeinketten
    I_qa_Typ2(3) = 1;  % das 3te Gelenk P ist aktuiert
    I_qa = [I_qa_Typ1,repmat(I_qa_Typ2,1,5)];
    for ii = 1:RP.NLEG
      RP.Leg(ii).update_EE(zeros(3,1),zeros(3,1));
    end
  end
  serroblib_update_template_functions({RS1.mdlname, RS2.mdlname});
  RP.fill_fcn_handles(false, true);
  RP.update_actuation(I_qa);
  % Gelenkgrenzen setzen, damit IK-Neuversuche möglich sind
  for ii = 1:RP.NLEG
    RP.Leg(ii).qlim(RP.Leg(ii).MDH.sigma==0,:) = ...
      repmat([-pi,pi],sum(RP.Leg(ii).MDH.sigma==0),1);
    RP.Leg(ii).qlim(RP.Leg(ii).MDH.sigma==1,:) = ...
      repmat([-1,3],sum(RP.Leg(ii).MDH.sigma==1),1);
  end
  % allgemeine Einstellungen
  RP.Leg(1).I_EE = logical([1 1 1 1 1 0]);
  RP.update_EE_FG(logical([1 1 1 1 1 0])); % 3T2R; letzter Eintrag Null, da beta_3 weg
  % Startpose
  X0 = [[0.1;0.2;1.2];[5;10;-10]*pi/180]; % Plattform nur verdrehbar, keine Kipp-bwg
  q0 = (-0.5+rand(RP.NJ,1))*2*60*pi/180; % zwischen -60° und +60°
  q0(RP.MDH.sigma==1) = 0.4; % Schubgelenke eher positiv wählen
  fprintf('3T2R-PKM mit passiver Führungskette %d/%d: %s\n', robnr, 3, Name);
  %% IK berechnen
  [q,phi] = RP.invkin_ser(X0, q0);
  [Phi3_red,Phi3_voll] = RP.constr3(q, X0); % mit Fuehrungsbeinkette
  assert(all(size(Phi3_red)==[35,1]), 'Ausgabe 1 von constr3 muss 35x1 sein');
  assert(all(size(Phi3_voll)==[36,1]), 'Ausgabe 2 von constr3 muss 36x1 sein');
  % Letzten Euler-Winkel auf den tatsächlichen Wert setzen.
  X0(6) = X0(6) + Phi3_voll(4);
  % Aufruf der ZB-Modellierung 2. Ist für diese Art von Roboter aber
  % nicht zielführend.
  [Phi2_red,Phi2_voll] = RP.constr2(q, X0);
  assert(all(size(Phi2_red)==[36,1]), 'Ausgabe 1 von constr2 muss 36x1 sein');
  assert(all(size(Phi2_voll)==[36,1]), 'Ausgabe 2 von constr2 muss 36x1 sein');
  %% Roboter zeichnen
  figure(10+robnr);clf;
  hold on; grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
  view(3);
  s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I2L_LEG], 'straight', 0);
  RP.plot( q, X0, s_plot );
  hold off;
  %% Differentielle Kinematik über constr3
  % Siehe [Bejaoui2020_M963] Kap. 4.2.
  % Jacobi q-Anteile
  [G_q,G_q_voll] = RP.constr3grad_q(q, X0); % automatisches herausnehmen
  assert(all(size(G_q)==[35 35]), 'ZB-matrix G_q muss 35x35 sein');
  assert(all(size(G_q_voll)==[36 35]), 'ZB-matrix G_q_voll muss 36x35 sein');
  
  % Jacobi x-Anteile ( dim bei G_x_red von constr3grad noch nicht richtig)
  [G_x_red,G_x_voll] = RP.constr3grad_x(q, X0); % automatisches herausnehmen
  % Vorgehen von Li fuer x-Anteile
  G_x = G_x_red;
  G_eta = G_x_red(:,RP.I_EE_Task); % manuelles herausnehmen

  % Aufteilung der Ableitung nach den Gelenken in Gelenkklassen
  G_a = G_q(:,RP.I_qa); % aktiv, phi_dqa [STO19]
  G_d = G_q(:,RP.I_qd); % passiv, phi_dpa [STO19]
  % Jacobi-Matrix zur Berechnung der abhaengigen Gelenke und EE-Koordinaten,
  % hier noch nicht quadratisch 30x29
  G_dx = [G_d, G_x]; % Gl. 50, phi_dxp, hier p-Anteile ueber x-Anteilen [STO19]
  J_voll1 = G_dx \ G_a; % inv(G_dx) * G_a = inv(phi_dxp)*phi_dqa, Gl. 50 vollstaendige Jacobi-Matrix bezogen auf x-Koordinaten [STO19]
  Jinv_voll1 = G_q \ G_x; % vollstaendige inverse Jacobi-Matrix in x-Koord
  
  % Jacobi-Matrix zur Berechnung der abhaengigen Gelenke und EE-Koordinaten,
  % hier quadratisch 29x29,  eta und nicht x fuer EE
  G_deta = [G_d, G_eta]; % Gl. 50, phi_dxp, hier p-Anteile ueber x-Anteilen [STO19]
  J_voll2 = G_deta \ G_a; % inv(G_dx) * G_a = inv(phi_dxp)*phi_dqa, Gl. 50, Jacobi-Matrix bezogen auf eta-Koordinaten [STO19]
  Jinv_voll2 = G_q \ G_eta; % Reduzierte inverse Jacobi-Matrix in eta-Koord
  
  J_qa_eta = Jinv_voll2(RP.I_qa,:); % Inverse Jacobi-Matrix des Roboters (eta-Koord)
  J_eta_qa = J_voll2(sum(RP.I_qd)+1:end,:); % Jacobi-Matrix (wird mit qaD multi.)
  assert(all(size(J_qa_eta)==[5 5]), 'inverse Jacobi-Matri muss 5x5 sein');
  assert(all(size(J_eta_qa)==[5 5]), 'Jacobi-Matri muss 5x5 sein');
  
  % Prüfe inverse Jacobi-Matrix gegen nicht-invertierte
  matrix_test = J_eta_qa*J_qa_eta - eye(5); % 5x5
  if any(abs(matrix_test(:)) > 1e-4)
    error('Jacobi-Matrix und ihre Inverse passen nicht zueinander');
  end

  %% Trajektorie berechnen
  k=1; XE = X0';
  d1=0.1;
  h1=0.1;
  r1=10*pi/180;
  k=k+1; XE(k,:) = XE(k-1,:) + [0,0,0  r1,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,0, 0, 0,-r1,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,0,0, 0,r1,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,0,0, -r1,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [d1,0,0, 0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [-d1,0,0, -0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,d1,0, 0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,-d1,0, -0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,0,h1, 0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,-0,-h1, -0,0,0];
  [X,XD,XDD,T] = traj_trapez2_multipoint(XE, 1, 2e-1, 1e-1, 5e-3, 0);
  % Inverse Kinematik berechnen
  t1 = tic();
  fprintf('Berechne Trajektorien-IK für %d Zeitschritte\n', length(T));
  iksettings = struct('n_max', 5000, 'Phit_tol', 1e-8, 'Phir_tol', 1e-8, 'debug', true, ...
    'retry_limit', 0, 'mode_IK', 1, 'normalize', false);
  [Q, QD, QDD, Phi] = RP.invkin_traj(X,XD,XDD,T,q,iksettings); % hier muessen einige Zeilen auskommentiert werden
  fprintf('Trajektorien-IK in %1.2fs berechnet. Prüfe die Ergebnisse.\n', toc(t1));
  %% Trajektorie prüfen
  % Tatsächliche EE-Koordinaten mit vollständiger direkter Kinematik bestimmen
  for i = 1:length(T)
    if max(abs( Phi(i,:) )) > max(iksettings.Phit_tol,iksettings.Phir_tol) || any(isnan( Phi(i,:) ))
      error(['IK stimmt nicht bei i=%d/%d (%1.3fs/%1.3fs). Wahrscheinliche ', ...
        'Ursache: Ist innerhalb von n_max nicht konvergiert'], i, length(T), T(i), T(end));
    end
    % Direkte Kinematik berechnen
    Tc_ges = RP.fkine(Q(i,:)', NaN(6,1));
    % Schnitt-KS aller Beinketten bestimmen
    II_BiKS = 1;
    for j = 1:RP.NLEG
      II_BiKS = II_BiKS + RP.Leg(j).NL+1;
      T_Bi = Tc_ges(:,:,II_BiKS);
      R_0_Bj = T_Bi(1:3,1:3);
      R_Bj_P = eulxyz2r(RP.phi_P_B_all(:,j))';
      R_0_P = R_0_Bj * R_Bj_P;
      r_0_0_Bj = T_Bi(1:3,4);
      r_P_P_Bj = RP.r_P_B_all(:,j);
      r_0_Bj_P = -R_0_P*r_P_P_Bj;
      r_0_Ej = r_0_0_Bj + r_0_Bj_P;
      R_0_E = R_0_P * RP.T_P_E(1:3,1:3);
      if j == 1
        % die EE-Position aus der ersten Kette muss auch für folgende
        % Ketten eingehalten werden. Daher Speicherung.
        r_0_E_Legs = r_0_Ej;
        R_0_E_Legs = R_0_E;
        x_Legs = RP.t2x(rt2tr(R_0_E, r_0_Ej));
      else
        test_eepos = r_0_E_Legs-r_0_Ej;
        if any(abs(test_eepos)>2e-6) % muss größer als IK-Toleranz sein
          error('i=%d: EE-Position aus Beinkette %d stimmt nicht mit Beinkette 1 überein. Fehler %1.2e', i, j, max(abs(test_eepos)));
        end
        test_eerot = R_0_E_Legs\R_0_E-eye(3);
        if any(abs(test_eerot(:)) > 1e-6)
          error('i=%d: EE-Rotation aus Beinkette %d stimmt nicht mit Beinkette 1 überein. Fehler %1.2e', i, j, max(abs(test_eerot(:))));
        end
        test_x = x_Legs - RP.t2x(rt2tr(R_0_E, r_0_Ej));
        if any(abs(test_x(:)) > 1e-6)
          error('i=%d: EE-Koordinaten x aus Beinkette %d stimmt nicht mit Beinkette 1 überein. Fehler %1.2e', i, j, max(abs(test_x(:))));
        end
      end
      % Speichere letzten Euler-Winkel aus der ersten Beinkette
      X(i,6) = x_Legs(6);
    end
    % Neues xD berechnen
    XD(i,6) = 0; % Auf Null setzen, damit Aufruf auch mehrfach funktioniert.
    [~,Phi2D]=RP.constr2D(Q(i,:)', QD(i,:)', X(i,:)', XD(i,:)');
    XD(i,6) = Phi2D(4);

    % Berechne die Zeitableitung aller Zwangsbedingungs-Modellierungen
    % Alle müssen Null sein. Ansonsten bewegt sich die PKM auseinander.
    [~,Phi1D]=RP.constr1D(Q(i,:)', QD(i,:)', X(i,:)', XD(i,:)');
    if any(abs(Phi1D)>1e-2)
      error('Geschwindigkeit der Koppelpunkte mit constr1 ist nicht konsistent. Fehler %1.2e.', max(abs(Phi1D)));
    end
    [~,Phi2D]=RP.constr2D(Q(i,:)', QD(i,:)', X(i,:)', XD(i,:)');
    if any(abs(Phi2D)>1e-2)
      error('Geschwindigkeit der Koppelpunkte mit constr2 ist nicht konsistent. Fehler %1.2e.', max(abs(Phi2D)));
    end
    [~,Phi4D]=RP.constr4D(Q(i,:)', QD(i,:)', X(i,:)', XD(i,:)');
    if any(abs(Phi4D)>1e-2)
      error('Geschwindigkeit der Koppelpunkte mit constr4 ist nicht konsistent. Fehler %1.2e.', max(abs(Phi4D)));
    end
  end
  % Funktion für direkte Kinematik nochmal testen
  for j = 1:RP.NLEG
    [X3,XD3,XDD3] = RP.fkineEE_traj(Q, QD, QDD, j);
    % Die Beschleunigung des dritten Euler-Winkels wurde noch nicht
    % korrigiert. Wird basierend auf erster Beinkette gemacht.
    if j == 1
      XDD(:,6) = XDD3(:,6);
    end
    test_X = X(:,1:6) - X3(:,1:6);
    test_XD = XD(:,1:6) - XD3(:,1:6);
    test_XDD = XDD(:,1:6) - XDD3(:,1:6);
    if max(abs(test_X(:)))>1e-6
      Ifirst = find(any(abs(test_X)>1e-6,2), 1, 'first');
      error('Die Endeffektor-Trajektorie X aus Beinkette %d stimmt nicht gegen Beinkette 1. Erstes Vorkommnis: Zeitschritt %d', j, Ifirst);
    end
    if max(abs(test_XD(:)))>1e-6
      Ifirst = find(any(abs(test_XD)>1e-6,2), 1, 'first');
      error('Die Endeffektor-Trajektorie XD aus Beinkette %d stimmt nicht gegen Beinkette 1. Erstes Vorkommnis: Zeitschritt %d', j, Ifirst);
    end
    if max(abs(test_XDD(:)))>1e-6
      Ifirst = find(any(abs(test_XDD)>1e-6,2), 1, 'first');
      error('Die Endeffektor-Trajektorie XDD aus Beinkette %d stimmt nicht gegen Beinkette 1. Erstes Vorkommnis: Zeitschritt %d', j, Ifirst);
    end
  end
  Q_int = repmat(Q(1,:),length(T),1)+cumtrapz(T, QD);
  figure(20+robnr);clf;
  for i = 1:RP.NLEG
    for j = 1:RP.Leg(1).NJ
      subplot(RP.NLEG,6, sprc2no(RP.NLEG, 6, i, j)); hold all;
      hdl1=plot(T, Q    (:,RP.I1J_LEG(i)+j-1));
      hdl2=plot(T, Q_int(:,RP.I1J_LEG(i)+j-1));
      grid on;
      ylabel(sprintf('Leg %d, Joint %d', i, j));
    end
  end
  sgtitle('Konsistenz q-qD');
  legend([hdl1;hdl2], {'q', 'int(qD)'});
  linkxaxes;
  test_q_qD_kons = Q_int - Q;
  if max(abs(test_q_qD_kons(:))) > 1e-2
    error('Q und QD aus Traj.-IK sind nicht konsistent');
  end
  %% Animation
  rob_path = fileparts(which('robotics_toolbox_path_init.m'));
  resdir = fullfile(rob_path, 'examples_tests', 'results');
  mkdirs(resdir);
  s_anim = struct('gif_name', fullfile(resdir, sprintf('3T2R_PKM_PassivBK_%s.gif',RP.mdlname)));
  fhdl=figure(2);clf;
  set(fhdl,'units','normalized','outerposition',[0 0 1 1],'color','w'); % Vollbild
  hold on;
  plot3(X(:,1), X(:,2), X(:,3));
  grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
  view(3);
  title(Name);
  RP.anim( Q(1:20:length(T),:), X(1:20:length(T),:), s_anim, s_plot);
end
