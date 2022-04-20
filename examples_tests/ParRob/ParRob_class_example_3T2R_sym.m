% Beispielskript für symmetrische 3T2R-PKM
% 
% Ergebnis des Beispielskripts:
% * Nachweis, dass diese Art von PKM kinematisch modellierbar ist
% * PKM Nr. 1 (5UPU) funktioniert nicht (siehe [HL02])
% * PKM Nr. 3 (5RPUR) funktionert (siehe [TG11])
%
% Offene Punkte und Besonderheiten:
% * Die PKM werden ohne die Transformation Bi-P für die PKM-Plattform
%   modelliert. Das Vorgehen ist also leicht anders als in ParRobLib
% 
% Quellen:
% [STO19] Schappler, M. et al.: Modeling Parallel Robot Kinematics for 3T2R
% and 3T3R Tasks using Reciprocal Sets of Euler Angles, MDPI Robotics KaRD2
% [TG11] Tale-Masouleh, M.; Gosselin, C.: Singularity analysis of 5-RPUR
% parallel mechanisms (3T2R)
% [HL02] Z. Huang, Q. C. Li: General Methodology for Type Synthesis of
% Symmetrical Lower-Mobility Parallel Manipulators and Several Novel Manipulators
% [Bejaoui2020_M963] Bejaoui, Abderahman: Modellierung und Struktursynthese 
% paralleler Roboter mit Freiheitsgrad Fünf

% MA Bejaoui (Bejaoui2020_M963; Betreuer: Moritz Schappler), 2020-04
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

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

for robnr = 1:3 % 5_UPU, 5_RUU, 5_RPUR;
  if robnr == 1 % 5_UPU
    RS = serroblib_create_robot_class('S5RRPRR12');
    RP = ParRob('P5_UPU_3T2R');
    RP.create_symmetric_robot(5, RS, 1.1, 0.3);
    pkin_all = zeros(8,5);
    for ii=1:RP.NLEG
      RP.Leg(ii).update_mdh(pkin_all(1:size(RP.Leg(ii).pkin),1));
    end
    I_qa_Typ = false(5,1);
    I_qa_Typ(3) = 1; % das dritte Gelenk (P) ist aktuiert
  elseif robnr == 2 % 5_RUU
    RS = serroblib_create_robot_class('S5RRRRR2');
    RP = ParRob('P5_RUU_3T2R');
    RP.create_symmetric_robot(5, RS, 1.5, 0.5);
    pkin_all  = [1.0, 1.5]';
    for ii=1:RP.NLEG
      RP.Leg(ii).update_mdh(pkin_all(1:size(RP.Leg(ii).pkin),1));
    end
    I_qa_Typ = false(5,1);
    I_qa_Typ(1) = 1; % das erste Gelenk (P) ist aktuiert
  elseif robnr == 3 % 5_RPUR; siehe [TG11]
    RS = serroblib_create_robot_class('S5RPRRR8V1');
    RP = ParRob('P5_RPUR_3T2R');
    RP.NLEG = 5;
    RP.Leg = copy(RS);
    for ii=2:RP.NLEG
      RP.Leg(ii) = copy(RS);
    end
    RP.initialize();
    pkin = zeros(length(RS.pkin),1);
    pkin(strcmp(RS.pkin_names, 'a5')) = 0.9;
    for ii=1:RP.NLEG
      RP.Leg(ii).update_mdh(pkin);
      RP.Leg(ii).update_EE(zeros(3,1),r2eulxyz(roty(pi/2)));
    end
    % Orientierung der Gestell-Koppelpunkt-KS
    RP.Leg(1).update_base( [0.8;0;0], r2eulxyz(rotx(pi/2)*rotz(pi/2))); % Fuehrungsbeinkette RPUR
    RP.Leg(2).update_base( [0.4;0.6;0], r2eulxyz(rotx((pi)/2)*rotz(pi/2))); % Folgebeinketten RPUR
    RP.Leg(3).update_base( [-0.4;0.6;0], r2eulxyz(rotx(pi/2)*rotz(pi/2))); % Folgebeinkette RPUR
    RP.Leg(4).update_base( [-0.4;-0.6;0], r2eulxyz(rotx(pi/2)*rotz(pi/2))); % Folgebeinkette RPUR
    RP.Leg(5).update_base( [0.4;-0.6;0], r2eulxyz(rotx(pi/2)*rotz(pi/2))); % Folgebeinkette RPUR
    % Reihenfolge wie im Bild in [TG11]
    RP.r_P_B_all(:,1) = [0.4;0;0];
    RP.r_P_B_all(:,2) = [0.2;0.3;0];
    RP.r_P_B_all(:,3) = [-0.2;0.3;0];
    RP.r_P_B_all(:,4) = [-0.2;-0.3;0];
    RP.r_P_B_all(:,5) = [0.2;-0.3;0];
    RP.phi_P_B_all(:) = 0;
    
    I_qa_Typ = zeros(5,1);
    I_qa_Typ(2) = 1; % das 2te Gelenk P ist aktuiert
  elseif robnr == 5 % 5_RPUR; siehe [TG11]
    RS = serroblib_create_robot_class('S5RPRRR10V1');
    pkin = zeros(length(RS.pkin),1);
    pkin(strcmp(RS.pkin_names, 'a5')) = 1.4;
    RS.update_mdh(pkin);
    RP = ParRob('P5RPUR_3T2R');
    RP.create_symmetric_robot(5, RS);
    RP.initialize();
    RP.align_base_coupling(1, 0.8);
    RP.align_platform_coupling(8, 0.4);
    I_qa_Typ = zeros(5,1); % Typ1: S5PRRRR1
    I_qa_Typ(2) = 1; % das 2te Gelenk P ist aktuiert
  end
  assert(~any(isnan(RP.phi_P_B_all(:))), 'Strukturparameter sind NaN. Falsche Initialisierung');
  serroblib_update_template_functions({RS.mdlname});
  RP.fill_fcn_handles(false, true);
  RP.update_actuation(repmat(I_qa_Typ,5,1));
  % qlim setzen, damit Neuversuche in IK besser funktionieren
  for i = 1:RP.NLEG
    % Begrenze die Winkel der Kugel- und Kardangelenke auf +/- 360°
    RP.Leg(i).qlim = repmat([-2*pi, 2*pi], RP.Leg(i).NQJ, 1);
    % Begrenze die Länge der Schubgelenke
    if any(RP.Leg(i).MDH.sigma)
      RP.Leg(i).qlim(RP.Leg(i).MDH.sigma==1,:) = [0.1, 2.0];
    end
  end
  % Setze EE-FG der Beinketten auf 3T2R: Entspricht den strukturellen FG
  % der Beinketten. Die Eigenschaft I_EE_Task ist trotzdem 3T3R (für IK).
  for ii=1:RP.NLEG
    RP.Leg(ii).I_EE = logical([1 1 1 1 1 0]);
  end
  % allgemeine Einstellungen
  RP.update_EE_FG([1 1 1 1 1 0]); % Strukturelle FG der PKM
  % Startpose
  X0 = [[0.1;0.1;1.0];[0;0;0]*pi/180]; % Plattform nur verdrehbar, keine Kipp-bwg
  % Anfangswerte für IK durch ausprobieren
  if robnr == 3
    q0 = rand(RP.NJ,1);
  else
    q0 = pi/2*2*(-0.5+rand(RP.NJ,1)); % Zufallswinkel zwischen -90° und +90°)
  end
  q0(RP.MDH.sigma==1) = 0.5; % positive Längen
  q0(RP.I1J_LEG(2):end) = NaN; % Setze NaN, damit Beinketten 2-5 das Ergebnis von Beinkette 1 als Startwert bekommen.
  % Indizes für die Zwangsbedingungen (sind für constr3 auch in der ParRob
  % Klasse implementiert. Für constr2 nicht.)
  I_constr2 = 1:6*RP.NLEG;
  I_constr2(4:6:end) = []; % immer die vierte Komponente entfernen (reziproker Winkel)
  I_constr3 = 1:6*RP.NLEG;
  I_constr3(4) = [];
  %% IK berechnen
  X0(6) = 0;
  [q, phi] = RP.invkin_ser(X0, q0);
  [Phi3_red,Phi3_voll] = RP.constr3(q, X0);
  assert(all(size(Phi3_red)==[29,1]), 'Ausgabe 1 von constr3 muss 29x1 sein');
  assert(all(size(Phi3_voll)==[30,1]), 'Ausgabe 2 von constr3 muss 30x1 sein');
  % Letzten Euler-Winkel auf den tatsächlichen Wert setzen. Dieser ist in
  % der Zwangsbedingung enthalten.
  X0(6) = X0(6) + Phi3_voll(4);
  % Berechne die Zwangsbedingungen nochmal neu mit korrigiertem X(6)
  [~,Phi3_voll] = RP.constr3(q, X0); % mit Fuehrungsbeinkette
  [Phi2_red,Phi2_voll] = RP.constr2(q, X0);
  assert(all(size(Phi2_red)==[25,1]), 'Ausgabe 1 von constr2 muss 25x1 sein');
  assert(all(size(Phi2_voll)==[30,1]), 'Ausgabe 2 von constr2 muss 30x1 sein');
  
  figure(10+robnr);clf;
  hold on; grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
  view(3);
  s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I2L_LEG], 'ks_platform', 1:6, 'straight', 0);
  RP.plot(q, X0, s_plot );
  title(RP.mdlname, 'interpreter', 'none');
  if any(abs(phi) > 1e-6)
    error('IK konnte für %s nicht erfolgreich berechnet werden.', RP.mdlname);
  end
  if all(abs(Phi3_voll) < 1e-6) && all(abs(Phi2_voll) < 1e-6)
    fprintf('Rob %d. %s: Die ZB-Modellierung 2 und 3 ergibt eine korrekte PKM\n', robnr, RP.mdlname);
  end
  %% Differentielle Kinematik über constr3
  % Siehe [Bejaoui2020_M963] Kap. 4.3.3. Dort wird der Nachteil der 29x25
  % Gradientenmatrix beschrieben. Das Ergebnis ist für funktionierende PKM
  % identisch zur Modellierung constr2.
  [G_q_red_3,G_q_voll_3] = RP.constr3grad_q(q, X0);
  [G_x_red_3,G_x_voll_3] = RP.constr3grad_x(q, X0);
  assert(all(size(G_q_red_3)==[29,25]), 'Ausgabe 1 von constr3grad_q muss 29x25 sein');
  assert(all(size(G_q_voll_3)==[30,25]), 'Ausgabe 2 von constr3grad_q muss 30x25 sein');
  assert(all(size(G_x_red_3)==[29,5]), 'Ausgabe 1 von constr3grad_x muss 29x5 sein');
  assert(all(size(G_x_voll_3)==[30,6]), 'Ausgabe 2 von constr3grad_x muss 30x6 sein');
  G_q_3 = G_q_voll_3(I_constr3,:);
  G_x_3 = G_x_voll_3(I_constr3,1:5);
  Jinv_3 = G_q_3 \ G_x_3;
  J_qa_x_3 = Jinv_3(RP.I_qa,:);
  assert(all(size(J_qa_x_3)==[5 5]), 'inverse Jacobi-Matri muss 5x5 sein');
  fprintf('Rob %d. %s: Rang der %dx%d Jacobi der aktiven Gelenke: %d/%d. Konditionszahl: %1.2e\n', ...
    robnr, RP.mdlname, size(J_qa_x_3,1), size(J_qa_x_3,2), rank(J_qa_x_3), sum(RP.I_EE), cond(J_qa_x_3));
  G_a_3 = G_q_3(:,RP.I_qa); % aktiv, phi_dqa [STO19]
  G_d_3 = G_q_3(:,RP.I_qd); % passiv, phi_dpa [STO19]
  % Jacobi-Matrix zur Berechnung der abhaengigen Gelenke und EE-Koordinaten
  G_dxp_3 = [G_d_3, G_x_3]; % Gl. 50, phi_dxp, hier p-Anteile ueber x-Anteilen [STO19]
  J_3 = G_dxp_3 \ G_a_3; % inv(G_dx) * G_a = inv(phi_dxp)*phi_dqa, Gl. 50 vollstaendige Jacobi-Matrix bezogen auf x-Koordinaten [STO19]
  % Pruefe inverse Jacobi-Matrix gegen nicht-invertierte
  if cond(J_qa_x_3) < 1e4
    % Funktioniert nur, wenn die Konditionszahl halbwegs gut ist
    J_x_qa_3 = J_3(sum(RP.I_qd)+1:end,:);
    matrix_test_3_debug = J_qa_x_3*J_x_qa_3- eye(5);
    matrix_test_3 = J_x_qa_3*J_qa_x_3- eye(5);
    if any(abs(matrix_test_3(:)) > 1e-4)
      error('constr3grad: Jacobi-Matrix und ihre Inverse passen nicht zueinander');
    end
  else
    if any(robnr==[1 2])
      addstr = sprintf('Dieses Verhalten ist erwartet.');
    else
      addstr = '';
    end
    fprintf(['Rob %d. %s: Die Konditionszahl ist zu groß (%1.2e). Eine Berechnung der Traj', ...
      'ektorie ist nicht möglich. %s\n'], robnr, RP.mdlname, cond(J_qa_x_3), addstr);
    continue;
  end
  %% Differentielle Kinematik über constr2
  % Wird in [Bejaoui2020_M963] Kap. 4.3.1 beschrieben.
  [G_q_red_2,G_q_voll_2] = RP.constr2grad_q(q, X0);
  [G_x_red_2,G_x_voll_2] = RP.constr2grad_x(q, X0);
  assert(all(size(G_q_red_2)==[25,25]), 'Ausgabe 1 von constr2grad_q muss 25x25 sein');
  assert(all(size(G_q_voll_2)==[30,25]), 'Ausgabe 2 von constr2grad_q muss 30x25 sein');
  assert(all(size(G_x_red_2)==[25,5]), 'Ausgabe 1 von constr2grad_x muss 25x5 sein');
  assert(all(size(G_x_voll_2)==[30,6]), 'Ausgabe 2 von constr2grad_x muss 30x6 sein');
  G_q_2 = G_q_voll_2(I_constr2,:);
  G_x_2 = G_x_voll_2(I_constr2,1:5);
  Jinv_2 = G_q_2 \ G_x_2; % vollstaendige inverse Jacobi-Matrix in x-Koord Gl. 49
  J_qa_x_2 = Jinv_2(RP.I_qa,:);
  test_Jinv_23 = Jinv_2 - Jinv_3;
  if any(abs(test_Jinv_23(:))>1e-6)
    % Wenn die Methoden nicht übereinstimmen, ist der dritte Euler-Winkel
    % bei den verschiedenen Beinketten nicht gleich.
    error('inverse Jacobi-Matrix stimmt nicht zwischen Methode 2 und 3 überein. PKM funktioniert nicht.');
  end

  %% Trajektorie berechnen
  % Einmal in jede Koordinatenrichtung einzeln bewegen
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
  iksettings = struct('n_max', 5000, 'Phit_tol', 1e-8, 'Phir_tol', 1e-8, 'debug', true, ...
    'retry_limit', 0, 'mode_IK', 1, 'normalize', false);
  [Q, QD, QDD, Phi] = RP.invkin_traj(X,XD,XDD,T,q,iksettings); % hier muessen einige Zeilen auskommentiert werden
  fprintf('Trajektorien-IK in %1.2fs berechnet. Prüfe die Ergebnisse.\n', toc(t1));

  %% Trajektorie prüfen
  t1 = tic();
  % Tatsächliche EE-Koordinaten mit vollständiger direkter Kinematik bestimmen
  for i = 1:length(T)
    if max(abs( Phi(i,:) )) > max(iksettings.Phit_tol,iksettings.Phir_tol) || any(isnan( Phi(i,:) ))
      warning(['IK stimmt nicht bei i=%d/%d (%1.3fs/%1.3fs). Wahrscheinliche ', ...
        'Ursache: Ist innerhalb von n_max nicht konvergiert'], i, length(T), T(i), T(end));
      return
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
    if any(abs(Phi1D)>1e-6)
      error('Geschwindigkeit der Koppelpunkte mit constr1 ist nicht konsistent. Fehler %1.2e.', max(abs(Phi1D)));
    end
    [~,Phi2D]=RP.constr2D(Q(i,:)', QD(i,:)', X(i,:)', XD(i,:)');
    if any(abs(Phi2D)>1e-6)
      error('Geschwindigkeit der Koppelpunkte mit constr2 ist nicht konsistent. Fehler %1.2e.', max(abs(Phi2D)));
    end
    [~,Phi4D]=RP.constr4D(Q(i,:)', QD(i,:)', X(i,:)', XD(i,:)');
    if any(abs(Phi4D)>1e-6)
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
  % Prüfe die Konsistenz von Geschwindigkeit und Position der Gelenke
  Q_int = repmat(Q(1,:),length(T),1)+cumtrapz(T, QD);
  figure(20+robnr);clf;
  for i = 1:RP.NLEG
    for j = 1:RP.Leg(1).NJ
      subplot(RP.NLEG,5, sprc2no(RP.NLEG, 5, i, j)); hold all;
      hdl1=plot(T,     Q(:,RP.I1J_LEG(i)+j-1));
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
  % Prüfe die Konsistenz von Geschwindigkeit und Position der Plattform
  X_int = repmat(X(1,:),length(T),1)+cumtrapz(T, XD);
  figure(30+robnr);clf;
  for i = 1:6
    subplot(2,3,i); hold all;
    hdl1=plot(T,     X(:,i));
    hdl2=plot(T, X_int(:,i));
    grid on;
    ylabel(sprintf('x %d', i));
  end
  sgtitle('Konsistenz x-xD');
  legend([hdl1;hdl2], {'x', 'int(xD)'});
  linkxaxes;
  fprintf('Prüfung der Trajektorie abgeschlossen (Dauer: %1.2f). Erstelle Animation.\n', toc(t1));
  %% Animation
  rob_path = fileparts(which('robotics_toolbox_path_init.m'));
  resdir = fullfile(rob_path, 'examples_tests', 'results');
  mkdirs(resdir);
  s_anim = struct('gif_name', fullfile(resdir, sprintf('3T2R_PKM_sym_test_%s.gif',RP.mdlname)));
  fhdl=figure(2);clf;
  set(fhdl,'units','normalized','outerposition',[0 0 1 1],'color','w'); % Vollbild
  hold on;
  plot3(X(:,1), X(:,2), X(:,3));
  grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
  view(3);
  title(RP.mdlname, 'interpreter', 'none');
  RP.anim( Q(1:20:length(T),:), X(1:20:length(T),:), s_anim, s_plot);
end
