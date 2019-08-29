% Beispiel für asymmetrische Parallelroboter mit 3T0R und 3T1R aus [Gogu2008]
% 
% Es werden fünf verschiedene Roboter generiert
% 
% Quelle:
% [Gogu2008] Gogu, Grigore:Structural Synthesis of Parallel Robots, Part 1:
% Methodology (2008), Springer

% Li, Junnan (MA bei Moritz Schappler)
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc

RobTitles = {}; % Für die Auswertungsbilder
for testcase = 1:5
   
  % FG für die Beinketten: Position und Orientierung der Koppel-KS muss
  % vollständig berechnet werden, damit die IK gelöst wird
  I_EE_Leg = logical([1 1 1 1 1 1]);
  
  %% Initialisierung: Speziell
  if testcase == 1
    %% Isoglide3-T3; [Gogu2008] S.154
    % 3xPRRR  S4PRRR2
    RobTitles{testcase} = 'Isoglide3-T3 ([Gogu2008] S.154)';
    RS1 = serroblib_create_robot_class('S4PRRR2');
    RS1.fill_fcn_handles(false);
    % ParRob-Klasse für PKM erstellen
    RP = ParRob('TestPKM3FG');
    % Einzelne Beinketten setzen
    RP.Leg = copy(RS1);
    RP.Leg(2) = copy(RS1);
    RP.Leg(3) = copy(RS1);
    RP.NLEG = 3;
    % PKM-Parameter definieren
    RP.align_base_coupling(1, 1.0);
    RP.align_platform_coupling(1, 0.3);
    RP.initialize();
    
    % Basis KS einstellen
    phiconv_W_0 = uint8(2);
    RP.Leg(1).update_base( [-0.5;1;0.5], [pi/2;0;pi], phiconv_W_0);
    RP.Leg(2).update_base( [-0.5;-0.5;0], [0;0;pi], phiconv_W_0);
    RP.Leg(3).update_base( [1;-1;0], [0;0;-pi/2], phiconv_W_0);
    RP.align_platform_coupling(1, 0.3);
    RP.initialize();
    
    % Kinematik-Parameter der Beinketten neu belegen (hier mit Zufallswerten)
    pkin_all = [1.2,1.2
                1.2,1.2
                1.2,1.2]';
    for i = 1:RP.NLEG
      RP.Leg(i).update_mdh(pkin_all(1:size(RP.Leg(i).pkin),i));% rufen update_EE_FG
    end
        
    % EE-KS mit Null-Rotation vorbelegen
    for i = 1:RP.NLEG
      RP.Leg(i).update_EE(zeros(3,1),zeros(3,1));
    end
    RP.Leg(1).update_EE([], [0;0;pi/2]);
    RP.Leg(2).update_EE([], [pi/2;0;0]);
    RP.Leg(3).update_EE([], [0;pi/2;0]);
    I_EE = logical([1 1 1 0 0 0]);   
  elseif testcase == 2
    %% Isoglide4-T3R1; [Gogu2008] S.156
    RobTitles{testcase} = 'Isoglide4-T3R1 ([Gogu2008] S.156)';
    RS1 = serroblib_create_robot_class('S4PRRR1');
    RS1.fill_fcn_handles(false);
    RS2 = serroblib_create_robot_class('S5PRRRR2');
    RS2.fill_fcn_handles(false);
    % ParRob-Klasse für PKM erstellen
    RP = ParRob('TestPKM3FG');
    % Einzelne Beinketten setzen
    RP.Leg = copy(RS1);
    RP.Leg(2) = copy(RS2);
    RP.Leg(3) = copy(RS2);
    RP.Leg(4) = copy(RS2);
    RP.NLEG = 4;
    % PKM-Parameter definieren
    RP.align_base_coupling(1, 1.0);
    RP.align_platform_coupling(1, 0.3);
    RP.initialize();
    
    %Basis KS einstellen
    phiconv_W_0 = uint8(2);
    RP.Leg(1).update_base( [1;1;0], [0;0;-pi/2], phiconv_W_0);
    RP.Leg(2).update_base( [-0.5;1;0], [0;-pi/2;0], phiconv_W_0);
    RP.Leg(3).update_base( [-0.5;-0.5;0], [pi/2;pi*3/2;pi/2], phiconv_W_0);
    RP.Leg(4).update_base( [1;-1;0], [-pi/2;0;-pi/2], phiconv_W_0);
    RP.align_platform_coupling(1, 0.3);
    RP.initialize();
    
    % Kinematik-Parameter der Beinketten neu belegen (hier mit Zufallswerten)
    pkin_all = [0.5,0.75,0.75,0.15,0.15,0.25,0;
      0.5,0.75,0.75,0.15,0.15,0.25,0;
      0.5,0.75,0.75,0.15,0.15,0.25,0;
      0.5,0.75,0.75,0.15,0.15,0.25,0]';%14x4
    for i = 1:RP.NLEG
      RP.Leg(i).update_mdh(pkin_all(1:size(RP.Leg(i).pkin),i));% rufen update_EE_FG
    end
    
    % EE-KS mit Null-Rotation vorbelegen
    for i = 1:RP.NLEG
      RP.Leg(i).update_EE(zeros(3,1),zeros(3,1));
    end
    RP.Leg(1).update_EE([], [0;0;pi]);
    I_EE = logical([1 1 1 0 0 1]);

  elseif testcase == 3
    %% Isoglide4-T3R1; [Gogu2008] S.158
    RobTitles{testcase} = 'Isoglide4-T3R1 ([Gogu2008] S.158)';
    RS1 = serroblib_create_robot_class('S5PRRRR2');
    RS1.fill_fcn_handles(false);
    % ParRob-Klasse für PKM erstellen
    RP = ParRob('TestPKM3FG');
    % Einzelne Beinketten setzen
    RP.Leg = copy(RS1);
    RP.Leg(2) = copy(RS1);
    RP.Leg(3) = copy(RS1);
    RP.Leg(4) = copy(RS1);
    RP.NLEG = 4;
    % PKM-Parameter definieren
    RP.align_base_coupling(1, 1.0);
    RP.align_platform_coupling(1, 0.3);
    RP.initialize();
    
    %Basis KS einstellen
    phiconv_W_0 = uint8(2);
    RP.Leg(1).update_base( [1;1;0], [0;0;-pi/2], phiconv_W_0);
    RP.Leg(2).update_base( [-0.5;1;0], [0;-pi/2;0], phiconv_W_0);
    RP.Leg(3).update_base( [-0.5;-0.5;0], [pi/2;pi*3/2;pi/2], phiconv_W_0);
    RP.Leg(4).update_base( [1;-1;0], [-pi/2;0;-pi/2], phiconv_W_0);
    RP.align_platform_coupling(1, 0.3);
    RP.initialize();
    
    % Kinematik-Parameter der Beinketten neu belegen (hier mit Zufallswerten)
    pkin_all = [0.5,0.75,0.75,0.15,0.15,0.25;
      0.5,0.75,0.75,0.15,0.15,0.25;
      0.5,0.75,0.75,0.15,0.15,0.25;
      0.5,0.75,0.75,0.15,0.15,0.25]';%14x4
    for i = 1:RP.NLEG
      RP.Leg(i).update_mdh(pkin_all(1:size(RP.Leg(i).pkin),i));% rufen update_EE_FG
    end
    
    % EE-KS mit Null-Rotation vorbelegen
    for i = 1:RP.NLEG
      RP.Leg(i).update_EE(zeros(3,1),zeros(3,1));
    end
    RP.Leg(1).update_EE([], [pi/2;0;0]);
    I_EE = logical([1 1 1 0 0 1]);
  elseif testcase == 4
    %% Isoglide4-T3R1; [Gogu2008] S.159
    RobTitles{testcase} = 'Isoglide4-T3R1 ([Gogu2008] S.159)';
    RS1 = serroblib_create_robot_class('S4PRRR1');
    RS1.fill_fcn_handles(false);
    RS2 = serroblib_create_robot_class('S5PRRRR2');
    RS2.fill_fcn_handles(false);
    RS3 = serroblib_create_robot_class('S6PRRRRR6');
    RS3.fill_fcn_handles(false);
    % ParRob-Klasse für PKM erstellen
    RP = ParRob('TestPKM3FG');
    % Einzelne Beinketten setzen
    RP.Leg = copy(RS1);
    RP.Leg(2) = copy(RS3);
    RP.Leg(3) = copy(RS3);
    RP.Leg(4) = copy(RS2);
    RP.NLEG = 4;
    % PKM-Parameter definieren
    RP.align_base_coupling(1, 1.0);
    RP.align_platform_coupling(1, 0.3);
    RP.initialize();
    
    %Basis KS einstellen
    phiconv_W_0 = uint8(2);
    RP.Leg(1).update_base( [1;1;0], [0;0;-pi/2], phiconv_W_0);
    RP.Leg(2).update_base( [-0.5;1;0], [0;-pi/2;0], phiconv_W_0);
    RP.Leg(3).update_base( [-0.5;-0.5;0], [pi/2;pi*3/2;pi/2], phiconv_W_0);
    RP.Leg(4).update_base( [1;-1;0], [-pi/2;0;-pi/2], phiconv_W_0);
    RP.align_platform_coupling(1, 0.3);
    RP.initialize();
    
    % Kinematik-Parameter der Beinketten neu belegen (hier mit Zufallswerten)
    pkin_all = [0.5,0.75,0.75,0.15,0.15,0.25,0,0,0,0,0,0,0,0;
      0.5,0.75,0.75,0,0,0,0,0,0,0,0,0,0,0;
      0.5,0.75,0.75,0,0,0,0,0,0,0,0,0,0,0;
      0.5,0.75,0.75,0.15,0.25,0,0,0,0,0,0,0,0,0]';%14x4
    for i = 1:RP.NLEG
      RP.Leg(i).update_mdh(pkin_all(1:size(RP.Leg(i).pkin),i));% rufen update_EE_FG
    end
    
    % EE-KS mit Null-Rotation vorbelegen
    for i = 1:RP.NLEG
      RP.Leg(i).update_EE(zeros(3,1),zeros(3,1));
    end
    I_EE = logical([1 1 1 0 0 1]);
  elseif testcase == 5
    %% Isoglide4-T3R1; [Gogu2008] S.160
    RobTitles{testcase} = 'Isoglide4-T3R1 ([Gogu2008] S.160)';
    RS1 = serroblib_create_robot_class('S4PRRR1');
    RS1.fill_fcn_handles(false);
    RS2 = serroblib_create_robot_class('S6PRRRRR6');
    RS2.fill_fcn_handles(false);
    % ParRob-Klasse für PKM erstellen
    RP = ParRob('TestPKM3FG');
    % Einzelne Beinketten setzen
    RP.Leg = copy(RS1);
    RP.Leg(2) = copy(RS2);
    RP.Leg(3) = copy(RS2);
    RP.Leg(4) = copy(RS2);
    RP.NLEG = 4;
    % PKM-Parameter definieren
    RP.align_base_coupling(1, 1.0);
    RP.align_platform_coupling(1, 0.3);
    RP.initialize();
    
    %Basis KS einstellen
    phiconv_W_0 = uint8(2);
    RP.Leg(1).update_base( [1;1;0], [0;0;-pi/2], phiconv_W_0);
    RP.Leg(2).update_base( [-0.5;1;0], [0;-pi/2;0], phiconv_W_0);
    RP.Leg(3).update_base( [-0.5;-0.5;0], [pi/2;pi*3/2;pi/2], phiconv_W_0);
    RP.Leg(4).update_base( [1;-1;0], [-pi/2;0;-pi/2], phiconv_W_0);
    RP.align_platform_coupling(1, 0.3);
    RP.initialize();
    
    % Kinematik-Parameter der Beinketten neu belegen (hier mit Zufallswerten)
    pkin_all = [0.5,0.75,0.75,0.15,0.15,0.25,0,0,0,0,0,0,0,0;
      0.5,0.75,0.75,0,0,0,0,0,0,0,0,0,0,0;
      0.5,0.75,0.75,0,0,0,0,0,0,0,0,0,0,0;
      0.5,0.75,0.75,0,0,0,0,0,0,0,0,0,0,0]';%14x4
    for i = 1:RP.NLEG
      RP.Leg(i).update_mdh(pkin_all(1:size(RP.Leg(i).pkin),i));% rufen update_EE_FG
    end
    
    % EE-KS mit Null-Rotation vorbelegen
    for i = 1:RP.NLEG
      RP.Leg(i).update_EE(zeros(3,1),zeros(3,1));
    end
    I_EE = logical([1 1 1 0 0 1]);
  else
    error('Noch nicht implementiert');
  end
  %% Teste FG der Beinketten
  % LJN: beurteilen ob eingegebene I_EE m�glich
  I_EE_Max = ones(1,6);
  for i = 1:RP.NLEG
    I_EE_Max = I_EE_Max & RP.Leg(i).I_EE;
  end
  if (I_EE & I_EE_Max) ~= I_EE
    error('Gezählte FG der Beinketten passen nicht zur gewünschten Plattformmobilität')
  end
  RP.update_EE_FG(I_EE, I_EE, repmat(logical(I_EE_Leg),RP.NLEG,1));

  %% PKM testen
  % Definition der Test-Pose
  X_E = [ [0;0;1]; [0;0;0]*pi/180 ];
  q0 = rand(RP.NJ,1);
  q0(RP.I_qa) = 0.2;

  RP.constr1(q0,X_E);
  Gq = RP.constr1grad_q(q0,X_E);% ZB noch weiter verstehen. EE_FG x Anzahl_q?
  % IK für Roboter berechnen
  [q,phi] = RP.invkin_ser(X_E, q0, struct('Phit_tol', 1e-13, 'Phir_tol', 1e-13));
  
  q(isnan(q)) = q0(isnan(q)); % nicht lösbare Beinkette belegen, für Plot
  Phi_real = RP.constr1(q, X_E);
  
  %% FG mit Jacobi-Matrix testen
  [G_q_red,G_q] = RP.constr1grad_q(q, X_E);
  [G_x_red,G_x] = RP.constr1grad_x(q, X_E);
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
  fprintf('%s: Rang der vollständigen Jacobi der inversen Kinematik (%dx%d): %d/%d\n', ...
    RP.mdlname, size(G_q,1), size(G_q,2), rank(G_q), RP.NJ);
  fprintf('%s: Rang der vollständigen Jacobi der direkten Kinematik (%dx%d): %d/%d\n', ...
    RP.mdlname, size(G_dx,1), size(G_dx,2), rank(G_dx), sum(RP.I_EE)+sum(RP.I_qd));
  fprintf('%s: Rang der Jacobi der aktiven Gelenke (%dx%d): %d/%d\n', ...
    RP.mdlname, size(G_a,1), size(G_a,2), rank(G_a), sum(RP.I_EE));
  
  qaD = 100*rand(sum(RP.I_qa),1);
  qdDxD = G_dx \ G_a * qaD;
  xD_test = qdDxD(sum(RP.I_qd)+1:end);
  if any(abs(xD_test(~RP.I_EE)) > 1e-10) % Genauigkeit hier ist abhängig von Zwangsbed.
    error('Falsche Koordinaten werden laut Jacobi-Matrix  durch die Antriebe bewegt')
  end

  % PKM zeichnen
  figure(testcase);clf;
  hold on;grid on;
  xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
  view(3);
  s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
  RP.plot( q, X_E, s_plot );
  title(sprintf('%s: %dT%dR', RobTitles{testcase}, sum(RP.I_EE(1:3)), sum(RP.I_EE(4:6))));
end