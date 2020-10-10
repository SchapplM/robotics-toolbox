% Asym. 3T2R PKM mit PUU-Leading, RUU-Leading und UPU-Leading



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

% 5PUS_RRS, 5UPS_RUU, 5UPS_RRS

for robnr = 1:3
  if robnr == 1
    RS1 = serroblib_create_robot_class('S5RRRRR12V1');% passive Fuehrungsbeinkette US , Denavit-Hartenberg falsch
    RS1.fill_fcn_handles(false);
    RS2 = serroblib_create_robot_class('S6PRRRRR6'); % Folgebeinketten (P)US
    RS2.fill_fcn_handles(false);
    % ParRob-Klasse fuer PKM erstellen
    RP = ParRob('OneUS_FivePUS_3T2R');
    % Beinketten definieren
    RP.NLEG = 6;
    RP.Leg = copy(RS1); % Fuehrungsbeinkette RRS
    pkin_RS1 = [0.5;1.5;pi/2;0;0;0;0];
    RP.Leg(1).update_mdh(pkin_RS1(1:size(RP.Leg(1).pkin),1));
    
    for ii=2:RP.NLEG % Folgebeinketten 4 PUS
      RP.Leg(ii) = copy(RS2);
    end
    %   a2 a3  a4  a5 a6 2  3     4  d2 d3 d4 d5 d6 t1
    pkin_RS2 = [0, 0, 1.0, 0, 0, 0, pi/2, 0, 0, 0, 0, 0, 0 ,0 ]';
    for ii=2:RP.NLEG % Folgebeinketten 4 UPS( vielleicht Verallgemeinerung fuer spaeter)
      RP.Leg(ii).update_mdh(pkin_RS2(1:size(RP.Leg(ii).pkin),1));
    end
    RP.align_base_coupling(8, [0.5;0.3;0.2]);  % Wahl ist wichtig fuer Loesbarkeit der IK
    RP.align_platform_coupling(4, [0.2;0.1]); % Wahl ist wichtig fuer Loesbarkeit der IK
    RP.initialize();
    I_qa_Typ1 = zeros(1,5); % keine Aktuierung fuer passives Fuehrungsbein
    I_qa_Typ2 = zeros(1,6); % Folgebeinketten
    I_qa_Typ2(1) = 1;  % das 3te Gelenk P ist aktuiert
    I_qa = [I_qa_Typ1,repmat(I_qa_Typ2,1,5)];
    RP.update_actuation(I_qa);
    
  elseif robnr == 2
    RS1 = serroblib_create_robot_class('S5RRRRR2');% passive Fuehrungsbeinkette RUU
    RS1.fill_fcn_handles(false);
    RS2 = serroblib_create_robot_class('S6RRPRRR14V3'); % Folgebeinketten U(P)S
    RS2.fill_fcn_handles(false);
    % ParRob-Klasse fuer PKM erstellen
    RP = ParRob('OneRUU_FiveUPS_3T2R');
    % Beinketten definieren
    RP.NLEG = 6;
    RP.Leg = copy(RS1); % Fuehrungsbeinkette RUU
    pkin_RS1 = [0.5 , 1]';
    RP.Leg(1).update_mdh(pkin_RS1(1:size(RP.Leg(1).pkin),1));
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
    RP.update_actuation(I_qa);
    
  elseif robnr == 3
    RS1 = serroblib_create_robot_class('S5RRRRR12V1');% passive Fuehrungsbeinkette US , Denavit-Hartenberg falsch
    RS1.fill_fcn_handles(false);
    RS2 = serroblib_create_robot_class('S6RRPRRR14V3'); % Folgebeinketten U(P)S
    RS2.fill_fcn_handles(false);
    % ParRob-Klasse fuer PKM erstellen
    RP = ParRob('OneUS_FiveUPS_3T2R');
    % Beinketten definieren
    RP.NLEG = 6;
    RP.Leg = copy(RS1); % Fuehrungsbeinkette RRS
    %pkin_RS1 = zeros(9,1);
    pkin_RS1 = [0.5;1.5;pi/2;0;0;0;0];
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
    RP.update_actuation(I_qa);
    for ii = 1:RP.NLEG
      RP.Leg(ii).update_EE(zeros(3,1),zeros(3,1));
    end
  end
  % generalle Einstellungen
  I_EE = logical([1 1 1 1 1 0]);
  I_EE_Task = logical([1 1 1 1 1 0]); % 3T2R , die Null , da beta_3 weg
  RP.update_EE_FG(I_EE,I_EE_Task);
  % Startpose
  X_E = [[0.1;0.2;1.2];[5;10;-10]*pi/180]; % Plattform nur verdrehbar, keine Kipp-bwg
  q0 = rand(RP.NJ,1);
  q0(RP.I_qa) = 0.5;
  q = q0; % qs in constr2 und q sind ungleich ( also aktive Gelenke)
  %% IK
  [q,phi] = RP.invkin_ser(X_E, q);
  [Phi3_red,Phi3_voll] = RP.constr3(q, X_E); % mit Fuehrungsbeinkette
  [Phi2_red,Phi2_voll] = RP.constr2(q, X_E);
  X_E(6) = X_E(6) + Phi3_voll(4);
  
  figure(2*robnr);clf;
  hold on; grid on;
  xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
  view(3);
  s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I2L_LEG], 'straight', 0);
  RP.plot( q, X_E, s_plot );
  hold off;
  
  % Jacobi q-Anteile
  [G_q_red,G_q_voll] = RP.constr3grad_q(q, X_E); % automatisches herausnehmen
  % Vorgehen von Li fuer q-Anteile
  G_q = G_q_voll(RP.I_constr_red,:); % manuelles herausnehmen
  % Vergleich zwischen manuellem und automatischem Herausnehmen
  if  any(abs(G_q_red - G_q) > 1e-2)
    warning('Vergleich zwischen manuellem und automatischem Herausnehmen fuer G_q falsch \n');
  else
    fprintf('Vergleich zwischen manuellem und automatischem Herausnehmen fuer G_q richtig\n');
  end
  
  % Jacobi x-Anteile ( dim bei G_x_red von constr3grad noch nicht richtig)
  [G_x_red,G_x_voll] = RP.constr3grad_x(q, X_E); % automatisches herausnehmen
  % Vorgehen von Li fuer x-Anteile
  G_x = G_x_voll(RP.I_constr_red,:);
  G_eta = G_x_voll(RP.I_constr_red,RP.I_EE_Task); % manuelles herausnehmen
  % Vergleich zwischen manuellem und automatischem Herausnehmen
  if  any(abs(G_x_red - G_eta) > 1e-2)
    warning('Vergleich zwischen manuellem und automatischem Herausnehmen fuer G_x falsch \n');
  else
    fprintf('Vergleich zwischen manuellem und automatischem Herausnehmen fuer G_x richtig \n');
  end
  
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
  
  % Prüfe inverse Jacobi-Matrix gegen nicht-invertierte
  matrix_test = J_eta_qa*J_qa_eta - eye(5); % 5x5
  if any(abs(matrix_test(:)) > 1e-4)
    error('Jacobi-Matrix und ihre Inverse passen nicht zueinander');
  end
  % FG test
  qaD = 100*rand(sum(RP.I_qa),1);
  qdDxD = J_voll1 * qaD;
  xD_test = qdDxD(sum(RP.I_qd)+1:end);
  if any(abs(xD_test(~RP.I_EE)) > 1e-4) % Genauigkeit hier ist abhaengig von Zwangsbed.
    fprintf('Falsche Koordinaten werden laut Jacobi-Matrix  durch die Antriebe bewegt\n');
  end
  
  %% Trajektorie ( aktuell ueber constr3grad
  
  k=1; XE = X_E';
  d1=0.1;
  h1=0.2;
  r1=10*pi/180;
  k=k+1; XE(k,:) = XE(k-1,:) + [0,0,0  r1,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,0, 0, 0,-r1,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,0,0, 0,r1,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,0,0, -r1,0,0];
  [X,XD,XDD,T] = traj_trapez2_multipoint(XE, 1, 2e-1, 1e-1, 5e-3, 0);
  %return
  % Inverse Kinematik berechnen
  iksettings = struct('n_max', 5000, 'Phit_tol', 1e-3, 'Phir_tol', 1e-3, 'debug', true, ...
    'retry_limit', 0, 'mode_IK', 1);
  warning off
  [Q, QD, QDD, Phi] = RP.invkin_traj(X,XD,XDD,T,q,iksettings); % hier muessen einige Zeilen auskommentiert werden
  warning on
  % Tatsächliche EE-Koordinaten mit vollständiger direkter Kinematik bestimmen
  for i = 1:length(T)
    if max(abs( Phi(i,:) )) > 1e-3 || any(isnan( Phi(i,:) ))
      warning('IK stimmt nicht bei i=%d. Wahrscheinliche Ursache: Ist innerhalb von n_max nicht konvergiert', i);
      return
    end
  end
  %% Animation
  rob_path = fileparts(which('robotics_toolbox_path_init.m'));
  resdir = fullfile(rob_path, 'examples_tests', 'results');
  mkdirs(resdir);
  s_anim = struct('gif_name', fullfile(resdir, sprintf('%s.gif',RP.mdlname)));
  figure(2);clf;
  hold on;
  plot3(X(:,1), X(:,2), X(:,3));
  grid on;
  xlabel('x [m]');
  ylabel('y [m]');
  zlabel('z [m]');
  view(3);
  title('Animation der kartesischen Trajektorie');
  RP.anim( Q(1:20:length(T),:), X(1:20:length(T),:), s_anim, s_plot);
%   return
end
