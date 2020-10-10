









% PKM 3 (5PUU) bei IK noch nicht l?sbar
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
% 5_UPU, 5_RUU, 5_PUU, 5_RPUR
for robnr = [1 2 4]
  if robnr == 1
    RS = serroblib_create_robot_class('S5RRPRR12');% Fuehrungsbeinkette
    RS.fill_fcn_handles(false);
    RP = ParRob('Five_UPU_3T2R');
    RP.create_symmetric_robot(5, RS, 1.1, 0.3);
    pkin_all = zeros(8,5);
    for ii=1:RP.NLEG
      RP.Leg(ii).update_mdh(pkin_all(1:size(RP.Leg(ii).pkin),1));
    end
    I_qa_Typ = zeros(1,5); % Typ1: S5PRRRR1
    I_qa_Typ(3) = 1; % das 2te Gelenk P ist aktuiert
    I_qa = repmat(I_qa_Typ,1,5);
    RP.update_actuation(I_qa);
    
  elseif robnr == 2
    RS = serroblib_create_robot_class('S5RRRRR2');
    RP = ParRob('Five_RUU_3T2R');
    RP.create_symmetric_robot(5, RS, 1.5, 0.5);
    pkin_all  = [1.0, 1.5]';
    for ii=1:RP.NLEG
      RP.Leg(ii).update_mdh(pkin_all(1:size(RP.Leg(ii).pkin),1));
    end
    I_qa_Typ = zeros(1,5); % Typ1: S5PRRRR1
    I_qa_Typ(1) = 1; % das 2te Gelenk P ist aktuiert
    I_qa = repmat(I_qa_Typ,1,5);
    RP.update_actuation(I_qa);
    
  elseif robnr == 3
    RS = serroblib_create_robot_class('S5PRRRR10');% Fuehrungsbeinkette
    RS.fill_fcn_handles(false);
    RP = ParRob('Five_PUU_3T2R');
    RP.NLEG = 5;
    RP.Leg = copy(RS);
    for ii=2:RP.NLEG
      RP.Leg(ii) = copy(RS);
    end
    pkin_all  = [0, 0, 0.5, 0, pi/2, pi/2, 0, 0, 0, 0, 0]';
    for ii=1:RP.NLEG
      RP.Leg(ii).update_mdh(pkin_all(1:size(RP.Leg(ii).pkin),1));
    end
    RP.initialize();
    RP.align_base_coupling(1, 1.1);
    RP.align_platform_coupling(8, 0.2);
    RP.Leg(1).update_base( 3*RP.r_P_B_all(:,1), r2eulxyz(rotx(-pi)*roty(-pi))); % Fuehrungsbeinkette PUU
    RP.Leg(2).update_base( 3*RP.r_P_B_all(:,2), r2eulxyz(rotz((-3*pi/5)))); % Folgebeinketten PUU
    RP.Leg(3).update_base( 3*RP.r_P_B_all(:,3), r2eulxyz(rotz((-pi/5)))); % Folgebeinkette PUU
    RP.Leg(4).update_base( 3*RP.r_P_B_all(:,4), r2eulxyz(rotz((pi/5)))); % Folgebeinkette PUU
    RP.Leg(5).update_base( 3*RP.r_P_B_all(:,5), r2eulxyz(rotz((3*pi/5)))); % Folgebeinkette PUU
    
    I_qa_Typ = zeros(1,5); % Typ1: S5PRRRR1
    I_qa_Typ(1) = 1; % das 2te Gelenk P ist aktuiert
    I_qa = repmat(I_qa_Typ,1,5);
    RP.update_actuation(I_qa);
    for ii=1:RP.NLEG
      RP.Leg(ii).update_EE(zeros(3,1),r2eulxyz(roty(-pi/2)));
    end
    
  elseif robnr == 4
    RS = serroblib_create_robot_class('S5RPRRR8V1');% Fuehrungsbeinkette
    RS.fill_fcn_handles(false);
    RP = ParRob('Five_RPUR_3T2R');
    RP.NLEG = 5;
    RP.Leg = copy(RS);
    for ii=2:RP.NLEG
      RP.Leg(ii) = copy(RS);
    end
    RP.initialize();
    pkin_all  = [ 0, 0.0, 0.9, 0.0, 0, 0.0, 0, 0, 0]';
    for ii=1:RP.NLEG
      RP.Leg(ii).update_mdh(pkin_all(1:size(RP.Leg(ii).pkin),1));
      RP.Leg(ii).update_EE(zeros(3,1),r2eulxyz(roty(pi/2)));
    end
    % Orientierung der Gestell-Koppelpunkt-KS
    RP.Leg(1).update_base( [0.8;0;0], r2eulxyz(rotx(pi/2)*rotz(pi/2))); % Fuehrungsbeinkette RPUR
    RP.Leg(2).update_base( [0.4;0.6;0], r2eulxyz(rotx((pi)/2)*rotz(pi/2))); % Folgebeinketten RPUR
    RP.Leg(3).update_base( [-0.4;0.6;0], r2eulxyz(rotx(pi/2)*rotz(pi/2))); % Folgebeinkette RPUR
    RP.Leg(4).update_base( [-0.4;-0.6;0], r2eulxyz(rotx(pi/2)*rotz(pi/2))); % Folgebeinkette RPUR
    RP.Leg(5).update_base( [0.4;-0.6;0], r2eulxyz(rotx(pi/2)*rotz(pi/2))); % Folgebeinkette RPUR
    % Reihenfolge wie im Bild
    RP.r_P_B_all(:,1) = [0.4;0;0];
    RP.r_P_B_all(:,2) = [0.2;0.3;0];
    RP.r_P_B_all(:,3) = [-0.2;0.3;0];
    RP.r_P_B_all(:,4) = [-0.2;-0.3;0];
    RP.r_P_B_all(:,5) = [0.2;-0.3;0];
    
    I_qa_Typ = zeros(1,5); % Typ1: S5PRRRR1
    I_qa_Typ(2) = 1; % das 2te Gelenk P ist aktuiert
    I_qa = repmat(I_qa_Typ,1,5);
    RP.update_actuation(I_qa);
  end
  % generalle Einstellungen
  I_EE = logical([1 1 1 1 1 0]);
  I_EE_Task = logical([1 1 1 1 1 0]); % 3T2R , die Null , da beta_3 weg
  RP.update_EE_FG(I_EE,I_EE_Task);
  % Startpose
  X_E = [[0.1;0.1;1.0];[0;0;20]*pi/180]; % Plattform nur verdrehbar, keine Kipp-bwg
  q0 = rand(RP.NJ,1);
  q0(RP.I_qa) = 0.5;
  q = q0; % qs in constr2 und q sind ungleich ( also aktive Gelenke)
  %% IK
  [q,phi] = RP.invkin_ser(X_E, q);
  [Phi3_red,Phi3_voll] = RP.constr3(q, X_E); % mit Fuehrungsbeinkette
  [Phi2_red,Phi2_voll] = RP.constr2(q, X_E);
  X_E(6) = X_E(6) + Phi3_voll(4);
  
  figure(robnr);clf;
  hold on; grid on;
  xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
  view(3);
  s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I2L_LEG], 'straight', 0);
  RP.plot( q, X_E, s_plot );
  hold off;

  I_EE_Legs_3T2R = repmat(I_EE_Task,5,1);
  RP.update_EE_FG(I_EE,I_EE_Task,I_EE_Legs_3T2R);
  %% Differentiale Kinematik ueber constr2grad
  [G_q_red_2,G_q_voll_2] = RP.constr2grad_q(q, X_E); % jetzt passt
  G_q_2 = G_q_voll_2(RP.I_constr_red,:);
  
  if  any(abs(G_q_red_2 - G_q_2) > 1e-2)
    warning('Vergleich zwischen manuellem und automatischem Herausnehmen fuer G_q falsch \n');
  else
    fprintf('Vergleich zwischen manuellem und automatischem Herausnehmen fuer G_q richtig\n');
  end
  
  [G_x_red_2,G_x_voll_2] = RP.constr2grad_x(q, X_E); % ljn: ZYX?
  G_x_2 = G_x_voll_2(RP.I_constr_red,RP.I_EE_Task);
  if  any(abs(G_x_red_2 - G_x_2) > 1e-2)
    warning('Vergleich zwischen manuellem und automatischem Herausnehmen fuer G_q falsch \n');
  else
    fprintf('Vergleich zwischen manuellem und automatischem Herausnehmen fuer G_q richtig\n');
  end
  
  G_a_2 = G_q_2(:,RP.I_qa); % aktiv, phi_dqa [STO19]
  G_d_2 = G_q_2(:,RP.I_qd); % passiv, phi_dpa [STO19]
  % Jacobi-Matrix zur Berechnung der abhaengigen Gelenke und EE-Koordinaten,
  % hier noch nicht quadratisch 30x29
  G_dxp_2 = [G_d_2, G_x_2]; % Gl. 50, phi_dxp, hier p-Anteile ueber x-Anteilen [STO19]
  J_2 = G_dxp_2 \ G_a_2; % inv(G_dx) * G_a = inv(phi_dxp)*phi_dqa, Gl. 50 vollstaendige Jacobi-Matrix bezogen auf x-Koordinaten [STO19]
  Jinv_2 = G_q_2 \ G_x_2; % vollstaendige inverse Jacobi-Matrix in x-Koord Gl. 49
  
  J_qa_x_2 = Jinv_2(RP.I_qa,:);
  J_x_qa_2 = J_2(sum(RP.I_qd)+1:end,:);
  % Pruefe inverse Jacobi-Matrix gegen nicht-invertierte
  matrix_test_2_debug = J_qa_x_2*J_x_qa_2- eye(5);
  matrix_test_2 = J_x_qa_2*J_qa_x_2- eye(5);
  if any(abs(matrix_test_2(:)) > 1e-4)
    warning('constr2grad: Jacobi-Matrix und ihre Inverse passen nicht zueinander');
  end
  
  %% Trajektorie ( aktuell ueber constr3grad fuer 5_RPUR )
  if robnr == 4
    qaD = 100*rand(sum(RP.I_qa),1);
    qdDxD = J_2 * qaD;
    xD_test = qdDxD(sum(RP.I_qd)+1:end);
    xD_test(6) = 0; % manuell
    if any(abs(xD_test(~RP.I_EE)) > 1e-4) % Genauigkeit hier ist abhaengig von Zwangsbed.
      fprintf('Falsche Koordinaten werden laut Jacobi-Matrix  durch die Antriebe bewegt\n');
    end
    
    k=1; XE = X_E';
    d1=0.1;
    h1=0.2;
    r1=10*pi/180;
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0,0  r1,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0, 0, 0,-r1,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0,0, 0,r1,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0,0, -r1,0,0];
    [X,XD,XDD,T] = traj_trapez2_multipoint(XE, 1, 2e-1, 1e-1, 5e-3, 0);
    
    % Inverse Kinematik berechnen
    iksettings = struct('n_max', 500, 'Phit_tol', 1e-6, 'Phir_tol', 1e-6, 'debug', true, ...
      'retry_limit', 100, 'mode_IK', 1, 'reci',true);
    warning off
    [Q, QD, QDD, Phi] = RP.invkin_traj(X,XD,XDD,T,q,iksettings); % hier muessen einige Zeilen auskommentiert werden
    warning on
    % Tatsächliche EE-Koordinaten mit vollständiger direkter Kinematik bestimmen
    for i = 1:length(T)
      if max(abs( Phi(i,:) )) > 1e-4 || any(isnan( Phi(i,:) ))
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
  end
  
end