% Symmetrische 3T2R-PKM: 5UPU, 5RUU, 5PUU, 5RPUR
% 
% Ergebnis des Beispielskripts:
% * Nachweis, dass diese Art von PKM kinematisch modellierbar ist

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

for robnr = 4%[1 2 4] % 5_UPU, 5_RUU, 5_PUU, 5_RPUR
  if robnr == 1 % 5_UPU
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
    
  elseif robnr == 2 % 5_RUU
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
    
  elseif robnr == 3 % 5_PUU
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
    
  elseif robnr == 4 % 5_RPUR
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
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
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
  if robnr ~= 4
    warning('Trajektorie für Roboter %d noch nicht validiert', robnr);
    continue;
  end
%% Trajektorie ( aktuell ueber constr3grad fuer 5_RPUR )
  qaD = 100*rand(sum(RP.I_qa),1);
  qdDxD = J_2 * qaD;
  xD_test = qdDxD(sum(RP.I_qd)+1:end);
  xD_test(6) = 0; % manuell
  if any(abs(xD_test(~RP.I_EE)) > 1e-4) % Genauigkeit hier ist abhaengig von Zwangsbed.
    fprintf('Falsche Koordinaten werden laut Jacobi-Matrix  durch die Antriebe bewegt\n');
  end
  %% Trajektorie berechnen
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
  t1 = tic();
  iksettings = struct('n_max', 5000, 'Phit_tol', 1e-8, 'Phir_tol', 1e-8, 'debug', true, ...
    'retry_limit', 0, 'mode_IK', 1, 'normalize', false);
  warning off
  [Q, QD, QDD, Phi] = RP.invkin_traj(X,XD,XDD,T,q,iksettings); % hier muessen einige Zeilen auskommentiert werden
  warning on
  fprintf('Trajektorien-IK in %1.2fs berechnet. Prüfe die Ergebnisse.\n', toc(t1));
%   return
  %% Trajektorie prüfen
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
      else
        test_eepos = r_0_E_Legs-r_0_Ej;
        if any(abs(test_eepos)>2e-6) % muss größer als IK-Toleranz sein
          error('i=%d: EE-Position aus Beinkette %d stimmt nicht mit Beinkette 1 überein. Fehler %1.2e', i, j, max(abs(test_eepos)));
        end
        test_eerot = R_0_E_Legs\R_0_E-eye(3);
        if any(abs(test_eerot(:)) > 1e-6)
          error('i=%d: EE-Rotation aus Beinkette %d stimmt nicht mit Beinkette 1 überein. Fehler %1.2e', i, j, max(abs(test_eerot(:))));
        end
      end
      T_E_Leg_j = rt2tr(R_0_E, r_0_Ej);
      x_i = RP.t2x(T_E_Leg_j);
      % Berechnet letzten Euler-Winkel neu aus der Beinkette
      X(i,6) = x_i(6);
    end

    % Neues xD berechnen
    XD(i,6) = 0; % Auf Null setzen, damit Aufruf auch mehrfach funktioniert.
    [~,Phi2D]=RP.constr2D(Q(i,:)', QD(i,:)', X(i,:)', XD(i,:)');
    XD(i,6) = Phi2D(4);

    % TODO: Das xD(6) muss vielleicht neu berechnet werden. Dadurch auch
    % das x(6). Wäre der Fall, wenn die 6. Koordinate abhängig ist und
    % nicht einfach nur Null bleibt.
    % TODO: Diese ZB-Funktionen können nicht funktionieren! (Dritte
    % Rotation ist nicht passend zur Plattform-Pose)
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
  return
  %% Animation
  rob_path = fileparts(which('robotics_toolbox_path_init.m'));
  resdir = fullfile(rob_path, 'examples_tests', 'results');
  mkdirs(resdir);
  s_anim = struct('gif_name', fullfile(resdir, sprintf('%s.gif',RP.mdlname)));
  figure(2);clf;
  hold on;
  plot3(X(:,1), X(:,2), X(:,3));
  grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
  view(3);
  title('Animation der kartesischen Trajektorie');
  RP.anim( Q(1:20:length(T),:), X(1:20:length(T),:), s_anim, s_plot);
  
end