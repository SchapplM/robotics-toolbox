% Testskript zur Implementierung der Limitierung der Rotation um die
% Z-Achse des Endeffektors
%   - zunächst für 3T2R und später auch für 2T2R (Translation entlang Z)
%  

close all
clear
clc
% Unterdrücke Warnung für Schlechte Konditionierung der Jacobi-Matrix
warning('off', 'MATLAB:rankDeficientMatrix');
warning('off', 'Coder:MATLAB:rankDeficientMatrix'); % gleiche Warnung aus mex-Datei
if isempty(which('serroblib_path_init.m'))
  warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end

%% Benutzereingaben
% Prüfung repräsentativer Roboter

Traj_TB = true;    % false = Sepehr

if Traj_TB
  Robots = {
  %           {'S4RRPR1', 'S4RRPR1_KUKA1'}, ...
  %           {'S5RRRRR1', 'S5RRRRR1_KUKA1'}, ...
            {'S6RRRRRR10V2', 'S6RRRRRR10V2_KUKA1'}, ...
  %           {'S7RRRRRRR1', 'S7RRRRRRR1_LWR4P'}
            };
else
  Robots = {
  %           {'S4RRPR1', 'S4RRPR1_KUKA1'}, ...
  %           {'S5RRRRR1', 'S5RRRRR1_KUKA1'}, ...
            {'S6RRRRRR10V2', 'S6RRRRRR10V2_KUKA46'}, ...
  %           {'S7RRRRRRR1', 'S7RRRRRRR1_LWR4P'}
            };
end

% Einstellungen
use_mex_functions = false; % mit mex geht es etwas schneller, dafür ist debuggen schwieriger
minimal_test = true; % Nur sehr wenige zufällige Winkel testen (geht schneller)
debug_mode = false;  % Diverse Plots schalten

calc_traj = true;  % IK für Trajektorie berechnen
calc_ep = false;

traj_anim = false;  % Animation der Trajektorie
% Endeffektor-Transformation ungleich Null festlegen, um zu prüfen, ob die
% Implementierung davon richtig ist
r_W_E = [0.05;0.07;0.09];
phi_W_E = [5; 10; 20]*pi/180;

%% Alle Robotermodelle durchgehen
for Robot_Data = Robots
  SName = Robot_Data{1}{1};
  RName = Robot_Data{1}{2};
   
  %% Klasse für seriellen Roboter erstellen
  % Instanz der Roboterklasse erstellen
  % serroblib_create_template_functions({SName}, false, false);
  RS = serroblib_create_robot_class(SName, RName);
  RS.fill_fcn_handles(use_mex_functions, true);
  % Grenzen festlegen (für Zusatz-Optimierung)
  RS.qlim = repmat([-pi, pi], RS.NQJ, 1);
  RS.qDlim = repmat([-4*pi, 4*pi], RS.NQJ, 1); % 2rpm
  RS.qDDlim = repmat([-100, 100], RS.NQJ, 1); % entspricht 1.5 rpm in 100ms
  qlim   = cat(1, RS.qlim);
  qDlim  = cat(1, RS.qDlim);
  qDDlim = cat(1, RS.qDDlim);
  
  RS.update_EE(r_W_E, phi_W_E, []);
  % Test-Einstellungen generieren
  TSS = RS.gen_testsettings(true, false);
  if minimal_test
    TSS.Q = TSS.Q(1:100,:);
  end
  % Bereich reduzieren: Bei Addition von Zufallswinkeln darf nicht pi
  % überschritten werden.
  TSS.Q(abs(TSS.Q(:))>150*pi/180) = 0;
  
  RS.I_EE = logical([1 1 1 1 1 1]);
  RS.I_EE_Task = logical([1 1 1 1 1 0]);
  fprintf('Starte Untersuchung für %s\n', RS.descr);
  
 
  %% Roboter in Nullstellung plotten (mit im Gelenkraum entworfener Trajektorie)
  if debug_mode
    s_plot = struct( 'ks', [RS.NJ+2, RS.NJ+2], 'straight', 0);
    figure(1);clf;
    hold on;
    grid on;
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    view(3);
    RS.plot( zeros(RS.NJ,1), s_plot );
    title(sprintf('Nullstellung: %s', RS.descr));
  end
  
  %% Roboter in Grundstellung plotten (mit im Gelenkraum entworfener Trajektorie)
  if debug_mode
    % qref = [0;90;0;90;0;0]*pi/180;
    qref = RS.qref;
    s_plot = struct( 'ks', [RS.NJ+2, RS.NJ+2], 'straight', 0);
    figure(2);clf;
    hold on;
    grid on;
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    view(3);
    RS.plot( qref, s_plot );
    title(sprintf('Grundstellung: %s', RS.descr));
    plot3(X(:,1), X(:,2), X(:,3));
  end
    
  %% Kartesische Trajektorie (Trajektorie von Sepehr verwendet)
  % Würfel-Trajektorie erstellen
  s.normalize=false;
  
  if Traj_TB
    q0 = RS.qref; % Wahl von TB
    x0 = [1,1,0.5, 0,0,0]'; % Wahl von TB
    k=1; XE = x0';
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0.5,0,0, 0,0,0]; % Wahl von TB
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,-1,0,0, 10*pi/180,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,-1,0,0, 20*pi/180,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ -0.5,0,0, 30*pi/180,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,0,0.25, -10*pi/180,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0.5,0,0,  -10*pi/180,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,1,0,    -10*pi/180,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,0,0.5,  -10*pi/180,0,0];
  else
  %   q0 = RS.qref+[0;-25;-25;0;120;0]*pi/180;   % Original von Sepehr
    q0 = RS.qref+[0;-25;-25;0;80;0]*pi/180;   % Abwandlung von TB für Gelenkgrenzen
    T_E = RS.fkineEE(q0);
    x0 = [T_E(1:3,4); r2eul(T_E(1:3,1:3), RS.phiconv_W_E)];

    k=1; XE = x0';
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0.2,-0.2,-0.3, -pi/2,-3*pi/4,0]; % parallel q0=120
    % Beginn Trajektorie
    d1=0.5;
    k=k+1; XE(k,:) = XE(k-1,:) + [-d1/2,0,0, 0,pi/3.2,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ -d1/2,0,0, 0,pi/4,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,d1/2,0, 0,pi/4,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,d1/2,0, 0,pi/4,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ d1/2,0,0, 0,pi/4,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ d1/2,0,0, 0,pi/4,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,-d1/2,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,-d1/2,0, 0,0,0];
  end
 
  Task_save = RS.I_EE_Task;
  RS.I_EE_Task = logical([1 1 1 1 1 1]);
  [q_new, phi_new, ~, Stats_new] = RS.invkin2(RS.x2tr(XE(1,:)'+[0;0;0.1;0;0;0]), q0, struct('retry_on_limitviol', true, ...
                                                                                                      'Phit_tol', 1e-12, 'Phir_tol', 1e-12));
  RS.I_EE_Task = Task_save;
  if any(abs(phi_new)>1e-12)
    error('IK des Startpunkts fehlgeschlagen');
  end
  q0 = q_new;
  
  % Startkonfiguration q0 auf Verletzung der Grenzen testen 
  q0norm = (q0-qlim(:,1)) ./ (qlim(:,2) - qlim(:,1));
  if any(q0norm >= 1) || any(q0norm <= 0) % Winkel mit +- 2*pi berücksichtigen
    error('q0 verletzt bereits die Grenzen');
    q0norm'
  end

  % Alle Soll-Punkte der Trajektorie auf Verletzung der Grenzen testen 
  Task_save = RS.I_EE_Task;
  RS.I_EE_Task = logical([1 1 1 1 1 1]);
  q_nn = zeros(6,size(XE,1));
  Stats_nn_save = NaN(1001,5,size(XE,1));
  qnn_norm = zeros(6,size(XE,1));
  for nn = 1:size(XE,1)
    if nn == 1
      [q_nn(:,1), ~, ~, Stats_nn] = RS.invkin2(RS.x2tr(XE(1,:)'), q0, struct('retry_on_limitviol', false, ...
                                                                             'Phit_tol', 1e-12, 'Phir_tol', 1e-12));
    else
      [q_nn(:,nn), ~, ~, Stats_nn] = RS.invkin2(RS.x2tr(XE(nn,:)'), q_nn(:,nn-1), struct('retry_on_limitviol', false, ...
                                                                                         'Phit_tol', 1e-12, 'Phir_tol', 1e-12)); 
    end
    Stats_nn_save(:,:,nn) = Stats_nn.h(:,2:end);
    qnn_norm(:,nn) = (q_nn(:,nn)-qlim(:,1)) ./ (qlim(:,2) - qlim(:,1));
    if any(qnn_norm(:,nn) >= 1) || any(qnn_norm(:,nn) <= 0) % Winkel mit +- 2*pi berücksichtigen
      warning('qnn für Punkt %d verletzt bereits die Grenzen', nn);
      qnn_norm(:,nn)'
    end
  end
  RS.I_EE_Task = Task_save;
  
  [X,XD,XDD,T] = traj_trapez2_multipoint(XE, 1, 1e-1, 1e-2, 1e-3, 0.25);

  % Gelenkkräfte und inverse Kinematik berechnen
  [Q, QD, QDD,Phi_prev,~,Stats_prev] = RS.invkin2_traj(X,XD,XDD,T,q0,struct('n_max', 50, 'Phit_tol', 1e-4, 'Phir_tol', 1e-4 ));
  
  % IK-Ergebnis testen
  for i = 1:length(T)
    phi_test_voll = RS.constr2(Q(i,:)', RS.x2tr(X(i,:)'), true);
    I_IK2 = [1 2 3 6 5 4];
    I_IK = I_IK2(RS.I_EE_Task);
    phi_test_red  = phi_test_voll(I_IK);
    if max(abs(phi_test_red)) > 1e-4
      error('IK stimmt nicht');
    end
  end
  %% Roboter in Grundstellung plotten (mit im Arbeitsraum entworfener Trajektorie)
  if debug_mode
    s_plot = struct( 'ks', [RS.NJ+2, RS.NJ+2], 'straight', 0);
    figure(6);clf;
    hold on;
    grid on;
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    view(3);
    RS.plot( q0, s_plot );
    title(sprintf('Grundstellung: %s', RS.descr));
    plot3(X(:,1), X(:,2), X(:,3));
    RS.plot( RS.qref, s_plot );
    
% s_anim = struct( 'gif_name', '');
% figure(4);clf;
% hold on;
% plot3(X(:,1), X(:,2), X(:,3));
% grid on;
% xlabel('x [m]');
% ylabel('y [m]');
% zlabel('z [m]');
% view(3);
% title('Animation der Gelenktrajektorie');
% RS.anim( Q(1:50:end,:), [], s_anim, s_plot);
  end
  
  %% Roboter in Grundstellung mit Koordinatensystemen
  if debug_mode
    s_plot = struct( 'ks', [RS.NJ+2, RS.NJ+2], 'straight', 0);
    s_plot_KS = struct( 'ks', [8, RS.NJ+2], 'straight', 0, 'mode', 2);
    figure(7);clf;
    hold on;
    grid on;
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    view(3);
    RS.plot( q0, s_plot );
    title(sprintf('Grundstellung: %s', RS.descr));
    plot3(X(:,1), X(:,2), X(:,3));
    RS.plot( RS.qref, s_plot );
    
    
    Task_save = RS.I_EE_Task;
    RS.I_EE_Task = logical([1 1 1 1 1 1]);
    q_ii_save = NaN(6,size(XE,1));
    q_ii_save_pre = q0;
    for ii = 1:size(XE,1)
      [q_ii, phi_ii, ~, Stats_ii] = RS.invkin2(RS.x2tr(XE(ii,:)'), q_ii_save_pre, struct('retry_on_limitviol', true, ...
                                                                                         'Phit_tol', 1e-12, 'Phir_tol', 1e-12));      
      q_ii_save(:,ii) = q_ii;
      if any(abs(phi_ii)>1e-12)
        error('IK des Startpunkts fehlgeschlagen');
      end
      q_ii_save_pre = q_ii;
    end
    RS.I_EE_Task = Task_save;
    
    for iii = 1:size(XE,1)
      hold on;
      RS.plot( q_ii_save(:,iii), s_plot_KS );
    end
    
  end
  %% Einzelpunkte aus Trajektorie
  if calc_ep
    fprintf('Trajektorie als Einzelpunkte\n');
    i_phiconv = uint8([2]);
    fig_count = 0;
    Phirt_tol = 1e-12;
    n_max = 2500;
    h5_save_nolimred   = NaN(1+n_max,size(XE,1));
    h5_save_limred     = h5_save_nolimred;
    phiz_save_nolimred = h5_save_nolimred;
    phiz_save_limred   = h5_save_nolimred;
    iter_limred        = NaN(size(XE,1),1);
    iter_nolimred      = iter_limred;
    Q_save_nolimred    = NaN(1+n_max,6,size(XE,1));
    Q_save_limred      = Q_save_nolimred;
    wn5_active_nolimred = zeros(1+n_max,size(XE,1));
    wn5_active_limred = wn5_active_nolimred;
    angle_nolimred = NaN(size(XE,1),1);
    angle_limred = NaN(size(XE,1),1);
    
    q0_ep = RS.qref + [0;0;0;0;0;-120]*pi/180;
    for i = 1:size(XE,1)
      RS.phiconv_W_E = i_phiconv;
      eulstr = euler_angle_properties(i_phiconv);

      % Ziel- und Anfangs-Konfiguration definieren
      RS.I_EE_Task = logical([1 1 1 1 1 1]);
      [q_ziel, phi_ziel] = RS.invkin2(RS.x2tr(XE(i,:)'), q0_ep, struct('n_max', n_max, ...
        'Phit_tol', Phirt_tol, 'Phir_tol', Phirt_tol, 'reci', false));
      T_E_ziel = RS.fkineEE(q_ziel);
      %     qs = q_ziel-20*pi/180*(0.5-rand(RS.NQJ,1)); % Anfangswinkel 20° neben der Endstellung
      %     q_start = qs;
      q_start = q0_ep;
      T_E_start = RS.fkineEE(q_start);

%       % Einmalig Auftreffwinkel auf XY-Ebene berechnen
%       Tr0E_soll = RS.x2tr(XE(i,:)');
%       e_z_OE = Tr0E_soll(:,3);
%       angle_Z_XY_value = asin(abs(sum(e_z_OE.*[0;0;1]))/(sqrt(e_z_OE(1)^2 + ...
%         e_z_OE(2)^2 + e_z_OE(3)^2)*sqrt(1)))/pi*180;
%       if abs(angle_Z_XY_value) <= 45
%         xz_modus_temp = true;
%       else
%         xz_modus_temp = false;
%       end

      % Aufgaben-FHG auf 3T2R
      RS.I_EE_Task = logical([1 1 1 1 1 0]);

      s_wn_3T2R_regular = [0.99;0.01];
%       s_wn_3T2R_regular = [0;0];    % TODO!
      s_wn_3T2R_cond    = 0;
      s_wn_3T2R_coll    = 0;
      s_wn_3T2R_limred  = 0;
      s_wn_3T2R_ges     = [s_wn_3T2R_regular; s_wn_3T2R_cond; s_wn_3T2R_coll; s_wn_3T2R_limred];
      s_ep_3T2R = struct( ...
        'n_min', 0, 'n_max', 2500, 'Phit_tol', Phirt_tol, 'Phir_tol', Phirt_tol, ...
        'scale_lim', 0, 'reci', true, 'wn', s_wn_3T2R_ges, 'retry_limit', 0);
      s_ep_3T2R.xlim = [NaN(5,2); [-45 45]*pi/180]; % in [rad] übergeben
      
      
      s_wn_limred_regular = [0;0];
      s_wn_limred_cond    = 0;
      s_wn_limred_coll    = 0;
      s_wn_limred_limred  = 1;
      s_wn_limred_ges     = [s_wn_limred_regular; s_wn_limred_cond; s_wn_limred_coll; s_wn_limred_limred];
      s_ep_limred = s_ep_3T2R;
      s_ep_limred.wn = s_wn_limred_ges; % wn(5) muss getestet werden
      s_ep_limred.xlim = [NaN(5,2); [-45 45]*pi/180]; % in [rad] übergeben


      % IK aufrufen, um Residuum zu bekommen
      [q_IK_nolimred, phi_IK_nolimred, ~, Stats_nolimred] = RS.invkin2(RS.x2tr(XE(i,:)'), q0_ep, s_ep_3T2R);
      h5_save_nolimred(:,i)   = Stats_nolimred.xlimred(:);
      phiz_save_nolimred(:,i) = Stats_nolimred.PHI(:,4);
      iter_nolimred(i,1) = Stats_nolimred.iter;
      Q_save_nolimred(:,:,i) = Stats_nolimred.Q(:,:);
      % Test-Berechnung ob wn(5) aktiv ist/sein sollte
      for nn = 1:(Stats_nolimred.iter+1)
        Stats_Phi_temp = Stats_nolimred.PHI(nn,:);
        if all(abs(Stats_Phi_temp(logical([1 1 1 0 1 1])))<1e-3)  % vierten Eintrag ignorieren
          wn5_active_nolimred(nn,i) = 1;
        end
      end
      if Stats_nolimred.iter == 2500
        warning('3T3R: Maximale Iteration bei Punkt %d erreicht', i);
      end
      
      [q_IK, phi_IK, ~, Stats_limred] = RS.invkin2(RS.x2tr(XE(i,:)'), q0_ep, s_ep_limred);
      h5_save_limred(:,i)   = Stats_limred.xlimred(:);
      phiz_save_limred(:,i) = Stats_limred.PHI(:,4);
      iter_limred(i,1) = Stats_limred.iter;    
      Q_save_limred(:,:,i) = Stats_limred.Q(:,:);
      % Test-Berechnung ob wn(5) aktiv ist/sein sollte
      for nn = 1:(Stats_limred.iter+1)
        Stats_Phi_temp = Stats_limred.PHI(nn,:);
        if all(abs(Stats_Phi_temp(logical([1 1 1 0 1 1])))<1e-3)  % vierten Eintrag ignorieren
          wn5_active_limred(nn,i) = 1;
        end
      end
      if Stats_limred.iter == 2500
        warning('limred: Maximale Iteration bei Punkt %d erreicht', i);
      end
      
      if any(abs(phi_IK_nolimred) > Phirt_tol)
        error('Inverse Kinematik OHNE limred fehlerhaft bei Punkt %d', i);
      end
      if any(abs(phi_IK) > Phirt_tol)
        error('Inverse Kinematik MIT limred fehlerhaft bei Punkt %d', i);
      end

    end

    %% Visualisierung von phi_z für ausgesuchte Punkte
    leg1 = zeros(2,1);
    leg2 = zeros(2,1);
    for jj = 1:size(XE,1)
      f0 = figure(50*4+jj);clf;
      f0.Position = [200 200 2000 1000];
      index_phiz_limred = 1:(iter_limred(jj)+1);
      index_phiz_nolimred = 1:(iter_nolimred(jj)+1);

      
      % Subplots Erste Spalte -------------------------------------
      subplot(4,12,1:6); hold on;
      plot(index_phiz_nolimred, phiz_save_nolimred(1:index_phiz_nolimred(end),jj)*180/pi, 'm', 'LineWidth',1);
      xlim_vek = repmat([s_ep_limred.xlim(6,1) s_ep_limred.xlim(6,2)]*180/pi,index_phiz_nolimred(end),1);
      plot(index_phiz_nolimred, xlim_vek(:,1), 'r--', 'LineWidth',1);
      plot(index_phiz_nolimred, xlim_vek(:,2), 'r--', 'LineWidth',1);
      plot(index_phiz_nolimred, xlim_vek(:,1)*0.8, 'b--', 'LineWidth',1);
      plot(index_phiz_nolimred, xlim_vek(:,2)*0.8, 'b--', 'LineWidth',1);
      axe = gca;
      axe.XLim = [1 index_phiz_nolimred(end)];
      ylabel('phi_z in °'); grid on;
      title('OHNE limred');
      
      % h(5)
      subplot(4,12,13:18); hold on;
      plot(index_phiz_nolimred, h5_save_nolimred(1:index_phiz_nolimred(end),jj), 'm', 'LineWidth', 1');
      axe = gca;
      axe.XLim = [1 index_phiz_nolimred(end)];
      ylabel('Optimierungskriterium h(5)'); grid on;
      for kk = 1:index_phiz_nolimred(end)
        if isinf(h5_save_nolimred(kk,jj))
          plot(kk, 0,'*g');
        end
      end
      hold on;
      grid on;
      xlabel('Iteration lfd Nr');
      
      % Subplots Zweite Spalte -------------------------------------
      subplot(4,12,7:12); hold on;
      plot(index_phiz_limred, phiz_save_limred(1:index_phiz_limred(end),jj)*180/pi, 'm', 'LineWidth',1);
      xlim_vek = repmat([s_ep_limred.xlim(6,1) s_ep_limred.xlim(6,2)]*180/pi,index_phiz_limred(end),1);
      plot(index_phiz_limred, xlim_vek(:,1), 'r--', 'LineWidth',1);
      plot(index_phiz_limred, xlim_vek(:,2), 'r--', 'LineWidth',1);
      plot(index_phiz_limred, xlim_vek(:,1)*0.8, 'b--', 'LineWidth',1);
      plot(index_phiz_limred, xlim_vek(:,2)*0.8, 'b--', 'LineWidth',1);
      axe = gca;
      axe.XLim = [1 index_phiz_limred(end)];
      grid on;
      title('MIT limred');
      
      % h(5)
      subplot(4,12,19:24); hold on;
      leg1(1) = plot(index_phiz_limred, h5_save_limred(1:index_phiz_limred(end),jj), 'm', 'LineWidth',1, 'DisplayName','h(5) aus IK');
      hinf_used = false;  % Abfrage bzgl. Legendeneintrag
      for kk = 1:index_phiz_limred(end)
        if isinf(h5_save_limred(kk,jj))
          hinf_used = true;
          leg1(2) = plot(kk, 0,'*g', 'DisplayName','h(5) = inf');
        end
      end
      hold on;
      grid on;
      xlabel('Iteration lfd Nr');
      if hinf_used
        legend(leg1(1:2), 'AutoUpdate','off');
      else
        legend(leg1(1:1), 'AutoUpdate','off');
      end
      axe = gca;
%       axe.XLim = [92 index_phiz_limred(end)];
      axe.XLim = [1 index_phiz_limred(end)];
      
      % Zeile 3
      % Subplots Erste Spalte -------------------------------------
      subplot(4,12,25:30); hold on;
      wn5_used = false;
      for kk = 1:index_phiz_nolimred(end)
        if wn5_active_nolimred(kk,jj) == 1
          wn5_used = true;
          leg2(1) = plot(kk, 1,'om', 'DisplayName','wn(5) aktiv');
        elseif wn5_active_nolimred(kk,jj) == 0
          leg2(2) = plot(kk, 0,'or', 'DisplayName','wn(5) inaktiv');
        end
      end
      hold on;
      axe = gca;
      axe.XLim = [1 index_phiz_nolimred(end)];
      axe.YLim = [-1 2];
      ylabel('Zustand wn(5)'); grid on;
      xlabel('Iteration lfd Nr');
      if hinf_used
        legend(leg2(1:2), 'AutoUpdate','off');
      else
        legend(leg2(2:2), 'AutoUpdate','off');
      end
      
      % Zeile 3
      % Subplots Zweite Spalte -------------------------------------
      subplot(4,12,31:36); hold on;
      wn5_used = false;
      for kk = 1:index_phiz_limred(end)
        if wn5_active_limred(kk,jj) == 1
          wn5_used = true;
          leg2(1) = plot(kk, 1,'om', 'DisplayName','wn(5) aktiv');
        elseif wn5_active_limred(kk,jj) == 0
          leg2(2) = plot(kk, 0,'or', 'DisplayName','wn(5) inaktiv');
        end
      end    
      hold on;
      axe = gca;
      axe.XLim = [1 index_phiz_limred(end)];
      axe.YLim = [-1 2];
      ylabel('Zustand wn(5)'); grid on;
      xlabel('Iteration lfd Nr');      
      
      % Zeile 4
      % Subplots Erste Spalte -------------------------------------
      for kkk = 1:RS.NQJ
        subplot(4,12,36+kkk);hold on;
        plot(index_phiz_nolimred, Q_save_nolimred(1:index_phiz_nolimred(end),kkk,jj)/RS.qunitmult_eng_sci(kkk));
        plot([0;index_phiz_nolimred(end)], RS.qlim(kkk,1)*[1;1]/RS.qunitmult_eng_sci(kkk), 'r--');
        plot([0;index_phiz_nolimred(end)], RS.qlim(kkk,2)*[1;1]/RS.qunitmult_eng_sci(kkk), 'r--');
        xlabel('t [s]');
        ylabel(sprintf('q_%d / %s', kkk, RS.qunit_eng{kkk}));
        grid on;
        title(sprintf('Zeitverlauf Achse %d',kkk));
  %       subplot(4,6,sprc2no(4,6,2,k));hold on;
  %       plot(T, QD_work(:,k)/RS.qunitmult_eng_sci(k));
  %       plot([0;T(end)], RS.qDlim(k,1)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
  %       plot([0;T(end)], RS.qDlim(k,2)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
  %       xlabel('t [s]');
  %       ylabel(sprintf('qD_%d / %s/s', k, RS.qunit_eng{k}));
  %       grid on;
      end

      % Zeile 4
      % Subplots Zweite Spalte -------------------------------------
      for kkk = 1:RS.NQJ
        subplot(4,12,42+kkk);hold on;
        plot(index_phiz_limred, Q_save_limred(1:index_phiz_limred(end),kkk,jj)/RS.qunitmult_eng_sci(kkk));
        plot([0;index_phiz_limred(end)], RS.qlim(kkk,1)*[1;1]/RS.qunitmult_eng_sci(kkk), 'r--');
        plot([0;index_phiz_limred(end)], RS.qlim(kkk,2)*[1;1]/RS.qunitmult_eng_sci(kkk), 'r--');
        xlabel('t [s]');
        ylabel(sprintf('q_%d / %s', k, RS.qunit_eng{kkk}));
        grid on;
        title(sprintf('Zeitverlauf Achse %d',kkk));
  %       subplot(4,6,sprc2no(4,6,2,k));hold on;
  %       plot(T, QD_work(:,k)/RS.qunitmult_eng_sci(k));
  %       plot([0;T(end)], RS.qDlim(k,1)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
  %       plot([0;T(end)], RS.qDlim(k,2)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
  %       xlabel('t [s]');
  %       ylabel(sprintf('qD_%d / %s/s', k, RS.qunit_eng{k}));
  %       grid on;
      end  
    txt = ['Punkt Nr. ',num2str(jj)];
    sgtitle(txt);
    end
  
  end
  
  %% Trajektorie
  if calc_traj
    fprintf('Teste gesamte Trajektorie\n');
    fprintf('ACHTUNG! Verwendung von korrekten Structs für Traj überprüfen!\n');
    % Gelenkkräfte und inverse Kinematik berechnen
    Phirt_tol = 1e-8;
    RS.I_EE_Task = logical([1 1 1 1 1 0]);
    
%     reguläre 3T2R mit 

%     s_traj_3T2R_regular       = [0.99;0.01;1;0];  % qlim quad+hyp wn(1+2), qDlim quad+hyp wn(3+4)
%     s_traj_3T2R_regular       = [0;0;0;0];          % qlim quad+hyp wn(1+2), qDlim quad+hyp wn(3+4)
%     s_traj_3T2R_cond_v1       = 0;                  % Konditionszahl wn(5)
%     s_traj_3T2R_qlim_v2       = [0;0];              % qlim quad wn(6) und qlim hyp wn(7)
%     s_traj_3T2R_cond_v2       = 0;                  % Konditionszahl wn(8)
%     s_traj_3T2R_collision     = [0;0];              % Kollisionsvermeidung wn(9+10)
%     s_traj_3T2R_limredcoord   = [0;0;0];            % wn(11+12) für qD und qDD und wn(13) für qD
%     s_traj_3T2R_ges = [s_traj_3T2R_regular; s_traj_3T2R_cond_v1; s_traj_3T2R_qlim_v2; s_traj_3T2R_cond_v2; ...
%                          s_traj_3T2R_collision; s_traj_3T2R_limredcoord];
      s_traj_3T2R_ges = zeros(13,1); % wn(:)=0 -> keine Opt.Krit
      s_traj_3T2R   = struct('n_min', 50, 'n_max', 1500, 'Phit_tol', Phirt_tol, 'Phir_tol', Phirt_tol, ...
                             'reci', true, 'I_EE', RS.I_EE_Task, 'wn', s_traj_3T2R_ges);
                                
%     s_traj_3T2R_ges = ones(13,1); % hierdurch werden die Kriterien berechnet                
%     s_traj_3T2R   = struct('n_min', 50, 'n_max', 1500, 'Phit_tol', Phirt_tol, 'Phir_tol', Phirt_tol, ...
%                            'reci', true, 'I_EE', RS.I_EE_Task, 'wn', s_traj_3T2R_ges, ...
%                            'K', zeros(RS.NJ,1), 'Kn', zeros(RS.NJ,1)); % mit K und Kn = 0 keine NRbew.
    
    % limredcoord: phi_z begrenzen
    s_traj_limred_regular       = [0;0;0;0];          % qlim quad+hyp wn(1+2), qDlim quad+hyp wn(3+4)
    s_traj_limred_cond_v1       = 0;                  % Konditionszahl wn(5)
    s_traj_limred_qlim_v2       = [0;0];              % qlim quad wn(6) und qlim hyp wn(7)
    s_traj_limred_cond_v2       = 0;                  % Konditionszahl wn(8)
    s_traj_limred_collision     = [0;0];              % Kollisionsvermeidung wn(9+10)
%     s_traj_limred_limredcoord   = [5;5;5];          % wn(11+12) für qD und qDD und wn(13) für qD
    s_traj_limred_limredcoord   = [1;1;0];            % wn(11+12) für qD und qDD und wn(13) für qD
    s_traj_limred_ges = [s_traj_limred_regular; s_traj_limred_cond_v1; s_traj_limred_qlim_v2; s_traj_limred_cond_v2; ...
                                  s_traj_limred_collision; s_traj_limred_limredcoord];                                     
    s_traj_limred = struct('n_min', 50, 'n_max', 1500, 'Phit_tol', Phirt_tol, 'Phir_tol', Phirt_tol, ...
                           'reci', true, 'I_EE', RS.I_EE_Task, 'wn', s_traj_limred_ges);  
                         
    s_traj_limred.xlim   = [NaN(5,2); [-45 45]*pi/180]; % in [rad] übergeben
    s_traj_limred.xDlim  = [NaN(5,2); [-0.21 0.21]];    % 0.21rad/s = 2rpm laut unitconversion.io/de/rpm-zu-rads-konvertierung
    s_traj_limred.xDDlim = [NaN(5,2); [-21 21]];        % vorläufige Konvertierung wie 4*pi zu 100 bei qD und qDD 
    
    
%     q0_traj = q0 + [0;0;0;0;0;-80]*pi/180;
    q0_traj = q0 + [0;0;0;0;0;-80]*pi/180;
%     q0_traj = q0 + [0;0;0;0;0;0]*pi/180;
    
    [Q_nolimred, QD_nolimred, QDD_nolimred, Phi_nolimred, ~, Stats_Traj_nolimred] = RS.invkin2_traj(X,XD,XDD,T,q0_traj,s_traj_3T2R);
    fprintf('\nBerechnung der Trajektorien Referenz-IK beendet.\n');
    
    [Q_limred, QD_limred, QDD_limred, Phi_limred, ~, Stats_Traj_limred] = RS.invkin2_traj(X,XD,XDD,T,q0_traj,s_traj_limred);
    fprintf('Berechnung der Trajektorien Opt.-IK wn11 und wn12 beendet.\n');
    
    s_traj_limred2 = s_traj_limred;
    s_traj_limred2.wn(11:13) = [0;0;1];
    [Q_limred2, QD_limred2, QDD_limred2, Phi_limred2, ~, Stats_Traj_limred2] = RS.invkin2_traj(X,XD,XDD,T,q0_traj,s_traj_limred2);
    fprintf('Berechnung der Trajektorien Opt.-IK wn13 beendet.\n');
    
    s_traj_limred3 = s_traj_limred;
    s_traj_limred3.wn(11:13) = [1;1;1];
    [Q_limred3, QD_limred3, QDD_limred3, Phi_limred3, ~, Stats_Traj_limred3] = RS.invkin2_traj(X,XD,XDD,T,q0_traj,s_traj_limred3);
    fprintf('Berechnung der Trajektorien Opt.-IK wn11 wn12 wn13 beendet.\n');
    
    [X_ist_nolimred, XD_ist_nolimred, ~] = RS.fkineEE_traj(Q_nolimred, QD_nolimred, QDD_nolimred);
    X_ist_nolimred_int = repmat(X_ist_nolimred(1,:), length(T), 1) + cumtrapz(T, XD_ist_nolimred);
    X_ist_nolimred(:,4:6) = normalizeAngle(X_ist_nolimred(:,4:6), X_ist_nolimred_int(:,4:6));
    
    [X_ist_limred, XD_ist_limred, ~] = RS.fkineEE_traj(Q_limred, QD_limred, QDD_limred);
    X_ist_limred_pre = X_ist_limred;
    XD_ist_limred_pre = XD_ist_limred;
    X_ist_limred_int = repmat(X_ist_limred(1,:), length(T), 1) + cumtrapz(T, XD_ist_limred);
    X_ist_limred(:,4:6) = normalizeAngle(X_ist_limred(:,4:6), X_ist_limred_int(:,4:6));  
    
    [X_ist_limred2, XD_ist_limred2, ~] = RS.fkineEE_traj(Q_limred2, QD_limred2, QDD_limred2);
    X_ist_limred_pre2 = X_ist_limred2;
    XD_ist_limred_pre2 = XD_ist_limred2;
    X_ist_limred_int2 = repmat(X_ist_limred2(1,:), length(T), 1) + cumtrapz(T, XD_ist_limred2);
    X_ist_limred2(:,4:6) = normalizeAngle(X_ist_limred2(:,4:6), X_ist_limred_int2(:,4:6));     
    
    [X_ist_limred3, XD_ist_limred3, ~] = RS.fkineEE_traj(Q_limred3, QD_limred3, QDD_limred3);
    X_ist_limred_pre3 = X_ist_limred3;
    XD_ist_limred_pre3 = XD_ist_limred3;
    X_ist_limred_int3 = repmat(X_ist_limred3(1,:), length(T), 1) + cumtrapz(T, XD_ist_limred3);
    X_ist_limred3(:,4:6) = normalizeAngle(X_ist_limred3(:,4:6), X_ist_limred_int3(:,4:6));  
    
    
    % IK-Ergebnis testen
    for i = 1:length(T)
      phi_test_voll_nolimred  = RS.constr2(Q_nolimred(i,:)', RS.x2tr(X(i,:)'), true);
      phi_test_voll_limred    = RS.constr2(Q_limred(i,:)'  , RS.x2tr(X(i,:)'), true);
      phi_test_voll_limred2   = RS.constr2(Q_limred2(i,:)' , RS.x2tr(X(i,:)'), true);
      phi_test_voll_limred3   = RS.constr2(Q_limred3(i,:)' , RS.x2tr(X(i,:)'), true);
      
      if RS.I_EE_Task == logical([1 1 1 1 1 0])
        I_IK2 = [1 2 3 6 5 4];
      else
        error('Achtung falscher FHG?');
      end
      I_IK = I_IK2(RS.I_EE_Task);
      phi_test_red_nolimred  = phi_test_voll_nolimred(I_IK);
      phi_test_red_limred    = phi_test_voll_limred(I_IK);
      phi_test_red_limred2   = phi_test_voll_limred2(I_IK);
      phi_test_red_limred3   = phi_test_voll_limred3(I_IK);
      if max(abs(phi_test_red_nolimred)) > Phirt_tol
        error('IK OHNE Limitierung stimmt nicht');
      end
      if max(abs(phi_test_red_limred)) > Phirt_tol
        error('IK MIT Limitierung stimmt nicht');
      end
      if max(abs(phi_test_red_limred2)) > Phirt_tol
        error('IK MIT Limitierung2 stimmt nicht');
      end
      if max(abs(phi_test_red_limred3)) > Phirt_tol
        error('IK MIT Limitierung3 stimmt nicht');
      end
    end
    
    fprintf('Berechnung der Trajektorien IK beendet.\n');
    
    
    % Auswertung für Plots
    phiz_traj_diff_nolimred      = X(:,6) - X_ist_nolimred(:,6);
    phiz_traj_diff_limred        = X(:,6) - X_ist_limred(:,6);
    phiz_traj_diff_limred_pre    = X(:,6) - X_ist_limred_pre(:,6);
    phiz_traj_diff_limred2       = X(:,6) - X_ist_limred2(:,6);
    phiz_traj_diff_limred_pre2   = X(:,6) - X_ist_limred_pre2(:,6);
    phiz_traj_diff_limred3       = X(:,6) - X_ist_limred3(:,6);
    phiz_traj_diff_limred_pre3   = X(:,6) - X_ist_limred_pre3(:,6);
    
    fprintf('Auswertung der Plots beendet\n');
    
    %% Plots
    figure(100*3+1);
    set(100*3+1, 'Name', sprintf('Verlauf von phi_z für gesamte Trajektorie OHNE limred'), 'NumberTitle', 'off', 'Position', get(0, 'Screensize'));
    sgtitle('Verlauf von phi_z und h');
    index_traj = 1:size(T,1);

    % ---------------------------- Subplot erste Spalte -------------------
    % Plot von phi_z nolimred
    subplot(4,4,1); hold on;
    hdl1 = plot(T, phiz_traj_diff_nolimred*180/pi, 'm', 'LineWidth',1);
    xlim_vek = repmat([s_traj_limred.xlim(6,1) s_traj_limred.xlim(6,2)]*180/pi,index_traj(end),1);
    plot(T, xlim_vek(:,1), 'r--', 'LineWidth',1);
    plot(T, xlim_vek(:,2), 'r--', 'LineWidth',1);
    xlim_vek_thresh = repmat([s_traj_limred.xlim(6,1) s_traj_limred.xlim(6,2)]*180/pi*0.8,index_traj(end),1);
    plot(T, xlim_vek_thresh(:,1), 'b--', 'LineWidth',1);
    plot(T, xlim_vek_thresh(:,2), 'b--', 'LineWidth',1);
    hdl1.Parent.XLim = [0 T(end)];
%     hdl1.Parent.YLim = [min(Stats_Traj_nolimred.PHI(1:index_traj(end),4)*180/pi)-5 max(Stats_Traj_nolimred.PHI(1:index_traj(end),4)*180/pi)+5];
    title('nolimred');
    ylabel('phi_z in °'); grid on;
    xlabel('Zeit in s');
    
    % ---------------------------- Subplot zweite Spalte -------------------
    % Plot von phi_z limred
    subplot(4,4,2); hold on;
    leg4 = zeros(1,2);
    leg4(1) = plot(T, phiz_traj_diff_limred*180/pi, 'm', 'LineWidth',1, 'DisplayName','phiz MIT normalizeAngle');
    leg4(2) = plot(T, phiz_traj_diff_limred_pre*180/pi, 'k--', 'LineWidth',1, 'DisplayName','phiz OHNE normalizeAngle');
    plot(T, xlim_vek(:,1), 'r--', 'LineWidth',1);
    plot(T, xlim_vek(:,2), 'r--', 'LineWidth',1);
    plot(T, xlim_vek_thresh(:,1), 'b--', 'LineWidth',1);
    plot(T, xlim_vek_thresh(:,2), 'b--', 'LineWidth',1);
    ax_test = gca;
    ax_test.XLim = [0 T(end)];
    %     hdl4.Parent.YLim = [min(Stats_Traj_limred.PHI(1:index_traj(end),4)*180/pi)-5 max(Stats_Traj_limred.PHI(1:index_traj(end),4)*180/pi)+5];
    title('limred 1 1 0');
    ylabel('phi_z in °'); grid on;
    xlabel('Zeit in s');
    legend(leg4(1:2), 'AutoUpdate','off');
    
    % Plot von h(7) limred
    leg2 = zeros(1,4);
    subplot(4,4,6); hold on;
    leg2(1) = plot(T, Stats_Traj_limred.h(1:index_traj(end),8), 'm', 'LineWidth',1, 'DisplayName','h aus IK-Traj');
%     leg2(2) = plot(index_traj, h7_limred, 'k', 'LineWidth',1, 'DisplayName','h aus X-ist mit integr');
%     leg2(3) = plot(index_traj, h7_limred_pre, 'y--', 'LineWidth',1, 'DisplayName','h aus X-ist ohne integr');
    ax_test = gca;
    ax_test.XLim = [0 T(end)];
    hinf_used = false;  % Abfrage bzgl. Legendeneintrag
    for kk = 1:index_traj(end)
%       if isinf(h7_limred(kk))
      if isinf(Stats_Traj_limred.h(kk,8))
        hinf_used = true;
        leg2(2) = plot(kk, 0,'*g', 'DisplayName','h = inf');
      end
    end
    hold on;
    ylabel('Optim.krit h(7)'); grid on;
    xlabel('Zeit in s');
    %   legend(leg(1:2));
    %     legend(leg(1:2), 'AutoUpdate','off');
    if hinf_used
      legend(leg2(1:2), 'AutoUpdate','off');
    else
      legend(leg2(1:1), 'AutoUpdate','off');
    end
    
    % Plot von h(8) limred
    subplot(4,4,10); hold on;
    hdl6 = plot(T, Stats_Traj_limred.h(1:index_traj(end),9), 'm', 'LineWidth',1);
%     plot(index_traj, h8_limred, 'k', 'LineWidth',1);
%     plot(index_traj, h8_limred_pre, 'y--', 'LineWidth',1);
    ax_test = gca;
    ax_test.XLim = [0 T(end)];
    for kk = 1:index_traj(end)
      if isinf(Stats_Traj_limred.h(kk,9))
        plot(kk, 0,'*g');
      end
    end
    hold on;
    ylabel('Optim.krit h(8)'); grid on;
    xlabel('Zeit in s');
    
    % Plot von phi_z Geschwindigkeit
    subplot(4,4,14); hold on;
    plot(T, Stats_Traj_limred.phi_zD(1:index_traj(end),1)*180/pi, 'm', 'LineWidth',1);
    ax_test = gca;
    ax_test.XLim = [0 T(end)];
    hold on;
    ylabel('Geschwindigkeit von phi_z in °/s'); grid on;
    xlabel('Zeit in s');

    % ---------------------------- Subplot dritte Spalte -------------------
    % Plot von phi_z limred
    subplot(4,4,3); hold on;
    plot(T, phiz_traj_diff_limred2*180/pi, 'm', 'LineWidth',1);
    plot(T, phiz_traj_diff_limred_pre2*180/pi, 'k--', 'LineWidth',1);
    plot(T, xlim_vek(:,1), 'r--', 'LineWidth',1);
    plot(T, xlim_vek(:,2), 'r--', 'LineWidth',1);
    plot(T, xlim_vek_thresh(:,1), 'b--', 'LineWidth',1);
    plot(T, xlim_vek_thresh(:,2), 'b--', 'LineWidth',1);
    ax_test = gca;
    ax_test.XLim = [0 T(end)];
    title('limred 0 0 1');
    ylabel('phi_z in °'); grid on;
    xlabel('Zeit in s');
    
    % Plot von h(7) limred
    subplot(4,4,7); hold on;
    plot(T, Stats_Traj_limred2.h(1:index_traj(end),8), 'm', 'LineWidth',1, 'DisplayName','h aus IK-Traj');
    ax_test = gca;
    ax_test.XLim = [0 T(end)];
    hinf_used = false;  % Abfrage bzgl. Legendeneintrag
    for kk = 1:index_traj(end)
      if isinf(Stats_Traj_limred2.h(kk,8))
        hinf_used = true;
        plot(kk, 0,'*g', 'DisplayName','h = inf');
      end
    end
    hold on;
    ylabel('Optim.krit h(7)'); grid on;
    xlabel('Zeit in s');

    % Plot von h(8) limred
    leg3 = zeros(1,2);
    hinf_used = false;
%     huneq0_used = false;
    subplot(4,4,11); hold on;
    plot(T, Stats_Traj_limred2.h(1:index_traj(end),9), 'm', 'LineWidth',1);
    ax_test = gca;
    ax_test.XLim = [0 T(end)];
    for kk = 1:index_traj(end)
      if isinf(Stats_Traj_limred2.h(kk,9))
        leg3(1) = plot(kk, 0,'*g', 'DisplayName', 'h = inf');
        hinf_used = true;
      end
%       if Stats_Traj_limred2.h(kk,9) ~= 0 && ~isinf(Stats_Traj_limred2.h(kk,9))
%         leg3(2) = plot(kk, 0,'*r', 'DisplayName', 'h ~= 0');
%         huneq0_used = true;
%       end
    end
    hold on;
    ylabel('Optim.krit h(8)'); grid on;
    xlabel('Zeit in s');
%     if hinf_used && huneq0_used
%       legend(leg3(1:2), 'AutoUpdate','off');
%     elseif hinf_used && ~huneq0_used
%       legend(leg3(1:1), 'AutoUpdate','off');
%     elseif ~hinf_used && huneq0_used
%       legend(leg3(2:2), 'AutoUpdate','off');
%     end
    if hinf_used
      legend(leg3(1:1), 'AutoUpdate','off');
    end

    % Plot von phi_z Geschwindigkeit
    subplot(4,4,15); hold on;
    plot(T, Stats_Traj_limred2.phi_zD(1:index_traj(end),1)*180/pi, 'm', 'LineWidth',1);
    ax_test = gca;
    ax_test.XLim = [0 T(end)];
    hold on;
    ylabel('Geschwindigkeit von phi_z in °/s'); grid on;
    xlabel('Zeit in s');
    

    % ---------------------------- Subplot vierte Spalte -------------------
    % Plot von phi_z limred
    subplot(4,4,4); hold on;
    plot(T, phiz_traj_diff_limred3*180/pi, 'm', 'LineWidth',1);
    plot(T, phiz_traj_diff_limred_pre3*180/pi, 'k--', 'LineWidth',1);
    plot(T, xlim_vek(:,1), 'r--', 'LineWidth',1);
    plot(T, xlim_vek(:,2), 'r--', 'LineWidth',1);
    plot(T, xlim_vek_thresh(:,1), 'b--', 'LineWidth',1);
    plot(T, xlim_vek_thresh(:,2), 'b--', 'LineWidth',1);
    ax_test = gca;
    ax_test.XLim = [0 T(end)];
    title('limred 1 1 1');
    ylabel('phi_z in °'); grid on;
    xlabel('Zeit in s');
    
    % Plot von h(7) limred
    subplot(4,4,8); hold on;
    plot(T, Stats_Traj_limred3.h(1:index_traj(end),8), 'm', 'LineWidth',1, 'DisplayName','h aus IK-Traj');
    ax_test = gca;
    ax_test.XLim = [0 T(end)];
    hinf_used = false;  % Abfrage bzgl. Legendeneintrag
    for kk = 1:index_traj(end)
      if isinf(Stats_Traj_limred3.h(kk,8))
        hinf_used = true;
        plot(kk, 0,'*g', 'DisplayName','h = inf');
      end
    end
    hold on;
    ylabel('Optim.krit h(7)'); grid on;
    xlabel('Zeit in s');

    % Plot von h(8) limred
    subplot(4,4,12); hold on;
    plot(T, Stats_Traj_limred3.h(1:index_traj(end),9), 'm', 'LineWidth',1);
    ax_test = gca;
    ax_test.XLim = [0 T(end)];
    for kk = 1:index_traj(end)
      if isinf(Stats_Traj_limred3.h(kk,9))
        plot(kk, 0,'*g');
      end
%       if Stats_Traj_limred3.h(kk,9) ~= 0 && ~isinf(Stats_Traj_limred3.h(kk,9))
%         plot(kk, 0,'*r');
%       end
    end
    hold on;
    ylabel('Optim.krit h(8)'); grid on;
    xlabel('Zeit in s');
    
    % Plot von phi_z Geschwindigkeit
    subplot(4,4,16); hold on;
    plot(T, Stats_Traj_limred3.phi_zD(1:index_traj(end),1)*180/pi, 'm', 'LineWidth',1);
    ax_test = gca;
    ax_test.XLim = [0 T(end)];
    hold on;
    ylabel('Geschwindigkeit von phi_z in °/s'); grid on;
    xlabel('Zeit in s');
    
    
    
    
    fprintf('Plot beendet\n'); 

    if traj_anim
      s_anim = struct( 'gif_name', '');
      figure(900);clf;
      hold on;
      plot3(X(:,1), X(:,2), X(:,3));
      grid on;
      xlabel('x [m]');
      ylabel('y [m]');
      zlabel('z [m]');
      view(3);
      title('Animation der kartesischen Trajektorie');
%       RS.anim( Q_nolimred(1:50:end,:), [], s_anim, s_plot);
      RS.anim( Q_limred(1:50:end,:), [], s_anim, s_plot);
    end
    
%% Trajektorie aus Gelenkraum visualisieren
  for qplots = 1:4
    f = figure(100*4+qplots);clf;
    f.Position = [200 200 2000 1000];
    if qplots == 1
      Q_work  = Q_nolimred; 
      QD_work = QD_nolimred;
      QDD_work = QDD_nolimred;
    elseif qplots == 2
      Q_work  = Q_limred; 
      QD_work = QD_limred;
      QDD_work = QDD_limred;
    elseif qplots == 3 
      Q_work  = Q_limred2; 
      QD_work = QD_limred2;
      QDD_work = QDD_limred2;
    elseif qplots == 4
      Q_work  = Q_limred3; 
      QD_work = QD_limred3;
      QDD_work = QDD_limred3;
    end
    % Gelenkkräfte berechnen
    TAU = NaN(size(Q_work,1),RS.NQJ);
    for k = 1:size(Q_work,1)
      q_k = Q_work(k,:)';
      qD_k = QD_work(k,:)';
      qDD_k = QDD_work(k,:)';

      tau_k = RS.invdyn(q_k, qD_k, qDD_k);
      TAU(k,:) = tau_k;
    end
    for k = 1:RS.NQJ
      subplot(4,6,sprc2no(4,6,1,k));hold on;
      plot(T, Q_work(:,k)/RS.qunitmult_eng_sci(k));
      plot([0;T(end)], RS.qlim(k,1)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
      plot([0;T(end)], RS.qlim(k,2)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
      xlabel('t [s]');
      ylabel(sprintf('q_%d / %s', k, RS.qunit_eng{k}));
      grid on;
      title(sprintf('Zeitverlauf Gelenkgrößen Achse %d',k));
      subplot(4,6,sprc2no(4,6,2,k));hold on;
      plot(T, QD_work(:,k)/RS.qunitmult_eng_sci(k));
      plot([0;T(end)], RS.qDlim(k,1)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
      plot([0;T(end)], RS.qDlim(k,2)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
      xlabel('t [s]');
      ylabel(sprintf('qD_%d / %s/s', k, RS.qunit_eng{k}));
      grid on;
      subplot(4,6,sprc2no(4,6,3,k));hold on;
      plot(T, QDD_work(:,k)/RS.qunitmult_eng_sci(k));
      plot([0;T(end)], RS.qDDlim(k,1)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
      plot([0;T(end)], RS.qDDlim(k,2)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
      xlabel('t [s]');
      ylabel(sprintf('qDD_%d / %s/s^2', k, RS.qunit_eng{k}));
      grid on;
      subplot(4,6,sprc2no(4,6,4,k));hold on;
      plot(T, TAU(:,k));
      xlabel('t [s]');
      ylabel(sprintf('\\tau_%d / %s', k, RS.tauunit_sci{k}));
      grid on;
      
      if     qplots == 1, sgtitle('Ref. Traj');
      elseif qplots == 2, sgtitle('Opt. Traj 1 1 0');
      elseif qplots == 3, sgtitle('Opt. Traj 0 0 1');
      elseif qplots == 4, sgtitle('Opt. Traj 1 1 1');
      end
    end
    linkxaxes
    
  end

  end
end
 