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

Robots = {
%           {'S4RRPR1', 'S4RRPR1_KUKA1'}, ...
%           {'S5RRRRR1', 'S5RRRRR1_KUKA1'}, ...
          {'S6RRRRRR10V2', 'S6RRRRRR10V2_KUKA1'}, ...
%           {'S7RRRRRRR1', 'S7RRRRRRR1_LWR4P'}
          };

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
  
  %% Startpose bestimmen //alt
%   s = struct('n_max', 1000, 'Phit_tol', 1e-12, 'Phir_tol', 1e-12, ...
%     'reci', true, 'scale_lim', 0); % keine Vorgabe von K oder Kn (Standard-Werte)
%   x0Ref = zeros(6,1);
%   if strcmp(SName, 'S6RRRRRR10V2')
%     % Werte aus [1].
%     T_part = transl([1.15; 0.2; -0.2;]);
%     x0Ref(1:3) = T_part(1:3,4) + [-0.200;0;0.400];
%     x0Ref(4:6) = [pi;0;0];
%   elseif strcmp(SName, 'S3RRR1')
%     x0Ref(1:2) = [0.3; 0.3];
%   end
%   X0 = x0Ref;
%   q0 = zeros(RS.NJ,1);
%   if strcmp(SName, 'S6RRRRRR10V2')
%     q0(2) = pi/2;
%     q0_ik_fix = q0 + [0;25;-35;0;15;0]*pi/180;
%   else
%     q0_ik_fix = q0;
%   end
%   [qs, Phi] = RS.invkin2(RS.x2tr(X0), q0_ik_fix, s);
%   if any(abs(Phi) > 1e-8)
%     error('Inverse Kinematik konnte in Startpose nicht berechnet werden');
%   end
  
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
  TAU = NaN(size(Q,1),RS.NQJ);
  for k = 1:size(Q,1)
    q_k = Q(k,:)';
    qD_k = QD(k,:)';
    qDD_k = QDD(k,:)';

    tau_k = RS.invdyn(q_k, qD_k, qDD_k);
    TAU(k,:) = tau_k;
  end

  %% Trajektorie definieren (Linie) //alt
%   x_dist = 1;
%   y_span = 2;
%   p_intervall = 11;
%   y_steps = 2/10;
%   z_height = 0;
%   X_start = [0.5,0,1,0,0,0];
%   X0 = [x_dist,y_span/2,z_height,0,0,0]'; % Start auf der linken Seite der Bahn
%   X1 = X0;
%   k=1; XL = X1';
%   for k = 1:(p_intervall-1)
%     k=k+1; XL(k,:) = XL(k-1,:) + [0,-y_steps,0,0,0,0];
%   end
  
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
    qref=RS.qref;
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
    
%% Trajektorie aus Gelenkraum visualisieren
  if debug_mode
    figure(3);clf;
    for k = 1:RS.NQJ
      subplot(4,6,sprc2no(4,6,1,k));hold on;
      plot(T, Q(:,k)/RS.qunitmult_eng_sci(k));
      plot([0;T(end)], RS.qlim(k,1)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
      plot([0;T(end)], RS.qlim(k,2)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
      xlabel('t [s]');
      ylabel(sprintf('q_%d / %s', k, RS.qunit_eng{k}));
      grid on;
      title(sprintf('Zeitverlauf Gelenkgrößen Achse %d',k));
      subplot(4,6,sprc2no(4,6,2,k));hold on;
      plot(T, QD(:,k)/RS.qunitmult_eng_sci(k));
      plot([0;T(end)], RS.qDlim(k,1)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
      plot([0;T(end)], RS.qDlim(k,2)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
      xlabel('t [s]');
      ylabel(sprintf('qD_%d / %s/s', k, RS.qunit_eng{k}));
      grid on;
      subplot(4,6,sprc2no(4,6,3,k));hold on;
      plot(T, QDD(:,k)/RS.qunitmult_eng_sci(k));
      xlabel('t [s]');
      ylabel(sprintf('qDD_%d / %s/s^2', k, RS.qunit_eng{k}));
      grid on;
      subplot(4,6,sprc2no(4,6,4,k));hold on;
      plot(T, TAU(:,k));
      xlabel('t [s]');
      ylabel(sprintf('\\tau_%d / %s', k, RS.tauunit_sci{k}));
      grid on;
    end
    linkxaxes

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
  

  
  %% Kartesische Trajektorie (Trajektorie von Sepehr verwendet)
  % Würfel-Trajektorie erstellen
  s.normalize=false;
%   q0 = RS.qref+[0;-25;-25;0;120;0]*pi/180;
  q0 = RS.qref;
  %q0 = RS.qref+[0;-25;-25;0;-50;0]*pi/180;
  % cond(RS.jacobig(q0))
  T_E = RS.fkineEE(q0);
%   x0 = [T_E(1:3,4); r2eul(T_E(1:3,1:3), RS.phiconv_W_E)];
  x0 = [1,1,0.5, 0,0,0]';
  
  
  % Start in Grundstellung
  k=1; XE = x0';
%   k=k+1; XE(k,:) = XE(k-1,:) + [ 0.2,-0.2,-0.3, -pi/2,-3*pi/4,0]; % parallel q0=120
  k=k+1; XE(k,:) = XE(k-1,:) + [ 0.5,0,0, 0,0,0];
  % Beginn Trajektorie
  d1=0.5;
  
  %parallel
  % %
  % k=k+1; XE(k,:) = XE(k-1,:) + [-d1/2,0,0, 0,pi/3.2,0];
  % k=k+1; XE(k,:) = XE(k-1,:) + [ -d1/2,0,0, 0,pi/4,0];
  % k=k+1; XE(k,:) = XE(k-1,:) + [ 0,d1/2,0, 0,pi/4,0];
  % k=k+1; XE(k,:) = XE(k-1,:) + [ 0,d1/2,0, 0,pi/4,0];
  % k=k+1; XE(k,:) = XE(k-1,:) + [ d1/2,0,0, 0,pi/4,0];
  % k=k+1; XE(k,:) = XE(k-1,:) + [ d1/2,0,0, 0,pi/4,0];
  % k=k+1; XE(k,:) = XE(k-1,:) + [ 0,-d1/2,0, 0,0,0];
  % k=k+1; XE(k,:) = XE(k-1,:) + [ 0,-d1/2,0, 0,0,0];

  %Tobias_Test
  % %
  k=k+1; XE(k,:) = XE(k-1,:) + [ 0,-1,0, 0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [ 0,-1,0, 0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [ -0.5,0,0, 0,0,0];
  
  Task_save = RS.I_EE_Task;
  RS.I_EE_Task = logical([1 1 1 1 1 1]);
  [q_new, phi_new, ~, Stats_new] = RS.invkin2(RS.x2tr(XE(1,:)'+[0;0;0.2;0;0;150*pi/180]), q0, struct('retry_on_limitviol', true, ...
                                                                          'Phit_tol', 1e-12, 'Phir_tol', 1e-12));
  RS.I_EE_Task = Task_save;
  if any(abs(phi_new)>1e-12)
    error('IK des Startpunkts fehlgeschlagen');
  end
  q0 = q_new;
  
  [X,XD,XDD,T] = traj_trapez2_multipoint(XE, 1, 1e-1, 1e-2, 1e-3, 0.25);

  % Gelenkkräfte und inverse Kinematik berechnen
  [Q, QD, QDD,Phi_prev,~,Stats_prev] = RS.invkin2_traj(X,XD,XDD,T,q0,struct('n_max', 50, 'Phit_tol', 1e-4, 'Phir_tol', 1e-4 ));
  TAU = RS.invdyn_traj(Q, QD, QDD);
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

  %   for i = 4:4%size(XE,1) % Beispielpunkt aus Mail mit Moritz
  %   for i = 7:7%size(XE,1) % Wechselt bei iter = 58 von 179.68° auf -179.59° UND 10
  %   for i = 1:1%size(XE,1) % befindet sich am Start bereits innerhalb der Toleranz, dann geht alles gut
  %   for i = 2:2%size(XE,1) % Ähnlcihes Verhalten wie 4

    angle_nolimred = NaN(size(XE,1),1);
    angle_limred = NaN(size(XE,1),1);
    for i = 2:2%size(XE,1)
      RS.phiconv_W_E = i_phiconv;
      eulstr = euler_angle_properties(i_phiconv);

      % Ziel- und Anfangs-Konfiguration definieren
      RS.I_EE_Task = logical([1 1 1 1 1 1]);
      [q_ziel, phi_ziel] = RS.invkin2(RS.x2tr(XE(i,:)'), q0, struct('n_max', 2500, ...
        'Phit_tol', Phirt_tol, 'Phir_tol', Phirt_tol, 'reci', false));
      T_E_ziel = RS.fkineEE(q_ziel);
      %     qs = q_ziel-20*pi/180*(0.5-rand(RS.NQJ,1)); % Anfangswinkel 20° neben der Endstellung
      %     q_start = qs;
      q_start = q0;
      T_E_start = RS.fkineEE(q_start);

      % Einmalig Auftreffwinkel auf XY-Ebene berechnen
      Tr0E_soll = RS.x2tr(XE(i,:)');
      e_z_OE = Tr0E_soll(:,3);
      angle_Z_XY_value = asin(abs(sum(e_z_OE.*[0;0;1]))/(sqrt(e_z_OE(1)^2 + ...
        e_z_OE(2)^2 + e_z_OE(3)^2)*sqrt(1)))/pi*180;
      if abs(angle_Z_XY_value) <= 45
        xz_modus_temp = true;
      else
        xz_modus_temp = false;
      end

      % Aufgaben-FHG auf 3T2R
      RS.I_EE_Task = logical([1 1 1 1 1 0]);

      s_wn_3T2R_regular = [0.99;0.01];
      s_wn_3T2R_cond    = 0;
      s_wn_3T2R_coll    = 0;
      s_wn_3T2R_limred  = 0;
      s_wn_3T2R_ges     = [s_wn_3T2R_regular; s_wn_3T2R_cond; s_wn_3T2R_coll; s_wn_3T2R_limred];
      s_ep_3T2R = struct( ...
        'n_min', 0, 'n_max', 2500, 'Phit_tol', Phirt_tol, 'Phir_tol', Phirt_tol, ...
        'scale_lim', 0, 'reci', true, 'wn', s_wn_3T2R_ges, 'retry_limit', 0);

      s_wn_limred_regular = [0;0];
      s_wn_limred_cond    = 0;
      s_wn_limred_coll    = 0;
      s_wn_limred_limred  = 5;
      s_wn_limred_ges     = [s_wn_limred_regular; s_wn_limred_cond; s_wn_limred_coll; s_wn_limred_limred];
      s_ep_limred = s_ep_3T2R;
      s_ep_limred.wn = s_wn_limred_ges; % wn(5) muss getestet werden
      s_ep_limred.xlim = [NaN(5,2); [-45 45]*pi/180]; % in [rad] übergeben


      % IK aufrufen, um Residuum zu bekommen
      [q_IK_nolimred, phi_IK_nolimred, ~, Stats_nolimred] = RS.invkin2(RS.x2tr(XE(i,:)'), q0, s_ep_3T2R);
  %     angle_nolimred(i) = Stats_nolimred.PHI(Stats_nolimred.iter,4)*180/pi;
      [q_IK, phi_IK, ~, Stats] = RS.invkin2(RS.x2tr(XE(i,:)'), q0, s_ep_limred);
  %     angle_limred(i) = Stats.PHI(Stats.iter,4)*180/pi;
  %     Stats_limred_save(i) = Stats; % für spätere Auswertung sichern

      if any(abs(phi_IK_nolimred) > Phirt_tol)
        error('Inverse Kinematik OHNE limred fehlerhaft');
      end
      if any(abs(phi_IK) > Phirt_tol)
        error('Inverse Kinematik MIT limred fehlerhaft');
      end

      % Plots
      % Start(rot), Ziel aus Planung(blau) und Ziel aus IK(grün)
      % Roboter in Grundstellung plotten (mit im Gelenkraum entworfener Trajektorie)
      qref = q0;
      s_plot = struct( 'ks', [RS.NJ+2, RS.NJ+2], 'straight', 0);
      fig_count = fig_count+1;
      figure(fig_count);clf;
      hold on;
      grid on;
      xlabel('x [m]');
      ylabel('y [m]');
      zlabel('z [m]');
      view(3);
      RS.plot( qref, s_plot );
      title(sprintf('rot. Residuum | Testpunkt %d | phiconv %d', i, i_phiconv));
      % Schnittpunkt mit XY-Ebene bestimmen / Startkonfiguration
      t_0E    = T_E_start(1:3,4);
      e_z_OE  = T_E_start(1:3,3);
      g_0E = zeros(3,1);
      if xz_modus_temp == false
        lambda = (-t_0E(3))/e_z_OE(3);
        g_0E(1) = t_0E(1) + lambda*e_z_OE(1);
        g_0E(2) = t_0E(2) + lambda*e_z_OE(2);
        g_0E(3) = 0;
      elseif xz_modus_temp == true
        lambda = (-t_0E(2))/e_z_OE(2);
        g_0E(1) = t_0E(1) + lambda*e_z_OE(1);
        g_0E(2) = 0;
        g_0E(3) = t_0E(3) + lambda*e_z_OE(3);
      else
        error('Fehler mit XZ_Modus');
      end
      p_intersect_start = g_0E;
      % Plot Schnittpunkt Gerade EE-Z-Achse zu KS0-XY-Ebene
      plot3(  [T_E_start(1,4);p_intersect_start(1)],...
        [T_E_start(2,4);p_intersect_start(2)],...
        [T_E_start(3,4);p_intersect_start(3)],'r')
      % Plot Schnittpunkt Gerade EE-Z-Achse / KS0-XY-Ebene bzw. XZ
      if xz_modus_temp == true
        plot3(p_intersect_start(1), 0, p_intersect_start(3),'or')
      else
        plot3(p_intersect_start(1), p_intersect_start(2), 0,'or')
      end
      txt = '\leftarrow Startpose';
      pos_test = T_E_start(1:3,4) + 1/2*(p_intersect_start-T_E_start(1:3,4));
      text(pos_test(1),pos_test(2),pos_test(3),txt)

      % Trajektorienpunkte
      for kk = 1:size(XE,1)
        hold on;
        plot3(XE(kk,1), XE(kk,2), XE(kk,3),'xb', 'LineWidth',2.0)
      end

      % Roboter in Endstellung aus Planung plotten
      qref2 = q_ziel;
      s_plot2 = struct( 'ks', [RS.NJ+2, RS.NJ+2], 'straight', 0);
      hold on;
      grid on;
      view(3);
      RS.plot( qref2, s_plot2 );
      % Schnittpunkt mit XY-Ebene bestimmen / Zielkonfiguration
      t_0E    = T_E_ziel(1:3,4);
      e_z_OE  = T_E_ziel(1:3,3);
      g_0E = zeros(3,1);
      if xz_modus_temp == false
        lambda = (-t_0E(3))/e_z_OE(3);
        g_0E(1) = t_0E(1) + lambda*e_z_OE(1);
        g_0E(2) = t_0E(2) + lambda*e_z_OE(2);
        g_0E(3) = 0;
      elseif xz_modus_temp == true
        lambda = (-t_0E(2))/e_z_OE(2);
        g_0E(1) = t_0E(1) + lambda*e_z_OE(1);
        g_0E(2) = 0;
        g_0E(3) = t_0E(3) + lambda*e_z_OE(3);
      else
        error('Fehler mit XZ_Modus');
      end
      p_intersect_ziel_planung = g_0E;
      % Plot Schnittpunkt Gerade EE-Z-Achse zu KS0-XY-Ebene
      plot3( [T_E_ziel(1,4);p_intersect_ziel_planung(1)],...
        [T_E_ziel(2,4);p_intersect_ziel_planung(2)],...
        [T_E_ziel(3,4);p_intersect_ziel_planung(3)],'b')
      % Plot Schnittpunkt Gerade EE-Z-Achse / KS0-XY-Ebene bzw. XZ
      if xz_modus_temp == true
        plot3(p_intersect_ziel_planung(1), 0, p_intersect_ziel_planung(3),'ob')
      else
        plot3(p_intersect_ziel_planung(1), p_intersect_ziel_planung(2), 0,'ob')
      end
      txt = '\leftarrow Zielpose aus Vorgabe';
      pos_test = T_E_ziel(1:3,4) + 1/2*(p_intersect_ziel_planung-T_E_ziel(1:3,4));
      text(pos_test(1),pos_test(2),pos_test(3),txt)

      % Plot Translatorisches Residuum als Linie
      plot3(  [p_intersect_start(1);p_intersect_ziel_planung(1)],...
        [p_intersect_start(2);p_intersect_ziel_planung(2)],...
        [p_intersect_start(3);p_intersect_ziel_planung(3)],'m')
      txt = '\leftarrow translatorisches Residuum zum Ziel';
      pos_test = p_intersect_start + 1/2*(p_intersect_ziel_planung-p_intersect_start);
      text(pos_test(1),pos_test(2),pos_test(3),txt)

      % Roboter in Zielstellung aus IK plotten
      q_test = q_IK;
      qref3 = q_test;
      s_plot3 = struct( 'ks', [RS.NJ+2, RS.NJ+2], 'straight', 0);
      hold on;
      grid on;
      view(3);
      RS.plot( qref3, s_plot3 );
      T_E_ziel_neu = RS.fkineEE(q_test);
      % Schnittpunkt mit XY-Ebene bestimmen
      t_0E    = T_E_ziel_neu(1:3,4);
      e_z_OE  = T_E_ziel_neu(1:3,3);
      g_0E = zeros(3,1);
      if xz_modus_temp == false
        lambda = (-t_0E(3))/e_z_OE(3);
        g_0E(1) = t_0E(1) + lambda*e_z_OE(1);
        g_0E(2) = t_0E(2) + lambda*e_z_OE(2);
        g_0E(3) = 0;
      elseif xz_modus_temp == true
        lambda = (-t_0E(2))/e_z_OE(2);
        g_0E(1) = t_0E(1) + lambda*e_z_OE(1);
        g_0E(2) = 0;
        g_0E(3) = t_0E(3) + lambda*e_z_OE(3);
      else
        error('Fehler mit XZ_Modus');
      end
      p_intersect_ziel_IK = g_0E;
      % Plot Schnittpunkt Gerade EE-Z-Achse zu KS0-XY-Ebene
      plot3(  [T_E_ziel_neu(1,4);p_intersect_ziel_IK(1)],...
        [T_E_ziel_neu(2,4);p_intersect_ziel_IK(2)],...
        [T_E_ziel_neu(3,4);p_intersect_ziel_IK(3)],'g')
      txt = '\leftarrow Zielpose aus 3T2R-IK';
      pos_test = T_E_ziel_neu(1:3,4) + 1/3*(p_intersect_ziel_IK-T_E_ziel_neu(1:3,4));
      text(pos_test(1),pos_test(2),pos_test(3),txt)
      % Plot Schnittpunkt Gerade EE-Z-Achse / KS0-XY-Ebene bzw. XZ
      if xz_modus_temp == true
        plot3(p_intersect_ziel_IK(1), 0, p_intersect_ziel_IK(3),'og')
      else
        plot3(p_intersect_ziel_IK(1), p_intersect_ziel_IK(2), 0,'og')
      end

      % Berechne Abweichung der Z-Rotation
      %     zrot_diff_rad = q_ziel(6) - q_IK(6);
      %     zrot_diff_grad = zrot_diff_rad/pi*180;
      zrot_diff_grad = Stats.PHI(Stats.iter,4)*180/pi;
      txt = ['\Delta_{phiz} : ' num2str(zrot_diff_grad) '^°'];
      text(0,0,0,txt)
    end

    %% Auswertung Traj als Einzelpunkte
  %   %ToDO: Schleife implementieren, damit mehrere Punkte nacheinander
  %   %überprüft werden. Für erste Tests nicht nur Trajektorie verwenden, sondern ~100? Punkte im Arbeitsbereich
  %   Pkt_Nr = [1:1:size(TSS.n)]';
  %   % Auswertung hier, damit übersichtlicher
  %   limred_success   = NaN(size(angle_limred,1),1);
  %   nolimred_success = NaN(size(angle_nolimred,1),1);
  %   for kk = 1:size(angle_limred,1)
  %     if (angle_limred(kk) >= s_ep_limred.xlim(6,1)) && (angle_limred(kk) <= s_ep_limred.xlim(6,2))
  %       limred_success(kk) = 1;
  %       if (angle_nolimred(kk) >= s_ep_limred.xlim(6,1)) && (angle_nolimred(kk) <= s_ep_limred.xlim(6,2))
  %         nolimred_success(kk) = 1;
  %       end
  %     else
  %       % wird durch Initialisierung mit NaN abgebildet
  %     end
  %   end
  %   T = table(Pkt_Nr,angle_nolimred,angle_limred,limred_success)
  %   
  %   sucess_chance = sum(limred_success == 1)/size(TSS.n)*100;
  %   fprintf('Von %d verschiedenen Punkten konnten %d korrekt in die Grenze bzgl. xlim bewegt werden.\n', size(TSS.n), limred_success); 
  %   fprintf('%d Punkte waren bereits ohne limred innerhalb der Grenzen.\n', sum(nolimred_success == 1)); 
  %   fprintf('Dies entspricht einer Wahrscheinlichkeit von %d durch limred OHNE die bereits vorher zulässigen phi_z.\n', sucess_chance);


    %% Visualisierung von phi_z für ausgesuchte Punkte

    j = [4];    % Gewählte Punkte
    workingStats = Stats;
    index_phiz = 1:workingStats.iter;
    fighdl = change_current_figure(100*2+1);clf;
    set(100*2+1, 'Name', sprintf('Verlauf von phi_z für ausgewählte Punkte über die gesamte Iteration'), 'NumberTitle', 'off', 'Position', get(0, 'Screensize'));
    sgtitle('Verlauf von phi_z und h');

    % Plot von phi_z_gesamt
    subplot(2,4,1); hold on;
  %   plot(index_phiz, Stats_limred_save(j(1)).PHI(:,4)*180/pi, 'r--', 'LineWidth',2.0);
  %   plot(index_phiz, Stats_limred_save(j(2)).PHI(:,4)*180/pi, 'b--', 'LineWidth',2.0);
  %   plot(index_phiz, Stats_limred_save(j(3)).PHI(:,4)*180/pi, 'm--', 'LineWidth',2.0);
    hdl1 = plot(index_phiz, workingStats.PHI(1:index_phiz(end),4)*180/pi, 'm', 'LineWidth',1);
    xlim_vek = repmat([s_ep_limred.xlim(6,1) s_ep_limred.xlim(6,2)]*180/pi,index_phiz(end),1);
    plot(index_phiz, xlim_vek(:,1), 'r--', 'LineWidth',1);
    plot(index_phiz, xlim_vek(:,2), 'r--', 'LineWidth',1);
    hdl1.Parent.XLim = [1 index_phiz(end)];
    hdl1.Parent.YLim = [min(workingStats.PHI(1:index_phiz(end),4)*180/pi)-5 max(workingStats.PHI(1:index_phiz(end),4)*180/pi)+5];
  %   title('phi_z');
    ylabel('phi_z in °'); grid on;


    % Plot von h(5)_gesamt
    subplot(2,4,5); hold on;
    hdl2 = plot(index_phiz, workingStats.h(1:index_phiz(end),6), 'r', 'LineWidth',1);
    hdl2.Parent.XLim = hdl1.Parent.XLim ;
    hold on;
    h5_new = workingStats.h(1:index_phiz(end),6);
    leg = zeros(1,2);
    for kk = 1:index_phiz(end)
      if isinf(h5_new(kk))
        leg(1) = plot(kk, 0,'*g', 'DisplayName','h = inf');
      end
      if kk > 1 && kk < index_phiz(end)
        if isinf(h5_new(kk-1)) && isinf(h5_new(kk+1))
          leg(2) = plot(kk, h5_new(kk),'*b', 'DisplayName','h linksrechts = inf');
        end
      end
    end
  %   title('Optimierungskriterium h');
    ylabel('Optim.krit h(5)'); grid on;
    xlabel('Iteration lfd Nr');
  %   legend(leg(1:2));
    legend(leg(1:2), 'AutoUpdate','off');

    % Plot von phi_z_range1
    subplot(2,4,2); hold on;
  %   plot(index_phiz, Stats_limred_save(j(1)).PHI(:,4)*180/pi, 'r--', 'LineWidth',2.0);
  %   plot(index_phiz, Stats_limred_save(j(2)).PHI(:,4)*180/pi, 'b--', 'LineWidth',2.0);
  %   plot(index_phiz, Stats_limred_save(j(3)).PHI(:,4)*180/pi, 'm--', 'LineWidth',2.0);
    hdl3 = plot(index_phiz, workingStats.PHI(1:index_phiz(end),4)*180/pi, 'm', 'LineWidth',1);
    xlim_vek = repmat([s_ep_limred.xlim(6,1) s_ep_limred.xlim(6,2)]*180/pi,index_phiz(end),1);
    plot(index_phiz, xlim_vek(:,1), 'r--', 'LineWidth',1);
    plot(index_phiz, xlim_vek(:,2), 'r--', 'LineWidth',1);
    grid on;
    hdl3.Parent.XLim = [12 50];
    hdl3.Parent.YLim = [0 50];

    % Plot von h(5)_range1
    subplot(2,4,6); hold on;
    hdl4 = plot(index_phiz, workingStats.h(1:index_phiz(end),6), 'r', 'LineWidth',1);
    hdl4.Parent.XLim = hdl3.Parent.XLim ;
    hold on;
    h5_new = workingStats.h(1:index_phiz(end),6);
    for kk = 1:index_phiz(end)
      if isinf(h5_new(kk))
        plot(kk, 0,'*g', 'DisplayName','h = inf');
      end
      if kk > 1 && kk < index_phiz(end)
        if isinf(h5_new(kk-1)) && isinf(h5_new(kk+1))
          plot(kk, h5_new(kk),'*b', 'DisplayName','h linksrechts = inf');
        end
      end
    end
    grid on;

    % Plot von phi_z_range2
    subplot(2,4,3); hold on;
  %   plot(index_phiz, Stats_limred_save(j(1)).PHI(:,4)*180/pi, 'r--', 'LineWidth',2.0);
  %   plot(index_phiz, Stats_limred_save(j(2)).PHI(:,4)*180/pi, 'b--', 'LineWidth',2.0);
  %   plot(index_phiz, Stats_limred_save(j(3)).PHI(:,4)*180/pi, 'm--', 'LineWidth',2.0);
    hdl5 = plot(index_phiz, workingStats.PHI(1:index_phiz(end),4)*180/pi, 'm', 'LineWidth',1);
    xlim_vek = repmat([s_ep_limred.xlim(6,1) s_ep_limred.xlim(6,2)]*180/pi,index_phiz(end),1);
    plot(index_phiz, xlim_vek(:,1), 'r--', 'LineWidth',1);
    plot(index_phiz, xlim_vek(:,2), 'r--', 'LineWidth',1);
    grid on;
    hdl5.Parent.XLim = [40 140];
    hdl5.Parent.YLim = [40 50];

    % Plot von h(5)_range2
    subplot(2,4,7); hold on;
    hdl6 = plot(index_phiz, workingStats.h(1:index_phiz(end),6), 'r', 'LineWidth',1);
    hdl6.Parent.XLim = hdl5.Parent.XLim ;
    hold on;
    h5_new = workingStats.h(1:index_phiz(end),6);
    for kk = 1:index_phiz(end)
      if isinf(h5_new(kk))
        plot(kk, 0,'*g', 'DisplayName','h = inf');
      end
      if kk > 1 && kk < index_phiz(end)
        if isinf(h5_new(kk-1)) && isinf(h5_new(kk+1))
          plot(kk, h5_new(kk),'*b', 'DisplayName','h linksrechts = inf');
        end
      end
    end
    grid on;

    % Plot von phi_z_range3
    subplot(2,4,4); hold on;
  %   plot(index_phiz, Stats_limred_save(j(1)).PHI(:,4)*180/pi, 'r--', 'LineWidth',2.0);
  %   plot(index_phiz, Stats_limred_save(j(2)).PHI(:,4)*180/pi, 'b--', 'LineWidth',2.0);
  %   plot(index_phiz, Stats_limred_save(j(3)).PHI(:,4)*180/pi, 'm--', 'LineWidth',2.0);
    hdl6 = plot(index_phiz, workingStats.PHI(1:index_phiz(end),4)*180/pi, 'm', 'LineWidth',1);
    xlim_vek = repmat([s_ep_limred.xlim(6,1) s_ep_limred.xlim(6,2)]*180/pi,index_phiz(end),1);
    plot(index_phiz, xlim_vek(:,1), 'r--', 'LineWidth',1);
    plot(index_phiz, xlim_vek(:,2), 'r--', 'LineWidth',1);
    grid on;
    hdl6.Parent.XLim = [130 index_phiz(end)];
    hdl6.Parent.YLim = [40 50];

    % Plot von h(5)_range3
    subplot(2,4,8); hold on;
    hdl7 = plot(index_phiz, workingStats.h(1:index_phiz(end),6), 'r', 'LineWidth',1);
    hdl7.Parent.XLim = hdl6.Parent.XLim ;
    hold on;
    h5_new = workingStats.h(1:index_phiz(end),6);
    for kk = 1:index_phiz(end)
      if isinf(h5_new(kk))
        plot(kk, 0,'*g', 'DisplayName','h = inf');
      end
      if kk > 1 && kk < index_phiz(end)
        if isinf(h5_new(kk-1)) && isinf(h5_new(kk+1))
          plot(kk, h5_new(kk),'*b', 'DisplayName','h linksrechts = inf');
        end
      end
    end
    grid on;
  
  end
  
  
  %% Trajektorie
  
  if calc_traj
    fprintf('Teste gesamte Trajektorie\n');
    fprintf('ACHTUNG! Verwendung von korrekten Structs für Traj überprüfen!\n');
    % Gelenkkräfte und inverse Kinematik berechnen
    Phirt_tol = 1e-8;
    RS.I_EE_Task = logical([1 1 1 1 1 0]);
    
    % reguläre 3T2R mit "Standardoptimierung"
    s_traj_3T2R_regular       = [0.99;0.01;20;0];   % qlim quad+hyp wn(1+2), qDlim quad+hyp wn(3+4)
    s_traj_3T2R_cond_v1       = 0;                  % Konditionszahl wn(5)
    s_traj_3T2R_qlim_v2       = [0;0];              % qlim quad wn(6) und qlim hyp wn(7)
    s_traj_3T2R_cond_v2       = 0;                  % Konditionszahl wn(8)
    s_traj_3T2R_collision     = [0;0];              % Kollisionsvermeidung wn(9+10)
    s_traj_3T2R_limredcoord   = [0;0;0];            % wn(11+12) für qD und qDD und wn(13) für qD
    s_traj_3T2R_ges = [s_traj_3T2R_regular; s_traj_3T2R_cond_v1; s_traj_3T2R_qlim_v2; s_traj_3T2R_cond_v2; ...
                                  s_traj_3T2R_collision; s_traj_3T2R_limredcoord];
    s_traj_3T2R   = struct('n_min', 50, 'n_max', 1500, 'Phit_tol', Phirt_tol, 'Phir_tol', Phirt_tol, ...
                           'reci', true, 'I_EE', RS.I_EE_Task, 'wn', s_traj_3T2R_ges);
    
    % limredcoord: phi_z begrenzen
    s_traj_limred_regular       = [0;0;0;0];   % qlim quad+hyp wn(1+2), qDlim quad+hyp wn(3+4)
    s_traj_limred_cond_v1       = 0;                  % Konditionszahl wn(5)
    s_traj_limred_qlim_v2       = [0;0];              % qlim quad wn(6) und qlim hyp wn(7)
    s_traj_limred_cond_v2       = 0;                  % Konditionszahl wn(8)
    s_traj_limred_collision     = [0;0];              % Kollisionsvermeidung wn(9+10)
%     s_traj_limred_limredcoord   = [5;5;5];            % wn(11+12) für qD und qDD und wn(13) für qD
    s_traj_limred_limredcoord   = [10;20;30];          % wn(11+12) für qD und qDD und wn(13) für qD
    s_traj_limred_ges = [s_traj_limred_regular; s_traj_limred_cond_v1; s_traj_limred_qlim_v2; s_traj_limred_cond_v2; ...
                                  s_traj_limred_collision; s_traj_limred_limredcoord];                                     
    s_traj_limred = struct('n_min', 50, 'n_max', 1500, 'Phit_tol', Phirt_tol, 'Phir_tol', Phirt_tol, ...
                           'reci', true, 'I_EE', RS.I_EE_Task, 'wn', s_traj_limred_ges);  
                         
    s_traj_limred.xlim   = [NaN(5,2); [-45 45]*pi/180]; % in [rad] übergeben
    s_traj_limred.xDlim  = [NaN(5,2); [-0.21 0.21]]; % 0.21rad/s = 2rpm laut unitconversion.io/de/rpm-zu-rads-konvertierung
    s_traj_limred.xDDlim = [NaN(5,2); [-21 21]]; % vorläufige Konvertierung wie 4*pi zu 100 bei qD und qDD 
    
    
    [Q_nolimred, QD_nolimred, QDD_nolimred, Phi_nolimred, ~, Stats_Traj_nolimred] = RS.invkin2_traj(X,XD,XDD,T,q0,s_traj_3T2R);
    [Q_limred, QD_limred, QDD_limred, Phi_limred, ~, Stats_Traj_limred] = RS.invkin2_traj(X,XD,XDD,T,q0,s_traj_limred);
    
    [X_ist_nolimred, XD_ist_nolimred, ~] = RS.fkineEE_traj(Q_nolimred, QD_nolimred, QDD_nolimred);
    X_ist_nolimred_int = repmat(X_ist_nolimred(1,:), length(T), 1) + cumtrapz(T, XD_ist_nolimred);
    X_ist_nolimred(:,4:6) = normalizeAngle(X_ist_nolimred(:,4:6), X_ist_nolimred_int(:,4:6));
    
    [X_ist_limred, XD_ist_limred, ~] = RS.fkineEE_traj(Q_limred, QD_limred, QDD_limred);
    X_ist_limred_pre = X_ist_limred;
    XD_ist_limred_pre = XD_ist_limred;
    X_ist_limred_int = repmat(X_ist_limred(1,:), length(T), 1) + cumtrapz(T, XD_ist_limred);
    X_ist_limred(:,4:6) = normalizeAngle(X_ist_limred(:,4:6), X_ist_limred_int(:,4:6));  
    
    % IK-Ergebnis testen
    for i = 1:length(T)
      phi_test_voll_nolimred = RS.constr2(Q_nolimred(i,:)', RS.x2tr(X(i,:)'), true);
      phi_test_voll_limred = RS.constr2(Q_limred(i,:)', RS.x2tr(X(i,:)'), true);
      
      if RS.I_EE_Task == logical([1 1 1 1 1 0])
        I_IK2 = [1 2 3 6 5 4];
      else
        error('Achtung falscher FHG?');
      end
      I_IK = I_IK2(RS.I_EE_Task);
      phi_test_red_nolimred  = phi_test_voll_nolimred(I_IK);
      phi_test_red_limred  = phi_test_voll_limred(I_IK);
      if max(abs(phi_test_red_nolimred)) > Phirt_tol
        error('IK OHNE Limitierung stimmt nicht');
      end
      if max(abs(phi_test_red_limred)) > Phirt_tol
        error('IK MIT Limitierung stimmt nicht');
      end
    end
    for i = 1:length(T)
      phi_test_voll = RS.constr2(Q_limred(i,:)', RS.x2tr(X(i,:)'), true);
%       phi_z_limred(i) = phi_test_voll(4);
      if RS.I_EE_Task == logical([1 1 1 1 1 0])
        I_IK2 = [1 2 3 6 5 4];
      else
        error('Achtung falscher FHG?');
      end
      I_IK = I_IK2(RS.I_EE_Task);
      phi_test_red  = phi_test_voll(I_IK);
      if max(abs(phi_test_red)) > Phirt_tol
        error('IK MIT Limitierung stimmt nicht');
      end
    end
    
    %% Auswertung für Plots
    phiz_traj_diff_nolimred = X(:,6) - X_ist_nolimred(:,6);
    phiz_traj_diff_limred   = X(:,6) - X_ist_limred(:,6);
    phiz_traj_diff_limred_pre   = X(:,6) - X_ist_limred_pre(:,6);
      
    for kk = 1:size(T,1)
      % Optimierungskriterium h7 und h8 für nolimred berechnen
      h7_nolimred(kk) = invkin_optimcrit_limits2(X_ist_nolimred(kk,6), [s_traj_limred.xlim(6,1) s_traj_limred.xlim(6,2)], ...
                                                                       [s_traj_limred.xlim(6,1) s_traj_limred.xlim(6,2)]*0.8);
      h8_nolimred(kk) = invkin_optimcrit_limits2(XD_ist_nolimred(kk,6), [s_traj_limred.xDlim(6,1) s_traj_limred.xDlim(6,2)], ...
                                                                        [s_traj_limred.xDlim(6,1) s_traj_limred.xDlim(6,2)]*0.8);
                                                                      
      % Optimierungskriterium h7 und h8 für limred berechnen -> zur Überprüfung
      h7_limred(kk) = invkin_optimcrit_limits2(X_ist_limred(kk,6), [s_traj_limred.xlim(6,1) s_traj_limred.xlim(6,2)], ...
                                                                   [s_traj_limred.xlim(6,1) s_traj_limred.xlim(6,2)]*0.8);
      h8_limred(kk) = invkin_optimcrit_limits2(XD_ist_limred(kk,6), [s_traj_limred.xDlim(6,1) s_traj_limred.xDlim(6,2)], ...
                                                                    [s_traj_limred.xDlim(6,1) s_traj_limred.xDlim(6,2)]*0.8);                                                          
      h7_limred_pre(kk) = invkin_optimcrit_limits2(X_ist_limred_pre(kk,6), [s_traj_limred.xlim(6,1) s_traj_limred.xlim(6,2)], ...
                                                                           [s_traj_limred.xlim(6,1) s_traj_limred.xlim(6,2)]*0.8);
      h8_limred_pre(kk) = invkin_optimcrit_limits2(XD_ist_limred_pre(kk,6), [s_traj_limred.xDlim(6,1) s_traj_limred.xDlim(6,2)], ...
                                                                            [s_traj_limred.xDlim(6,1) s_traj_limred.xDlim(6,2)]*0.8);    
    end
    fprintf('Auswertung der Plots beendet\n');
    %% Plots
    figure(100*3+1);
    set(100*3+1, 'Name', sprintf('Verlauf von phi_z für gesamte Trajektorie OHNE limred'), 'NumberTitle', 'off', 'Position', get(0, 'Screensize'));
    sgtitle('Verlauf von phi_z und h');
    index_traj = 1:size(T,1);

    
    % ---------------------------- Subplot linke Seite -------------------
    % Plot von phi_z nolimred
    subplot(3,2,1); hold on;
    hdl1 = plot(index_traj, phiz_traj_diff_nolimred*180/pi, 'm', 'LineWidth',1);
    xlim_vek = repmat([s_traj_limred.xlim(6,1) s_traj_limred.xlim(6,2)]*180/pi,index_traj(end),1);
    plot(index_traj, xlim_vek(:,1), 'r--', 'LineWidth',1);
    plot(index_traj, xlim_vek(:,2), 'r--', 'LineWidth',1);
%     hdl1.Parent.XLim = [1 index_traj(end)];
%     hdl1.Parent.YLim = [min(Stats_Traj_nolimred.PHI(1:index_traj(end),4)*180/pi)-5 max(Stats_Traj_nolimred.PHI(1:index_traj(end),4)*180/pi)+5];
    title('nolimred');
    ylabel('phi_z in °'); grid on;
    
    % Plot von h(7) nolimred
    subplot(3,2,3); hold on;
    hdl2 = plot(index_traj, h7_nolimred, 'm', 'LineWidth',1);
%     hdl2.Parent.XLim = hdl1.Parent.XLim ;
    hold on;
    h7_new = h7_nolimred;
    leg1 = zeros(1,2);
    hinf_used = false;  % Abfrage bzgl. Legendeneintrag
    for kk = 1:index_traj(end)
      if isinf(h7_new(kk))
        hinf_used = true;
        leg1(1) = plot(kk, 0,'*g', 'DisplayName','h = inf');
      end
      if kk > 1 && kk < index_traj(end)
        if isinf(h7_new(kk-1)) && isinf(h7_new(kk+1))
          hinf_used = true;
          leg1(2) = plot(kk, h7_new(kk),'*b', 'DisplayName','h linksrechts = inf');
        end
      end
    end
    grid on;
    ylabel('Optim.krit h(7)'); grid on;
    xlabel('Iteration lfd Nr');
%     legend(leg(1:2));
    if hinf_used
      legend(leg1(1:2), 'AutoUpdate','off');
    end
    
    % Plot von h(8) nolimred
    subplot(3,2,5); hold on;
    hdl3 = plot(index_traj, h8_nolimred, 'm', 'LineWidth',1);
%     hdl3.Parent.XLim = hdl2.Parent.XLim ;
    hold on;
    h8_new = h8_nolimred;
    for kk = 1:index_traj(end)
      if isinf(h8_new(kk))
        plot(kk, 0,'*g', 'DisplayName','h = inf');
      end
      if kk > 1 && kk < index_traj(end)
        if isinf(h8_new(kk-1)) && isinf(h8_new(kk+1))
          plot(kk, h8_new(kk),'*b', 'DisplayName','h linksrechts = inf');
        end
      end
    end
    grid on;
    ylabel('Optim.krit h(8)'); grid on;
    xlabel('Iteration lfd Nr');

    % ---------------------------- Subplot rechte Seite -------------------
    % Plot von phi_z limred
    subplot(3,2,2); hold on;
    leg4 = zeros(1,2);
    leg4(1) = plot(index_traj, phiz_traj_diff_limred*180/pi, 'm', 'LineWidth',1);
    leg4(2) = plot(index_traj, phiz_traj_diff_limred_pre*180/pi, 'm--', 'LineWidth',1);
    plot(index_traj, xlim_vek(:,1), 'r--', 'LineWidth',1);
    plot(index_traj, xlim_vek(:,2), 'r--', 'LineWidth',1);
    %     hdl4.Parent.XLim = [1 index_traj(end)];
    %     hdl4.Parent.YLim = [min(Stats_Traj_limred.PHI(1:index_traj(end),4)*180/pi)-5 max(Stats_Traj_limred.PHI(1:index_traj(end),4)*180/pi)+5];
    title('limred');
    ylabel('phi_z in °'); grid on;
    legend(leg4(1:2), 'AutoUpdate','off');
    
    % Plot von h(7) limred
    leg2 = zeros(1,4);
    subplot(3,2,4); hold on;
    leg2(1) = plot(index_traj, Stats_Traj_limred.h(1:index_traj(end),8), 'm', 'LineWidth',1, 'DisplayName','h aus IK-Traj');
    leg2(2) = plot(index_traj, h7_limred, 'k', 'LineWidth',1, 'DisplayName','h aus X-ist mit integr');
    leg2(3) = plot(index_traj, h7_limred_pre, 'y--', 'LineWidth',1, 'DisplayName','h aus X-ist ohne integr');
    %     hdl5.Parent.XLim = hdl4.Parent.XLim ;
    hinf_used = false;  % Abfrage bzgl. Legendeneintrag
    for kk = 1:index_traj(end)
      if isinf(h7_limred(kk))
        hinf_used = true;
        leg2(4) = plot(kk, 0,'*g', 'DisplayName','h = inf');
      end
    end
    hold on;
    ylabel('Optim.krit h(7)'); grid on;
    xlabel('Iteration lfd Nr');
    %   legend(leg(1:2));
    %     legend(leg(1:2), 'AutoUpdate','off');
    if hinf_used
      legend(leg2(1:4), 'AutoUpdate','off');
    else
      legend(leg2(1:3), 'AutoUpdate','off');
    end
    
    % Plot von h(8) limred
    subplot(3,2,6); hold on;
    hdl6 = plot(index_traj, Stats_Traj_nolimred.h(1:index_traj(end),9), 'm', 'LineWidth',1);
    plot(index_traj, h8_limred, 'k', 'LineWidth',1);
    plot(index_traj, h8_limred_pre, 'y--', 'LineWidth',1);
    %     hdl6.Parent.XLim = hdl4.Parent.XLim ;
    hold on;
    ylabel('Optim.krit h(8)'); grid on;
    xlabel('Iteration lfd Nr');
    %   legend(leg(1:2));
    %     legend(leg(1:2), 'AutoUpdate','off');
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
    
    %% Visualisierung von h (und später phi_z) der Trajektorie
    
%     labels = NaN(size(Stats_Traj_limred.h,2)-6,1);  % 6 Kriterien gibt es bereits
%     index_phiz_traj = 1:size(Stats_Traj_limred.h,1);
%     change_current_figure(100*7+1);clf;
%     set(100*7+1, 'Name', sprintf('Traj: Verlauf von der Optimierungskriterien zur Limitierung von phi_z'), 'NumberTitle', 'off');
%     sgtitle('Verlauf von h(7) und h(8)');
%     
%     % Plot von h(7) und h(8)
%     subplot(2,1,1); hold on;
%     plot(index_phiz_traj, Stats_Traj_limred.h(:,7), 'r--', 'LineWidth',2.0);
%     plot(index_phiz_traj, Stats_Traj_limred.h(:,8), 'b--', 'LineWidth',2.0);
%     title('Optimierungskriterium h(7) und h(8)');
%     ylabel('Optimierungskriterium h'); grid on;
%     xlabel('Iteration lfd Nr');
%    
%     for kkk = 1:size(labels,1)
%       labels(kkk) = sprintf('Optim.Krit h(%d)', 6+kkk);
%     end
%     %   legend([hdl1, hdl2, hdl3], {'Ohne AR, 2°-Schritte.', 'Ohne AR, bester', 'Mit Aufgabenredundenz'});
%     legend(labels(1),labels(2));
    
  end


  
  
  
end
 