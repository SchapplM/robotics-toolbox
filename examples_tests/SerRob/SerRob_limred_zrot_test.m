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
respath = fileparts(which('SerRob_limred_zrot_test_v1.m'));

use_mex_functions = true; % mit mex geht es etwas schneller, dafür ist debuggen schwieriger
minimal_test = true; % Nur sehr wenige zufällige Winkel testen (geht schneller)
debug_mode = false;  % Diverse Plots schalten
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
  
  I_EE_3T3R = logical([1 1 1 1 1 1]);
  I_EE_3T2R = logical([1 1 1 1 1 0]);
  RS.I_EE = I_EE_3T3R;
  RS.I_EE_Task = I_EE_3T2R;
  
  fprintf('Starte Untersuchung für %s\n', RS.descr);
  
  %% Roboter in Nullstellung plotten (mit im Gelenkraum entworfener Trajektorie)
  if debug_mode
    s_plot = struct( 'ks', [RS.NJ+2, RS.NJ+2], 'straight', 0);
    change_current_figure(1);clf;
    hold on;
    grid on;
    xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
    view(3);
    RS.plot( zeros(RS.NJ,1), s_plot );
    title(sprintf('Nullstellung: %s', RS.descr));
  end
  
  %% Roboter in Grundstellung plotten (mit im Gelenkraum entworfener Trajektorie)
  if debug_mode
    % qref = [0;90;0;90;0;0]*pi/180;
    qref = RS.qref;
    s_plot = struct( 'ks', [RS.NJ+2, RS.NJ+2], 'straight', 0);
    change_current_figure(2);clf;
    hold on;
    grid on;
    xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
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
  RS.I_EE_Task = I_EE_3T3R;
  [q0_new, phi_new, ~, Stats_new] = RS.invkin2(RS.x2tr(XE(1,:)'+[0;0;0;0;0;-80*pi/180]), q0, struct('retry_on_limitviol', true, ...
                                                                                             'Phit_tol', 1e-12, 'Phir_tol', 1e-12));
  % bei Bedarf kann über/unter dem ersten Punkt begonnen werden
%   [q0_new, phi_new, ~, Stats_new] = RS.invkin2(RS.x2tr(XE(1,:)'+[0;0;0.1;0;0;0]), q0, struct('retry_on_limitviol', true, ...
%                                                                                             'Phit_tol', 1e-12, 'Phir_tol', 1e-12));
  RS.I_EE_Task = Task_save;
  if any(abs(phi_new)>1e-12)
    error('IK des Startpunkts fehlgeschlagen');
  end
  q0 = q0_new;
  
  % Startkonfiguration q0 auf Verletzung der Grenzen testen 
  q0norm = (q0-qlim(:,1)) ./ (qlim(:,2) - qlim(:,1));
  if any(q0norm >= 1) || any(q0norm <= 0) % Winkel mit +- 2*pi berücksichtigen
    error('q0 verletzt bereits die Grenzen');
    q0norm'
  end

  % Alle Soll-Punkte der Trajektorie auf Verletzung der Grenzen testen 
  Task_save = RS.I_EE_Task;
  RS.I_EE_Task = I_EE_3T3R;
  q_nn = zeros(6,size(XE,1));
  qnn_norm = zeros(6,size(XE,1));
  for nn = 1:size(XE,1)
    if nn == 1
      [q_nn(:,1), ~, ~, Stats_nn] = RS.invkin2(RS.x2tr(XE(1,:)'), q0, struct('retry_on_limitviol', false, ...
                                                                             'Phit_tol', 1e-12, 'Phir_tol', 1e-12));
    else
      [q_nn(:,nn), ~, ~, Stats_nn] = RS.invkin2(RS.x2tr(XE(nn,:)'), q_nn(:,nn-1), struct('retry_on_limitviol', false, ...
                                                                                         'Phit_tol', 1e-12, 'Phir_tol', 1e-12)); 
    end
    qnn_norm(:,nn) = (q_nn(:,nn)-qlim(:,1)) ./ (qlim(:,2) - qlim(:,1));
    if any(qnn_norm(:,nn) >= 1) || any(qnn_norm(:,nn) <= 0) % Winkel mit +- 2*pi berücksichtigen
      warning('qnn für Punkt %d verletzt die Grenzen', nn);
      qnn_norm(:,nn)'
    end
  end
  RS.I_EE_Task = Task_save;
  
  [X,XD,XDD,T] = traj_trapez2_multipoint(XE, 1, 1e-1, 1e-2, 1e-3, 0.25);    % Trajektorie interpolieren

  % Trajektorie: IK mit 3T2R ohne Zielkriterien
  [Q, QD, QDD,Phi_prev,~,Stats_prev] = RS.invkin2_traj(X,XD,XDD,T,q0,struct('n_max', 50, 'Phit_tol', 1e-4, 'Phir_tol', 1e-4 ));
  
  % IK-Ergebnis testen
  for i = 1:length(T)
    phi_test_voll = RS.constr2(Q(i,:)', RS.x2tr(X(i,:)'), true);
    I_IK2 = [1 2 3 6 5 4];
    I_IK = I_IK2(RS.I_EE_Task);
    phi_test_red  = phi_test_voll(I_IK);
    if max(abs(phi_test_red)) > 1e-4
      error('Traj-IK für 3T2R ohne Zielkriterien stimmt nicht.');
    end
  end
  
  %% Roboter in Grundstellung plotten (mit im Arbeitsraum entworfener Trajektorie)
  if debug_mode
    s_plot = struct( 'ks', [RS.NJ+2, RS.NJ+2], 'straight', 0);
    change_current_figure(6);clf;
    hold on;
    grid on;
    xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
    view(3);
    RS.plot( q0, s_plot );
    title(sprintf('Grundstellung: %s', RS.descr));
    plot3(X(:,1), X(:,2), X(:,3));
    RS.plot( RS.qref, s_plot );
    
    if traj_anim
      s_anim = struct( 'gif_name', '');
      change_current_figure(4);clf;
      hold on;
      plot3(X(:,1), X(:,2), X(:,3));
      grid on;
      xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
      view(3);
      title('Animation der Gelenktrajektorie');
      RS.anim( Q(1:50:end,:), [], s_anim, s_plot);
    end
  end
  
  %% Roboter in Grundstellung mit Koordinatensystemen
  % Die Verwendung von ['mode', 2] bei Robotern ohne hinterlegtes
  % CAD-Modell eignet sich gut, um die einzlenen Koordinatensysteme der
  % Trajektorienpunkte zu plotten
  
  if debug_mode
    s_plot = struct( 'ks', [RS.NJ+2, RS.NJ+2], 'straight', 0);
    s_plot_KS = struct( 'ks', [8, RS.NJ+2], 'straight', 0, 'mode', 2);
    change_current_figure(7);clf;
    hold on;
    grid on;
    xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
    view(3);
    % Plot des Roboters mit Trajektorie
    RS.plot( q0, s_plot );
    title(sprintf('Grundstellung: %s', RS.descr));
    plot3(X(:,1), X(:,2), X(:,3));
    RS.plot( RS.qref, s_plot );
    
    Task_save = RS.I_EE_Task;
    RS.I_EE_Task = I_EE_3T3R;
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
    
    % Plot der Koordinatensysteme
    for iii = 1:size(XE,1)
      hold on;
      RS.plot( q_ii_save(:,iii), s_plot_KS );
    end
    
  end
  
  %% Einzelpunkt-IK | Initialisierung und Berechnung
  fprintf('Starte Berechnungen für Trajektorie als Einzelpunkte\n');
  i_phiconv = uint8([2]);
  Phirt_tol = 1e-10; % Fehler der IK mit 1e-12 (vermutlich durch Ungenauigkeit bei DLS)
  n_max = 2500;
  amount_optcrit = 1+2; % 1(3T2R) + 2(limred hyp + quadhyp)
  
  % Initialisierung
  h6_ep = NaN(1+n_max, amount_optcrit, size(XE,1)); % h7 kann auch ausgelesen werden, wenn wn(7) nicht aktiv ist
  h7_ep = h6_ep;
  phiz_ep = h6_ep;
  iter_ep = NaN(size(XE,1), amount_optcrit);
  Q_ep = NaN(1+n_max, 6, size(XE,1), amount_optcrit);
  limred_crit_active = zeros(1+n_max, amount_optcrit, size(XE,1));
  warning_iter_count = 0;
  warning_phi_count = 0;
  warnplot_needed_vector = NaN;
  warnplot_needed_vector_counter = 1;
  warnplot_needed_state = 0;
  phiz_oob         = zeros(amount_optcrit-1,size(XE,1));  % oob: out-of-bounds bzgl. xlim
  phiz_oob_vector  = NaN;
  phiz_oob_vector_counter = 1;
  phiz_oob_state = 0;
  phiz_oob_thresh  = zeros(amount_optcrit-1,size(XE,1));  % zwischen xlim und 0.8*xlim
  phiz_oob_thresh_vector  = NaN;
  phiz_oob_thresh_vector_counter = 1;
  phiz_oob_thresh_state = 0;
  
  for i = 1:size(XE,1)
    RS.phiconv_W_E = i_phiconv;
    eulstr = euler_angle_properties(i_phiconv);
    RS.I_EE_Task = I_EE_3T2R; % Aufgaben-FHG auf 3T2R
    
    % Start-Gelenkwinkel
%     q0_ep = RS.qref + [0;0;0;0;0;-80]*pi/180; % spez. Wert festlegen
    q0_ep = RS.qref - 20*pi/180*(0.5-rand(RS.NQJ,1)); % Anfangswinkel 20° variieren

    % Structs definieren
    s_ep_wn = zeros(RS.idx_ik_length.wnpos,amount_optcrit);  % 3T2R mit wn(:) = 0
    % limred mit hyperbolischem Ansatz
    s_ep_wn(RS.idx_ikpos_wn.xlim_par,2) = 0;
    s_ep_wn(RS.idx_ikpos_wn.xlim_hyp,2) = 1;
    % limred mit quadratischem und hyperbolischem Ansatz
    s_ep_wn(RS.idx_ikpos_wn.xlim_par,3) = 1;
    s_ep_wn(RS.idx_ikpos_wn.xlim_hyp,3) = 1;
    opt1 = s_ep_wn(6:7,2);   % Sicherung für späteren Plot
    opt2 = s_ep_wn(6:7,3);   % Sicherung für späteren Plot
    s_ep = struct( ...
      'n_min', 0, 'n_max', n_max, 'Phit_tol', Phirt_tol, 'Phir_tol', Phirt_tol, ...
      'scale_lim', 0, 'reci', true, 'wn', zeros(RS.idx_ik_length.wnpos,1), 'retry_limit', 0);
    s_ep.xlim = [NaN(5,2); [-45 45]*pi/180]; % in [rad] übergeben

    % Inverse Kinematik für die Auswertung mit Stats
    for k = 1:amount_optcrit
      s_ep.wn = s_ep_wn(:,k); % k verschiedene Optimierungskriterien
      [q_IK_ep, phi_IK_ep, ~, Stats_IK_ep] = RS.invkin2(RS.x2tr(XE(i,:)'), q0_ep, s_ep);
      h6_ep(:,k,i)   = Stats_IK_ep.h(:,1+RS.idx_ikpos_hn.xlim_par);
      h7_ep(:,k,i)   = Stats_IK_ep.h(:,1+RS.idx_ikpos_hn.xlim_hyp);
      phiz_ep(:,k,i) = Stats_IK_ep.PHI(:,4);
      iter_ep(i,k)   = Stats_IK_ep.iter;
      Q_ep(:,:,i,k)  = Stats_IK_ep.Q(:,:);
      
      % Test-Berechnung ob wn(5) bzw. wn(6) aktiv ist (phi<1e-3, siehe invkin)
      for nn = 1:(Stats_IK_ep.iter+1)
        Stats_Phi_temp = Stats_IK_ep.PHI(nn,:);
        if all(abs(Stats_Phi_temp(I_EE_3T2R))<1e-3)  % vierten Eintrag ignorieren
          limred_crit_active(nn,k,i) = 1;
        end
      end
      
      % Warnung bzgl. Erreichen der maximalen Iteration -> Algorithmus steckt eventuell fest
      if Stats_IK_ep.iter == n_max
        warning('Maximale Iteration bei Punkt %d für Optimierungsmethode %d erreicht!', i, k);
        warning_iter_count = warning_iter_count + 1;
        warnplot_needed_state = 1;
      end
      
      % Warnung, wenn aktuelle IK ungültig -> Fehlerausgabe am Ende
      if any(abs(phi_IK_ep) > Phirt_tol)
        warning('Inverse Kinematik bei Punkt %d für Optimierungsmethode %d fehlerhaft!', i, k); 
        warning_phi_count = warning_phi_count + 1;
        warnplot_needed_state = 1;
      end
      
      % Auswertung 1 - Teil 1: Endergebnis aus der IK für Optimierungsmethode liegt NICHT innerhalb von xlim
%       if i==1&&k==3 % Fehler simulieren
%         phiz_ep(Stats_IK_ep.iter+1,k,i) = -55*pi/180;
%       end
%       if i==2&&k==2
%         phiz_ep(Stats_IK_ep.iter+1,k,i) = 55*pi/180;
%       end
%       if i==3&&k==2
%         phiz_ep(Stats_IK_ep.iter+1,k,i) = 15*pi/180;
%       end
      if k > 1
        if phiz_ep(Stats_IK_ep.iter+1,k,i) <= s_ep.xlim(RS.NQJ,1) || phiz_ep(Stats_IK_ep.iter+1,k,i) >= s_ep.xlim(RS.NQJ,2)
          phiz_oob(k-1,i) = 1;  % k-1, damit Vektor nicht mit 3T3R befüllt wird
          phiz_oob_state = 1;
        else
          if phiz_ep(Stats_IK_ep.iter+1,k,i) <= s_ep.xlim(RS.NQJ,1)*0.8 || phiz_ep(Stats_IK_ep.iter+1,k,i) >= s_ep.xlim(RS.NQJ,2)*0.8
            phiz_oob_thresh(k-1,i) = 1;
            phiz_oob_thresh_state = 1;
          end 
        end
      end
    end
    
    % Auswertung 1 - Teil 2: Endergebnis aus der IK für Optimierungsmethode liegt NICHT innerhalb von xlim
    if phiz_oob_state == 1
      for k_it = 1:amount_optcrit-1
        if  phiz_oob(k_it,i) == 1
          phiz_oob_vector(phiz_oob_vector_counter) = i; % aktuellen Punkt anhängen
          phiz_oob_vector_counter = phiz_oob_vector_counter + 1; 
          break;  % sobald eine Stelle gefunden wurde, kann ausgebrochen werden
                  % es wird ohnehin ein Plot für alle Opt.Krit eines Punktes erstellt
        end
      end
      phiz_oob_state = 0; % zurücksetzen, damit es für andere Punkte erkannt werden kann
    end
    % Auswertung 2: Endergebnis aus der IK für Optimierungsmethode liegt NICHT innerhalb von xlim*0.8
    if phiz_oob_thresh_state == 1
      for k_it = 1:amount_optcrit-1
        if  phiz_oob_thresh(k_it,i) == 1
          phiz_oob_thresh_vector(phiz_oob_thresh_vector_counter) = i;
          phiz_oob_thresh_vector_counter = phiz_oob_thresh_vector_counter + 1; 
          break;
        end
      end
      phiz_oob_thresh_state = 0; % zurücksetzen, damit es für andere Punkte erkannt werden kann
    end
    
    % Vektor für Warnungen zusammenstellen
    if warnplot_needed_state == 1
      warnplot_needed_vector(warnplot_needed_vector_counter) = i; % aktuellen Punkt anhängen
      warnplot_needed_vector_counter = warnplot_needed_vector_counter + 1;
      warnplot_needed_state = 0;  % zurücksetzen
    end 
  end
  
  % Ausgabe der Statistik
  fprintf('\nInverse Kinematik für Einzelpunkte abgeschlossen!\n'); 
  if warning_phi_count ~= 0
    error('Fehlerhafte IK-Berechnung von %d Einzelpunkten! Siehe Konsole für betroffene Punkte und prüfe Ursache.', warning_phi_count); 
  else
    fprintf('\nStatistik: \n'); 
    fprintf('Warnungen bezüglich erreichter max. Iteration bei den Punkten: '); 
    for nnn = 1:size(warnplot_needed_vector,2)
      fprintf(' %d', warnplot_needed_vector(nnn));
    end
    fprintf('\n');
    
    if ~isnan(phiz_oob_vector(1))
      warning('Fehlerhafte IK mit limred (letztes phiz liegt außerhalb von xlim = [%d %d] bei den Punkten: ', s_ep.xlim(6,1)*180/pi, s_ep.xlim(6,2)*180/pi); 
      for nnn = 1:size(phiz_oob_vector,2)
        fprintf(' %d', phiz_oob_vector(nnn));
      end
      fprintf('\n');
    else
      fprintf('KEINE fehlerhafte IK mit limred -> letztes phiz aller Punkte liegt innerhalb von xlim = [%d %d])', s_ep.xlim(6,1)*180/pi, s_ep.xlim(6,2)*180/pi); 
      fprintf('\n');
    end
    
    if ~isnan(phiz_oob_thresh_vector(1))
      fprintf('Letztes phiz zwischen Schwellwert und xlim bei den Punkten: '); 
      for nnn = 1:size(phiz_oob_thresh_vector,2)
        fprintf(' %d', phiz_oob_thresh_vector(nnn));
      end
      fprintf('\n\n');
    else
      fprintf('KEIN phiz liegt zwischen Schwellwert und xlim.'); 
      fprintf('\n\n');
    end
    
  end
 
  %% Einzelpunkt-IK | Plot Sets
  plot_task = 4;  % 1:Warnungen, 2: Auswertung1(außerhalb xlim), 3: Auswertung2(zwischen Schwellwert und xlim), 4:selbstgewählte Punkte
  % STANDARD = 2
  
  switch plot_task
    case 1
      plot_vector = warnplot_needed_vector;
      fprintf('Starte Plot für Warnungen! Standard ist Plot der Auswertung1, bitte notfalls ändern!\n');
      noplot = 0;
    case 2
      if isnan(phiz_oob_vector(1))
        fprintf('Kein Plot für Einzelpunkte nötig, da die IK mit limred erfolgreich berechnet wurde!\n'); 
        fprintf('Bei Bedarf können Warnungen oder bestimmte Punkte geplottet werden. Siehe Code.\n'); 
        noplot = 1;
      else    
        plot_vector = phiz_oob_vector;
        fprintf('Starte Standard-Plot für fehlerhafte IK mit limred (phiz außerhalb von xlim)!\n');
        noplot = 0;
      end
    case 3
      plot_vector = phiz_oob_thresh_vector;
      fprintf('Starte Plot für Punkte, bei welchen das letzte phiz zwischen Schwellwert und xlim liegt. Standard ist Plot der Auswertung1, bitte notfalls ändern!\n'); 
      noplot = 0;
    case 4
      plot_vector = [1 2 3 4 5 6 7];
      fprintf('Starte Plot für selbstgewählte Punkte! Standard ist Plot der Auswertung1, bitte notfalls ändern!\n'); 
      noplot = 0;
  end
  
  if noplot ~= 1
    for ii = 1:size(plot_vector,2)
      pkt = plot_vector(1,ii);
      f3 = change_current_figure(100+ii);clf;
      ylable_first_plot_incolumn = ones(4,1);   % 4 = Anzahl der Zeilen
      for kk = 1:amount_optcrit
        index_phiz = 1:(iter_ep(pkt,kk)+1);

        % Subplots Erste Zeile: Verlauf von phiz -------------------------------------
        subplot(5,18,(RS.NQJ*(kk-1)+1):kk*RS.NQJ); hold on;
        plot(index_phiz, phiz_ep(1:index_phiz(end),kk,pkt)*180/pi, 'm', 'LineWidth',1);
        xlim_vek = repmat([s_ep.xlim(RS.NQJ,1) s_ep.xlim(RS.NQJ,2)]*180/pi,index_phiz(end),1);
        plot(index_phiz, xlim_vek(:,1), 'r--', 'LineWidth',1);
        plot(index_phiz, xlim_vek(:,2), 'r--', 'LineWidth',1);
        plot(index_phiz, xlim_vek(:,1)*0.8, 'b--', 'LineWidth',1);
        plot(index_phiz, xlim_vek(:,2)*0.8, 'b--', 'LineWidth',1);
        axe = gca;
        axe.XLim = [index_phiz(1) index_phiz(end)];
        if ylable_first_plot_incolumn(1) == true
          ylabel('phi_z in °');
          ylable_first_plot_incolumn(1) = false;
        end
        xlabel('Iteration lfd. Nr.'); grid on;
        switch kk
          case 1
            title('OHNE limred: wn=[0 0 0 0 0]');
          case 2
            title(sprintf('Optimierungsmethode 1: wn=[0 0 0 %d %d]', opt1(1), opt1(2)));
          case 3
            title(sprintf('Optimierungsmethode 2: wn=[0 0 0 %d %d]', opt2(1), opt2(2)));
        end

        % Subplots Zweite Zeile: Verlauf von h(6) -------------------------------------
        if s_ep_wn(RS.idx_ikpos_wn.xlim_par,kk) == 1
          subplot(5,18,amount_optcrit*RS.NQJ+RS.NQJ*(kk-1)+1:amount_optcrit*RS.NQJ+RS.NQJ*(kk-1)+RS.NQJ); hold on;
          plot(index_phiz, h6_ep(1:index_phiz(end),kk,pkt), 'm', 'LineWidth', 1');
          axe = gca;
          axe.XLim = [index_phiz(1) index_phiz(end)];
          if ylable_first_plot_incolumn(2) == true
            ylabel('Optimierungskriterium h(6)');
            ylable_first_plot_incolumn(2) = false;
          end
          xlabel('Iteration lfd. Nr.'); grid on;
        end

        % Subplots Dritte Zeile: Verlauf von h(7) -------------------------------------
        if s_ep_wn(RS.idx_ikpos_wn.xlim_hyp,kk) == 1
          subplot(5,18,amount_optcrit*RS.NQJ*2+RS.NQJ*(kk-1)+1:amount_optcrit*RS.NQJ*2+RS.NQJ*(kk-1)+RS.NQJ); hold on;
          plot(index_phiz, h7_ep(1:index_phiz(end),kk,pkt), 'm', 'LineWidth', 1');
          % wenn h(6) = inf, dann grünen Asterisk plotten
          for nn = 1:index_phiz(end)
            if isinf(h7_ep(nn,kk,pkt))
              plot(nn, 0,'*g', 'DisplayName','h(7) = inf');
            end
          end
          hold on;
          grid on;
          axe = gca;
          axe.XLim = [index_phiz(1) index_phiz(end)];
          if ylable_first_plot_incolumn(3) == true
            ylabel('Optimierungskriterium h(7)');
            ylable_first_plot_incolumn(3) = false;
          end
          xlabel('Iteration lfd. Nr.'); grid on;
        end

        % Subplots Vierte Zeile: Verlauf von limred_crit_active -------------------------------------
        if s_ep_wn(RS.idx_ikpos_wn.xlim_par,kk) == 1 || s_ep_wn(RS.idx_ikpos_wn.xlim_hyp,kk) == 1
          subplot(5,18,amount_optcrit*RS.NQJ*3+RS.NQJ*(kk-1)+1:amount_optcrit*RS.NQJ*3+RS.NQJ*(kk-1)+RS.NQJ); hold on;
          for nn = 1:index_phiz(end)
            if limred_crit_active(nn,kk,pkt) == 1
              plot(nn, 1,'om', 'DisplayName','limred aktiv');
            elseif limred_crit_active(nn,kk,pkt) == 0
              plot(nn, 0,'or', 'DisplayName','limred inaktiv');
            end
          end
          hold on;
          axe = gca;
          axe.XLim = [index_phiz(1) index_phiz(end)];
          axe.YLim = [-1 2];
          ylabel('Zustand limred: 0/1'); grid on;
          xlabel('Iteration lfd. Nr.');
        end

        % Subplots Fünfte Zeile: Verlauf der Gelenkwinkel -------------------------------------
        for kkk = 1:RS.NQJ
          subplot(5,18,amount_optcrit*RS.NQJ*4+RS.NQJ*(kk-1)+kkk);hold on;
          plot(index_phiz, Q_ep(1:index_phiz(end),kkk,pkt,kk)/RS.qunitmult_eng_sci(kkk));
          plot([0;index_phiz(end)], RS.qlim(kkk,1)*[1;1]/RS.qunitmult_eng_sci(kkk), 'r--');
          plot([0;index_phiz(end)], RS.qlim(kkk,2)*[1;1]/RS.qunitmult_eng_sci(kkk), 'r--');
          xlabel('lfd. Nr.');
          ylabel(sprintf('q_%d / %s', k, RS.qunit_eng{kkk}));
          grid on;
        end
      end
      txt = ['Punkt Nr. ',num2str(pkt)];
      sgtitle(txt);
      txt2 = ['limred_3T2R_EP_Verlauf_',num2str(pkt)];
      saveas(gcf, [txt2, '.png'])
    end
  end
  %% Trajektorien-IK | Initialisierung und Berechnung
  fprintf('\n\nStarte Berechnungen für gesamte Trajektorie');
  Phirt_tol = 1e-8;
  n_max = 1500;
  RS.I_EE_Task = I_EE_3T2R;
  Namen_Methoden = cell(1,5);
  % Initialisierung
  phiz_traj_diff_k = NaN(size(T,1),size(Namen_Methoden,2));
  phiz_traj_diff_k_pre = phiz_traj_diff_k;
  h8_k = NaN(size(T,1),size(Namen_Methoden,2));
  h9_k = h8_k;
  h10_k = h8_k;
  phizD_traj =  NaN(size(T,1),1);
  Q_k_ges   = NaN(size(T,1),RS.NQJ,size(Namen_Methoden,2));
  QD_k_ges  = Q_k_ges;
  QDD_k_ges = Q_k_ges;
  limred_fail_flag_traj = NaN(1,1);  % ohne 3T2R-Referenz
  limred_fail_counter = 1;
  traj_plot_needed = 0;
  
  q0_traj = q0 + [0;0;0;0;0;0]*pi/180;    % Startdrehung vorgeben
  % Vorgabe per invkin zum Startpunkt ist sinnvoller als q(6)
  
  wn_limred_save = NaN(5,length(Namen_Methoden)); % erstes Element bezieht sich auf Größe wn(11:15)
  
  for k = 1:length(Namen_Methoden)
    % Default-Struct
    s_traj_wn = zeros(RS.idx_ik_length.wntraj,1); % wn(:)=0 -> keine Opt.Krit
    s_traj    = struct('n_min', 50, 'n_max', n_max, 'Phit_tol', Phirt_tol, 'Phir_tol', Phirt_tol, ...
                       'reci', true, 'I_EE', RS.I_EE_Task, 'wn', s_traj_wn);
    s_traj.xlim   = [NaN(5,2); [-45 45]*pi/180]; % in [rad] übergeben
    s_traj.xDlim  = [NaN(5,2); [-0.21 0.21]];    % 0.21rad/s = 2rpm laut unitconversion.io/de/rpm-zu-rads-konvertierung
%     s_traj.xDDlim = [NaN(5,2); [-21 21]];        % vorläufige Konvertierung wie 4*pi zu 100 bei qD und qDD
    I_13bis17 = [RS.idx_iktraj_wnP.xlim_par, RS.idx_iktraj_wnD.xlim_par, ...
      RS.idx_iktraj_wnP.xlim_hyp, RS.idx_iktraj_wnD.xlim_hyp, ...
      RS.idx_iktraj_wnP.xDlim_par];
    switch k
      case 1
        name_method = sprintf('3T2R-IK mit wn(:)=0');
        name_method_save1 = name_method;
      case 2
        % wn(17:18)=1: hyp. für qD und qDD
        s_traj.wn(RS.idx_iktraj_wnP.xlim_hyp) = 1;
        s_traj.wn(RS.idx_iktraj_wnD.xlim_hyp) = 1;
        name_method = sprintf('3T2R-IK mit xlim_hyp PD=1');
        name_method_save2 = name_method;
      case 3
        % wn(19)=1: quadr. Dämpfung für qD
        s_traj.wn(RS.idx_iktraj_wnP.xDlim_par) = 1;
        name_method = sprintf('3T2R-IK mit wnP.xDlim_par=1');
        name_method_save3 = name_method;
      case 4
        % wn(15:17)=1: Einfluss von fehlendem wn(13:14) testen
        % TODO: D-Verstärkung ist zu hoch
        s_traj.wn(RS.idx_iktraj_wnP.xlim_hyp) = 1;
        s_traj.wn(RS.idx_iktraj_wnD.xlim_hyp) = 1;
        s_traj.wn(RS.idx_iktraj_wnP.xDlim_par) = 1;
        name_method = sprintf('3T2R-IK mit wn(15:17)=1');
        name_method_save4 = name_method;
      case 5  % vollständiges Optimierungskriterium immer ans Ende stellen
        % wn(13:17)=1: Alle Optimierungskriterien aktivieren
        s_traj.wn(RS.idx_iktraj_wnP.xlim_par) = 1;
        s_traj.wn(RS.idx_iktraj_wnD.xlim_par) = 1;
        s_traj.wn(RS.idx_iktraj_wnP.xlim_hyp) = 1;
        s_traj.wn(RS.idx_iktraj_wnD.xlim_hyp) = 1;
        s_traj.wn(RS.idx_iktraj_wnP.xDlim_par) = 1;
        name_method = sprintf('3T2R-IK mit wn(13:17)=1');
        name_method_save5 = name_method;
    end
    wn_limred_save(:,k) = s_traj.wn(I_13bis17);
    
    % IK berechnen
    [Q_k, QD_k, QDD_k, Phi_k, ~, Stats_Traj_k] = RS.invkin2_traj(X,XD,XDD,T,q0_traj,s_traj);
    Q_k_ges(:,:,k) = Q_k;
    QD_k_ges(:,:,k) = QD_k;
    QDD_k_ges(:,:,k) = QDD_k;
    fprintf('\nBerechnung der Trajektorie mit "%s" beendet.', name_method);
    % Actual platform trajectory
    [X_ist_k, XD_ist_k, ~] = RS.fkineEE_traj(Q_k, QD_k, QDD_k);
    % Save original vector
    X_ist_k_pre = X_ist_k;
    % Get platform pose from integration to avoid restriction to +/- pi
    X_ist_k_int = repmat(X_ist_k(1,:), length(T), 1) + cumtrapz(T, XD_ist_k);
    % Normalize platform angles from direct calculation using angles from
    % integration as center. Gives exact solution without limitation to +/-pi
    X_ist_k(:,4:6) = normalizeAngle(X_ist_k(:,4:6), X_ist_k_int(:,4:6));
    
    % IK-Ergebnis testen
    for i = 1:length(T)
      phi_test_voll_k  = RS.constr2(Q_k(i,:)', RS.x2tr(X(i,:)'), true);
      if RS.I_EE_Task == logical([1 1 1 1 1 0])
        I_IK2 = [1 2 3 6 5 4];
      else
        error('Achtung falscher FHG?');
      end
      I_IK = I_IK2(RS.I_EE_Task);
      phi_test_red_k = phi_test_voll_k(I_IK);
      if max(abs(phi_test_red_k)) > Phirt_tol
        error('IK für "%s" stimmt nicht. Fehlersuche nötig! Eventuell ungültige Gelenkwinkel(&-Grenzen)', name_method);
      end
    end
    
    % Auswertung für den Plot: phiz
    phiz_traj_diff_k(:,k)     = -X(:,6) + X_ist_k(:,6);
    phiz_traj_diff_k_pre(:,k) = -X(:,6) + X_ist_k_pre(:,6);
    if k > 1
      if phiz_traj_diff_k(size(T,1),k) <= s_traj.xlim(RS.NQJ,1) || phiz_traj_diff_k(size(T,1),k) >= s_traj.xlim(RS.NQJ,2)
        limred_fail_flag_traj(1,limred_fail_counter) = k;
        limred_fail_counter = limred_fail_counter + 1;
        traj_plot_needed = 1;
      end
    end
    % Auswertung für den Plot: h(8) bis h(10)
    h8_k(:,k) = Stats_Traj_k.h(:,1+RS.idx_iktraj_hn.xlim_par);
    h9_k(:,k) = Stats_Traj_k.h(:,1+RS.idx_iktraj_hn.xlim_hyp);
    h10_k(:,k) = Stats_Traj_k.h(:,1+RS.idx_iktraj_hn.xDlim_par);
    % Auswertung für den Plot: phizD
    phizD_traj(:,k) = Stats_Traj_k.phi_zD(:,1);
  end
  fprintf('\nBerechnung und Auswertung der Trajektorien-IK beendet.\n');
  
  if ~isnan(limred_fail_flag_traj(1,1))
    warning('Bei der Trajektorienberechnung befindet sich der Endpunkt bei einer Optimierungsmethode NICHT innerhalb der Grenzen xlim:');
    fprintf('Optimierungsmethoden: ');
    for nnn = 1:size(limred_fail_flag_traj,2)
      fprintf('%d ', limred_fail_flag_traj(1,nnn));
    end
    if any(limred_fail_flag_traj == length(Namen_Methoden))  % length(Namen_Methoden)=vollständiges Optimierungskriterium
     warning('Optimierungskriterium %d ist fehlerhaft. Unbedingt Plot zu Gelenkwinkeln überprüfen!', length(Namen_Methoden)); 
     warning('Nachfolgend der Plot der Trajektorie bzgl. allen Optimierungsmethoden.'); 
    else
     fprintf('\n\nNachfolgend der Plot zur Trajektorie mit allen Optimierungsmethoden.\n');
     fprintf('Sollte die vollständige Optimierung mit allen Kriterien erfolgreich sein, kann das Ergebnis dennoch als positiv bewertet werden!\n');
    end
    else
    fprintf('\n\nKein Plot zur Trajektorie nötig, da alle Optimierungskriterien die Endpunkte innerhalb der Grenzen positioniert haben.\n');
  end
  
  %% Trajektorien-IK | Plot
  if traj_plot_needed
    f4 = change_current_figure(100+1);clf;
    sgtitle('Verlauf von phiz, h(7:9) und phizD');
    index_traj = 1:size(T,1);
    ylable_first_plot_incolumn = ones(5,1);   % 4 = Anzahl der Zeilen

    for kk = 1:size(Namen_Methoden,2)
      % Subplots Erste Zeile: Verlauf von phiz -------------------------------------
      subplot(5,size(Namen_Methoden,2),kk); hold on;
      plot(T, phiz_traj_diff_k(:,kk)*180/pi, 'm', 'LineWidth',1);
      plot(T, phiz_traj_diff_k_pre(:,kk)*180/pi, 'k--', 'LineWidth',1);
      xlim_vek = repmat([s_traj.xlim(RS.NQJ,1) s_traj.xlim(RS.NQJ,2)]*180/pi,index_traj(end),1);
      plot(T, xlim_vek(:,1), 'r--', 'LineWidth',1);
      plot(T, xlim_vek(:,2), 'r--', 'LineWidth',1);
      xlim_vek_thresh = repmat([s_traj.xlim(6,1) s_traj.xlim(6,2)]*180/pi*0.8,index_traj(end),1);
      plot(T, xlim_vek_thresh(:,1), 'b--', 'LineWidth',1);
      plot(T, xlim_vek_thresh(:,2), 'b--', 'LineWidth',1);
      axe = gca;
      axe.XLim = [0 T(end)];
      switch kk
        case 1
          title(sprintf('Optimierungsmethode 1: wn=[%d %d %d %d %d]', wn_limred_save(1,kk), wn_limred_save(2,kk), wn_limred_save(3,kk), wn_limred_save(4,kk), wn_limred_save(5,kk)));
        case 2
          title(sprintf('Optimierungsmethode 2: wn=[%d %d %d %d %d]', wn_limred_save(1,kk), wn_limred_save(2,kk), wn_limred_save(3,kk), wn_limred_save(4,kk), wn_limred_save(5,kk)));
        case 3
          title(sprintf('Optimierungsmethode 3: wn=[%d %d %d %d %d]', wn_limred_save(1,kk), wn_limred_save(2,kk), wn_limred_save(3,kk), wn_limred_save(4,kk), wn_limred_save(5,kk)));
        case 4
          title(sprintf('Optimierungsmethode 4: wn=[%d %d %d %d %d]', wn_limred_save(1,kk), wn_limred_save(2,kk), wn_limred_save(3,kk), wn_limred_save(4,kk), wn_limred_save(5,kk)));
        case 5
          title(sprintf('Optimierungsmethode 5: wn=[%d %d %d %d %d]', wn_limred_save(1,kk), wn_limred_save(2,kk), wn_limred_save(3,kk), wn_limred_save(4,kk), wn_limred_save(5,kk)));
      end
      if ylable_first_plot_incolumn(1) == true
        ylabel('Verlauf von phiz');
        ylable_first_plot_incolumn(1) = false;
      end
      xlabel('Zeit in s'); grid on;

      % Subplots Zweite Zeile: Verlauf von h(8) -------------------------------------
      if wn_limred_save(1,kk) == 1 || wn_limred_save(2,kk) == 1
        subplot(5,size(Namen_Methoden,2),size(Namen_Methoden,2)+kk); hold on;
        plot(T, h8_k(:,kk), 'm', 'LineWidth',1);
        axe = gca;
        axe.XLim = [0 T(end)];
        hold on;
        if ylable_first_plot_incolumn(2) == true
          ylabel('Optim.krit h(8)');
          ylable_first_plot_incolumn(2) = false;
        end
        xlabel('Zeit in s'); grid on;
      end

      % Subplots Dritte Zeile: Verlauf von h(9) -------------------------------------
      if wn_limred_save(3,kk) == 1 || wn_limred_save(4,kk) == 1
        subplot(5,size(Namen_Methoden,2),size(Namen_Methoden,2)*2+kk); hold on;
        plot(T, h9_k(:,kk), 'm', 'LineWidth',1);
        for nn = 1:index_traj(end)
          if isinf(h9_k(nn,kk))
            plot(T(nn), 0,'*g', 'LineWidth',1);
          end
        end
        hold on; grid on;
        axe = gca;
        axe.XLim = [0 T(end)];
        xlabel('Zeit in s');
        if ylable_first_plot_incolumn(3) == true
          ylabel('Optim.krit h(9)');
          ylable_first_plot_incolumn(3) = false;
        end
      end

      % Subplots Vierte Zeile: Verlauf von h(10) -------------------------------------
      if wn_limred_save(5,kk) == 1
        subplot(5,size(Namen_Methoden,2),size(Namen_Methoden,2)*3+kk); hold on;
        plot(T, h10_k(:,kk), 'm', 'LineWidth',1);
        axe = gca;
        axe.XLim = [0 T(end)];
        hold on;
        if ylable_first_plot_incolumn(4) == true
          ylabel('Optim.krit h(10)');
          ylable_first_plot_incolumn(4) = false;
        end
        xlabel('Zeit in s'); grid on;
      end

      % Subplots Fünfte Zeile: Verlauf von phizD -------------------------------------
      subplot(5,size(Namen_Methoden,2),size(Namen_Methoden,2)*4+kk); hold on;
      plot(T, phizD_traj(:,kk), 'm', 'LineWidth',1);
      axe = gca;
      axe.XLim = [0 T(end)];
      hold on;
      if ylable_first_plot_incolumn(5) == true
        ylabel('Geschwindigkeit von phi_z in °/s');
        ylable_first_plot_incolumn(5) = false;
      end
      xlabel('Zeit in s'); grid on;

    end                      
    fprintf('\nPlot für Trajektorie beendet\n'); 
  end
  %% q, qD, qDD und tau bei Bedarf plotten
  
  gelenkplot_wanted = 0;
  
  if gelenkplot_wanted
    for qplots = 5:5%size(Namen_Methoden,2) % Kommentar entfernen, wenn alle Opt.Methoden erwünscht sind
      f = change_current_figure(300*4+qplots);clf;
      Q_work   = Q_k_ges(:,:,qplots);
      QD_work  = QD_k_ges(:,:,qplots);
      QDD_work = QDD_k_ges(:,:,qplots);
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

        if     qplots == 1, sgtitle(sprintf('Optimierungsmethode 1: wn=[%d %d %d %d %d]', wn_limred_save(1,qplots), wn_limred_save(2,qplots), wn_limred_save(3,qplots), wn_limred_save(4,qplots), wn_limred_save(5,qplots)));
        elseif qplots == 2, sgtitle(sprintf('Optimierungsmethode 2: wn=[%d %d %d %d %d]', wn_limred_save(1,qplots), wn_limred_save(2,qplots), wn_limred_save(3,qplots), wn_limred_save(4,qplots), wn_limred_save(5,qplots)));
        elseif qplots == 3, sgtitle(sprintf('Optimierungsmethode 3: wn=[%d %d %d %d %d]', wn_limred_save(1,qplots), wn_limred_save(2,qplots), wn_limred_save(3,qplots), wn_limred_save(4,qplots), wn_limred_save(5,qplots)));
        elseif qplots == 4, sgtitle(sprintf('Optimierungsmethode 4: wn=[%d %d %d %d %d]', wn_limred_save(1,qplots), wn_limred_save(2,qplots), wn_limred_save(3,qplots), wn_limred_save(4,qplots), wn_limred_save(5,qplots)));
        elseif qplots == 5, sgtitle(sprintf('Optimierungsmethode 5: wn=[%d %d %d %d %d]', wn_limred_save(1,qplots), wn_limred_save(2,qplots), wn_limred_save(3,qplots), wn_limred_save(4,qplots), wn_limred_save(5,qplots)));
        end
      end
      linkxaxes

    end
  end
  
  
end
 
