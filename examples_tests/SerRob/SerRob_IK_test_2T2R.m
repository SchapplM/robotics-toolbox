% Teste die Roboterklasse SerRob mit verschiedenen Systemen bzgl Inverse
% Kinematik für die 2T2R-Anpassungen durch Tobias Blum
% 
% Test-Szenario:
% * Translatorisches Residuum visuell bzgl. korrekter Berechnung prüfen
% 
% Quelle:
% [SchapplerBluJob2022] Schappler, M. et al.: Geometric Model for Serial-
% Chain Robot Inverse Kinematics in the Case of Two Translational DoF with
% Spatial Rotation and Task Redundancy, Submitted to ARK 2022 
% [Blum2021] Blum, T.: Inverse Kinematik aufgabenredundanter Roboter für
% Aufgaben mit zwei translatorischen und zwei rotatorischen Freiheits-
% graden, Masterarbeit M-03/2021-1013

% Tobias Blum, Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 202021
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

close all
clear
clc
% Unterdrücke Warnung für Schlechte Konditionierung der Jacobi-Matrix
if isempty(which('serroblib_path_init.m'))
  warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end

%% Benutzereingaben
% Prüfung repräsentativer Roboter
Robots = {{'S6RRRRRR10V2', 'S6RRRRRR10V2_KUKA1'}};

% Einstellungen
usr_plot = false;
use_mex_functions = true; % mit mex geht es etwas schneller, dafür ist debuggen schwieriger
minimal_test = false; % Nur sehr wenige zufällige Winkel testen (geht schneller)
% Endeffektor-Transformation ungleich Null festlegen, um zu prüfen, ob die
% Implementierung davon richtig ist
r_W_E = [0.1;0.1;0.1];
phi_W_E = [5; 5; 5]*pi/180;

%% Alle Robotermodelle durchgehen
for Robot_Data = Robots
  SName = Robot_Data{1}{1};
  RName = Robot_Data{1}{2};
   
  %% Klasse für seriellen Roboter erstellen
  % Instanz der Roboterklasse erstellen
  serroblib_update_template_functions({SName});
  RS = serroblib_create_robot_class(SName, RName);
  RS.fill_fcn_handles(use_mex_functions, true);
  % Grenzen festlegen (für Zusatz-Optimierung)
  RS.qlim = repmat([-pi, pi], RS.NQJ, 1);
  RS.update_EE(r_W_E, phi_W_E, []);
  % Test-Einstellungen generieren
  TSS = RS.gen_testsettings(true, false);
  if minimal_test
    TSS.Q = TSS.Q(1:5,:);
  end
  fprintf('Starte Untersuchung für %s\n', RS.descr);
  %% Init 
  % Bereich reduzieren: Bei Addition von Zufallswinkeln darf nicht pi
  % überschritten werden.
  TSS.Q(abs(TSS.Q(:))>150*pi/180) = 0;
  
  %% Test 1: Translatorisches Residuum
  if usr_plot
  fprintf('%s: Test 1: Translatorisches Residuum bei 2T2R\n', SName);  
  for i_phiconv = uint8([2 6])
    for i = 1:size(TSS.Q,1)
      RS.phiconv_W_E = i_phiconv;
      eulstr = euler_angle_properties(i_phiconv);
      
      % Ziel- und Anfangs-Konfiguration definieren
      q_ziel  = TSS.Q(i,:)';
      T_E_ziel = RS.fkineEE(q_ziel);
      xE = [T_E_ziel(1:3,4); r2eul(T_E_ziel(1:3,1:3), RS.phiconv_W_E)];
      
      q_start = q_ziel-20*pi/180*(0.5-rand(RS.NQJ,1)); % Anfangswinkel 20° neben der Endstellung
      T_E_start = RS.fkineEE(q_start);
      
      % Einmalig Auftreffwinkel auf XY-Ebene berechnen
      Tr0E_soll = RS.x2tr(xE);
      e_z_OE = Tr0E_soll(:,3);
      angle_Z_XY_value = asin(abs(sum(e_z_OE.*[0;0;1])));
      if abs(angle_Z_XY_value) <= pi/4
        xz_modus_temp = true;
      else
        xz_modus_temp = false;
      end
      
      % IK aufrufen, um translatorisches Residuum zu bekommen
      [q_test, phi_test,~] = RS.invkin2(RS.x2tr(xE), q_start, struct( ...
        'scale_lim', 0, 'retry_limit', 20, 'I_EE', logical([1 1 0 1 1 0])));
      
      % Plots: Start(rot), Ziel aus Planung(blau) und Ziel aus IK(grün)
      % Roboter in Grundstellung plotten (mit im Gelenkraum entworfener Trajektorie)
      qref = q_start;
      s_plot = struct( 'ks', [6, RS.NJ+2], 'straight', 0);
      change_current_figure(1);clf;
      hold on; grid on; view(3);
      xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
      RS.plot(qref, s_plot);
      title(sprintf('trans. Residuum | Testpunkt %d | phiconv %d', i, i_phiconv));
      % Schnittpunkt mit XY-Ebene bestimmen / Startkonfiguration
      t_0E    = T_E_start(1:3,4);
      e_z_OE  = T_E_start(1:3,3);
      g_0E = zeros(3,1);
      if xz_modus_temp == false
        lambda = (-t_0E(3))/e_z_OE(3);
        g_0E(1) = t_0E(1) + lambda*e_z_OE(1);
        g_0E(2) = t_0E(2) + lambda*e_z_OE(2);
        g_0E(3) = 0;
      else % if xz_modus_temp == true
        lambda = (-t_0E(2))/e_z_OE(2);
        g_0E(1) = t_0E(1) + lambda*e_z_OE(1);
        g_0E(2) = 0;
        g_0E(3) = t_0E(3) + lambda*e_z_OE(3);
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
      
      % Roboter in Endstellung aus Planung plotten
      qref2 = q_ziel;
      s_plot2 = struct( 'ks', [6, RS.NJ+2], 'straight', 0);
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
      else %if xz_modus_temp == true
        lambda = (-t_0E(2))/e_z_OE(2);
        g_0E(1) = t_0E(1) + lambda*e_z_OE(1);
        g_0E(2) = 0;
        g_0E(3) = t_0E(3) + lambda*e_z_OE(3);
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
      
      % Plot Translatorisches Residuum als Linie
      plot3(  [p_intersect_start(1);p_intersect_ziel_planung(1)],...
              [p_intersect_start(2);p_intersect_ziel_planung(2)],...
              [p_intersect_start(3);p_intersect_ziel_planung(3)],'m')
      
      % Roboter in Zielstellung aus IK plotten
      qref3 = q_test;
      s_plot3 = struct( 'ks', [6, RS.NJ+2], 'straight', 0);
      hold on; grid on; view(3);
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
      else %if xz_modus_temp == true
        lambda = (-t_0E(2))/e_z_OE(2);
        g_0E(1) = t_0E(1) + lambda*e_z_OE(1);
        g_0E(2) = 0;
        g_0E(3) = t_0E(3) + lambda*e_z_OE(3);
      end
      p_intersect_ziel_IK = g_0E;
      
      % Plot Schnittpunkt Gerade EE-Z-Achse zu KS0-XY-Ebene
      plot3(  [T_E_ziel_neu(1,4);p_intersect_ziel_IK(1)],...
              [T_E_ziel_neu(2,4);p_intersect_ziel_IK(2)],...
              [T_E_ziel_neu(3,4);p_intersect_ziel_IK(3)],'g')
      % Plot Schnittpunkt Gerade EE-Z-Achse / KS0-XY-Ebene bzw. XZ
      if xz_modus_temp == true
        plot3(p_intersect_ziel_IK(1), 0, p_intersect_ziel_IK(3),'og')
      else
        plot3(p_intersect_ziel_IK(1), p_intersect_ziel_IK(2), 0,'og')
      end
    end
  end 
  fprintf('Test 1 durchgeführt. Erfolgreich, wenn die jeweils grüne Linie mit der blauen Linie übereinstimmt.\n');
  end
  %% Test 2: Gradient der kinematischen Zwangsbedingungen
  fprintf('%s: Test 2: Gradient der kinematischen Zwangsbedingungen testen\n', SName);  
  for i_phiconv = uint8([2 6]) % Prüfe für xyz und yxz
    n_niO = 0;
    n_iO = 0;
    eulstr = euler_angle_properties(i_phiconv);
    RS.phiconv_W_E = i_phiconv;

    for i = 1:size(TSS.Q,1)
      % Ziel- und Anfangs-Konfiguration definieren
      q_ziel  = TSS.Q(i,:)';
      T_E_ziel = RS.fkineEE(q_ziel);
      xE = [T_E_ziel(1:3,4); r2eul(T_E_ziel(1:3,1:3), RS.phiconv_W_E)];

      q_start = q_ziel-20*pi/180*(0.5-rand(RS.NQJ,1)); % Anfangswinkel 20° neben der Endstellung
      T_E_start = RS.fkineEE(q_start);
      if cond(RS.jacobig(q_start)) > 1e10, continue; end % Ignoriere Singularität

      % Einmalig Auftreffwinkel auf XY-Ebene berechnen
      Tr0E_soll = RS.x2tr(xE);
      e_z_OE = Tr0E_soll(:,3);
      angle_Z_XY_value = asin(abs(sum(e_z_OE.*[0;0;1])));
      if abs(angle_Z_XY_value) <= pi/4
        xz_modus_temp = true;
      else
        xz_modus_temp = false;
      end

      % Anfängliche Zwangsbedingungen und -gradienten für q_start berechnen
      if xz_modus_temp == true
        Phi_0 = RS.constr3(q_start, Tr0E_soll, true, true);
        Phidq_0 = RS.constr3grad(q_start, Tr0E_soll, true, true);
      elseif xz_modus_temp == false
        Phi_0 = RS.constr3(q_start, Tr0E_soll, true, false);
        Phidq_0 = RS.constr3grad(q_start, Tr0E_soll, true, false);
      end

      for id = 1:size(TSS.Q,2) % Alle Komponenten der Gelenkkoordinaten einmal verschieben
        % Neue Koordinaten q1 durch Verschiebung in einer Komponente
        dq = zeros(size(TSS.Q,2),1);
        dq(id) = 1e-4;
        q1 = q_start+dq;

        % Zwangsbedingungen für verschobene Koordinaten q1 berechnen
        if xz_modus_temp == true
          Phi_1 = RS.constr3(q1, RS.x2tr(xE), true, true);
        elseif xz_modus_temp == false
          Phi_1 = RS.constr3(q1, RS.x2tr(xE), true, false);
        end

        % Prüfe neue ZB aus Ableitung gegen direkt berechnete (linksseitiger
        % Differenzenquotient)
        Phi_1_g = Phi_0 + Phidq_0*dq;
        test1   = Phi_1 - Phi_1_g;

        % Prüfe das Inkrement der ZB-Änderung. Ist dieses Qualitativ
        % gleich, kann man davon ausgehen, dass die Lösung richtig ist.
        dPhi_grad = Phidq_0*dq;
        dPhi_diff = Phi_1 - Phi_0;
        RelErr = dPhi_grad./dPhi_diff - 1;
        RelErr(isnan(RelErr)) = 0; % 0=0 -> relativer Fehler 0
        I_relerr = abs(RelErr) > 5e-2; % Indizes mit Fehler größer 5%
        I_abserr = abs(test1)  > 8e10*eps(1+max(abs(Phi_1))); % Absoluter Fehler über Toleranz
        if any( I_relerr & I_abserr ) % Fehler bei Überschreitung von absolutem und relativem Fehler
%           error('%s: Zwangsbedingungs-Ableitung nach q stimmt nicht mit Zwangsbedingungen überein (Var. 1; Delta q%d)', PName, id);
          warning(['%s: %d/%d: Deltaq%d: Zwangsbedingungs-Ableitung stimmt ', ...
            'nicht mit Zwangsbedingungen überein. Fehler %1.1e'], SName, ...
            i, size(TSS.Q,1), id, max(abs(test1)));
          n_niO = n_niO + 1;
        else
          % fprintf('%d/%d: Gelenkverschiebung Nr. %d: Zwangsbedingungen erfolgreich getestet\n', i, size(TSS.Q,1), id);
          n_iO = n_iO + 1;
        end
      end
    end
    % Statistik
    fprintf(['%d/%d verschobene Gelenkkoordinaten waren erfolgreich bei ', ...
      '%s-Euler-Winkeln. %d erfolglos, Rest singulär\n'], n_iO, size(TSS.Q,1), eulstr, n_niO);
    if n_niO/(n_iO+n_niO) > 0.02
      error('Es gab in mehr als 2%% der Fälle (%d/%d mal) Abweichung beim Residuum', n_niO, n_iO+n_niO);
    end
  end

  %% Test 3: Klassen-Funktionen gegen Funktions-Vorlagen testen
  fprintf('%s: Test 3: Funktions-Vorlagen testen\n', SName);
  res = matlabfcn2mex({[RS.mdlname,'_constr3grad_tq'], ...
    [RS.mdlname,'_constr3_trans']});
  assert(res==0, 'Kompilieren der constr3-Funktionen fehlgeschlagen');
  for i = 1:size(TSS.Q,1)
    q_ziel  = TSS.Q(i,:)';
    T_E_ziel = RS.fkineEE(q_ziel);
    xE = [T_E_ziel(1:3,4); r2eul(T_E_ziel(1:3,1:3), RS.phiconv_W_E)];
    Tr0E_soll = RS.x2tr(xE);
    q = q_ziel-20*pi/180*(0.5-rand(RS.NQJ,1)); % Anfangswinkel 20° neben der Endstellung
    T_E_start = RS.fkineEE(q);

    for XZ_Modus = [false, true]
      % Einmalig Auftreffwinkel auf XY-Ebene berechnen
      phi = RS.constr3_trans(q, Tr0E_soll, XZ_Modus);
      phi_tpl = eval(sprintf(['%s_constr3_trans(q, Tr0E_soll, RS.pkin, RS.T_N_E, ', ...
        'uint8(RS.I_EElink), XZ_Modus);'], RS.mdlname));
      assert(all(abs(phi-phi_tpl) < 1e-10), ['Template-Funktion für ', ...
        'constr3_trans stimmt nicht überein']);
      
      phidq = RS.constr3grad_tq(q, XZ_Modus);
      phidq_tpl = eval(sprintf(['%s_constr3grad_tq(q, RS.pkin, RS.T_N_E, ', ...
        'uint8(RS.I_EElink), XZ_Modus);'], RS.mdlname));
      test_phidq_abs = phidq-phidq_tpl;
      I_relerror = test_phidq_abs ./ phidq_tpl > 1e-3;
      I_abserror = abs(test_phidq_abs) > 1e-9;
      assert(~any(I_relerror(:)&I_abserror(:)), ['Template-Funktion ', ...
        'für constr3grad_tq stimmt nicht überein']);
    end
  end
end
 
