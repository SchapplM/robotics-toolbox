% Teste die kinematischen Zwangsbedingungen aus der Roboterklasse ParRob
% 
% Getestet:
% kinematischen Zwangsbedingungen:
% * constr1..., ...
% Gradienten der Zwangsbedingungen
% * constr1grad..., ...
% Diese Funktionen sind die Grundlage für die inverse Kinematik
% Die rotatorischen Zwangsbedingungen können für unterschiedliche
% Euler-Winkel-Konventionen aufgestellt werden. Hier werden alle getestet.
% 
% TODO: Die numerischen Toleranzen sind relativ hoch gewählt. Zu prüfen:
% Ist die Schrittweite zu groß oder sind diese Grenzwerte nachvollziehbar?

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc

% Beispielsysteme
% Entsprechen 6UPS, 4xSCARA, 3RRR, 3RPR
RobotNames = {'P6RRPRRR14', 'P4RRRP1', 'P3RRR1', 'P3RPR1'};

% Einstellungen
use_mex_functions = true; % mit mex geht es etwas schneller

% Endeffektor-Transformation: Nehme irgendwelche Werte um zu prüfen, ob die
% Transformation korrekt implementiert ist
r_P_E   = [0.1;0.2;0.3];
phi_P_E = [20; 40; 50]*pi/180;
% Basis-Transformation: Nehme auch hier irgendwelche Werte
r_W_0   = [0.1;0.2;0.3];
phi_W_0 = [20; 40; 50]*pi/180;

%% Alle Robotermodelle durchgehen
for NNN = RobotNames
  %% PKM initialisieren
  PName = NNN{1};
  N_LEG = str2double(PName(2));
  SName = sprintf('S%d%s',N_LEG,PName(3:end));
  fprintf('Starte Untersuchung für %s\n', PName);
  
  RS = serroblib_create_robot_class(SName);
  serroblib_addtopath({RS.mdlname});
  RS.fill_fcn_handles(use_mex_functions);
  RS.mex_dep()
  
  % Kinematik-Parameter der Beine setzen
  % NaN-Parameter sind frei wählbar und nicht vorbelegt.
  n_NaN = sum(isnan([RS.MDH.a;RS.MDH.d]));
  dd = 0.6 / n_NaN * 2; % Alle Längen gleich
  RS.MDH.b(isnan(RS.MDH.b)) = dd;
  RS.MDH.a(isnan(RS.MDH.a)) = dd;
  RS.MDH.d(isnan(RS.MDH.d)) = dd;
  
  tt = pi/4; % Wenn Winkel ein Parameter sind, darf dieser nicht Null werden
  RS.MDH.alpha(isnan(RS.MDH.alpha)) = tt;
  RS.MDH.theta(isnan(RS.MDH.theta)) = tt;
  RS.update_pkin();
  
  % Aufbau einer Symmetrischen PKM
  RP = ParRob(PName);
  RP.create_symmetric_robot(N_LEG, RS, 0.5, 0.2);
  
  RP.initialize();
  X = [ [0.0;0.0;0.5]; [0;0;0]*pi/180 ];
  xE = X + [[0.1; 0.05; 0]; [0; 0; 0]*pi/180];
  
  % Kinematik-Parameter anpassen: Zufällige Rotation zu Koppelpunkt-KS der
  % Beinketten. Damit wird geprüft, ob die Funktionen allgemein funktionieren
  for i = 1:RP.NLEG
    RP.Leg(i).update_EE([], rand(3,1));
    RP.phi_P_B_all(:,i) = rand(3,1);
  end
  RP.update_base(r_W_0, phi_W_0);
  RP.update_EE(r_P_E, phi_P_E, []);
  % EE-FG eintragen. TODO: FG allgemein festlegen
  if RP.NLEG == 3
    RP.update_EE_FG(logical([1 1 0 0 0 1]));
  elseif RP.NLEG == 4
    RP.update_EE_FG(logical([1 1 1 0 0 1]));
  else
    RP.update_EE_FG(logical([1 1 1 1 1 1]));
  end
  %% Initialisierung der Test-Variablen
  q0 = rand(RP.NJ,1);
  qD = rand(RP.NJ,1);
  n = 100;
  Q_test = rand(n, RP.NJ);
  X_test = rand(n, 6);
  X_test(:,~RP.I_EE) = 0;
  %% Alle Funktionen einmal aufrufen
  % Prüfe dabei, ob eine Ausgabe NaN wird. Das darf bei der Eingabe nicht
  % sein.
  testNaN = cell(40,1);
  [testNaN{1},testNaN{2}]=RP.fkine(q0, xE);
  [testNaN{3},testNaN{4}]=RP.fkine_legs(q0);
  [testNaN{5},testNaN{6}]=RP.fkine_platform(xE);

  testNaN{7}=RP.constr1(q0, xE);
  testNaN{8}=RP.constr1_trans(q0, xE);
  testNaN{9}=RP.constr1_rot(q0, xE);
  [testNaN{10},testNaN{11}]=RP.constr1grad_q(q0, xE);
  [testNaN{12},testNaN{13}]=RP.constr1grad_tq(q0);
  [testNaN{14},testNaN{15}]=RP.constr1grad_rq(q0, xE);
  [testNaN{16},testNaN{17}]=RP.constr1grad_x(q0, xE);
  
  [testNaN{18}]=RP.constr2(q0, xE);
  [testNaN{19},testNaN{20}]=RP.constr2_trans(q0, xE);
  [testNaN{21},testNaN{22}]=RP.constr2_rot(q0, xE);
  [testNaN{23},testNaN{24}]=RP.constr2grad_q(q0, xE);
  [testNaN{25},testNaN{26}]=RP.constr2grad_x(q0, xE);
  if all(RP.I_EE == [1 1 1 1 1 1]) % Benutze ZB Definition 3 nur für räumliche Systeme
    [testNaN{27},testNaN{28}]=RP.constr3(q0, xE);
    [testNaN{31},testNaN{32}]=RP.constr3_rot(q0, xE);
    [testNaN{33},testNaN{34}]=RP.constr3grad_q(q0, xE);
    [testNaN{37},testNaN{38}]=RP.constr3grad_rq(q0, xE);
    [testNaN{39},testNaN{40}]=RP.constr3grad_x(q0, xE);
  end
  for i = 1:40
    test = testNaN{i};
    if any(isnan(test(:)))
      error('Die aufgerufenen Funktionen für Argument %d liefert NaN', i);
    end
  end
  fprintf('%s: Alle Funktionen einmal ausgeführt\n', PName);

  %% Gradienten der kinematischen Zwangsbedingungen testen
  % Teste, ob die Gradienten dem Differenzenquotienten der
  % Zwangsbedingungen entsprechen.
  
  for i_phiconv = uint8([2 4 6 7 9 11]) % Test für alle Euler-Winkel-Konventionen
    eulstr = euler_angle_properties(i_phiconv);
    RP.phiconv_W_E = i_phiconv;
    for i = 1:size(Q_test,1)
      q0 = Q_test(i,:)';
      x0 = X_test(i,:)';

      % Zwangsbedingungen und -gradienten für q0/x0 berechnen
      [~,Phi1_0] = RP.constr1(q0, x0);
      [~,Phi1dq_0] = RP.constr1grad_q(q0, x0);
      [~,Phi1dx_0] = RP.constr1grad_x(q0, x0);
      
      [~,Phi2_0] = RP.constr2(q0, x0);
      [~,Phi2dq_0] = RP.constr2grad_q(q0, x0);
      [~,Phi2dx_0] = RP.constr2grad_x(q0, x0);
      
      if all(RP.I_EE == [1 1 1 1 1 1])
      [~,Phi3_0] = RP.constr3(q0, x0);
      [~,Phi3dq_0] = RP.constr3grad_q(q0, x0);
      [~,Phi3dx_0] = RP.constr3grad_x(q0, x0);   
      else
      Phi3_0=NaN*Phi2_0; Phi3dq_0=NaN*Phi2dq_0;Phi3dx_0=NaN*Phi2dx_0;
      end
      
      % Alle Komponenten der Gelenkkoordinaten einmal verschieben und
      % ZB-Gradienten testen (Geometrische Matrix der inversen Kinematik)
      for id = 1:length(q0) 
        % Neue Koordinaten q1 durch Verschiebung in einer Komponente
        dq = zeros(length(q0),1);
        dq(id) = 1e-4;
        q1 = q0+dq;

        % Zwangsbedingungen für verschobene Koordinaten q1 berechnen
        [~,Phi1_1] = RP.constr1(q1, x0);
        [~,Phi2_1] = RP.constr2(q1, x0);
        [~,Phi3_1] = RP.constr3(q1, x0);
        
        % Prüfe neue ZB aus Ableitung gegen direkt berechnete (linksseitiger
        % Differenzenquotient)
        Phi1_1_g = Phi1_0 + Phi1dq_0*dq;
        test1 = Phi1_1 - Phi1_1_g;
        
        Phi2_1_g = Phi2_0 + Phi2dq_0*dq;
        test2 = Phi2_1 - Phi2_1_g;  
        Phi3_1_g = Phi3_0 + Phi3dq_0*dq;
        test3 = Phi3_1 - Phi3_1_g;
        
        % Vielfache von 2*Pi entfernen
        test1(abs(abs(test1)-2*pi) < 1e-3 ) = 0;
        test2(abs(abs(test2)-2*pi) < 1e-3 ) = 0;
        test3(abs(abs(test3)-2*pi) < 1e-3 ) = 0;
        
        % Prüfe das Inkrement der ZB-Änderung. Ist dieses Qualitativ
        % gleich, kann man davon ausgehen, dass die Lösung richtig ist.
        dPhi_grad1 = Phi1dq_0*dq;
        dPhi_diff1 = Phi1_1 - Phi1_0;
        RelErr = dPhi_grad1./dPhi_diff1-1;
        RelErr(isnan(RelErr)) = 0; % 0=0 -> relativer Fehler 0
        I_relerr = abs(RelErr) > 5e-2; % Indizes mit Fehler größer 5%
        I_abserr = abs(test1) > 8e10*eps(1+max(abs(Phi1_1))); % Absoluter Fehler über Toleranz
        if any( I_relerr & I_abserr ) % Fehler bei Überschreitung von absolutem und relativem Fehler
          error('%s: Zwangsbedingungs-Ableitung nach q stimmt nicht mit Zwangsbedingungen überein (Var. 1; Delta q%d)', PName, id);
        end
        
        dPhi_grad2 = Phi2dq_0*dq;
        dPhi_diff2 = Phi2_1 - Phi2_0;
        RelErr = dPhi_grad2./dPhi_diff2-1;
        RelErr(isnan(RelErr)) = 0; % 0=0 -> relativer Fehler 0
        I_relerr = abs(RelErr) > 5e-2; % Indizes mit Fehler größer 5%
        I_abserr = abs(test2) > 8e10*eps(max(1+abs(Phi2_1))); % Absoluter Fehler über Toleranz
        if any( I_relerr & I_abserr ) % Fehler bei Überschreitung von absolutem und relativem Fehler
          error('%s: Zwangsbedingungs-Ableitung nach q stimmt nicht mit Zwangsbedingungen überein (Var. 2; Delta q%d)', PName, id);
        end  
        
        if all(RP.I_EE == [1 1 1 1 1 1]) % Benutze ZB Definition 3 nur für räumliche Systeme
        dPhi_grad3 = Phi3dq_0*dq;
        dPhi_diff3 = Phi3_1 - Phi3_0;
        RelErr = dPhi_grad3./dPhi_diff3-1;
        RelErr(isnan(RelErr)) = 0; % 0=0 -> relativer Fehler 0
        I_relerr = abs(RelErr) > 5e-2; % Indizes mit Fehler größer 5%
        I_abserr = abs(test3) > 8e10*eps(max(1+abs(Phi3_1))); % Absoluter Fehler über Toleranz
        I_nan = isnan(Phi3_0);
        if any( I_nan | I_relerr & I_abserr ) % Fehler bei Überschreitung von absolutem und relativem Fehler
          error('%s: Zwangsbedingungs-Ableitung nach q stimmt nicht mit Zwangsbedingungen überein (Var. 3; Delta q%d)', PName, id);
        end
        end
      end
      % Alle Komponenten der EE-Koordinaten einmal verschieben und
      % ZB-Gradienten testen (Geometrische Matrix der direkten Kinematik)
      for id = 1:6 
        % Neue Koordinaten x1 durch Verschiebung in einer Komponente
        dx = zeros(6,1);
        dx(id) = 1e-4;
        x1 = x0+dx;

        % Zwangsbedingungen für verschobene Koordinaten x1 berechnen
        [~,Phi1_1] = RP.constr1(q0, x1);
        [~,Phi2_1] = RP.constr2(q0, x1);
        [~,Phi3_1] = RP.constr3(q0, x1);
        
        % Prüfe neue ZB aus Ableitung gegen direkt berechnete (linksseitiger
        % Differenzenquotient)
        Phi1_1_g = Phi1_0 + Phi1dx_0*dx;
        test1 = Phi1_1 - Phi1_1_g;
        
        Phi2_1_g = Phi2_0 + Phi2dx_0*dx;
        test2 = Phi2_1 - Phi2_1_g;
        
        Phi3_1_g = Phi3_0 + Phi3dx_0*dx;
        test3 = Phi3_1 - Phi3_1_g;
        
        % Vielfache von 2*Pi entfernen (Rotationsfehler von 2*Pi ist kein
        % "Fehler" sondern nur in der Darstellung begründet)
        test1(abs(abs(test1)-2*pi) < 1e-3 ) = 0;
        test2(abs(abs(test2)-2*pi) < 1e-3 ) = 0;
        test3(abs(abs(test3)-2*pi) < 1e-3 ) = 0;

        % Prüfe das Inkrement der ZB-Änderung. Ist dieses Qualitativ
        % gleich, kann man davon ausgehen, dass die Lösung richtig ist.
        dPhi_grad1 = Phi1dx_0*dx;
        dPhi_diff1 = Phi1_1 - Phi1_0;
        RelErr = dPhi_grad1./dPhi_diff1-1;
        RelErr(isnan(RelErr)) = 0; % 0/0 -> relativer Fehler 0
        I_relerr = abs(RelErr) > 5e-2; % Indizes mit Fehler größer 5%
        I_abserr = abs(test1) > 8e10*eps(1+max(abs(Phi1_1))); % Absoluter Fehler über Toleranz
        if any( I_relerr & I_abserr ) % Fehler bei Überschreitung von absolutem und relativem Fehler
          error('%s: Zwangsbedingungs-Ableitung nach x stimmt nicht mit Zwangsbedingungen überein (Var. 1; Delta x%d)', PName, id);
        end
        
        dPhi_grad2 = Phi2dx_0*dx;
        dPhi_diff2 = Phi2_1 - Phi2_0;
        RelErr = dPhi_grad2./dPhi_diff2-1;
        RelErr(isnan(RelErr)) = 0; % 0/0 -> relativer Fehler 0
        I_relerr = abs(RelErr) > 5e-2; % Indizes mit Fehler größer 5%
        I_abserr = abs(test2) > 8e10*eps(max(1+abs(Phi2_1))); % Absoluter Fehler über Toleranz
        if any( I_relerr & I_abserr ) % Fehler bei Überschreitung von absolutem und relativem Fehler
          error('%s: Zwangsbedingungs-Ableitung nach x stimmt nicht mit Zwangsbedingungen überein (Var. 2; Delta x%d)', PName, id);
        end
        if all(RP.I_EE == [1 1 1 1 1 1]) % Benutze ZB Definition 3 nur für räumliche Systeme
        dPhi_grad3 = Phi3dx_0*dx;
        dPhi_diff3 = Phi3_1 - Phi3_0;
        RelErr = dPhi_grad3./dPhi_diff3-1;
        RelErr(isnan(RelErr)) = 0; % 0/0 -> relativer Fehler 0
        I_relerr = abs(RelErr) > 5e-2; % Indizes mit Fehler größer 5%
        I_abserr = abs(test3) > 8e10*eps(max(1+abs(Phi3_1))); % Absoluter Fehler über Toleranz
        if any( I_relerr & I_abserr ) % Fehler bei Überschreitung von absolutem und relativem Fehler
          error('%s: Zwangsbedingungs-Ableitung nach x stimmt nicht mit Zwangsbedingungen überein (Var. 3; Delta x%d)', PName, id);
        end
        end
      end
    end
    fprintf('%s: Konsistenz der Zwangsbedingungs-Gradienten erfolgreich getestet für %s-Euler-Winkel.\n', PName, eulstr);
  end
  %% PKM-Jacobi-Matrix testen gegen Zwangsbedingungen
  % TODO: Bei PKM mit <6 FG passt die Auswahl der Zwangsbedingungen noch
  % nicht, wenn reziproke Winkel gewählt werden.
  if RP.NLEG == 6
    phiconv_sel = uint8([2 4 6 7 9 11]); % Test für alle Euler-Winkel-Konventionen
  else
    phiconv_sel = uint8(2); % Koordinatendefinition funktioniert nur wenn z-Rotation am Ende ist
  end
  for i_phiconv = phiconv_sel  
    eulstr = euler_angle_properties(i_phiconv);
    RP.phiconv_W_E = i_phiconv;
    for i = 1:size(Q_test,1)
      % Jacobi-Matrix testen: Prüfe ob sich bei konsistenter Änderung von q
      % und x die Zwangsbedingungen nicht ändern.
      x0 = X_test(i,:)';
      q0 = Q_test(i,:)';
      % Alternativ: IK berechnen. ZB müssen Null bleiben
      % q0 = RP.invkin_ser(X_test(i,:)', rand(RP.NJ,1), struct('Phit_tol', 1e-12, 'Phir_tol', 1e-12));
      
      % Benutze hier die Zwangsbedingungen für die reduzierten
      % EE-Koordinaten (sonst schlägt der Test fehl).
      Phi1_0 = RP.constr1(q0, x0);
      Phi1dq_0 = RP.constr1grad_q(q0, x0);
      Phi1dx_0 = RP.constr1grad_x(q0, x0);
      
      % Inverse PKM-Jacobi-Matrix
      Jinv1_voll = -Phi1dq_0 \ Phi1dx_0;
      % Jacobi-Zusammenhang zwischen qD und xD ausnutzen: 
      % Inverse differentielle Kinematik (x->q)
      delta_x_red = (0.5-rand(sum(RP.I_EE),1));
      delta_q1 = Jinv1_voll * delta_x_red;
      % Faktor zur Normierung von delta_q auf den Wert 1e-3. Dann gilt noch
      % die Kleinwinkelnäherung und die Beträge sind noch nicht zu klein.
      % Ansonsten gibt es Probleme mit Singularitäten
      k1 = max(abs(delta_q1))*1e3;
      delta_q1 = delta_q1/k1;
      delta_x_red1 = delta_x_red/k1;
      delta_x1 = zeros(6,1); % Umrechnen auf 6 EE-FG für Funktionen
      delta_x1(RP.I_EE) = delta_x_red1;
      % Berechne neue Koordinaten q1,x1 konsistent mit den Zwangsbed.
      q1 = q0 + delta_q1;
      x1 = x0 + delta_x1;
      Phi1_1 = RP.constr1(q1, x1);
      test_Phi1 = Phi1_0 - Phi1_1; % Fehler-Wert: Die ZB müssen gleich bleiben.
      if max(abs(test_Phi1)) > 1e11*eps(1+max(abs(Phi1_0)))
        error('%s: Inverse PKM-Jacobi-Matrix ist nicht mit Zwangsbedingungen konsistent.', PName);
      end
      
      % Wie sieht das Ergebnis bei zufälliger Wahl von delta_q aus?
      test_rand_worst = zeros(length(Phi1_1),1); % schlechtester Fehler bei zufälliger Wahl
      for kk = 1:10
        % Wähle beliebiges delta_x für die Berechnung von delta_q. Die
        % Größenordnung muss gleich sein.
        delta_q_rand = Jinv1_voll * diff(minmax2(delta_x_red'))*2*(0.5-rand(sum(RP.I_EE),1));
        q1_rand = q1+delta_q_rand;
        % Berechne damit die Zwangsbedingungen neu und bestimme den Fehler
        Phi1_1_rand = RP.constr1(q1_rand, x1);
        test_rand = Phi1_0-Phi1_1_rand;
        % Speichere den größten Fehler ab und vergleiche mit Lösung oben
        test_rand_worst(abs(test_rand_worst)<abs(test_rand)) = test_rand(abs(test_rand_worst)<abs(test_rand));
      end
      if max(abs(test_Phi1)) > 0.5*max(abs(test_rand_worst))
        warning('%s: Bei zufälliger Wahl von delta_q ist das Ergebnis nicht viel besser als bei gegebener Berechnung. Wahrscheinlich Singularität.', PName);
      end
      
      % Andere ZB-Definitionen vergleichen:
      % Gradientenmatrizen und inverse Jacobi-Matrix
      Phi2dq_0 = RP.constr2grad_q(q0, x0);
      Phi2dx_0 = RP.constr2grad_x(q0, x0);
      Jinv2_voll = -Phi2dq_0 \ Phi2dx_0; % TODO: Hier noch falsche Zeilen-Auswahl bei <6FG
      if all(RP.I_EE == [1 1 1 1 1 1]) % Benutze ZB Definition 3 nur für räumliche Systeme
      Phi3dq_0 = RP.constr3grad_q(q0, x0);
      Phi3dx_0 = RP.constr3grad_x(q0, x0);
      Jinv3_voll = -Phi3dq_0 \ Phi3dx_0; % TODO: Hier noch falsche Zeilen-Auswahl bei <6FG
      else
      Phi3dq_0=Phi2dq_0*NaN; Phi3dx_0=Phi2dx_0*NaN;
      Jinv3_voll=NaN*Jinv2_voll;
      end
      % Änderung der Gelenkwinkel bei gegebener Plattformänderung
      % Führe auch wieder eine Anpassung der Schrittweite mit Faktor k2 und
      % k3 ein. Wenn andere ZB-Definitionen schlechter konditioniert sind,
      % gibt es sonst Probleme
      delta_q2 = Jinv2_voll * delta_x_red;
      k2 = max(abs(delta_q2))*1e3;
      delta_q2 = delta_q2/k2;
      delta_x_red2 = delta_x_red/k2;
      delta_x2 = zeros(6,1);
      delta_x2(RP.I_EE) = delta_x_red2;
      delta_q3 = Jinv3_voll * delta_x_red;
      k3 = max(abs(delta_q3))*1e3;
      delta_q3 = delta_q3/k3;
      delta_x_red3 = delta_x_red/k3;
      delta_x3 = zeros(6,1);
      delta_x3(RP.I_EE) = delta_x_red3;
      % Daraus resultierende Änderung der Zwangsbedingungen
      Phi2_0 = RP.constr2(q0, x0);
      Phi2_1 = RP.constr2(q0 + delta_q2, x0+delta_x2);
      Phi3_0 = RP.constr3(q0, x0);
      Phi3_1 = RP.constr3(q0 + delta_q3, x0+delta_x3);
      test_Phi2 = Phi2_0 - Phi2_1;
      test_Phi3 = Phi3_0 - Phi3_1;
      if max(abs(test_Phi2)) > 5e11*eps(1+max(abs(Phi2_0)))
        error('Die inverse Jacobi-Matrix der PKM nach Var. 2 ist nicht konsistent zu den Zwangsbedingungen');
      end
      if max(abs(test_Phi3)) > 5e11*eps(1+max(abs(Phi3_0)))
        error('Die inverse Jacobi-Matrix der PKM nach Var. 3 ist nicht konsistent zu den Zwangsbedingungen');
      end
      % Folgende Tests funktionieren nur, wenn die Zwangsbedingungen Null
      % sind. Dafür muss oben die inverse Kinematik für `q0` benutzt werden
      % Vergleich der resultierenden Gelenkwinkeländerungen bei
      % unterschiedlichen ZB-Definitionen
      % Die Winkeländerungen sind unterschiedlich, da die Definitionen und
      % die Gradienten anders sind. Ist aber nicht falsch! Entspricht
      % anderer Gewichtung der Translations- und Rotationskomponenten
      % Wenn Phi=0 ist, stimmen die Werte überein.
      if max(abs(Phi1_0)) < 1e-10 && max(abs([Phi2_0;Phi3_0])) > 1e-10
        error('Wenn Zwangsbed. Var. 1 Null sind, müssen die anderen Var. auch Null sein.');
      end
      test_q12 = delta_q2 - delta_q1;
      test_q13 = delta_q3 - delta_q1;
      test_q23 = delta_q2 - delta_q3;
      if max(abs(Phi1_0)) < 1e-10 && max(abs([test_q12;test_q13;test_q23])) > 1e-10
        error('Wenn ZB Null sind, müssen die Winkeländerungen bei versch. Var. gleich sein.');
      end
      % Vergleich der inversen PKM-Jacobi-Matrix bei verschiedenen ZB-Def.
      % Sind unterschiedlich, wegen unterschiedlicher Def. (nicht falsch!)
      testJ12 = Jinv1_voll - Jinv2_voll;
      testJ13 = Jinv1_voll - Jinv3_voll;
      testJ23 = Jinv2_voll - Jinv3_voll;
      if max(abs(Phi1_0)) < 1e-10 && max(abs([testJ12(:);testJ13(:);testJ23(:)])) > 1e-10
        error('Wenn ZB Null sind, müssen inv. Jacobi-Matrizen bei versch. Var. gleich sein.');
      end
    end
    fprintf('%s: Konsistenz der PKM-Jacobi-Matrix erfolgreich getestet für %s-Euler-Winkel.\n', PName, eulstr);
  end
  RS.phiconv_W_E = 2; % zurücksetzen auf Standard XYZ
  fprintf('fertig mit %s\n', PName);
end
