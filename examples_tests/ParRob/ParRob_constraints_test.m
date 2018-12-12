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
  RP = RP.create_symmetric_robot(N_LEG, RS, 0.5, 0.2);
  
  RP.initialize();
  X = [ [0.0;0.0;0.5]; [0;0;0]*pi/180 ];
  xE = X + [[0.1; 0.05; 0]; [0; 0; 10]*pi/180];
  
  % Kinematik-Parameter anpassen: Zufällige Rotation zu Koppelpunkt-KS der
  % Beinketten. Damit wird geprüft, ob die Funktionen allgemein funktionieren
  for i = 1:RP.NLEG
    RP.Leg(i).update_EE([], rand(3,1));
  end
  RP.update_base(r_W_0, phi_W_0);
  RP.update_EE(r_P_E, phi_P_E, []);
  %% Initialisierung der Test-Variablen
  q0 = rand(RP.NJ,1);
  qD = rand(RP.NJ,1);
  n = 100;
  Q_test = rand(n, RP.NJ);
  X_test = rand(n, 6);
  
  %% Alle Funktionen einmal aufrufen
  
  RP.fkine(q0, xE);
  RP.fkine_legs(q0);
  RP.fkine_platform(xE);
  RP.constr1(q0, xE);
  RP.constr1_trans(q0, xE);
  RP.constr1_rot(q0, xE);
  RP.constr1grad_q(q0, xE);
  RP.constr1grad_x(q0, xE);

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
      Phi1_0 = RP.constr1(q0, x0);
      Phi1dq_0 = RP.constr1grad_q(q0, x0);
      Phi1dx_0 = RP.constr1grad_x(q0, x0);
      % Alle Komponenten der Gelenkkoordinaten einmal verschieben und
      % ZB-Gradienten testen (Geometrische Matrix der inversen Kinematik)
      for id = 1:length(q0) 
        % Neue Koordinaten q1 durch Verschiebung in einer Komponente
        dq = zeros(length(q0),1);
        dq(id) = 1e-4;
        q1 = q0+dq;

        % Zwangsbedingungen für verschobene Koordinaten q1 berechnen
        Phi1_1 = RP.constr1(q1, x0);

        % Prüfe neue ZB aus Ableitung gegen direkt berechnete (linksseitiger
        % Differenzenquotient)
        Phi1_1_g = Phi1_0 + Phi1dq_0*dq;
        test1 = Phi1_1-Phi1_1_g;
        % Vielfache von 2*Pi entfernen
        test1(abs(abs(test1)-2*pi) < 1e-6 ) = 0;
        
        % Prüfe das Inkrement der ZB-Änderung. Ist dieses Qualitativ
        % gleich, kann man davon ausgehen, dass die Lösung richtig ist.
        dPhi_grad = Phi1dq_0*dq;
        dPhi_diff = Phi1_1 - Phi1_0;
        RelErr = dPhi_grad./dPhi_diff;
        RelErr(isnan(RelErr)) = 1; % 0=0 -> relativer Fehler 0 -> 100%
        % if any(abs(RelErr-1) > 0.10)
        %   [dPhi_grad, dPhi_diff]
        %   % error('Relativer Fehler größer als 10%');
        % end
        
        if max(abs(test1)) > 8e10*eps(max(1+abs(Phi1_1)))
          error('%s: Zwangsbedingungs-Ableitung nach q stimmt nicht mit Zwangsbedingungen überein', PName);
        end
      end
      % Alle Komponenten der EE-Koordinaten einmal verschieben und
      % ZB-Gradienten testen (Geometrische Matrix der direkten Kinematik)
      for id = 1:6 
        % Neue Koordinaten q1 durch Verschiebung in einer Komponente
        dx = zeros(6,1);
        dx(id) = 1e-4;
        x1 = x0+dx;

        % Zwangsbedingungen für verschobene Koordinaten q1 berechnen
        Phi1_1 = RP.constr1(q0, x1);

        % Prüfe neue ZB aus Ableitung gegen direkt berechnete (linksseitiger
        % Differenzenquotient)
        Phi1_1_g = Phi1_0 + Phi1dx_0*dx;
        test1 = Phi1_1-Phi1_1_g;
        % Vielfache von 2*Pi entfernen (Rotationsfehler von 2*Pi ist kein
        % "Fehler" sondern nur in der Darstellung begründet)
        test1(abs(abs(test1)-2*pi) < 1e-6 ) = 0;
        
        % Prüfe das Inkrement der ZB-Änderung. Ist dieses Qualitativ
        % gleich, kann man davon ausgehen, dass die Lösung richtig ist.
        dPhi_grad = Phi1dx_0*dx;
        dPhi_diff = Phi1_1 - Phi1_0;
        RelErr = dPhi_grad./dPhi_diff;
        RelErr(isnan(RelErr)) = 1; % 0=0 -> relativer Fehler 0 -> 100%
        % if any(abs(RelErr-1) > 0.10)
        %   [dPhi_grad, dPhi_diff]
        %   % error('Relativer Fehler größer als 10%');
        % end
        
        if max(abs(test1)) > 8e10*eps(max(1+abs(Phi1_1)))
          error('%s: Zwangsbedingungs-Ableitung nach x stimmt nicht mit Zwangsbedingungen überein', PName);
        end
      end
    end
    fprintf('%s: Konsistenz der Zwangsbedingungs-Gradienten erfolgreich getestet für %s-Euler-Winkel.\n', PName, eulstr);
  end
  RS.phiconv_W_E = 2; % zurücksetzen auf Standard XYZ
  fprintf('fertig mit %s\n', PName);
end