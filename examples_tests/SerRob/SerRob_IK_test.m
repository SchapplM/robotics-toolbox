% Teste die Roboterklasse SerRob mit verschiedenen Systemen bzgl Inverse
% Kinematik
% 
% Achtung: Dieses Testskript ist größtenteils veraltet! Warnungen können
% ignoriert werden. Es sollte aber fehlerfrei laufen.
% 
% Verschiedene Test-Szenarien:
% * 1) Aufruf aller Funktionen (zur Prüfung auf Syntax-Fehler)
% * 2) Inverse Kinematik zu erreichbaren Punkten im Kartesischen Arbeitsraum
% * 3) IK für erreichbare Richtung des Endeffektors im Arbeitsraum (Aufgabe
%   mit einer Symmetrieachse
% * 4) Aufruf der Zielfunktion für die Nullraumoptimierung
% * 5) IK mit Zusatzoptimierung von Kennzahlen durch eine Nullraumbewegung
% * 6) IK für komplette Trajektorie im Kartesischen Arbeitsraum
% 
% Test verschiedener Funktionen:
% * Klassen-Funktion (aus SerRob), die allgemein konfigurierbar ist
% * Roboterspezifische Funktion, die direkt an den jeweiligen Roboter
%   angepasst ist und dadurch kompilierbar ist (ermöglicht Faktor 50
%   schneller Ausführung)
% 
% Details zu den Tests:
% (5) Test der inversen Kinematik (mit Optimierung von Nebenbedingungen)
% * "ohne": Keine Optimierung der Nebenbedingungen
% * "mit1": Gleichzeitige Optimierung von Zielannäherung und Nebenbedingung
% * "mit2": Erst Zielannäherung mit IK, dann Optimierung der NB als
%   Nullraumbewegung
% 
% Bei Test 5 und 6 wird die Implementierung als Klassenmethode und als
% eigene Funktionsdatei verglichen. Da die Implementierungen nicht mehr
% identisch sind, sind diesbezügliche Warnungen nicht mehr relevant.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07, 2019-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

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

Robots = {{'S4RRPR1', 'S4RRPR1_KUKA1'}, ...
          {'S5RRRRR1', 'S5RRRRR1_KUKA1'}, ...
          {'S6RRRRRR10V2', 'S6RRRRRR10V2_KUKA1'}, ...
          {'S7RRRRRRR1', 'S7RRRRRRR1_LWR4P'}};
% Folgende Zeilen zur Prüfung einzelner Roboter einkommentieren:
% Robots = {{'S4RRPR1', 'S4RRPR1_KUKA1'}};
% Robots = {{'S5RRRRR1', 'S5RRRRR1_KUKA1'}};
% Robots = {{'S6RRRRRR10V2', 'S6RRRRRR10V2_KUKA1'}};
% Robots = {{'S7RRRRRRR1', 'S7RRRRRRR1_LWR4P'}};

% Einstellungen
use_mex_functions = true; % mit mex geht es etwas schneller, dafür ist debuggen schwieriger
minimal_test = true; % Nur sehr wenige zufällige Winkel testen (geht schneller)
% Endeffektor-Transformation ungleich Null festlegen, um zu prüfen, ob die
% Implementierung davon richtig ist
r_W_E = [0.1;0.2;0.3];
phi_W_E = [20; 40; 50]*pi/180;

%% Alle Robotermodelle durchgehen
for Robot_Data = Robots
  SName = Robot_Data{1}{1};
  RName = Robot_Data{1}{2};
  
  %% Code ggf neu generieren
  % Kann händisch aktiviert werden, wenn im HybrDyn-Repo Änderungen
  % durchgeführt werden:
  % serroblib_generate_code({SName}, true, false, 1) % Für alle Funktionen
  % serroblib_generate_code({SName}, true, false, 2) % Für Funktionen aus Vorlage
  
  %% Klasse für seriellen Roboter erstellen
  % Instanz der Roboterklasse erstellen
  % serroblib_create_template_functions({SName}, false, false);
  serroblib_update_template_functions({SName});
  RS = serroblib_create_robot_class(SName, RName);
%   RS.mex_dep(true);
  RS.fill_fcn_handles(use_mex_functions, true);
  
  % Grenzen festlegen (für Zusatz-Optimierung)
  RS.qlim = repmat([-pi, pi], RS.NQJ, 1);
  RS.update_EE(r_W_E, phi_W_E, []);
  
  % Test-Einstellungen generieren
  TSS = RS.gen_testsettings(true, false);
  if minimal_test
    TSS.Q = TSS.Q(1:3,:);
  end
  fprintf('Starte Untersuchung für %s\n', RS.descr);
  %% Init 
  % Bereich reduzieren: Bei Addition von Zufallswinkeln darf nicht pi
  % überschritten werden.
  TSS.Q(abs(TSS.Q(:))>150*pi/180) = 0;
  
  %% (Test 1) Alle Funktionen einmal aufrufen
  fprintf('%s: Test 1 (Funktionsaufrufe)\n', SName);
  q0 = TSS.Q(1,:)';
  qD0 = TSS.QD(1,:)';
  qDD0 = TSS.QDD(1,:)';
  T_E = RS.fkineEE(q0);
  Tr_E = T_E(1:3,:);
  xE = [T_E(1:3,4); r2eul(T_E(1:3,1:3), RS.phiconv_W_E)];
  
  RS.fkine(q0);
  RS.fkineEE(q0);
  RS.jacobiR(q0);
  RS.jacobig(q0);
  RS.jacobia(q0);
  RS.jacobit(q0);
  RS.jacobiw(q0);
  RS.jacobigD(q0,qD0);
  RS.jacobiaD(q0,qD0);
  RS.jacobitD(q0,qD0);
  RS.jacobiwD(q0,qD0);
  RS.jtraf(q0);
  
  RS.constr1(q0, Tr_E);
  RS.constr1grad(q0, Tr_E);
  RS.constr2(q0, Tr_E);
  RS.constr2grad(q0, Tr_E);
  RS.invkin(Tr_E, q0+0.1*ones(RS.NJ,1));
  RS.invkin2(Tr_E, q0+0.1*ones(RS.NJ,1));
  RS.invkin_traj(repmat(xE',2,1), zeros(2,6), zeros(2,6), [0;1], q0+0.1*ones(RS.NJ,1));
  RS.invkin2_traj(repmat(xE',2,1), zeros(2,6), zeros(2,6), [0;1], q0+0.1*ones(RS.NJ,1));
  fprintf('%s: Alle Funktionen einmal ausgeführt\n', SName);

  %% (Test 2) Inverse Kinematik (Normal) prüfen (für verschiedene Posen)
  % IK-Ziel ist immer erreichbar, Startwerte für iterativen Algorithmus
  % liegen in der Nähe der Zielkonfiguration
  % Benutze die EE-FG des Systems (aus I_EE): Die Bewegung ist also je nach
  % Roboter 3T3R (Industrieroboter), 3T0R (SCARA)
  fprintf('%s: Test 2 (IK Normal)\n', SName);
  for m = 2 % Nur nach Methode 2 prüfen
    for i_phiconv = uint8([2 4 6 7 9 11]) % Test für alle funktionierenden Euler-Winkel-Konventionen für 3T3R
      eulstr = euler_angle_properties(i_phiconv);
      RS.phiconv_W_E = i_phiconv;
      n_iO1 = 0; n_iO2 = 0; % Zähler für Erfolg der IK (SerRob/Spez.)
      T1 = 0; T2 = 0; % Zeit für IK (SerRob/Spez.)
      for i = 1:size(TSS.Q,1)
        q = TSS.Q(i,:)'; 
        T_E = RS.fkineEE(q); % Ziel-Pose (erreichbar, da aus direkter Kin.)
        xE = [T_E(1:3,4); r2eul(T_E(1:3,1:3), RS.phiconv_W_E)];
        q0 = q-20*pi/180*(0.5-rand(RS.NQJ,1)); % Anfangswinkel 20° neben der Endstellung
        % Berechnung mit SerRob
        tic();
        q_test1 = RS.invkin(RS.x2tr(xE), q0, struct('constr_m', m)); 
        T1 = T1 + toc;
        % Erfolg mit Klassen-Methode prüfen
        T_E_test1 = RS.fkineEE(q_test1);
        test_T1 = T_E\T_E_test1 - eye(4);
        if any(abs(test_T1(:)) > 1e-8) || any(isnan(test_T1(:)))
          warning('%s: DK/IK (Var. %d) stimmt nicht', SName, m);
        else
          n_iO1 = n_iO1+1;
        end
        % Berechnung mit Spez. Funktion
        tic();
        q_test2 = RS.invkin2(RS.x2tr(xE), q0); 
        T2 = T2 + toc;
        % Erfolg mit Spez. Funktion prüfen
        T_E_test2 = RS.fkineEE(q_test2);
        test_T2 = T_E\T_E_test2 - eye(4);
        if any(abs(test_T2(:)) > 1e-8) || any(isnan(test_T2(:)))
          warning('%s: DK/IK (Var. %d) stimmt nicht', SName, m);
        else
          n_iO2 = n_iO2+1;
        end
      end
      fprintf(['%s: Inverse Kinematik Variante %d getestet (%s-Euler-Winkel).', ...
        '\n\t%d/%d erfolgreich mit Klasse. %d/%d kompiliert.', ...
        '\n\tZeit: %1.1fms (Klassenmethode) vs %1.1fms (Funktion)\n'], ...
        SName, m, eulstr, n_iO1, size(TSS.Q,1), n_iO2, size(TSS.Q,1), ...
        1e3*T1/size(TSS.Q,1), 1e3*T2/size(TSS.Q,1));
    end
  end
  RS.phiconv_W_E = uint8(2); % zurücksetzen auf Standard XYZ
  fprintf('Einzelpunkt-IK für %s erfolgreich getestet\n', SName);

  %% (Test 3) Inverse Kinematik mit reduzierten FG (3T2R-Aufgabe)
  fprintf('%s: Test 3 (IK 3T2R)\n', SName);
  if all(RS.I_EE([4,5]))
    % IK-Ziel ist jetzt nicht vollständige Pose, sondern die Richtung der
    % z-Achse des Endeffektors (3T2R-Aufgabe)
    for i_phiconv = uint8([2, 6]) % Schleife für verschiedene Euler-Konventionen (Tait-Bryan), deren letzter Winkel z ist
      RS.phiconv_W_E = i_phiconv;
      eulstr = euler_angle_properties(i_phiconv);
      n_iO1 = 0; n_iO2 = 0; % Zähler für Erfolg der IK (SerRob/Spez.)
      T1 = 0; T2 = 0; % Zeit für IK (SerRob/Spez.)
      for i = 1:size(TSS.Q,1)
        % Ziel- und Anfangs-Konfiguration definieren (mit Symmetrieachse)
        q = TSS.Q(i,:)';
        T_E = RS.fkineEE(q);
        xE = [T_E(1:3,4); r2eul(T_E(1:3,1:3), RS.phiconv_W_E)];
        xE(6) = 0; % Rotation um z-Achse des EE interessiert nicht.
        q0 = q-20*pi/180*(0.5-rand(RS.NQJ,1)); % Anfangswinkel 20° neben der Endstellung
        T_E0 = RS.fkineEE(q0);
        % Berechnung mit SerRob
        tic();
        q_test1 = RS.invkin(RS.x2tr(xE), q0, struct('constr_m',2, 'I_EE', logical([1 1 1 1 1 0])));
        T1 = T1 + toc();
        % Prüfe Erfolg mit SerRob
        T_E_test1 = RS.fkineEE(q_test1);
        test_T1 = T_E\T_E_test1 - eye(4);
        test_T1 = test_T1(:,[3,4]); % Spalten mit x-y-Einheitsvektoren lassen sich nicht vergleichen.
        if any(abs(test_T1(:)) > 1e-8) || any(isnan(test_T1(:)))
          % Teilweise konvergiert die IK nicht, wenn der Abstand zu groß ist.
          warning('%s: DK/IK stimmt nicht für 3T2R-Aufgabe mit %s-Euler-Winkeln', SName, eulstr);
        else
          n_iO1 = n_iO1+1;
        end
        % Berechnung mit Spez. Funktion
        tic();
        q_test2 = RS.invkin2(RS.x2tr(xE), q0, struct('I_EE', logical([1 1 1 1 1 0])));
        T2 = T2 + toc();
        % Prüfe Erfolg mit Spez. Funktion
        T_E_test2 = RS.fkineEE(q_test2);
        test_T2 = T_E\T_E_test2 - eye(4);
        test_T2 = test_T2(:,[3,4]);
        if any(abs(test_T2(:)) > 1e-8) || any(isnan(test_T2(:)))
          % Teilweise konvergiert die IK nicht, wenn der Abstand zu groß ist.
          warning('%s: DK/IK stimmt nicht für 3T2R-Aufgabe mit %s-Euler-Winkeln', SName, eulstr);
        else
          n_iO2 = n_iO2+1;
        end
      end
      fprintf(['%s: Inverse Kinematik Variante 2 mit Aufgabenredundanz getestet (%s-Euler-Winkel).', ...
        '\n\t%d/%d erfolgreich mit Klasse. %d/%d kompiliert.', ...
        '\n\tZeit: %1.1fms (Klassenmethode) vs %1.1fms (Funktion)\n'], ...
        SName, eulstr, n_iO1, size(TSS.Q,1), n_iO2, size(TSS.Q,1), ...
        1e3*T1/size(TSS.Q,1), 1e3*T2/size(TSS.Q,1));
    end
    RS.phiconv_W_E = uint8(2); % zurücksetzen auf Standard XYZ
    fprintf('Einzelpunkt-IK für 3T2R-Aufgabe für %s erfolgreich getestet\n', SName);
  else
    fprintf('Einzelpunkt-IK für 3T2R-Aufgabe mit %s nicht möglich und daher nicht getestet.\n', SName);
  end
  %% (Test 5) Inverse Kinematik für 3T2R-/3T3R-Aufgabe (mit Zusatzoptimierung) prüfen
  % Gleiche Tests wie oben, aber zusätzliche Optimierung im Nullraum der
  % inversen Kinematik
  fprintf('%s: Test 5 (IK 3T2R und 3T3R mit Zusatzoptimierung)\n', SName);
  wn = zeros(RS.idx_ik_length.wnpos, 1);
  wn(RS.idx_ikpos_wn.qlim_par) = 1; % Auswahl der Nebenbedingungen zur Optimierung
  for tr = [false, true] % Schleife 3T3R bzw 3T2R (Aufgabenredundanz)
    if tr
      % EE-Koordinaten, die für die Aufgabe benötigt sind. "0" bedeutet,
      % dass der FG "egal" ist (Rotationssymmetrie der 3T2R-Aufgabe)
      I_EE_Task = logical([1 1 1 1 1 0]);
      % Zulässige Euler-Winkel-Konventionen für die Definition des
      % Residual-Vektors (letzter Winkel muss der Symmetrieachse (z)
      % entsprechen
      eulconv_Task = uint8([2, 6]);
    else
      I_EE_Task = logical([1 1 1 1 1 1]);
      eulconv_Task = uint8([2 4 6 7 9 11]);
    end
    for m = 2 % Nur Methode 2 prüfen (Methode 1 ist nicht als eigene Funktion implementiert)
      if RS.NQJ < 7-tr
        fprintf('%s: Roboter hat nur %d FG und für %dFG-Aufgaben keinen Nullraum\n', ...
          RS.mdlname, RS.NJ, 6-tr);
        continue
      end
      for i_phiconv = eulconv_Task % Schleife für verschiedene Euler-Konventionen
        RS.phiconv_W_E = i_phiconv;
        eulstr = euler_angle_properties(i_phiconv);
        n_iO1 = 0; n_iO2 = 0; % Zähler für erfolgreiche IK (ohne Nullraum)
        n_identq = 0; % Zähler für identische Ergebnisse (Klasse vs. Sequ.)
        T_ges = zeros(2,3); % Zeitmessung für Klasse/Sequ. (Zeilen) und 3 IK-Verfahren
        K_zopt = NaN(size(TSS.Q,1),6);
        Q_zopt = NaN(size(TSS.Q,1),3*size(TSS.Q,2));
        for i = 1:size(TSS.Q,1)
          % Start- und Ziel-Pose (erreichbar)
          q = TSS.Q(i,:)';
          T_E = RS.fkineEE(q);
          xE = [T_E(1:3,4); r2eul(T_E(1:3,1:3), RS.phiconv_W_E)];
          q0 = q-10*pi/180*(0.5-rand(RS.NQJ,1)); % Anfangswinkel 10° neben der Endstellung
           % Berechnung mit SerRob: Ohne Optimierung ("ohne")
          tic();
          q_ohne = RS.invkin(RS.x2tr(xE), q0, ...
            struct('I_EE', I_EE_Task, 'constr_m', m));
          T_ges(1,1)=T_ges(1,1)+toc();
          if any(isnan(q_ohne)), return; end
          % Berechnung mit SerRob: gleichzeitige Optimierung ("mit1")
          s = struct('K',5e-1*ones(RS.NJ,1), ...
                     'Kn',1e-2*ones(RS.NJ,1), ...
                     'wn',wn, ...
                     'I_EE', I_EE_Task, 'constr_m', m);
          tic()
          [q_mit1,~,Q_mit1] = RS.invkin(RS.x2tr(xE), q0, s);
          T_ges(1,2)=T_ges(1,2)+toc();
          % Berechnung mit SerRob: nachträgliche Optimierung ("mit2")
          tic();
          [q_mit2,Phi,Q_mit2] = RS.invkin(RS.x2tr(xE), q_ohne, ...
            struct('K',5e-1*ones(RS.NJ,1), ...
                   'Kn',1e-2*ones(RS.NJ,1), ...
                   'n_min', 50, ...
                   'wn',wn, ...
                   'I_EE', I_EE_Task, 'constr_m', m));
          T_ges(1,3)=T_ges(1,3)+toc();
          % Ergebnisse prüfen
          test_T_ohne = T_E\RS.fkineEE(q_ohne) - eye(4);
          test_T_mit1 = T_E\RS.fkineEE(q_mit1) - eye(4);
          test_T_mit2 = T_E\RS.fkineEE(q_mit2) - eye(4);
          if tr
            test_T_ohne=test_T_ohne(:,[3,4]);test_T_mit1=test_T_mit1(:,[3,4]);test_T_mit2=test_T_mit2(:,[3,4]);
          end
          test_T = [test_T_ohne(:); test_T_mit1(:); test_T_mit2(:)];
          if any(abs(test_T) > 1e-5) || any(isnan(test_T))
            warning('%s: DK/IK stimmt nicht (Var. %d, %s-Euler-Winkel)', SName, m, eulstr);
          else
            n_iO1 = n_iO1+1;
          end
          % Prüfe, ob Kriterium verbessert wurde
          K_zopt(i,1) = invkin_optimcrit_limits1(q_ohne, RS.qlim);
          K_zopt(i,2) = invkin_optimcrit_limits1(q_mit1, RS.qlim);
          K_zopt(i,3) = invkin_optimcrit_limits1(q_mit2, RS.qlim);
          if K_zopt(i,1) < K_zopt(i,2) || K_zopt(i,1) < K_zopt(i,3)
            warning('%d/%d: Die Zielfunktion ist mit Optimierung schlechter als ohne (Berechnung mit Klassenfunktion)', i, size(TSS.Q,1));
          end
          Q_zopt(i,:) = [q_ohne; q_mit1; q_mit2]';

          % Gleiche Rechnung wie oben, aber mit kompilierten Funktionen
          tic();
          q2_ohne = RS.invkin2(RS.x2tr(xE), q0, struct('I_EE', I_EE_Task));
          T_ges(2,1)=T_ges(2,1)+toc();
          s = struct('K',5e-1*ones(RS.NJ,1), ...
                     'Kn',1e-2*ones(RS.NJ,1), ...
                     'wn',wn, ...
                     'I_EE', I_EE_Task);
          tic();
          q2_mit1 = RS.invkin2(RS.x2tr(xE), q0, s); Q2_mit1 = NaN;
          T_ges(2,2)=T_ges(2,2)+toc();
          s = struct('K',5e-1*ones(RS.NJ,1), ...
                     'Kn',1e-2*ones(RS.NJ,1), ...
                     'n_min', 50, ...
                     'wn',wn, ...
                     'I_EE', I_EE_Task);
          tic();
          q2_mit2 = RS.invkin2(RS.x2tr(xE), q_ohne, s);
          T_ges(2,3)=T_ges(2,3)+toc();
          test_T2_ohne = T_E\RS.fkineEE(q2_ohne) - eye(4);
          test_T2_mit1 = T_E\RS.fkineEE(q2_mit1) - eye(4);
          test_T2_mit2 = T_E\RS.fkineEE(q2_mit2) - eye(4);
          if tr
            test_T2_ohne=test_T2_ohne(:,[3,4]);test_T2_mit1=test_T2_mit1(:,[3,4]);test_T2_mit2=test_T2_mit2(:,[3,4]);
          end
          test_T2 = [test_T2_ohne(:); test_T2_mit1(:); test_T2_mit2(:)];
          if any(abs(test_T2) > 1e-5) || any(isnan(test_T2))
            warning('%s: DK/IK stimmt nicht (Var. %d) Kompilierte Funktionen', SName, m);
          else
            n_iO2 = n_iO2+1;
          end
          K_zopt(i,4) = invkin_optimcrit_limits1(q2_ohne, RS.qlim);
          K_zopt(i,5) = invkin_optimcrit_limits1(q2_mit1, RS.qlim);
          K_zopt(i,6) = invkin_optimcrit_limits1(q2_mit2, RS.qlim);
          if K_zopt(i,4) < K_zopt(i,5) || K_zopt(i,4) < K_zopt(i,6)
            warning('%d/%d: Die Zielfunktion ist mit Optimierung schlechter als ohne (Berechnung mit Roboterspezifischer Funktion)', i, size(TSS.Q,1));
          end
          % Prüfe, ob Ergebnis exakt mit Klassen-Methode übereinstimmt
          test_ohne = q2_ohne - q_ohne;
          test_mit1 = q2_mit1 - q_mit1;
          test_mit2 = q2_mit2 - q_mit2;
          if max(abs([test_ohne; test_mit1; test_mit2])) > 1e-3
            % Debuggen, woran es liegt, dass die eigentlich identischen
            % Methoden unterschiedliche Ergebnisse liefern
            % Ursache: Schlechte Konditionierung in Verbindung mit
            % Rundungsfehlern
            % Aktuell: Funktionen sind gar nicht identisch. Wieder
            % aktivieren, falls Funktionen angeglichen werden.
%             change_current_figure(20);clf; 
%             subplot(3,1,1); hold on; grid on; plot(Q_mit1); set(gca, 'ColorOrderIndex',1); plot(Q2_mit1,'--');
%             Z_all = NaN(size(Q_mit1,1),2);
%             for jj = 1:size(Q_mit1,1), Z_all(jj,1)=invkin_optimcrit_limits1(Q_mit1(jj,:)');Z_all(jj,2)=invkin_optimcrit_limits1(Q2_mit1(jj,:)'); end
%             subplot(3,1,2);hold on; grid on; plot(Z_all(:,1),'-');plot(Z_all(:,2),'--');
% %             Phi_all1 = NaN/(
% %             for jj = 1:size(Q_mit1,1), Z_all(jj,1)=invkin_optimcrit_limits1(Q_mit1(jj,:)');Z_all(jj,2)=invkin_optimcrit_limits1(Q2_mit1(jj,:)'); end
%             subplot(3,1,3); hold on; grid on; plot(Q_mit1-Q2_mit1);
%             linkxaxes
%             warning('%d/%d: Ergebnis zwischen Klassenfunktion und kompilierter Funktion stimmt nicht (mögl. schlecht konditioniert)', i, size(TSS.Q,1));
          else
            n_identq = n_identq + 1;
          end            
        end
        %% Auswertung: Optimierung der Nebenbedingungen mit den drei Ansätzen
        % Bild: Optimierungskritierum mit den verschiedenen Ansätzen und
        % Implementierungen. Aussage des Bildes: Welches Verfahren führt
        % zum besten Optimierungskriterium.
        fhdl = change_current_figure(1);clf;
        set(fhdl, 'Name', sprintf('%s_Optim_NB',SName), 'NumberTitle', 'off');
        subplot(2,1,1); hold on
        title(sprintf('%s: Optimierung der Nebenbedingung',SName));
        plot(K_zopt(:,1), 'rs'); plot(K_zopt(:,4), 'r+');
        plot(K_zopt(:,2), 'kv'); plot(K_zopt(:,5), 'k^');
        plot(K_zopt(:,3), 'mo'); plot(K_zopt(:,6), 'm*');
        
        ylabel('Opt. Krit (kleiner = besser)');
        xlabel('lfd Nummer IK-Konfiguration');
        legend({'ohne (SerRob)', 'ohne (Spez.)', 'mit1 (SerRob)', 'mit1 (Spez.)', 'mit2 (SerRob)', 'mit2 (Spez.)'});
        grid on;
        subplot(2,1,2); hold on;
        title(sprintf('%s: Vergleich Optimierungskriterium bzgl IK ohne Opt.',SName));
        plot(K_zopt(:,1)-K_zopt(:,2), 'ko'); plot(K_zopt(:,4)-K_zopt(:,5), 'k+');
        plot(K_zopt(:,1)-K_zopt(:,3), 'gs'); plot(K_zopt(:,4)-K_zopt(:,6), 'g*');
        plot([1;size(K_zopt,1)], [0;0], 'r--');
        legend({'mit gleichz. Opt. (Klasse)', 'gleichz. (Spez.)', 'mit sequ. Opt. (Klasse)', 'sequ. (Spez.)'});
        xlabel('lfd Nummer IK-Konfiguration');
        ylabel('Diff. Opt. Krit (muss >0 sein)');
        grid on;
        if tr == 1, trstring = 'mit Aufgabenredundanz';
        else,       trstring = 'ohne Aufgabenredundanz';
        end

        % Bild: Verschiedene Gelenkwinkel
        fhdl = change_current_figure(2);clf;
        set(fhdl, 'Name', sprintf('%s_q',SName), 'NumberTitle', 'off');
        for j = 1:min(RS.NQJ,6)
          subplot(3,2,j); hold on
          if j == 1
            title(sprintf('%s: Verlauf der Gelenkwinkel für IK-Kombinationen',SName));
          end
          plot(Q_zopt(:,[j, 6+j, 12+j]));
          xlabel('lfd Nummer IK-Konfiguration');
          ylabel(sprintf('q%d', j));
          grid on;
        end
        fprintf('%s: Inverse Kinematik Variante %d mit Nullraumoptimerung %s getestet. %d/%d erfolgreich (%s-Euler-Winkel)\n', ...
          SName, m, trstring, n_iO1, size(TSS.Q,1), eulstr);
        fprintf('\tVergleich Zeiten Klasse/Roboterspezifisch: ohne %1.1fs/%1.1fs, mit gleichz. %1.1fs/%1.1fs, mit nacheinander %1.1fs/%1.1fs.\n', ...
          T_ges(1,1), T_ges(2,1), T_ges(1,2), T_ges(2,2), T_ges(1,3), T_ges(2,3));
        fprintf('\tVergleich Leistung Klasse/Roboterspezifisch: Erfolg %d/%d, identisches Ergebnis: %d, Schlechter nach Optimierung: %d/%d (gleichz.), %d/%d (sequ.)\n', ...
          n_iO1, n_iO2, n_identq, ...
          sum(K_zopt(:,1)-K_zopt(:,2)<=0), sum(K_zopt(:,4)-K_zopt(:,5)<=0), ...
          sum(K_zopt(:,1)-K_zopt(:,3)<=0), sum(K_zopt(:,4)-K_zopt(:,6)<=0));
      end
    end
  end
  %% (Test 6) Inverse Kinematik für komplette Trajektorie (3T3R) prüfen
  fprintf('%s: Test 6 (IK 3T3R für Trajektorie)\n', SName);
  % Trajektorien-IK als Funktion berücksichtigt auch Geschwindigkeit und
  % Beschleunigung benötigt dadurch weniger IK-Iterationen
  for m = 2 % Nur Methode 2 prüfen
    if RS.NQJ < 6
      fprintf('Roboter %s hat nur %d FG. Keine allgemeinen kartesischen Trajektorien verfolgbar.\n', SName, RS.NQJ);
      break;
    end
    for i_phiconv = uint8([2 4 6 7 9 11]) % Test für alle funktionierenden Euler-Winkel-Konventionen
      RS.phiconv_W_E = i_phiconv;
      eulstr = euler_angle_properties(i_phiconv);
      
      RS.phiconv_W_E = i_phiconv;
      % Trajektorie generieren
      T = zeros(3,1); % Zeitmessung für Trajektorie
      % Direkte Kinematik für einzelne Gelenkwinkel der Trajektorie
      X = RS.fkineEE_traj(TSS.Q,TSS.Q*0,TSS.Q*0);
      % Kartesische Trajektorie zwischen einzelnen Posen berechnen
      [X_t,XD_t,XDD_t,t] = traj_trapez2_multipoint(X, 2, 0.100, 0.050, 0.001, 0.100);
      % Trajektorie auf wenige Punkte reduzieren (dauert sonst zu lange)
      if minimal_test
        nn = 100;
      else
        nn = min(5000, length(t)); %#ok<UNRCH>
      end
      nn = min(nn, length(t));
      X_t = X_t(1:nn, :);
      XD_t = XD_t(1:nn, :);
      XDD_t = XDD_t(1:nn, :);
      t = t(1:nn,:);
      % Gelenktrajektorie mit Trajektorien-IK auf zwei Wege berechnen
      tic();
      [Q_IK, QD_IK, QDD_IK, PHI_IK] = RS.invkin_traj(X_t,XD_t,XDD_t, t, q0, struct('constr_m', m));
      T(1) = toc;
      if max(abs(PHI_IK(:))) > 1e-9
        warning('Trajektorie stimmt nicht mit Klassenmethode');
      end
      tic();
      [Q_IK2, QD_IK2, QDD_IK2, PHI_IK2] = RS.invkin2_traj(X_t,XD_t,XDD_t, t, q0);
      T(2) = toc;
      if max(abs(PHI_IK2(:))) > 1e-9
        warning('Trajektorie stimmt nicht mit eigener Funktion');
      end
      % Vergleichsaufruf ohne Eingabe von Geschwindigkeiten
      tic();
      [Q_IK3, ~, ~] = RS.invkin2_traj(X_t,XD_t*0,XDD_t*0, t, q0);
      T(3) = toc;
      % Berechnung vergleichen
      test1 = Q_IK - Q_IK2;
      test1(abs(abs(test1)-2*pi) < 1e-3) = 0; % 2pi-Fehler entfernen
      test1D = QD_IK - QD_IK2;
      test1DD = QDD_IK - QDD_IK2;
      if false && (max(abs(test1(:))) > 1e-6 || max(abs(test1D(:))) > 1e-5 || max(abs(test1DD(:))) > 1e-1)
        % Aktuell ist die Abweichung der Methoden kein Fehler, da die
        % Implementierung unterschiedlich ist.
        % Fehler suchen
        % t(find(sum(abs(test1D),2) > 1e-5))
        % test1DD(find(sum(abs(test1DD),2) > 1e-5), :)
        
        % Fehler: Berechne und Zeichne Debug-Informationen
        [X_IK,XD_IK,XDD_IK] = RS.fkineEE_traj(Q_IK,QD_IK,QDD_IK);
        % QD_IK3 = [zeros(1,RS.NQJ); diff(Q_IK)] ./ 0.001;
        change_current_figure(1);clf;
        subplot(3,2,sprc2no(3,2,1,1)); hold on;
        plot(t, Q_IK); set(gca, 'ColorOrderIndex', 1);
        plot(t, Q_IK2, '--');
        subplot(3,2,sprc2no(3,2,2,1)); hold on;
        plot(t, QD_IK); set(gca, 'ColorOrderIndex', 1);
        plot(t, QD_IK2, '--');
        % plot(t, QD_IK3, '.-');
        subplot(3,2,sprc2no(3,2,3,1)); hold on;
        plot(t, QDD_IK); set(gca, 'ColorOrderIndex', 1);
        plot(t, QDD_IK2, '--');
        subplot(3,2,sprc2no(3,2,1,2)); hold on;
        plot(t, X_t); set(gca, 'ColorOrderIndex', 1);
        plot(t, X_IK, '--');
        legend({'rx', 'ry', 'rz', 'phi1', 'phi2', 'phi3'});
        subplot(3,2,sprc2no(3,2,2,2)); hold on;
        plot(t, XD_t); set(gca, 'ColorOrderIndex', 1);
        plot(t, XD_IK, '--');
        subplot(3,2,sprc2no(3,2,3,2)); hold on;
        plot(t, XDD_t); set(gca, 'ColorOrderIndex', 1);
        plot(t, XDD_IK, '--');
        linkxaxes
        warning('Trajektorie stimmt nicht zwischen Berechnungsalternativen. Wahrscheinlich schlechte Kondition.');
      end
      fprintf('%s: IK für Traj mit %s-Euler-Winkeln berechnet. Pro Bahnpunkt/Gesamt: %1.1fms/%1.1fs für Klasse, %1.1fms/%1.1fs für komp. Insgesamt %d Bahnpunkte\n', ...
        SName, eulstr, 1e3*T(1)/nn, T(1), 1e3*T(2)/nn, T(2), nn);
      fprintf('\tZeitgewinn durch Berücksichtigung der Geschwindigkeit in der IK: %1.2fs -> %1.2fs (%1.0f%%)\n', ...
        T(3), T(2), 100*(T(2)/T(3)-1));
    end
    fprintf('Trajektorien-IK für %s erfolgreich getestet\n', SName);
  end
end

fprintf('Alle Tests erfolgreich durchgeführt\n');

%% Teste Konsistenz der Optimierungskriterien
% Der Gradient muss mit der Größe selbst übereinstimmen
Q_test = -0.5+rand(100,1);
Qlim_test = -0.5+rand(100,2);
Qlim_test(Qlim_test(:,1)>Qlim_test(:,2),2) = 1+Qlim_test(Qlim_test(:,1)>Qlim_test(:,2),2);
for jj = 1:3
  for i = 1:size(Q_test,1)
    q = Q_test(i,:);
    qlim = Qlim_test(i,:);
    deltaq = 1e-6;
    q1 = q+deltaq; % Test-Winkel
    
    % Setze Schwellwert für Spline-Interpolation fest: 10% der Spannweite
    % von der Grenze entfernt (jeweils oben und unten). Also 80% in der Mitte
    % des Gelenkbereichs ist das Kriterium 2 Null.
    qlim_thr = mean(qlim) + [-0.4, +0.4]*diff(qlim);
    assert(all(qlim_thr(:)>qlim(1)) && all(qlim_thr(:)<qlim(2)), ...
      'Schwellwert für Umschaltung muss innerhalb der Grenzen liegen');
    qlim_sw = mean([qlim;qlim_thr]);
    % Teste Stetigkeit der stückweise definierten Funktion
    if jj == 3
      h_left = NaN(4,1); h_right = NaN(4,1);
      h_left(1) = invkin_optimcrit_limits2(qlim_thr(1)-eps, qlim, qlim_thr);
      h_right(1) = invkin_optimcrit_limits2(qlim_thr(1)+eps, qlim, qlim_thr);
      h_left(2) = invkin_optimcrit_limits2(qlim_thr(2)-eps, qlim, qlim_thr);
      h_right(2) = invkin_optimcrit_limits2(qlim_thr(2)+eps, qlim, qlim_thr);
      h_left(3) = invkin_optimcrit_limits2(qlim_sw(1)-eps, qlim, qlim_thr);
      h_right(3) = invkin_optimcrit_limits2(qlim_sw(1)+eps, qlim, qlim_thr);
      h_left(4) = invkin_optimcrit_limits2(qlim_sw(2)-eps, qlim, qlim_thr);
      h_right(4) = invkin_optimcrit_limits2(qlim_sw(2)+eps, qlim, qlim_thr);
      assert(all(abs(h_left-h_right) < 1e-8), ['Eine Umschaltbedingung in ', ...
        'invkin_optimcrit_limits2 stimmt nicht']);
    end
    
    % Berechne Zielfunktion und Gradient für zwei Stellen
    if jj == 1
      [h, hdq] = invkin_optimcrit_limits1(q, qlim);
      h1 = invkin_optimcrit_limits1(q1, qlim);
    elseif jj == 2 % hyperbolisch ohne Spline
      if q < qlim(1) || q > qlim(2), continue; end
      [h, hdq] = invkin_optimcrit_limits2(q, qlim);
      h1 = invkin_optimcrit_limits2(q1, qlim);
    elseif jj == 3 % hyperbolisch mit Spline
      if q < qlim(1) || q > qlim(2), continue; end
      [h, hdq] = invkin_optimcrit_limits2(q, qlim, qlim_thr);
      h1 = invkin_optimcrit_limits2(q1, qlim, qlim_thr);
    else
      error('Fall nicht definiert');
    end
    h1_diff = h+hdq*deltaq; % berechne ZF an zweiter Stelle mit Gradient
    % Vergleiche direkte Berechnung mit Berechnung aus Gradienten.
    test_h_abs = h1_diff-h1;
    test_h_rel = test_h_abs/h;
    hdq_diff = (h1-h)/deltaq;
    test_hdq_abs = hdq-hdq_diff;
    test_hdq_rel = test_hdq_abs/hdq;
    % Fehlertoleranz 5%. Bei hyperbolischem Kriterium können sehr große
    % Zahlen entstehen. Dann ungenaue Linearisierung.
    if abs(test_h_abs) > 1e-5 && test_h_rel > 5e-2 || ...
        abs(test_hdq_abs) > 1e-3 && test_hdq_rel > 5e-2
      figure(1); clf; hold on;
      Qplot = unique([(min([q;qlim(1)]):1e-4:max([q;qlim(2)]))';q;q1]);
      Hplot = NaN(size(Qplot));
      for k = 1:length(Qplot)
        if jj == 1
          Hplot(k) = invkin_optimcrit_limits1(Qplot(k), qlim);
        elseif jj == 2
          Hplot(k) = invkin_optimcrit_limits2(Qplot(k), qlim);
        elseif jj ==3
          Hplot(k) = invkin_optimcrit_limits2(Qplot(k), qlim, qlim_thr);
        else
          error('Fall nicht definiert');
        end
      end
      plot(Qplot, Hplot);
      plot(q, h, 'ro');
      plot(q1, h1, 'ks');
      plot(q1, h1_diff, 'cv');
      plot([q;q1], [h;h1_diff], 'b-');
      xlabel('q'); ylabel('h'); grid on;
      xlim([q-deltaq, q1+deltaq]);
      error('invkin_optimcrit_limits%d: Gradient vs Zielfunktion falsch.', jj);
    end
  end
end
%% Teste Optimierungskriterium basierend auf der Konditionszahl
k_test = [linspace(1, 1000, 10000), logspace(3, 7, 200)]';
n = length(k_test);
h_test = NaN(n,1);
k_thr = 500;
for i = 1:n
  h_test(i) = invkin_optimcrit_condsplineact(k_test(i), k_thr);
end
figure(55);clf;
plot(k_test, h_test);
xlabel('Konditionszahl k');
ylabel('Optimierungskriterium h');
grid on;
title('Validiierung Kriterium Kondition mit Aktivierungsschwelle');
xlim([1, k_thr*1.1]);
