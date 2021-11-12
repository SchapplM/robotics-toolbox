% Teste Kinematische Zwangsbedingungen in der Roboterklasse SerRob
% Diese entsprechen den Jacobi-Matrizen bei der inversen Kinematik mit
% Euler-Winkeln als Fehlermaß in der Position
% 
% Test-Szenario:
% * Positions- und Orientierungsfehler wird aus Differenzenquotient und aus
%   Gradientenmatrix berechnet. Beide Rechenwege müssen übereinstimmen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc
if isempty(which('serroblib_path_init.m'))
  warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end
%% Benutzereingaben
Robots = {{'S5RRRRR1', 'S5RRRRR1_KUKA1'}, ...
          {'S6RRRRRR10V2', 'S6RRRRRR10V2_KUKA1'}, ...
          {'S7RRRRRRR1', 'S7RRRRRRR1_LWR4P'}};
% Folgende Zeilen zur Prüfung einzelner Roboter einkommentieren:
% Robots = {{'S6RRRRRR10V2', 'S6RRRRRR10V2_KUKA1'}};
% Einstellungen
use_mex_functions = true; % mit mex geht es etwas schneller
% Endeffektor-Transformation ungleich Null festlegen, um zu prüfen, ob die
% Implementierung davon richtig ist
r_W_E = [0.1;0.2;0.3];
phi_W_E = [20; 40; 50]*pi/180;

%% Alle Robotermodelle durchgehen
for Robot_Data = Robots
  SName = Robot_Data{1}{1};
  RName = Robot_Data{1}{2};
  
  %% Klasse für seriellen Roboter erstellen
  % Instanz der Roboterklasse erstellen
  RS = serroblib_create_robot_class(SName, RName);
  % RS.mex_dep();
  serroblib_create_template_functions({SName},false);
%   matlabfcn2mex({[SName,'_constr4']}, true);
%   matlabfcn2mex({[SName,'_constr4grad']});
%   matlabfcn2mex({[SName,'_invkin_eulangresidual']}, true);
  RS.fill_fcn_handles(use_mex_functions, true);
  
  % Grenzen festlegen (für Zusatz-Optimierung)
  RS.qlim = repmat([-pi, pi], RS.NJ, 1);
  RS.update_EE(r_W_E, phi_W_E, []);
  
  % Test-Einstellungen generieren
  TSS = RS.gen_testsettings(true, false);
  
  fprintf('Starte Untersuchung für %s\n', RS.descr);
  
  %% Alle Funktionen einmal aufrufen
  q0 = TSS.Q(1,:)';
  qD0 = TSS.QD(1,:)';
  qDD0 = TSS.QDD(1,:)';
  T_E = RS.fkineEE(q0);
  if any(isnan(T_E(:)))
    error('Roboter %s nicht korrekt initialisiert.', SName);
  end
  xE = [T_E(1:3,4); r2eul(T_E(1:3,1:3), RS.phiconv_W_E)];
  
  RS.fkine(q0);
  RS.fkineEE(q0);
  RS.jacobiR(q0);
  RS.jacobig(q0);
  RS.jacobit(q0);
  RS.jacobiw(q0);
  RS.jtraf(q0);
  
  RS.constr1(q0, RS.x2tr(xE));
  RS.constr1grad(q0, RS.x2tr(xE));
  RS.constr2(q0, RS.x2tr(xE), false);
  RS.constr2grad(q0, RS.x2tr(xE), false);
  RS.constr2(q0, RS.x2tr(xE), true);
  RS.constr2grad(q0, RS.x2tr(xE), true);
  
  fprintf('%s: Alle Funktionen einmal ausgeführt\n', SName);

  %% Gradienten der kinematischen Zwangsbedingungen testen
  M_Namen = {'ZB Variante 1', 'ZB Variante 2', 'ZB Variante 2 reziprok', ...
    'ZB Variante 4 reziprok', 'ZB Variante 4 nicht reziprok'};
  for m = 1:5
    if m == 4 || m == 5
      % Methode ist nur sinnvoll für kinematische Ketten mit 5 Gelenken zur
      % Nutzung als Beinkette in 3T1R-PKM. Wird aber hier für alle getestet
      % EE-FG werden nicht direkt in der Funktion so benutzt, sondern
      % stattdessen x- und y-Komponente der Rotation kombiniert.
      RS.I_EE = [1 1 1 0 1 1];
    end
    for i_phiconv = uint8([2 4 6 7 9 11]) % Test für alle Euler-Winkel-Konventionen
      eulstr = euler_angle_properties(i_phiconv);
      RS.phiconv_W_E = i_phiconv;
      for i = 1:size(TSS.Q,1)
        q0 = TSS.Q(i,:)';
        if i > size(TSS.Q,1)/2 % die zweite Hälfte mit Phi=0
          T_E = RS.fkineEE(q0);
        else % die erste Hälfte mit Phi~=0
          T_E = RS.fkineEE(-0.5+rand(RS.NJ,1));
        end
        xE = [T_E(1:3,4); r2eul(T_E(1:3,1:3), RS.phiconv_W_E)];

        % Zwangsbedingungen und -gradienten für q0 berechnen
        if m == 1
          Phi_0 = RS.constr1(q0, RS.x2tr(xE));
          Phidq_0 = RS.constr1grad(q0, RS.x2tr(xE));
        elseif m == 2
          Phi_0 = RS.constr2(q0, RS.x2tr(xE), false);
          Phidq_0 = RS.constr2grad(q0, RS.x2tr(xE), false);
        elseif m == 3
          Phi_0 = RS.constr2(q0, RS.x2tr(xE), true);
          Phidq_0 = RS.constr2grad(q0, RS.x2tr(xE), true);
        elseif m == 4
          Phi_0_voll = RS.constr2(q0, RS.x2tr(xE), true);
          if all(abs(Phi_0_voll) < 1e-10)
            % Die Modellierung funktioniert nicht, wenn die ZB Null sind.
            continue
          end
          Phidq_0_voll = RS.constr2grad(q0, RS.x2tr(xE), true);
          Phi_0=eval(sprintf('%s_constr4(Phi_0_voll, RS.I_EE, true)', RS.mdlname));
          Phidq_0=eval(sprintf('%s_constr4grad(Phidq_0_voll, Phi_0_voll, RS.I_EE, true)', RS.mdlname));
        elseif m == 5
          Phi_0_voll = RS.constr2(q0, RS.x2tr(xE), false);
          if all(abs(Phi_0_voll) < 1e-10)
            % Die Modellierung funktioniert nicht, wenn die ZB Null sind.
            continue
          end
          Phidq_0_voll = RS.constr2grad(q0, RS.x2tr(xE), false);
          Phi_0=eval(sprintf('%s_constr4(Phi_0_voll, RS.I_EE, false)', RS.mdlname));
          Phidq_0=eval(sprintf('%s_constr4grad(Phidq_0_voll, Phi_0_voll, RS.I_EE, false)', RS.mdlname));
        end
        for id = 1:size(TSS.Q,2) % Alle Komponenten der Gelenkkoordinaten einmal verschieben
          % Neue Koordinaten q1 durch Verschiebung in einer Komponente
          dq = zeros(size(TSS.Q,2),1);
          dq(id) = 1e-6; % eher kleinen Schritt nehmen, damit geringe Abweichung von Linearisierung
          q1 = q0+dq;

          % Zwangsbedingungen für verschobene Koordinaten q1 berechnen
          if m == 1
            Phi_1 = RS.constr1(q1, RS.x2tr(xE));
          elseif m == 2
            Phi_1 = RS.constr2(q1, RS.x2tr(xE), false);
          elseif m == 3
            Phi_1 = RS.constr2(q1, RS.x2tr(xE), true);
          elseif m == 4
            Phi_1_voll = RS.constr2(q1, RS.x2tr(xE), true);
            Phi_1=eval(sprintf('%s_constr4(Phi_1_voll, RS.I_EE, true)', RS.mdlname));
          elseif m == 5
            Phi_1_voll = RS.constr2(q1, RS.x2tr(xE), false);
            Phi_1=eval(sprintf('%s_constr4(Phi_1_voll, RS.I_EE, false)', RS.mdlname));
          end

          % Prüfe neue ZB aus Ableitung gegen direkt berechnete (linksseitiger
          % Differenzenquotient)
          Phi_1_g = Phi_0 + Phidq_0*dq;
          test = Phi_1-Phi_1_g;
          dPhi_diff = Phi_1-Phi_0;
          I_2pierr = abs(abs(dPhi_diff)-2*pi) < 1e-2;
          dPhi_diff(I_2pierr) = angleDiff(Phi_0(I_2pierr),Phi_1(I_2pierr)); % 2pi-Fehler entfernen
          dPhi_grad = Phidq_0*dq;
          test_dPhi_abs = dPhi_grad-dPhi_diff;
          test_dPhi_rel = test_dPhi_abs./dPhi_grad;
          if any( abs(test_dPhi_abs)>1e-6 & abs(test_dPhi_rel)>5e-2 )
            error('%s: Zwangsbedingungs-Ableitung stimmt nicht mit Zwangsbedingungen überein (Methode %d)', SName, m);
          end
        end
      end
      fprintf('%s: Konsistenz der Zwangsbedingungs-Gradienten (Methode %s) erfolgreich getestet für %s-Euler-Winkel.\n', ...
        SName, M_Namen{m}, eulstr);
    end
  end
  fprintf('Zwangsbedingungen erfolgreich getestet für %s\n', SName);
end
