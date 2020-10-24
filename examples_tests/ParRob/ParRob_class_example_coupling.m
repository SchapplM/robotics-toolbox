% Roboterklasse für PKM mit verschiedene Koppelpunkt-Varianten testen
% Beispielsystem: 6RRRRRR (abgeleitet aus 6 Industrierobotern)

% Junnan Li, Hiwi bei Moritz Schappler, 2020-01
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc

%% Definitionen, Benutzereingaben
% Robotermodell entweder aus PKM-Bibliothek oder nur aus
% Seriell-Roboter-Bibliothek laden. Stellt keinen Unterschied dar.
use_parrob = false;
Basis_Radius = 1.3;
Basis_Abstand = 0.2; % bei Paarweise Methode
Steigungswinkel = pi/3;
Plat_Radius = 0.3;
Plat_Abstand = 0.2;
Traj_test = true;

rob_path = fileparts(which('robotics_toolbox_path_init.m'));
respath = fullfile(rob_path, 'examples_tests', 'results');

%% Klasse für PKM erstellen (basierend auf serieller Beinkette)
if isempty(which('serroblib_path_init.m'))
  warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end

% Typ des seriellen Roboters auswählen (S6RRRRRR10 = Seriell-Knickarm)
SName='S6RRRRRR10';
% Instanz der Roboterklasse erstellen
RS = serroblib_create_robot_class(SName);
RS.fill_fcn_handles(true, true);
rankloss_matrix = NaN(9,8);

for base_Coup = 1:9
  for plat_Coup = [1:6 8]
    %% PKM erstellen
    Parname = sprintf('P6RRRRRR10G%dP%dA1',base_Coup,plat_Coup);
    RP = ParRob(Parname);
    RP.create_symmetric_robot(6, RS, 1.0, 0.3);
    RP.initialize();
    RP.fill_fcn_handles(true, true);
    % Kinematik-Parameter des KR 30-3 anpassen und in allgemeines Modell
    % eintragen. Siehe auch: 
    %     RS_tmp = serroblib_create_robot_class('S6RRRRRR10V2', 'S6RRRRRR10V2_KUKA1');
    %     pkin_gen = S6RRRRRR10V2_pkin_var2gen(RS_tmp.pkin);
    pkin_gen = zeros(length(RP.Leg(1).pkin_names),1);
    % Nachbearbeitung einiger Kinematikparameter
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'d1')) = 0.0; % Ergibt kein Sinn für PKM
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'a2')) = 0.2;
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'a3')) = 0.65;
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'d4')) = 0.85;
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'a4')) = 0.2;
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'a5')) = 0.15;
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'alpha2')) = pi/2;
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'alpha4')) = pi/2;
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'a6')) = 0.1;
    pkin_gen(strcmp(RP.Leg(1).pkin_names,'d6')) = 0.0; % ist für P4 ungünstig
    for i = 1:RP.NLEG
      RP.Leg(i).update_mdh(pkin_gen);
    end
    I_qa = false(36,1);
    I_qa(1:6:36) = true;
    RP.update_actuation(I_qa);
    RP.fill_fcn_handles();
    %% Gestell- und Plattform-Konfiguration verändern
    if base_Coup <= 3
      base_par = Basis_Radius;
    elseif base_Coup == 4
      base_par = [Basis_Radius,Steigungswinkel];
    elseif base_Coup <= 7
      base_par = [Basis_Radius;Basis_Abstand];
    elseif base_Coup == 8
      base_par = [Basis_Radius;Basis_Abstand;Steigungswinkel];
    elseif base_Coup == 9
      base_par = Basis_Radius;
    else
      error('Gestell-Orientierungsmethode %d nicht implementiert', base_Coup)
    end
    if plat_Coup <= 3
      plat_par = Plat_Radius;
    elseif plat_Coup <=6
      plat_par = [Plat_Radius;Plat_Abstand];
    elseif plat_Coup == 8
      plat_par = Plat_Radius;
    else
      error('Plattform-Orientierungsmethode %d nicht implementiert', plat_Coup)
    end
    % Coupling einstellen
    RP.align_base_coupling(base_Coup, base_par);
    RP.align_platform_coupling(plat_Coup, plat_par);
    %% Grenzen für die Gelenkpositionen setzen
    % Dadurch wird die Schrittweite bei der inversen Kinematik begrenzt (auf 5%
    % der Spannbreite der Gelenkgrenzen) und die Konfiguration klappt nicht um.
    for i = 1:RP.NLEG
      % Begrenze die Winkel der Kugel- und Kardangelenke auf +/- 360°
      RP.Leg(i).qlim = repmat([-2*pi, 2*pi], RP.Leg(i).NQJ, 1);
    end
    qlim_pkm = cat(1, RP.Leg.qlim);
    %% Startpose bestimmen
    % Mittelstellung im Arbeitsraum
    X = [ [0.05;-0.05;0.5]; [5;-5;3]*pi/180 ];
    leg_config_ok = false(6,1);
    for ii = 1:50
      % Startwerte für numerische IK (definiere nur die der ersten Beinkette)
      q0 = qlim_pkm(:,1)+rand(36,1).*(qlim_pkm(:,2)-qlim_pkm(:,1));
      q0(1) = pi/2; % Erstes Gelenk sollte nach innen zeigen
      q0(2) = pi/2; % Zweites Gelenk sollte nicht nach unten klappen ("Ellenbogen unten")
      q0(3) = -0.7; % Ausprobieren zum Vermeiden von Ellenbogen unten
      % Für jede Koppelpunkt-Variante die Anfangswerte so anpassen, dass
      % die gewünschte (übersichtliche) Konfiguration herauskommt.
      if base_Coup == 1
        q0(1) = rand()*0.3+1.5; % Erstes Gelenk sollte nach innen zeigen
        q0(2) = rand()*0.3+1.5; % Zweites Gelenk sollte nicht nach unten klappen ("Ellenbogen unten")
        q0(3) = rand()*0.3-1; % Ausprobieren zum Vermeiden von Ellenbogen unten
        q0(4) = rand()*0.3;
        q0(5) = rand()*0.2+1.2;
        if plat_Coup == 3
          q0(1) = rand()*0.3+1.5;
          q0(2) = rand()*0.3-1;
          q0(3) = rand()*0.3-3;
          q0(4) = rand()*0.3-3;
          q0(5) = rand()*0.2+1.8;
          q0(6) = rand()*0.2+1.2;
        elseif plat_Coup == 5
          q0(4) = rand()*0.5 + 1.5;
          q0(5) = rand()*0.2+1.2;
          q0(6) = rand()*0.2+2;
        elseif plat_Coup == 6
          q0(4) = rand()*0.2 + 1.3;
          q0(5) = rand()*0.2+1.6;
          q0(6) = rand()*0.2-3;
        end
      elseif base_Coup == 2
        q0(1) = rand()*0.3-1.3; % Erstes Gelenk sollte nach innen zeigen
        q0(2) = rand()*0.5+1.3; % Zweites Gelenk sollte nicht nach unten klappen ("Ellenbogen unten")
        q0(3) = rand()*0.3-1; % Ausprobieren zum Vermeiden von Ellenbogen unten
        q0(4) = rand()*0.4-1.5;
        q0(5) = rand()*0.4+1.5;
        q0(6) = rand()*0.4-2.5;
      elseif base_Coup == 3
        q0(1) = rand()*0.3-2; % Erstes Gelenk sollte nach innen zeigen
        q0(2) = rand()*0.3; % Zweites Gelenk sollte nicht nach unten klappen ("Ellenbogen unten")
        q0(3) = rand()*0.3-3; % Ausprobieren zum Vermeiden von Ellenbogen unten
        q0(4) = rand()*0.2+2.8;
        q0(5) = rand()*0.3+1;
        q0(6) = -rand()*0.2 - 1.6;
      elseif base_Coup == 4
        q0(1) = rand()*0.3-2; % Erstes Gelenk sollte nach innen zeigen
        q0(2) = rand()*0.3+0.5; % Zweites Gelenk sollte nicht nach unten klappen ("Ellenbogen unten")
        q0(3) = rand()*0.3-pi; % Ausprobieren zum Vermeiden von Ellenbogen unten
        q0(4) = rand()*0.3-pi;
        q0(5) = rand()*0.3+1;
        q0(6) = rand()*0.3-1.5;
      elseif base_Coup == 5
        q0(1) = rand()*0.3+1.5; % Erstes Gelenk sollte nach innen zeigen
        q0(2) = rand()*0.3+1.5; % Zweites Gelenk sollte nicht nach unten klappen ("Ellenbogen unten")
        q0(3) = rand()*0.3-1; % Ausprobieren zum Vermeiden von Ellenbogen unten
        q0(4) = rand()*0.3;
        q0(5) = rand()*0.2+0.9;
      elseif base_Coup == 6
        q0(1) = rand()*0.3-1.3; % Erstes Gelenk sollte nach innen zeigen
        q0(2) = rand()*0.3-1.3; % Zweites Gelenk sollte nicht nach unten klappen ("Ellenbogen unten")
        q0(3) = rand()*0.5-3.2; % Ausprobieren zum Vermeiden von Ellenbogen unten
        q0(4) = rand()*0.3-2;
        q0(5) = rand()*0.3+1.6;
      elseif base_Coup == 7
        q0(1) = rand()*0.3-1.3; % Erstes Gelenk sollte nach innen zeigen
        q0(2) = rand()*0.3+0.1; % Zweites Gelenk sollte nicht nach unten klappen ("Ellenbogen unten")
        q0(3) = rand()*0.3-pi; % Ausprobieren zum Vermeiden von Ellenbogen unten
        q0(4) = rand()*0.5+2.5;
        q0(5) = rand()*0.3+1.1;
      elseif base_Coup == 8
        q0(1) = -pi*3/4; % Erstes Gelenk sollte nach innen zeigen
        q0(2) = pi/4; % Zweites Gelenk sollte nicht nach unten klappen ("Ellenbogen unten")
        q0(3) = pi; % Ausprobieren zum Vermeiden von Ellenbogen unten
        q0(4) = -pi*3/4;
        q0(5) = pi/4;
      elseif base_Coup == 9
        % TODO: Festlegen, falls zufällige Konfiguration unpassend aussieht
      end
      % Vorherige Lösungen einsetzen
      q0(7:end) = NaN; % Dadurch wird die Lösung der ersten Beinkette für die nächsten als Startwert gesetzt
      for jj = 1:RP.NLEG
        if leg_config_ok(jj)
          q0(RP.I1J_LEG(jj):RP.I2J_LEG(jj)) = q(RP.I1J_LEG(jj):RP.I2J_LEG(jj));
        end
      end
      % IK berechnen
      [q, Phis] = RP.invkin_ser(X, q0);
      if any(abs(Phis) > 1e-6)
        error('G%d-P%d: Inverse Kinematik (für jedes Bein einzeln) konnte in Startpose nicht berechnet werden', base_Coup, plat_Coup);
      end
      % Prüfe, ob Konfiguration richtig ist
      for jj = 1:RP.NLEG
        q_diff = q(RP.I1J_LEG(jj):RP.I2J_LEG(jj)) - q0(RP.I1J_LEG(jj):RP.I2J_LEG(jj));
        if any( abs(q_diff([1 2 3 4 5 6])) > 90*pi/180 & abs(q_diff([1 2 3 4 5 6])) < 270*pi/180)
          leg_config_ok(jj) = false;
        else
          leg_config_ok(jj) = true;
        end
      end
      if all(leg_config_ok)
        break;
      elseif ii == 50
        fprintf('Keine erfolgreiche Konfiguration gefunden\n');
      end     
    end
    %% Zwangsbedingungen in Startpose testen
    Phi1=RP.constr1(q, X);
    Phit1=RP.constr1_trans(q, X);
    Phir1=RP.constr1_rot(q, X);
    if any(abs(Phi1) > 1e-6)
      error('ZB in Startpose ungleich Null');
    end
    
    %% Jacobi-Matrizen auswerten
    G_q = RP.constr1grad_q(q, X);
    G_x = RP.constr1grad_x(q, X);
    
    % Aufteilung der Ableitung nach den Gelenken in Gelenkklassen
    % * aktiv/unabhängig (a),
    % * passiv+schnitt/abhängig (d)
    for aa = 1:3
      I_qa = false(36,1);
      I_qa(aa:6:36) = true;
      RP.update_actuation(I_qa);
      Par_name = RP.mdlname;
      Par_name(end) = sprintf('%d',aa);
      G_a = G_q(:,RP.I_qa);
      G_d = G_q(:,RP.I_qd);
      % Jacobi-Matrix zur Berechnung der abhängigen Gelenke und EE-Koordinaten
      G_dx = [G_d, G_x];
      
      fprintf('%s: Rang der vollständigen Jacobi der inversen Kinematik: %d/%d (Kondition %1.1f)\n', ...
        Par_name, rank(G_q), RP.NJ, cond(G_q));
      fprintf('%s: Rang der vollständigen Jacobi der direkten Kinematik: %d/%d (Kondition %1.1e)\n', ...
        Par_name, rank(G_dx), sum(RP.I_EE)+sum(RP.I_qd), cond(G_dx));
      fprintf('%s: Rang der Jacobi der aktiven Gelenke: %d/%d\n', ...
        Par_name, rank(G_a), sum(RP.I_EE));
      
      Jinv_num_voll = -inv(G_q) * G_x;
      Jinv_num = Jinv_num_voll(RP.I_qa,:);
      rank_J = rank(Jinv_num, 1e-6);
      fprintf('%s: Rang der inversen PKM-Jacobi: %d/%d (Kondition %1.1e)\n', ...
        Par_name, rank_J, sum(RP.I_qa), cond(Jinv_num));
    end
    % Inverse Jacobi-Matrix aus symbolischer Berechnung (mit Funktion aus HybrDyn)
    if ~isempty(which('parroblib_path_init.m'))
      Jinv_sym = RP.jacobi_qa_x(q, X);
      test_Jinv = Jinv_sym - Jinv_num;
      if max(abs(test_Jinv(:))) > 1e-10
        error('Inverse Jacobi-Matrix stimmt nicht zwischen numerischer und symbolischer Berechnung überein');
      else
        fprintf('Die inverse Jacobi-Matrix stimmt zwischen symbolischer und numerischer Berechnung überein\n');
      end
    end
    
    if rank_J < sum(RP.I_qa)
      rankloss_matrix(base_Coup,plat_Coup) = 1;
    else
      rankloss_matrix(base_Coup,plat_Coup) = 0;
    end
    
    %% Entwurfsparameter setzen
    for i = 1:RP.NLEG
      % Setze Schubgelenke als Hubzylinder
      RP.Leg(i).DesPar.joint_type(RP.I_qa((RP.I1J_LEG(i):RP.I2J_LEG(i)))) = 5;
      % Setze Segmente als Hohlzylinder mit Radius 50mm
      RP.Leg(i).DesPar.seg_par=repmat([5e-3,50e-3],RP.Leg(i).NL,1);
    end
    if plat_Coup < 4 || plat_Coup == 8 % Kreisförmig
      RP.DesPar.platform_par(2) = 5e-3;
    else % Paarweise
      RP.DesPar.platform_par(3) = 5e-3;
    end
    %% Roboter in Startpose plotten
    if base_Coup == 1 || plat_Coup == 1
      figure(base_Coup*10+plat_Coup); clf; hold on; grid on;% Bild der Entwurfsparameter
      set(base_Coup*10+plat_Coup, 'name', sprintf('G%dP%d', base_Coup, plat_Coup), 'numbertitle', 'off');
      xlabel('x in m');ylabel('y in m');zlabel('z in m'); view(3);
      s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG-1], 'straight', 1, 'mode', 4);
      RP.plot( q, X, s_plot );
      Fig2_title = sprintf('G%dP%d. Jacobi: Rang %d/%d; Kondition %1.3e', ...
        base_Coup, plat_Coup, rank(Jinv_num, 1e-6), sum(RP.I_qa), cond(Jinv_num));
      title(Fig2_title);
      drawnow
    end
    
    if Traj_test
      % Trajektorie mit beliebigen Bewegungen der Plattform
      X0 = [ [0.05;-0.05;0.5]; [5;-5;3]*pi/180 ];     
      XL = [X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
        X0'+1*[[ 0.0, 0.0, 0.05], [0.0, 0.0, 0.0]*pi/180]; ...
        X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 5.0]*pi/180]; ...
        X0'+1*[[ 0.0, 0.05, 0.0], [0.0, 0.0, 0.0]*pi/180]; ...
        X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 5.0, 0.0]*pi/180]; ...
        X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, -5.0]*pi/180]; ...
        X0'+1*[[ 0.05, 0.0, 0.0], [5.0, 0.0, 0.0]*pi/180]; ...
        X0'+1*[[ -0.05, 0.0, -0.05], [0.0, -5.0, 0.0]*pi/180]; ...
        X0'+1*[[ 0.0, 0.0, 0.0], [-5.0, 0.0, 0.0 ]*pi/180]];
      XL = [XL; XL(1,:)]; % Rückfahrt zurück zum Startpunkt.
      [X_t,XD_t,XDD_t,t] = traj_trapez2_multipoint(XL, 1, 0.1, 0.01, 1e-3, 1e-1);
      % Inverse Kinematik berechnen
      q0 = q; % Lösung der IK von oben als Startwert
      t0 = tic();
      % IK-Einstellungen: Sehr lockere Toleranzen, damit es schneller geht
      s = struct('Phit_tol', 1e-3, 'Phir_tol', 0.1*pi/180);
      [q1, Phi_num1] = RP.invkin1(X_t(1,:)', q0, s);
      if any(abs(Phi_num1) > 1e-2)
        warning('IK konvergiert nicht');
      end
      fprintf('Inverse Kinematik für Trajektorie berechnen: %d Bahnpunkte\n', length(t));
      [Q_t, ~, ~, Phi_t] = RP.invkin_traj(X_t, XD_t, XDD_t, t, q1, s);
      % Traj.-IK mit kompilierter Funktion: Geht nur mit parroblib_create_robot
      % [Q_t, ~, ~, Phi_t] = RP.invkin2_traj(X_t, XD_t, XDD_t, t, q1, s);
      if any(any(abs(Phi_t(:,RP.I_constr_t_red)) > s.Phit_tol)) || ...
          any(any(abs(Phi_t(:,RP.I_constr_r_red)) > s.Phir_tol))
        error('Fehler in Trajektorie zu groß. IK nicht berechenbar');
      end
      fprintf('%1.1fs nach Start. %d Punkte berechnet.\n', ...
        toc(t0), length(t));
    end
    
  end
end

return
%% Auswertung des Rangverlusts
comb_loss_status = NaN(8*4,3);
for i = 1:8
  for j = 1:4
    comb_loss_status(4*(i-1)+j,:) = [i,j,rankloss_matrix(i,j)];
  end
end
figure(200);clf;hold on;
I_loss = comb_loss_status(:,3) == 1;
plot(comb_loss_status(~I_loss,1), comb_loss_status(~I_loss,2), 'gv');
plot(comb_loss_status( I_loss,1), comb_loss_status( I_loss,2), 'rx');
xlabel('Gestell-Nummer'); ylabel('Plattform-Nummer');
title('Übersicht Rangverlust');
legend({'Voller Rang', 'Rangverlust'});