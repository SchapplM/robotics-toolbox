% Beispiel-Skript für 3PRRRR- und PUU-Roboter
% * Erstellung des Modells und Visualisierung
% * Inverse Kinematik für kartesische Trajektorie
% 
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

rob_path = fileparts(which('robotics_toolbox_path_init.m'));
resdir = fullfile(rob_path, 'examples_tests', 'results');
serrobpath = fileparts(which('serroblib_path_init.m'));
if isempty(serrobpath)
  warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end
if isempty(which('parroblib_path_init.m'))
  warning('Repo mit parallelen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end
%% Mögliche PKM bestimmen (zur Planung)
% Mögliche PKM anzeigen
[PNames_Kin, PNames_Akt] = parroblib_filter_robots([1 1 1 0 0 0], 0);
% Filtere nach PRRRR
PNames_Akt = PNames_Akt(contains(PNames_Akt,'P3PRRRR'));
% Filtere nach G1 (z-Achse senkrecht zur Ebene) und G4 (pyramidale z-Achse)
PNames_Akt = PNames_Akt(contains(PNames_Akt,'G1') | contains(PNames_Akt,'G4'));
% Nehme nur automatisch generierte Beinketten (aus Struktursynthese)
I_autogen = false(length(PNames_Akt),1);
for i = 1:length(PNames_Akt)
  % Informationen zur PKM und zur Beinkette laden
  [~, LEG_Names] = parroblib_load_robot(PNames_Akt{i});
  mdllistfile_Ndof = fullfile(serrobpath, sprintf('mdl_%sdof', ...
    LEG_Names{1}(2)), sprintf('S%s_list.mat',LEG_Names{1}(2)));
  l = load(mdllistfile_Ndof);
  % Gesuchte Beinkette aus Datenbank holen
  II = strcmp(l.Names_Ndof, LEG_Names{1});
  % Daten zur Modellherkunft auswerten
  Origin = l.BitArrays_Origin(II,:);
  if bitand(Origin, bin2dec('1110')) ~= 0 % entspricht Tabellenspalten in csv
    % Kette kommt aus Struktursynthese
    I_autogen(i) = true;
  end
end
PNames_Akt = PNames_Akt(I_autogen);

PKM_selection = {'P3PRRRR8G1P3A1', 'P3PRRRR4G1P2A1', 'P3PRRRR6G4P1A1'};
% Schleife über alle möglichen Strukturen
for PKMName = PKM_selection
  % Prüfe, ob genannte PKM überhaupt in gefilterter Liste enthalten ist
  if ~any(strcmp(PNames_Akt, PKMName{1}))
    error('Gesuchte PKM ist nicht in PKM-Datenbank vorhanden (oder gültig)');
  end
  %% Klasse für PKM erstellen
  % PKM laden (aus Bibliothek)
  RP = parroblib_create_robot_class(PKMName{1}, 0.5, 0.2);
  RP.fill_fcn_handles(true, true);
  % Kinematik-Parameter belegen
  fprintf('Kinematik-Parameter von %s:\n', RP.Leg(1).mdlname);
  disp(RP.Leg(1).pkin_names);
  pkin = rand(size(RP.Leg(1).pkin)); % Alle Parameter zufällig setzen
  RP.Leg(1).update_mdh(pkin);
  if rank(RP.Leg(1).jacobig(rand(RP.Leg(1).NJ,1))) < RP.Leg(1).NJ
    error('Mit zufälliger Parametrierung hat die Beinkette einen Rangverlust');
  end
  pkin = zeros(size(RP.Leg(1).pkin)); % Alle Parameter Null setzen
  % Die wichtigsten Parameter wieder auf sinnvolle Werte setzen
  if strcmp(RP.mdlname, 'P3PRRRR1G1P3A1')
    pkin(1) = 0.0;
    pkin(2) = 0.4;
  elseif strcmp(RP.mdlname, 'P3PRRRR8G1P3A1')
    pkin(strcmp(RP.Leg(1).pkin_names, 'theta1')) = 0;
    pkin(strcmp(RP.Leg(1).pkin_names, 'alpha2')) = 0;
    pkin(strcmp(RP.Leg(1).pkin_names, 'd2')) = 0.0;
    pkin(strcmp(RP.Leg(1).pkin_names, 'a2')) = 0.0;
    pkin(strcmp(RP.Leg(1).pkin_names, 'a4')) = 0.5;
  elseif strcmp(RP.mdlname, 'P3PRRRR4G1P2A1')
    pkin(strcmp(RP.Leg(1).pkin_names, 'theta1')) = pi/2;
    pkin(strcmp(RP.Leg(1).pkin_names, 'a2')) = 0.05;
    pkin(strcmp(RP.Leg(1).pkin_names, 'a3')) = 0.4;
    pkin(strcmp(RP.Leg(1).pkin_names, 'a4')) = 0.05;
    pkin(strcmp(RP.Leg(1).pkin_names, 'a5')) = 0.6;
  elseif strcmp(RP.mdlname, 'P3PRRRR6G4P1A1')
    pkin(strcmp(RP.Leg(1).pkin_names, 'theta1')) = pi/2;
    pkin(strcmp(RP.Leg(1).pkin_names, 'a2')) = 0.3;
    pkin(strcmp(RP.Leg(1).pkin_names, 'a4')) = 0.5;
    pkin(strcmp(RP.Leg(1).pkin_names, 'd2')) = 0.1;
    pkin(strcmp(RP.Leg(1).pkin_names, 'a3')) = 0.3;
    pkin(strcmp(RP.Leg(1).pkin_names, 'a5')) = 0.3;
    pkin(strcmp(RP.Leg(1).pkin_names, 'd5')) = 0.1;
  else
    error('Peine parameter hinterlegt');
  end
  for i = 1:RP.NLEG
    RP.Leg(i).update_mdh(pkin);
  end
  if rank(RP.Leg(1).jacobig(rand(RP.Leg(1).NJ,1))) < RP.Leg(1).NJ
    error('Mit aktueller Parametrierung hat die Beinkette einen Rangverlust');
  end
  % PKM ist oberhalb der Aufgabe angebracht (und zeigt nach unten)
  RP.update_base([0;0;1], [pi;0;0]);
  %% Zwangsbedingungen in Startpose testen
  XW_t0 = [0.05;0.05;0.6; 0; 0; 0];
  x0h = invtr(RP.T_W_0)*[XW_t0(1:3);1];
  X0_t0 = [x0h(1:3); XW_t0(4:6)];
  % Anfangs-Pose so, dass Bein bereits im Arbeitsraum mit richtiger
  % Orientierung liegt.
  q0_leg = rand(RP.Leg(1).NJ,1);
  if strcmp(RP.mdlname, 'P3PRRRR1G1P3A1')
    q0_leg(1) = -0.2; % Schubgelenk zuerst nach oben fahren
  elseif strcmp(RP.mdlname, 'P3PRRRR4G1P2A1')
    q0_leg(1) = 0.1;
    q0_leg(5) = -0.3;
  end
  q0 = repmat(q0_leg, 3, 1);

  % IK mit zwei Methoden testen. Sehr genau berechnen, damit die Prüfung
  % rotatorischer FG und der Jacobi-Matrizen im nächsten Abschnitt auch
  % sehr genau ist.
  [q, Phi_test1] = RP.invkin_ser(X0_t0, q0, struct('Phit_tol', 1e-13, 'Phir_tol', 1e-13));

  Phi1=RP.constr1(q, X0_t0);
  if any(abs(Phi1) > 1e-6) || any(isnan(Phi1))
    warning('ZB in Startpose ungleich Null');
    q = q0;
  end

  %% Anfangs-Konfiguration zeichnen
  figure(1);clf;
  hold on;grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
  view(3);
  s_plot = struct( 'ks_legs', [RP.I2L_LEG], ...
                   'ks_platform', RP.NLEG+[1,2], ...
                   'straight', 0);
  RP.plot( q, X0_t0, s_plot );

  %% Teste Jacobi-Matrizen
  if any(abs(Phi1) > 1e-6) || any(isnan(Phi1))
    % Folgende Rechnungen ergeben keinen Sinn
    return
  end
  
  G_q  = RP.constr1grad_q(q, X0_t0);
  G_x = RP.constr1grad_x(q, X0_t0);
  % Aufteilung der Ableitung nach den Gelenken in Gelenkklassen 
  % * aktiv/unabhängig (a),
  % * passiv+schnitt/abhängig (d)
  G_a = G_q(:,RP.I_qa);
  G_d = G_q(:,RP.I_qd);
  % Jacobi-Matrix zur Berechnung der abhängigen Gelenke und EE-Koordinaten
  G_dx = [G_d, G_x];

  % Gebe Rang der Einzel-Matrizen aus
  fprintf('%s: Rang der vollständigen Jacobi der inversen Kinematik: %d/%d\n', ...
    RP.mdlname, rank(G_q), RP.NJ);
  fprintf('%s: Rang der vollständigen Jacobi der direkten Kinematik: %d/%d\n', ...
    RP.mdlname, rank(G_dx), sum(RP.I_EE)+sum(RP.I_qd));
  fprintf('%s: Rang der Jacobi der aktiven Gelenke: %d/%d\n', ...
    RP.mdlname, rank(G_a), sum(RP.I_EE));
  % Berechne PKM-Jacobi
  Jinv_voll = -G_q \ G_x;
  Jinv_qD_xD = Jinv_voll(RP.I_qa,:);
  fprintf('%s: Rang der PKM-Jacobi (aktiv -> EE): %d/%d\n', ...
    RP.mdlname, rank(Jinv_qD_xD), 3);

  %% Trajektorie bestimmen
  % Start in Grundstellung
  k=1; XE_W = XW_t0';
  % Fahrt in Startstellung
  k=k+1; XE_W(k,:) = XE_W(k-1,:) + [ -0.05,0.05,0.05, 0,0,0];
  % Beginn Würfel
  d1=0.1;
  h1=0.1;
  k=k+1; XE_W(k,:) = XE_W(k-1,:) + [ d1,0,0, 0,0,0];
  k=k+1; XE_W(k,:) = XE_W(k-1,:) + [0,-d1,0  0,0,0];
  k=k+1; XE_W(k,:) = XE_W(k-1,:) + [-d1,0,0, 0,0,0];
  k=k+1; XE_W(k,:) = XE_W(k-1,:) + [0,0,-h1, 0,0,0];
  k=k+1; XE_W(k,:) = XE_W(k-1,:) + [0,0, h1, 0,0,0];
  k=k+1; XE_W(k,:) = XE_W(k-1,:) + [0,d1,0,  0,0,0];
  k=k+1; XE_W(k,:) = XE_W(k-1,:) + [0,0,-h1, 0,0,0];
  k=k+1; XE_W(k,:) = XE_W(k-1,:) + [ d1,0,0, 0,0,0];
  k=k+1; XE_W(k,:) = XE_W(k-1,:) + [0,-d1,0  0,0,0];
  k=k+1; XE_W(k,:) = XE_W(k-1,:) + [0,0, h1, 0,0,0];
  k=k+1; XE_W(k,:) = XE_W(k-1,:) + [0,0,-h1, 0,0,0];
  k=k+1; XE_W(k,:) = XE_W(k-1,:) + [-d1,0,0, 0,0,0];
  k=k+1; XE_W(k,:) = XE_W(k-1,:) + [0,d1,0,  0,0,0];
  k=k+1; XE_W(k,:) = XE_W(k-1,:) + [0,0, h1, 0,0,0];
  [X,XD,XDD,T] = traj_trapez2_multipoint(XE_W, 1, 1e-1, 1e-2, 1e-3, 0.25);
  Traj_W = struct('T', T, 'X', X, 'XD', XD, 'XDD', XDD);
  % Trajektorien-Eckpunkte in Basis-KS umrechnen
  Traj_0 = RP.transform_traj(Traj_W);
  Traj_0.X(:,4) = 0;
  %% Roboter in Startpose mit Beispieltrajektorie plotten
  % Roboter wurde vorher schon gezeichnet. Es fehlt nur die Trajektorie
  figure(1);
  plot3(Traj_W.X(:,1), Traj_W.X(:,2), Traj_W.X(:,3), 'r--');
  saveas(1, fullfile(resdir, sprintf('3T0R_PUU_Test_%s_Roboter.fig', PKMName{1})));
  %% Inverse Kinematik berechnen
  iksettings = struct('Phit_tol', 1e-8, 'Phir_tol', 1e-8);
  [Q, QD, QDD, Phi] = RP.invkin2_traj(Traj_0.X,Traj_0.XD,Traj_0.XDD,T,q0, iksettings);
  for i = 1:length(T)
    if max(abs( Phi(:) )) > 1e-4 || any(isnan(Phi(:)))
      warning('IK stimmt nicht. Wahrscheinliche Ursache: Ist innerhalb von n_max nicht konvergiert');
      return
    end
  end
  % Prüfe parasitäre Bewegung während der Trajektorie (die nicht in den
  % Zwangsbedingungen Phi enthalten sind)
  for jj = 1:length(T)
    [~,PhiD_jj] = RP.constr4D(Q(jj,:)', QD(jj,:)', Traj_0.X(jj,:)',Traj_0.XD(jj,:)');
    if any(abs(PhiD_jj)> max(iksettings.Phir_tol,iksettings.Phit_tol))
      error('Fehler in Zeitableitung der kin. Zwangsbed.: Parasitäre Bewegung!')
    end
  end
  %% Zeitverlauf der Trajektorie plotten
  figure(4);clf;
  subplot(3,2,sprc2no(3,2,1,1)); hold on;
  sgtitle('Zeitverlauf Bewegung');
  plot(T, Traj_0.X(:,RP.I_EE));
  legend({'$x$', '$y$', '$z$'}, 'interpreter', 'latex')
  grid on;
  ylabel('x_E');
  title('Plattform-Position');
  subplot(3,2,sprc2no(3,2,2,1));
  plot(T, Traj_0.XD(:,RP.I_EE));
  grid on;
  ylabel('xD_E');
  title('Plattform-Geschw.');
  subplot(3,2,sprc2no(3,2,3,1));
  plot(T, Traj_0.XDD(:,RP.I_EE));
  grid on;
  ylabel('xDD_E');
  title('Plattform-Beschl.');
  subplot(3,2,sprc2no(3,2,1,2));
  plot(T, Q);
  grid on;
  ylabel('Q');
  title('Gelenk-Position');
  subplot(3,2,sprc2no(3,2,2,2)); hold on;
  plot(T, Phi(:,RP.I_constr_t_red));
  plot(T([1 end]), iksettings.Phit_tol*[1;1], 'r--');
  plot(T([1 end]),-iksettings.Phit_tol*[1;1], 'r--');
  grid on;
  ylabel('\Phi_{trans}');
  title('Zwangsbed. transl.');
  subplot(3,2,sprc2no(3,2,3,2)); hold on;
  plot(T, Phi(:,RP.I_constr_r_red));
  plot(T([1 end]), iksettings.Phir_tol*[1;1], 'r--');
  plot(T([1 end]),-iksettings.Phir_tol*[1;1], 'r--');
  grid on;
  ylabel('\Phi_{rot}');
  title('Zwangsbed. rot.');
  saveas(4, fullfile(resdir, sprintf('3T0R_PUU_Test_%s_Zeitverlauf.fig', PKMName{1})));
  %% Animation
  mkdirs(resdir);
  s_anim = struct( 'mp4_name', fullfile(resdir, sprintf('3T0R_PUU_Test_%s_anim.mp4', PKMName{1})));
  figure(9);clf;
  set(9,'units','normalized','outerposition',[0 0 1 1]); % Vollbild
  hold on;
  plot3(Traj_W.X(:,1), Traj_W.X(:,2), Traj_W.X(:,3));
  grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
  view(3);
  title('Animation der kartesischen Trajektorie');
  RP.anim( Q(1:20:length(T),:), Traj_0.X(1:20:length(T),:), s_anim, s_plot);

end