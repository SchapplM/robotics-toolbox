% Zeichne den parallelen Roboter in einer gegebenen Pose
% 
% Eingabe:
% q
%   Gelenk-Winkel (bzw. verallgemeinerte Koordinaten) des Roboters
% x
%   Endeffektor-Pose (entsprechend Roboterdefinition)
% s
%   Struktur mit Plot-Einstellungen. Felder, siehe Quelltext
%   * ks_legs: Plot der durch Nummer gegebenen KS der Beinketten. 
%     Reihenfolge bzw. Nummerierung wie in ParRob/fkine_legs
%   * ks_platform: Plot der durch Nummer gegebenen KS der Plattform. 
%     Reihenfolge bzw. Nummerierung wie in ParRob/fkine_platform
%   * straight: Gerade oder winklige Verbindung der Beinketten

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Universität Hannover

function plot(Rob, q, x, s)

s_std = struct( ...
             'mode', 1, ... % Strichmodell
             'straight', 1, ...
             'nojoints', false, ...
             'ks_platform', Rob.NLEG+2, ... % EE-KS
             'ks_legs', Rob.I2L_LEG); % nur EE-KS jedes Beins
if nargin < 4
  % Keine Einstellungen übergeben. Standard-Einstellungen
  s = s_std;
end

% Prüfe Felder der Einstellungs-Struktur
for f2 = fields(s_std)'
  f = f2{1};
  if ~isfield(s, f),s.(f) = s_std.(f); end
end

% Einstellungs-Struktur für serielle Beinketten
s_ser = struct('ks', [], 'straight', s.straight, 'jointcolors', 'parallel', ...
  'mode', s.mode, 'nojoints', s.nojoints);
T_W_0 = Rob.T_W_0;
%% Koppelpunkte berechnen
r_A1_ges = NaN(Rob.NLEG, 3);
r_B1_ges = NaN(Rob.NLEG, 3);
for iLeg = 1:Rob.NLEG
  IJ_i = Rob.I1J_LEG(iLeg):Rob.I2J_LEG(iLeg);
  qs = q(IJ_i); % Gelenkwinkel dieser Kette
  [~,Tci_0] = Rob.Leg(iLeg).fkine(qs);
  r_A1_ges(iLeg,:) = eye(3,4)*T_W_0*[Rob.Leg(iLeg).r_W_0;1];
%   T_W_Ai = transl(Rob.Leg(iLeg).r_W_0) * ...
%           r2t( eul2r(Rob.Leg(iLeg).phi_W_0, Rob.Leg(iLeg).phiconv_W_0) );
  T_W_Bi = T_W_0*Tci_0(:,:,Rob.Leg(iLeg).I_EElink+1)*Rob.Leg(iLeg).T_N_E;
  r_B1_ges(iLeg,:) = T_W_Bi(1:3,4);
end

r_A1_ges = [r_A1_ges; r_A1_ges(1,:)]; % damit Viel-Eck geschlossen wird
r_B1_ges = [r_B1_ges; r_B1_ges(1,:)];

%% Basis
% Basis-Koppelpunkte als Viel-Eck plotten
hdl = plot3(r_A1_ges(:,1), r_A1_ges(:,2), r_A1_ges(:,3), 'k-');
set(hdl, 'DisplayName', 'Base');

% Basis-KS
trplot(Rob.T_W_0, 'frame', '0', 'rgb', 'length', 0.4)
%% Beinketten
% Alle seriellen Beinketten plotten (ohne KS)
for iLeg = 1:Rob.NLEG
  IJ_i = Rob.I1J_LEG(iLeg):Rob.I2J_LEG(iLeg);
  qs = q(IJ_i); % Gelenkwinkel dieser Kette
  r_A1_ges(iLeg,:) = Rob.Leg(iLeg).r_W_0;
  
  % Basis-KS der Beinkette temporär modifizieren, damit die
  % Basis-Transformation der PKM hier berücksichtigt werden kann.
  s_ser.T_W_0 = T_W_0 * Rob.Leg(iLeg).T_W_0;
  Rob.Leg(iLeg).plot(qs, s_ser);
  % Umbenennung der gezeichneten 3D-Körper (Ziel: "Leg1_Link2").
  for c = get(gca, 'children')'
    [~,tokens] = regexp(get(c, 'DisplayName'), '^Link_([\d]+)$', 'tokens', 'match');
    if ~isempty(tokens)
      set(c, 'DisplayName', sprintf('Leg_%d_%s', iLeg, tokens{1}));
    end
    [~,tokens] = regexp(get(c, 'DisplayName'), '^Joint_([\d]+)$', 'tokens', 'match');
    if ~isempty(tokens)
      set(c, 'DisplayName', sprintf('Leg_%d_%s', iLeg, tokens{1}));
    end
  end
end

% Koordinatensysteme der Beine
[~,TcLges_W] = Rob.fkine_legs(q);
k = 0;
for iLeg = 1:Rob.NLEG
  NLL = Rob.Leg(iLeg).NL;
  for j = 1:NLL+1
    k = k + 1;
    if ~any(s.ks_legs == k)
      continue
    end
    if j-1 == 0
      % Basis-KS der Beinkette
      name = sprintf('A%d', iLeg);
    elseif j == NLL+1
      % Virtuelles EE-KS der Beinkette
      name = sprintf('C%d', iLeg);
    else
      % Sonstige Körper-KS der Beinkette
      name = sprintf('%d,%d', iLeg, j-1);
    end
    trplot(TcLges_W(:,:,k), 'frame', name, 'rgb', 'length', 0.20)
  end
end

%% Kollisionskörper (der PKM) zeichnen
% Siehe dazu auch check_collisionset_simplegeom.m
% Kollisionskörper der einzelnen Beinketten werden bereits in SerRob/plot
% gezeichnet.
if any(s.mode == 5) || any(s.mode == 6)
  % Wandle das Datenformat von fkine_legs in das von invkin um. Das
  % virtuelle EE-KS der Beinketten wird weggelassen. Zusätzlich Basis-KS.
  Tc_PKM = NaN(4,4,Rob.NL-1+Rob.NLEG);
  Tc_PKM(:,:,1) = Rob.T_W_0; % PKM-Basis
  for kk = 1:Rob.NLEG
    Tc_PKM(:,:,1+kk+(-1+Rob.I1J_LEG(kk):Rob.I2J_LEG(kk))) = ...
      TcLges_W(:,:,kk*2+(-2+Rob.I1J_LEG(kk):-1+Rob.I2J_LEG(kk)));
  end
  for cbtype = 1:2
    if cbtype == 1
      collbodies_nonleg = Rob.collbodies_nonleg;
      color = 'b';
      if ~any(s.mode == 5)
        % Keine Kollisionskörper zeichnen, sondern nur Bauraumobjekt
        continue
      end
    else
      collbodies_nonleg = Rob.collbodies_instspc_nonleg;
      color = 'g';
      if ~any(s.mode == 6)
        % Keine Bauraumkörper zeichnen, sondern nur Kollisionskörper
        continue
      end
    end
    % Zeichne Kollisionskörper
    for j = 1:size(collbodies_nonleg.type,1)
      % Nummern der beteiligten Körper (0=PKM-Basis, 1=Beinkette1-Basis,...)
      i1 = collbodies_nonleg.link(j,1);
      i2 = collbodies_nonleg.link(j,2);
      % Transformationsmatrix der Körper
      T_body_i1 = Tc_PKM(:,:,i1+1);
      T_body_i2 = Tc_PKM(:,:,i2+1);
      if collbodies_nonleg.type(j) == 6 % Kapsel zum vorherigen
        r = collbodies_nonleg.params(j,1); % Radius ist einziger Parameter.
        pts_W = [T_body_i1(1:3,4)', T_body_i2(1:3,4)']; % Punkte bereits durch Körper-Nummern und Kinematik definiert.
        drawCapsule([pts_W, r], 'FaceColor', color, 'FaceAlpha', 0.2);
      elseif collbodies_nonleg.type(j) == 4 || ...  % Kugel mit Angabe des Mittelpunkts
          collbodies_nonleg.type(j) == 15 % explizit im Basis-KS
        r = collbodies_nonleg.params(j,4);
        pt_i = collbodies_nonleg.params(j,1:3);
        pt_W = (eye(3,4)*T_body_i1*[pt_i(1:3)';1])'; ... % Punkt
        drawSphere([pt_W, r], 'FaceColor', color, 'FaceAlpha', 0.2);
      elseif collbodies_nonleg.type(j) == 12 % Zylinder im Basis-KS
        r = collbodies_nonleg.params(j,7);
        pt1_i = collbodies_nonleg.params(j,1:3);
        pt2_i = collbodies_nonleg.params(j,4:6);
        pt1_W = (eye(3,4)*T_body_i1*[pt1_i(1:3)';1])';
        pt2_W = (eye(3,4)*T_body_i1*[pt2_i(1:3)';1])';
        drawCylinder([pt1_W,pt2_W, r], 'FaceColor', color, 'FaceAlpha', 0.2);
      elseif collbodies_nonleg.type(j) == 10 % Quader im Basis-KS
        q_W = eye(3,4)*T_body_i1*[collbodies_nonleg.params(j,1:3)';1];
        u1_W = T_body_i1(1:3,1:3)*collbodies_nonleg.params(j,4:6)';
        u2_W = T_body_i1(1:3,1:3)*collbodies_nonleg.params(j,7:9)';
        % letzte Kante per Definition senkrecht auf anderen beiden.
        u3_W = cross(u1_W,u2_W); u3_W = u3_W/norm(u3_W)*collbodies_nonleg.params(j,10);
        % Umrechnen in Format der plot-Funktion
        cubpar_c = q_W(:)+(u1_W(:)+u2_W(:)+u3_W(:))/2; % Mittelpunkt des Quaders
        cubpar_l = [norm(u1_W); norm(u2_W); norm(u3_W)]; % Dimension des Quaders
        cubpar_a = 180/pi*tr2rpy([u1_W(:)/norm(u1_W), u2_W(:)/norm(u2_W), u3_W(:)/norm(u3_W)],'zyx')'; % Orientierung des Quaders
        drawCuboid([cubpar_c', cubpar_l', cubpar_a'], ...
          'FaceColor', color, 'FaceAlpha', 0.1);
      else
        error('Fall %d nicht definiert', collbodies_nonleg.type(j));
      end
    end
  end
end
%% Plattform Plotten (von Plattform-Koppelpunkten her)
% Anpassung der Ist-Plattform-Pose für den Fall der Aufgabenredundanz
if all(Rob.I_EE == [1 1 1 1 1 0]) || Rob.I_EE_Task(6) ~= Rob.I_EE(6) % 3T2R, aber auch 3T1R/2T1R mit Redundanz
  % Transformation von Plattform zum Plattform-Koppelpunkt für die letzte
  % Beinkette (zur Nutzung von Variable von vorher)
  T_P_Bi = [eul2r(Rob.phi_P_B_all(:,end),Rob.phiconv_P_E(end)), Rob.r_P_B_all(:,end);[0 0 0 1]]; 
  % Von Welt-KS zum EE aus direkter Kinematik der letzten Beinkette.
  % Umrechnung auf Basis-KS (da xE immer darin ausgedrückt wird)
  T_0_E_lastleg = invtr(T_W_0)*T_W_Bi*invtr(T_P_Bi)*Rob.T_P_E;
  x_lastleg = Rob.t2x(T_0_E_lastleg); % Umrechnung in Minimalkoordinaten
  % Setze freien Rotations-FG so, dass es dem Ist-Zustand entspricht.
  % Dadurch wird vermieden, dass Soll- und Ist-Plattform-Pose gegeneinander
  % verdreht sind.
  x(6) = x_lastleg(6);
  % Rob.fkineEE_traj(q',[],[],5)
end
[~,Tc_Pges_W] = Rob.fkine_platform(x);

% Plattform als Objekt
if any(s.mode == 1)
  r_B2_ges = squeeze(Tc_Pges_W(1:3,4,1:Rob.NLEG))';
  r_B2_ges = [r_B2_ges; r_B2_ges(1,:)]; % damit Viel-Eck geschlossen wird
  plot3(r_B2_ges(:,1), r_B2_ges(:,2), r_B2_ges(:,3), ...
    'color', 'm','LineWidth',3);
end
if any(s.mode == 3)
  % Trägheitsellipse. Nur plotten, wenn Dynamikparameter gegeben.
  if ~any(isnan([Rob.DynPar.Icges(end,:), Rob.DynPar.mges(end), Rob.DynPar.rSges(end,:)]))
    inertia_ellipsoid( ...
      inertiavector2matrix(Rob.DynPar.Icges(end,:)), ...
      Rob.DynPar.mges(end)/2700, ... % Dichte von Aluminium zur Skalierung der Größe
      Rob.DynPar.rSges(end,:), ...
      Tc_Pges_W(:,:,end-1));
  end
end
hdl_plf = [];
if any(s.mode == 4)
  if any(Rob.DesPar.platform_method == [1 2 3 8])
    h = Rob.DesPar.platform_par(end); % Dicke der Kreisscheibe
    if h > 0
      rh_W_P1o = Tc_Pges_W(:,:,end-1)*[0;0;+h/2;1]; % Nutze Trafo zum Plattform-KS
      rh_W_P1u = Tc_Pges_W(:,:,end-1)*[0;0;-h/2;1];
      % Plattform als Kreisscheibe modellieren
      hdl_plf = drawCylinder([rh_W_P1u(1:3)', rh_W_P1o(1:3)', Rob.DesPar.platform_par(1)], ...
        'FaceColor', [0.7 0.7 0.7], 'edgeColor', 'k', 'FaceAlpha', 0.5, 'EdgeAlpha', 0.3);
    end
  elseif any(Rob.DesPar.platform_method == [4 5 6])
    h = Rob.DesPar.platform_par(end); % Dicke der Polygon-Platte
    % Polygon-Punkte zusammenstellen und zeichnen
    if h > 0
      hdl_plf = NaN(2,1);
      for jj = 1:2
        pts = squeeze(Tc_Pges_W(1:4,4,end-Rob.NLEG-1:end-2));
        pts2 = pts + Tc_Pges_W(:,:,end-1)*[0;0;+(-1)^jj*h/2;0];
        hdl_plf(jj) = fillPolygon3d(pts2', 'm');
        set(hdl_plf(jj), 'FaceColor', [0.7 0.7 0.7], 'edgeColor', 'k', 'FaceAlpha', 0.5, 'EdgeAlpha', 0.3)
      end
    end
  else
    error('Methode für platform_method nicht implementiert');
  end
end
for i = 1:length(hdl_plf)
  set(hdl_plf(i), 'DisplayName', 'Platform');
end
% Plattform-bezogene KS
for i = 1:size(Tc_Pges_W,3)
  if ~any(s.ks_platform == i)
    continue
  end
  if i == size(Tc_Pges_W,3)
    name = 'E';
  elseif i == size(Tc_Pges_W,3)-1
    name = 'P';
  else
    name = sprintf('B%d', i);
  end
  trplot(Tc_Pges_W(1:4,1:4,i), 'frame', name, 'rgb', 'length', 0.4)
end

axis auto


%% Plattform Plotten (von Bein-Endpunkten her; als Viel-Eck)
if any(s.mode == 1)
  plot3(r_B1_ges(:,1), r_B1_ges(:,2), r_B1_ges(:,3), '--', ...
    'color', 'c','LineWidth',2);
end
