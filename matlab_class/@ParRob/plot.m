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
             'ks_platform', Rob.NLEG+2, ... % EE-KS
             'ks_legs', Rob.I2L_LEG); % nur Basis- und EE-KS jedes Beins
if nargin < 3
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
  'mode', s.mode);
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
plot3(r_A1_ges(:,1), r_A1_ges(:,2), r_A1_ges(:,3), 'k-');

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
  T_0_0i_old = Rob.Leg(iLeg).T_W_0;
  Rob.Leg(iLeg).T_W_0 = T_W_0 * T_0_0i_old;
  Rob.Leg(iLeg).plot(qs, s_ser);
  Rob.Leg(iLeg).T_W_0 = T_0_0i_old;
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
    name = sprintf('%d,%d', iLeg, j-1);
    trplot(TcLges_W(:,:,k), 'frame', name, 'rgb', 'length', 0.20)
  end
end


%% Plattform Plotten (von Plattform-Koppelpunkten her)
% Anpassung der Ist-Plattform-Pose für den Fall der Aufgabenredundanz
if all(Rob.I_EE_Task==[1 1 1 1 1 0])
  % Transformation von Plattform zum Plattform-Koppelpunkt für die letzte
  % Beinkette (zur Nutzung von Variable von vorher)
  T_P_Bi = [eul2r(Rob.phi_P_B_all(:,end),Rob.phiconv_P_E(end)), Rob.r_P_B_all(:,end);[0 0 0 1]]; 
  % Von Welt-KS zum EE aus direkter Kinematik der letzten Beinkette
  T_W_E_lastleg = T_W_Bi*invtr(T_P_Bi)*Rob.T_P_E;
  x_lastleg = Rob.t2x(T_W_E_lastleg); % Umrechnung in Minimalkoordinaten
  % Setze freien Rotations-FG so, dass es dem Ist-Zustand entspricht.
  % Dadurch wird vermieden, dass Soll- und Ist-Plattform-Pose gegeneinander
  % verdreht sind.
  x(6) = x_lastleg(6);
  % Rob.fkineEE_traj(q',[],[],5)
end
[~,Tc_Pges_W] = Rob.fkine_platform(x);

% Plattform als Objekt
if s.mode == 1
  r_B2_ges = squeeze(Tc_Pges_W(1:3,4,1:Rob.NLEG))';
  r_B2_ges = [r_B2_ges; r_B2_ges(1,:)]; % damit Viel-Eck geschlossen wird
  plot3(r_B2_ges(:,1), r_B2_ges(:,2), r_B2_ges(:,3), ...
    'color', 'm','LineWidth',3);
end
if s.mode == 3
  % Trägheitsellipse. Nur plotten, wenn Dynamikparameter gegeben.
  if ~any(isnan([Rob.DynPar.Icges(end,:), Rob.DynPar.mges(end), Rob.DynPar.rSges(end,:)]))
    inertia_ellipsoid( ...
      inertiavector2matrix(Rob.DynPar.Icges(end,:)), ...
      Rob.DynPar.mges(end)/2700, ... % Dichte von Aluminium zur Skalierung der Größe
      Rob.DynPar.rSges(end,:), ...
      Tc_Pges_W(:,:,end-1));
  end
end
if s.mode == 4
  if any(Rob.DesPar.platform_method == [1 2 3 8])
    h = Rob.DesPar.platform_par(2); % Dicke der Kreisscheibe
    if h > 0
      rh_W_P1o = Tc_Pges_W(:,:,end-1)*[0;0;+h/2;1]; % Nutze Trafo zum Plattform-KS
      rh_W_P1u = Tc_Pges_W(:,:,end-1)*[0;0;-h/2;1];
      % Plattform als Kreisscheibe modellieren
      drawCylinder([rh_W_P1u(1:3)', rh_W_P1o(1:3)', Rob.DesPar.platform_par(1)], ...
        'FaceColor', [0.7 0.7 0.7], 'edgeColor', 'k', 'FaceAlpha', 0.5, 'EdgeAlpha', 0.3)
    end
  elseif any(Rob.DesPar.platform_method == [4 5 6])
    h = Rob.DesPar.platform_par(3); % Dicke der Polygon-Platte
    % Polygon-Punkte zusammenstellen und zeichnen
    if h > 0
      for jj = 1:2
        pts = squeeze(Tc_Pges_W(1:4,4,end-Rob.NLEG-1:end-2));
        pts2 = pts + Tc_Pges_W(:,:,end-1)*[0;0;+(-1)^jj*h/2;0];
        hdl=fillPolygon3d(pts2', 'm');
        set(hdl, 'FaceColor', [0.7 0.7 0.7], 'edgeColor', 'k', 'FaceAlpha', 0.5, 'EdgeAlpha', 0.3)
      end
    end
  else
    error('Methode für platform_method nicht implementiert');
  end
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
if s.mode == 1
  plot3(r_B1_ges(:,1), r_B1_ges(:,2), r_B1_ges(:,3), '--', ...
    'color', 'c','LineWidth',2);
end
