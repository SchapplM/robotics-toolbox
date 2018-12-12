% Zeichne den parallelen Roboter in einer gegebenen Pose
% 
% Eingabe:
% q
%   Gelenk-Winkel (bzw. verallgemeinerte Koordinaten) des Roboters
% x
%   Endeffektor-Pose (entsprechend Roboterdefinition)
% s
%   Struktur mit Plot-Einstellungen. Felder, siehe Quelltext

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
s_ser = struct('ks', []);
s_ser.straight = s.straight;
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
  T_W_Bi = T_W_0*Tci_0(:,:,end);
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
[~,Tc_Pges_W] = Rob.fkine_platform(x);

% Plattform als Objekt
r_B2_ges = squeeze(Tc_Pges_W(1:3,4,1:Rob.NLEG))';
r_B2_ges = [r_B2_ges; r_B2_ges(1,:)]; % damit Viel-Eck geschlossen wird
plot3(r_B2_ges(:,1), r_B2_ges(:,2), r_B2_ges(:,3), ...
  'color', 'm','LineWidth',3);

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
plot3(r_B1_ges(:,1), r_B1_ges(:,2), r_B1_ges(:,3), '--', ...
  'color', 'c','LineWidth',2);