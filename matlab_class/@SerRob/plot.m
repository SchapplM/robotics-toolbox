% Zeichne den Roboter in einer gegebenen Pose
% 
% Eingabe:
% qJ
%   Gelenk-Winkel (bzw. verallgemeinerte Koordinaten) des Roboters
% s
%   Struktur mit Plot-Einstellungen. Felder, siehe Quelltext
%   ks
%     Einträge für alle KS entsprechend der Liste der
%     Gelenk-Transformationen. NJ+2: End-Effektor
% 
% Vorbereitung:
% Vor Starten der Animation muss ein Figure geöffnet werden und vorbereitet
% werden mit passenden Formateinstellungen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Universität Hannover

function plot(Rob, qJ, s)

%% Initialisierung
s_std = struct( ...
             'mode', 1, ... % Strichmodell
             'straight', 1, ... % Linien gerade setzen oder entlang der MDH-Parameter
             'ks', [1, Rob.NJ+2]); % nur Basis- und EE-KS
if nargin < 3
  % Keine Einstellungen übergeben. Standard-Einstellungen
  s = s_std;
end



% Prüfe Felder der Einstellungs-Struktur
if ~isfield(s, 'mode'),s.mode = s_std.mode; end
if ~isfield(s, 'ks'),s.ks = s_std.ks; end
if ~isfield(s, 'straight'),s.straight = s_std.straight; end

[~,T_c_W] = Rob.fkine(qJ);

v = Rob.MDH.v;

%% Gelenke als Punkte zeichnen
O_xyz_ges = squeeze(T_c_W(1:3,4,:));
plot3(O_xyz_ges(1,:)', O_xyz_ges(2,:)', O_xyz_ges(3,:)', 'ro', 'MarkerSize', 8);

%% Gelenke zeichnen
% Gelenke als Objekte. TODO: An Größe des Roboters anpassen
d = 0.04; % Durchmesser der Zylinder und Quader
h = 0.15; % Höhe
% Anmerkungen:
% * Gelenk i bewegt Körper i
% * Gelenkachse i ist z-Achse des Körper-KS i (MDH-Notation)
% * Gelenk "sitzt" im Körper-KS i (MDH-Notation)
for i = 1:Rob.NJ
  if Rob.MDH.mu(i) == 1, cc = 'r';
  else,                  cc = 'b'; end

  R_W_i = T_c_W(1:3,1:3,i+1); % um eins verschoben (Transformations-Index 1 ist Basis)
  r_W_Oi = T_c_W(1:3,4,i+1);
  if Rob.MDH.sigma(i) == 0 % Drehgelenk
    r_W_P1 = r_W_Oi + R_W_i*[0;0;-h/2];
    r_W_P2 = r_W_Oi + R_W_i*[0;0; h/2];
    drawCylinder([r_W_P1', r_W_P2', d/2], 'EdgeColor', cc, ...
      'FaceAlpha', 0.3, 'FaceColor', 'w')
  elseif Rob.MDH.sigma(i) == 1 % Schubgelenk
    % Eckpunkte des Quaders definieren
    r_W_Q1 = r_W_Oi + R_W_i*[-d/2; -d/2; -h/2];
    r_W_Q1Q2 = R_W_i*[d;0;0];
    r_W_Q1Q3 = R_W_i*[0;d;0];
    r_W_Q1Q4 = R_W_i*[0;0;h];
    plot_cube2(r_W_Q1, r_W_Q1Q2, r_W_Q1Q3, r_W_Q1Q4, cc);
  else
    continue % kein Gelenk
  end
end
%% Segmente, also Verbindungen der einzelnen Gelenke zeichnen
if s.straight % if Rob.NQJ ~= Rob.NJ
  % hybride Roboter: Die Koordinatentransformationen sind eventuell nicht
  % einzeln bestimmbar. TODO: Prüfung für hybride Roboter
  for i = 1:Rob.NJ
    j = v(i)+1; % Körper-Index (Vorgänger)
    T1 = T_c_W(:,:,j);
    T2 = T_c_W(:,:,i+1);
    plot3([T1(1,4),T2(1,4)],[T1(2,4),T2(2,4)],[T1(3,4),T2(3,4)], ...
      'LineWidth',4,'Color','k')
  end
else
  q = Rob.jointvar(qJ);
  for i = 1:Rob.NJ
    j = v(i)+1;
    % MDH-Transformation in einzelnen Schritten nachrechnen, damit diese
    % geplottet werden können.
    T_k = NaN(4,4,3);
    T_k(:,:,1) = T_c_W(:,:,j);
    T_k(:,:,2) = T_k(:,:,1) * trotz(Rob.MDH.beta(i)) * transl([0;0;Rob.MDH.b(i)]);
    T_k(:,:,3) = T_k(:,:,2) * trotx(Rob.MDH.alpha(i)) * transl([Rob.MDH.a(i);0;0]);
    if Rob.MDH.sigma(i) == 0 % Drehgelenk
      theta = q(i) + Rob.MDH.offset(i);
      d = Rob.MDH.d(i);
    elseif Rob.MDH.sigma(i) == 1 % Schubgelenk
      theta = Rob.MDH.theta(i);
      d = q(i) + Rob.MDH.offset(i);
    else % statische Transformation (z.B. zu Schnitt-Koordinatensystemen)
      theta = Rob.MDH.theta(i);
      d = Rob.MDH.d(i);
    end
    T_k(:,:,4) = T_k(:,:,3) * trotz(theta) * transl([0;0;d]);
    
    for k = 1:3
      T1 = T_k(:,:,k);
      T2 = T_k(:,:,k+1);
      if abs(T1(1:3,4)-T2(1:3,4)) < 1e-10
        continue % nicht zeichnen, wenn keine Veränderung
      end
      plot3([T1(1,4),T2(1,4)],[T1(2,4),T2(2,4)],[T1(3,4),T2(3,4)], ...
        'LineWidth',4,'Color','k')
    end
  end
end

%% Verbindung zum Endeffektor
T_W_N = T_c_W(:,:,Rob.I_EElink+1);
[~,T_W_E] = Rob.fkineEE(qJ);
plot3([T_W_N(1,4),T_W_E(1,4)],[T_W_N(2,4),T_W_E(2,4)],[T_W_N(3,4),T_W_E(3,4)], ...
  'LineWidth',4,'Color','k')

%% Koordinatensysteme
for i = 1:size(T_c_W,3)
  if ~any(s.ks == i)
    continue
  end
  trplot(T_c_W(:,:,i), 'frame', sprintf('%d',i-1), 'rgb', 'length', 0.20)
end
if any(s.ks == Rob.NJ+2)
  trplot(T_W_E, 'frame', 'E', 'rgb', 'length', 0.20)
end

%% Formatierung
axis equal