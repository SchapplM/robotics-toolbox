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
%   mode
%     1: Strichmodell mit stilisierten Gelenken (Zylinder, Quader)
%     2: CAD-Modell des Roboters aus hinterlegten STL-Dateien der Körper
%     3: Trägheitsellipsen (basierend auf Masse und Trägheitstensor)
%     4: Darstellung der Körper mit Entwurfsparametern (z.B. Zylinder)
%   straight (nur aktiv, wenn `mode` auf 1 (Strichmodell)
%     1: direkte Verbindung zwischen den Gelenken
%     0: winklige Verbindung zwischen Gelenken entsprechend der
%        DH-Parameter a und d (Verschiebung in x- und z-Richtung)
%   jointcolors
%     'serial': Klasse entspricht einzelnem seriellem/hybridem Roboter
%     'parallel': Klassen entspricht Beinkette einer PKM
%   bodies
%     Enthält eine Liste der Körper, für die die CAD-Modelle gezeichnet
%     werden sollen (Standard: alle). Konvention: Basis = 0.
% 
% Ausgabe:
% hdl
%   Handles zu dem gezeichneten CAD-Modell des Roboters (falls Option
%   gewählt
% 
% Vorbereitung:
% Vor Starten der Animation muss ein Figure geöffnet werden und vorbereitet
% werden mit passenden Formateinstellungen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Universität Hannover

function hdl = plot(Rob, qJ, s)
hdl = [];
%% Initialisierung
s_std = struct( ...
             'mode', 1, ... % Strichmodell
             'straight', 1, ... % Linien gerade setzen oder entlang der MDH-Parameter
             'ks', [1, Rob.NJ+2], ... % nur Basis- und EE-KS
             'bodies', 0:Rob.NL, ... % CAD-Modelle für alle Körper
             'only_bodies', false, ... % Nur Körper zeichnen (STL/Ellipsoid); keine Gelenke/Sonstiges
             'jointcolors', 'serial'); 
if nargin < 3
  % Keine Einstellungen übergeben. Standard-Einstellungen
  s = s_std;
end



% Prüfe Felder der Einstellungs-Struktur und setze Standard-Werte, falls
% Eingabe nicht gesetzt
for f = fields(s_std)'
  if ~isfield(s, f{1})
    s.(f{1}) = s_std.(f{1});
  end
end

[~,T_c_W] = Rob.fkine(qJ);

v = Rob.MDH.v;

%% Gelenke als Punkte zeichnen
% O_xyz_ges = squeeze(T_c_W(1:3,4,:));
% plot3(O_xyz_ges(1,:)', O_xyz_ges(2,:)', O_xyz_ges(3,:)', 'ro', 'MarkerSize', 8);

%% Gelenke zeichnen (als Zylinder oder Quader)
if ~s.only_bodies && any(s.mode == [1 3 4])
  % Verschiedene Gelenkfarben für serielle/hybride Roboter und PKM
  % Ursache: Für PKM wird mu=2 für aktive Gelenke gesetzt. Bei
  % seriell-hybriden Ketten ist noch entscheidend, ob ein Gelenk abhängig
  % in der seriell-hybriden Kette ist (mu=0) oder Minimalkoordinate (mu=1)
  if     strcmp(s.jointcolors, 'serial')
    colors = {'b', 'r', 'k'};
  elseif strcmp(s.jointcolors, 'parallel')
    colors = {'c', 'b', 'r'};
  else
    error('nicht definiert');
  end
  % Gelenke als Objekte. TODO: An Größe des Roboters anpassen
  gd = 0.04; % Durchmesser der Zylinder und Quader
  gh = 0.15; % Höhe
  
  % Anmerkungen:
  % * Gelenk i bewegt Körper i
  % * Gelenkachse i ist z-Achse des Körper-KS i (MDH-Notation)
  % * Gelenk "sitzt" im Körper-KS i (MDH-Notation)
  for i = 1:Rob.NJ
    % mu: 2=PKM-aktiv, 1=seriell-aktiv, 0=seriell-passiv
    cc = colors{Rob.MDH.mu(i)+1};

    if s.mode == 4
      % Passe zu zeichnendes Gelenk von der Größe her an
      if Rob.DesPar.seg_type(i+1) == 1
        % Das Gelenk sollte etwas größer sein als das skizzierte Segment
        gh = max(Rob.DesPar.seg_par(i+1,2)*2, gh);
        gd = gh/4; % Gleiche Proportion des Gelenks
      else
        error('Noch nicht definiert');
      end
    end
    
    R_W_i = T_c_W(1:3,1:3,i+1); % um eins verschoben (Transformations-Index 1 ist Basis)
    r_W_Oi = T_c_W(1:3,4,i+1);
    % Gelenk so verschieben, dass es bei d-Verschiebung (z-Achse) in der
    % Mitte zwischen den benachbarten KS liegt (damit die Gelenke weniger
    % direkt ineinander liegen)
    if Rob.MDH.sigma(i) == 0 % Drehgelenk
      d = Rob.MDH.d(i);
    elseif Rob.MDH.sigma(i) == 1 % Schubgelenk
      d = qJ(i) + Rob.MDH.offset(i);
    else % statische Transformation (z.B. zu Schnitt-Koordinatensystemen)
      d = Rob.MDH.d(i);
    end
    r_W_Oimd = r_W_Oi - T_c_W(1:3,1:3,i+1)*[0;0;d];

    if Rob.MDH.sigma(i) == 0 % Drehgelenk
      if ~s.straight
        % Gelenke werden so auf der z-Achse verschoben, dass sie in der
        % Mitte zwischen den beiden KS sind (ist dann übersichtlicher)
        r_W_Gi = (r_W_Oi+r_W_Oimd)/2;
      else
        % Gelenk direkt im Gelenk-KS zeichnen. Bei schräger Verbindung
        % steht das Gelenk sonst in der Luft.
        r_W_Gi = r_W_Oi;
      end
      r_W_P1 = r_W_Gi + R_W_i*[0;0;-gh/2];
      r_W_P2 = r_W_Gi + R_W_i*[0;0; gh/2];
      drawCylinder([r_W_P1', r_W_P2', gd/2], 'EdgeColor', cc, ...
        'FaceAlpha', 0.3, 'FaceColor', 'w')
    elseif Rob.MDH.sigma(i) == 1 % Schubgelenk
      if i<Rob.NJ && Rob.MDH.a(i+1) ~= 0
        % Schubgelenke werden nicht auf der Achse verschoben, wenn es einen
        % Abstand a gibt. Dann sieht es so aus, als ob das folgende Segment
        % auf dem Quader befestigt ist.
        r_W_Gi = r_W_Oi;
      elseif s.straight
        % Bei schräger Verbindung muss das Gelenk direkt im Gelenk-KS
        % gezeichnet werden
        r_W_Gi = r_W_Oi;
      else
        % Bei der normalen Kinematik-Skizze wird das Schubgelenk auch in
        % die Mitte zwischen die KS gesetzt
        r_W_Gi = (r_W_Oi+r_W_Oimd)/2;
      end
      cubpar_c = r_W_Gi; % Mittelpunkt des Quaders
      cubpar_l = [gd; gd; gh]; % Dimension des Quaders
      cubpar_a = 180/pi*r2eulzyx(R_W_i); % Orientierung des Quaders
      drawCuboid([cubpar_c', cubpar_l', cubpar_a'], 'FaceColor', cc, 'FaceAlpha', 0.3);
    else
      continue % kein Gelenk
    end
  end
end
%% Trägheitsellipsen zeichnen
if s.mode == 3
  for i = 1:Rob.NL
    if any(isnan([Rob.DynPar.Icges(i,:), Rob.DynPar.mges(i), Rob.DynPar.rSges(i,:)]))
      continue % Nur plotten, wenn Dynamikparameter gegeben sind.
    end
    if ~any(s.bodies == i-1)
      continue % Dieser Körper soll nicht gezeichnet werden
    end
    if any(eig(inertiavector2matrix(Rob.DynPar.Icges(i,:))) < 0)
      continue
    end
    inertia_ellipsoid( ...
      inertiavector2matrix(Rob.DynPar.Icges(i,:)), ...
      Rob.DynPar.mges(i)/2700, ... % Dichte von Aluminium zur Skalierung der Größe
      Rob.DynPar.rSges(i,:), ...
      T_c_W(:,:,i));
  end
end
%% Segmente, also Verbindungen der einzelnen Gelenke zeichnen
if ~s.only_bodies && any(s.mode == [3 4]) || s.mode == 1 && s.straight
    % hybride Roboter: Die Koordinatentransformationen sind eventuell nicht
    % einzeln bestimmbar. TODO: Prüfung für hybride Roboter
    for i = 1:Rob.NJ
      j = v(i)+1; % Körper-Index (Vorgänger)
      T1 = T_c_W(:,:,j);
      T2 = T_c_W(:,:,i+1);
      if s.mode == 1 && s.straight
        % Als Linie zeichnen
        plot3([T1(1,4),T2(1,4)],[T1(2,4),T2(2,4)],[T1(3,4),T2(3,4)], ...
          'LineWidth',4,'Color','k')
      elseif s.mode == 3 && s.straight
        plot3([T1(1,4),T2(1,4)],[T1(2,4),T2(2,4)],[T1(3,4),T2(3,4)], ...
          'LineWidth',1,'Color','k')
      elseif s.mode == 4
        if Rob.MDH.sigma(i) == 0 % Drehgelenk
          % Als Zylinder entsprechend der Entwurfsparameter zeichnen
          % Bei Drehgelenken wird das aktuelle Gelenk direkt mit dem
          % vorherigen verbunden
          if Rob.DesPar.seg_type(i) == 1
            drawCylinder([T1(1:3,4)', T2(1:3,4)', Rob.DesPar.seg_par(i,2)/2], ...
              'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[0 1 0], ...
              'EdgeAlpha', 0.1); 
          else
            error('Segmenttyp ist nicht definiert');
          end
        else % Schubgelenk
          % Bei Schubgelenken wird die Konstruktionsart des Schubgelenkes
          % berücksichtigt.
          % Transformation zu Anfangs- und Endlage des Schubgelenkes
          T_qmin = T2 * transl([0;0;-qJ(i)+Rob.qlim(i,1)]); % Lage von T2, wenn q=qmin wäre
          T_qmax = T2 * transl([0;0;-qJ(i)+Rob.qlim(i,2)]); % Lage von T2, wenn q=qmax wäre

          if Rob.DesPar.joint_type(i) ==  4 % Schubgelenk ist Linearführung
            % Zeichne Linearführung als Quader
            cubpar_c = (T_qmin(1:3,4)+T_qmax(1:3,4))/2; % Mittelpunkt des Quaders
            cubpar_l = [Rob.DesPar.seg_par(i,2)*0.25*[1;1];Rob.qlim(i,2)-Rob.qlim(i,1)]; % Dimension des Quaders
            cubpar_a = 180/pi*r2eulzyx(T_qmin(1:3,1:3)); % Orientierung des Quaders
            drawCuboid([cubpar_c', cubpar_l', cubpar_a'], 'FaceColor', 'b', 'FaceAlpha', 0.3);
            % Normales Segment zum Start der Linearführung
            if Rob.DesPar.seg_type(i) == 1
              if Rob.qlim(i,2)*Rob.qlim(i,2)< 0 % Führung liegt auf der Höhe
                % Als senkrechte Verbindung unter Benutzung des a-Parameters der DH-Notation
                drawCylinder([T1(1:3,4)', (eye(3,4)*T1*[Rob.MDH.a(i);0;0;1])', Rob.DesPar.seg_par(i,2)/2], ...
                  'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[0 1 0], ...
                  'EdgeAlpha', 0.1);
              else
                % Keine senkrechte Verbindung (Schiene fängt woanders an)
                if Rob.qlim(i,2) < 0 % Schiene liegt komplett "links" vom KS. Gehe zum Endpunkt
                  drawCylinder([T1(1:3,4)', T_qmax(1:3,4)', Rob.DesPar.seg_par(i,2)/2], ...
                    'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[0 1 0], ...
                    'EdgeAlpha', 0.1);
                else % Schiene liegt "rechts" vom KS. Gehe zum Startpunkt
                  drawCylinder([T1(1:3,4)', T_qmin(1:3,4)', Rob.DesPar.seg_par(i,2)/2], ...
                    'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[0 1 0], ...
                    'EdgeAlpha', 0.1);
                end
              end
            end
          else % Schubgelenk ist Hubzylinder
            % Großer Zylinder, muss so lang sein wie maximaler Hub.
            T_grozylstart = T_qmin * transl([0;0;-(Rob.qlim(i,2)-Rob.qlim(i,1))]);
            drawCylinder([T_grozylstart(1:3,4)', T_qmin(1:3,4)', Rob.DesPar.seg_par(i,2)/2], ...
              'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[1 1 0], ...
              'EdgeAlpha', 0.1);
            % Kleinerer Zylinder, der herauskommt (aber so lang ist, wie
            % der maximale Hub)
            T_klezylstart = T2 * transl([0;0;-(Rob.qlim(i,2)-Rob.qlim(i,1))]);
            drawCylinder([T_klezylstart(1:3,4)', T2(1:3,4)', Rob.DesPar.seg_par(i,2)/3], ...
              'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[1 0 0], ...
              'EdgeAlpha', 0.1);
            % Normales Segment zum Start des Hubzylinders.
            if Rob.DesPar.seg_type(i) == 1
              if Rob.qlim(i,2) > 2*Rob.qlim(i,1) && Rob.qlim(i,2) > 0 % Zylinder liegt auf der Höhe des KS
                % Als senkrechte Verbindung unter Benutzung des a-Parameters der DH-Notation
                drawCylinder([T1(1:3,4)', (eye(3,4)*T1*[Rob.MDH.a(i);0;0;1])', Rob.DesPar.seg_par(i,2)/2], ...
                  'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[0 1 0], ...
                  'EdgeAlpha', 0.1);
              else
                % Keine senkrechte Verbindung (Zylinder fängt woanders an)
                if Rob.qlim(i,2) < 0 % Großer Zylinder liegt komplett "links" (von x-Achse schauend)
                  drawCylinder([T1(1:3,4)', T_qmin(1:3,4)', Rob.DesPar.seg_par(i,2)/2], ...
                    'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[0 1 0], ...
                    'EdgeAlpha', 0.1);
                else % Großer Zylinder liegt komplett "rechts"
                  drawCylinder([T1(1:3,4)', T_grozylstart(1:3,4)', Rob.DesPar.seg_par(i,2)/2], ...
                    'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[0 1 0], ...
                    'EdgeAlpha', 0.1);
                end
              end
            end
          end
        end
      end
    end % for i = 1:Rob.NJ
end
if ~s.only_bodies && ~s.straight && all(s.mode ~= [2 4])
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
        if s.mode == 1
          % Strichmodell: Dicke Linie zeichnen
          plot3([T1(1,4),T2(1,4)],[T1(2,4),T2(2,4)],[T1(3,4),T2(3,4)], ...
            'LineWidth',4,'Color','k');
        elseif s.mode == 3
          % Trägheitsellipsen: Dünnere Linie zeichnen
          plot3([T1(1,4),T2(1,4)],[T1(2,4),T2(2,4)],[T1(3,4),T2(3,4)], ...
            'LineWidth',1,'Color','k');
        end
      end
    end
end

%% CAD-Modelle zeichnen
if s.mode == 2 && ~isempty(Rob.CADstruct)
  hdl = NaN(length(Rob.CADstruct.link),1);
  for j = 1:length(Rob.CADstruct.link)
    if ~any(s.bodies == Rob.CADstruct.link(j))
      continue % Dieser Körper soll nicht gezeichnet werden
    end
    % Körper-KS in Welt-Koordinaten
    T_W_bmdh = T_c_W(:,:,Rob.CADstruct.link(j)+1);
    % Transformation vom Körper-KS zum CAD-KS
    T_mdh_CAD = Rob.CADstruct.T_body_visual(:,:,j);
    % Pfad zur CAD-Datei (STL-Format)
    STLfilepath = Rob.CADstruct.filepath{j};
    % Farbe des Körpers
    color = Rob.CADstruct.color{j};
    
    % STL Operation: read STL
    [v, f, n, c, ~] = stlread(STLfilepath, 0);
    
    % Umrechnen der STL von Millimeter nach Meter
    if Rob.CADstruct.unit(j) ~= 1
      u = Rob.CADstruct.unit(j);
      for i = 1:size(v,1)
        v(i,:) = u*v(i,:);
      end
    end
    % Move and rotate to link reference frame 
    for i = 1:size(n,1)
        n(i,:) = (T_W_bmdh(1:3,1:3)*n(i,:)')';
    end
    for i = 1:size(v,1)
        v(i,:) = (eye(3,4)*T_W_bmdh*T_mdh_CAD*[v(i,:)';1])';
    end
    % draw STL
    hdl(j) = patch('Faces', f, 'Vertices', v,'FaceVertexCData',c, ...
      'FaceColor', 'k', 'FaceAlpha', 0.53, ...
      'EdgeColor', color, 'EdgeAlpha', 0.54);
  end
end

  
%% Verbindung zum Endeffektor
if ~s.only_bodies
  T_W_N = T_c_W(:,:,Rob.I_EElink+1);
  T_W_E = T_W_N*Rob.T_N_E;
  if s.mode == 1
    plot3([T_W_N(1,4),T_W_E(1,4)],[T_W_N(2,4),T_W_E(2,4)],[T_W_N(3,4),T_W_E(3,4)], ...
      'LineWidth',4,'Color','k')
  elseif s.mode == 3
    plot3([T_W_N(1,4),T_W_E(1,4)],[T_W_N(2,4),T_W_E(2,4)],[T_W_N(3,4),T_W_E(3,4)], ...
      'LineWidth',1,'Color','k')  
  elseif s.mode == 4
    % Als Zylinder entsprechend der Entwurfsparameter zeichnen
    if Rob.DesPar.seg_type(i+1) == 1
      drawCylinder([T_W_N(1:3,4)', T_W_E(1:3,4)', Rob.DesPar.seg_par(end,2)/2], ...
        'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[0 1 0], ...
        'EdgeAlpha', 0.1); 
    else
      error('Segmenttyp ist nicht definiert');
    end
  end
end
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