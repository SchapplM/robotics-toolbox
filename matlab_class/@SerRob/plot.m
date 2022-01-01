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
%     5: Darstellung mit Kollisionsobjekten (Kapseln etc.)
%     6: Darstellung mit Bauraumbegrenzung (z.B. Quader erlaubten Bauraums)
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
%   nojoints
%     0: Gelenke werden gezeichnet (Zylinder/Quader) [standard]
%     1: Keine Gelenke zeichnen, nur Segmente
%   length_frames
%     Länge der Koordinatensysteme im Plot
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
             'T_W_0', Rob.T_W_0, ... % T_W_0 für Plot von PKM-Beinketten
             'ks', [1, Rob.NJ+2], ... % nur Basis- und EE-KS
             'bodies', 0:Rob.NL, ... % CAD-Modelle für alle Körper
             'only_bodies', false, ... % Nur Körper zeichnen (STL/Ellipsoid); keine Gelenke/Sonstiges
             'jointcolors', 'serial', ... % Farben für die Gelenke
             'nojoints', false, ...
             'length_frames', 0.20); 
if nargin < 3
  % Keine Einstellungen übergeben. Standard-Einstellungen
  s = s_std;
end
% Relativ neu hinzugefügtes Feld. Abwärts-Kompatibilität für gespeicherte
% Roboter-Klassen, z.B. aus Maßsynthese-Ergebnissen
if ~isfield(Rob.DesPar, 'joint_offset')
  Rob.DesPar.joint_offset = zeros(Rob.NJ,1);
end

% Prüfe Felder der Einstellungs-Struktur und setze Standard-Werte, falls
% Eingabe nicht gesetzt
for f = fields(s_std)'
  if ~isfield(s, f{1})
    s.(f{1}) = s_std.(f{1});
  end
end
% Transformationsmatrizen zu allen Körper-KS bestimmen (Basis-KS)
T_c_0 = Rob.fkine(qJ);
% Umrechnen in das Welt-Koordinatensystem. Benutze nicht das in der
% Roboterklasse eingespeicherte, sondern ein extern vorgegebenes. Dadurch
% für PKM einfachere Implementierung für Darstellung der Beinketten
T_c_W = NaN(size(T_c_0));
for i = 1:size(T_c_W,3)
  T_c_W(:,:,i) = s.T_W_0 * T_c_0(:,:,i);
end

v = Rob.MDH.v;

% Bestimme Gelenkvariablen. Bei hybriden Robotern unterschiedlich zu den
% Minimalkoordinaten qJ.
q_JV = Rob.jointvar(qJ);
%% Gelenke als Punkte zeichnen
% O_xyz_ges = squeeze(T_c_W(1:3,4,:));
% plot3(O_xyz_ges(1,:)', O_xyz_ges(2,:)', O_xyz_ges(3,:)', 'ro', 'MarkerSize', 8);

%% Gelenke zeichnen (als Zylinder oder Quader)
% Wenn CAD-Modell vorliegt, brauchen die Gelenke nicht gezeichnet werden.
if ~s.only_bodies && all(s.mode ~= 2) && ~s.nojoints
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
    hdl_joint = [];

    if any(s.mode == 4)
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
      d = q_JV(i) + Rob.MDH.offset(i);
      % Zusätzlicher Offset aus Entwurfsparametern: Schubachse wird mit
      % einem Segment verlängert. Dadurch Verschiebung der Schiene.
      r_W_Gi_offsetkorr = T_c_W(1:3,1:3,i+1)*[0;0;Rob.DesPar.joint_offset(i)];
    elseif Rob.MDH.sigma(i) == 2 % statische Transformation (z.B. zu Schnitt-Koordinatensystemen)
      d = Rob.MDH.d(i);
    else
      error('Fall für sigma nicht definiert');
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
      if Rob.DesPar.joint_type(i) == 2 % Kardangelenk 
        gh_plot = gh*0.7; % Zylinder kleiner zeichnen
      else
        gh_plot = gh; % normale Größe für Drehgelenk
      end
      r_W_P1 = r_W_Gi + R_W_i*[0;0;-gh_plot/2];
      r_W_P2 = r_W_Gi + R_W_i*[0;0; gh_plot/2];
      if Rob.DesPar.joint_type(i) == 3  && ...% Kugelgelenk
          ... % 3 Einzelgelenke zu Kugelgelenk zusammengefasst. Zeichne mittleres.
          i>1 && i<Rob.NJ && all(Rob.DesPar.joint_type([i-1,i+1])==3)
        hdl_joint = drawSphere([r_W_Gi', 1.3*gd/2], 'FaceColor', cc);
      elseif Rob.DesPar.joint_type(i) ~= 3 % Normaler Zylinder als Ersatzdarstellung für Drehgelenk
        hdl_joint = drawCylinder([r_W_P1', r_W_P2', gd/2], 'EdgeColor', cc, ...
          'FaceAlpha', 0.3, 'FaceColor', 'w');
      end
    elseif Rob.MDH.sigma(i) == 1 % Schubgelenk
      if i<Rob.NJ && Rob.MDH.a(i+1) ~= 0
        % Schubgelenke werden nicht auf der Achse verschoben, wenn es einen
        % Abstand a gibt. Dann sieht es so aus, als ob das folgende Segment
        % auf dem Quader befestigt ist.
        r_W_Gi = r_W_Oi;
      elseif s.straight && (Rob.MDH.a(i) ~= 0 || any(r_W_Gi_offsetkorr))
        % Bei schräger Verbindung muss das Gelenk direkt im Gelenk-KS
        % gezeichnet werden. Bei sowieso gerade Verbindung ignorieren.
        % Dann sieht die nächste Option besser aus
        r_W_Gi = r_W_Oi;
      else
        % Bei der normalen Kinematik-Skizze wird das Schubgelenk auch in
        % die Mitte zwischen die KS gesetzt
        r_W_Gi = (r_W_Oi+r_W_Oimd)/2;
      end
      cubpar_c = r_W_Gi - r_W_Gi_offsetkorr; % Mittelpunkt des Quaders
      cubpar_l = [gd; gd; gh]; % Dimension des Quaders
      cubpar_a = rotation3dToEulerAngles(R_W_i)'; % Orientierung des Quaders. Benutze Singularitäts-berücksichtigende Funktion
      hdl_joint = drawCuboid([cubpar_c', cubpar_l', cubpar_a'], 'FaceColor', cc, 'FaceAlpha', 0.3);
    else
      continue % kein Gelenk
    end
    if ~isempty(hdl_joint) % Benenne das gezeichnete Objekt (zum Wiederfinden)
      set(hdl_joint, 'DisplayName', sprintf('Joint_%d', i));
    end
  end
end
%% Trägheitsellipsen zeichnen
if any(s.mode == 3)
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
%% Kollisionskörper zeichnen
% Siehe dazu auch check_collisionset_simplegeom.m
if any(s.mode == 5) || any(s.mode == 6)
  for cbtype = 1:2
    if cbtype == 1
      collbodies = Rob.collbodies;
      color = 'b';
      if ~any(s.mode == 5)
        % Keine Kollisionskörper zeichnen, sondern nur Bauraumobjekt
        continue
      end
    else
      collbodies = Rob.collbodies_instspc;
      color = 'g';
      if ~any(s.mode == 6)
        % Keine Bauraumkörper zeichnen, sondern nur Kollisionskörper
        continue
      end
    end
    for j = 1:size(collbodies.type,1)
      i = collbodies.link(j,1);
      if ~any(s.bodies == i)
        continue % Dieser Körper soll nicht gezeichnet werden
      end
      T_body_i = T_c_W(:,:,i+1);
      if collbodies.type(j) == 6 % Kapsel zum vorherigen
        T_body_iv = T_c_W(:,:,v(i)+1);
        r = collbodies.params(j,1);
        pts_W = [T_body_i(1:3,4)', T_body_iv(1:3,4)'];
        hdl=drawCapsule([pts_W, r], 'FaceColor', color, 'FaceAlpha', 0.2);
        hdl = hdl(hdl~=0); % Entferne ein 0-Handle, das bei Degeneration der Kapsel auftreten kann
      elseif collbodies.type(j) == 5 % Zylinder zum vorherigen (schräg)
        T_body_iv = T_c_W(:,:,v(i)+1);
        r = collbodies.params(j,1);
        pts_W = [T_body_i(1:3,4)', T_body_iv(1:3,4)'];
        hdl=drawCylinder([pts_W, r], 'FaceColor', color, 'FaceAlpha', 0.2);
      elseif collbodies.type(j) == 3 || ... % Kapsel mit Angabe von 2 Punkten
          collbodies.type(j) == 13 % explizit im Basis-KS
        r = collbodies.params(j,7);
        pts_i = collbodies.params(j,1:6);
        pts_W = [eye(3,4)*T_body_i*[pts_i(1:3)';1]; ...   % Punkt 1
                 eye(3,4)*T_body_i*[pts_i(4:6)';1]]'; ... % Punkt 2
        hdl=drawCapsule([pts_W, r], 'FaceColor', color, 'FaceAlpha', 0.2);
      elseif collbodies.type(j) == 2 || ... % Zylinder mit Angabe von 2 Punkten
          collbodies.type(j) == 12 % explizit im Basis-KS
        r = collbodies.params(j,7);
        pts_i = collbodies.params(j,1:6);
        pts_W = [eye(3,4)*T_body_i*[pts_i(1:3)';1]; ...   % Punkt 1
                 eye(3,4)*T_body_i*[pts_i(4:6)';1]]'; ... % Punkt 2
        hdl=drawCylinder([pts_W, r], 'FaceColor', color, 'FaceAlpha', 0.2);
      elseif collbodies.type(j) == 1 || ... % Quader mit Angabe von 1 Punkt, 2 Kanten, 1 Länge
          collbodies.type(j) == 10 % explizit im Basis-KS
        q_W = eye(3,4)*T_body_i*[collbodies.params(j,1:3)';1];
        u1_W = T_body_i(1:3,1:3)*collbodies.params(j,4:6)';
        u2_W = T_body_i(1:3,1:3)*collbodies.params(j,7:9)';
        % letzte Kante per Definition senkrecht auf anderen beiden.
        u3_W = cross(u1_W,u2_W); u3_W = u3_W/norm(u3_W)*collbodies.params(j,10);
        % Umrechnen in Format der plot-Funktion
        cubpar_c = q_W(:)+(u1_W(:)+u2_W(:)+u3_W(:))/2; % Mittelpunkt des Quaders
        cubpar_l = [norm(u1_W); norm(u2_W); norm(u3_W)]; % Dimension des Quaders
        cubpar_a = 180/pi*tr2rpy([u1_W(:)/norm(u1_W), u2_W(:)/norm(u2_W), u3_W(:)/norm(u3_W)],'zyx')'; % Orientierung des Quaders
        hdl=drawCuboid([cubpar_c', cubpar_l', cubpar_a'], ...
          'FaceColor', color, 'FaceAlpha', 0.2);
      elseif collbodies.type(j) == 4 || ...  % Kugel mit Angabe des Mittelpunkts
          collbodies.type(j) == 15 % explizit im Basis-KS
        r = collbodies.params(j,4);
        pt_i = collbodies.params(j,1:3);
        pt_W = (eye(3,4)*T_body_i*[pt_i(1:3)';1])'; ... % Punkt
        hdl=drawSphere([pt_W, r], 'FaceColor', color, 'FaceAlpha', 0.2);
      elseif collbodies.type(j) == 9 % Punkt
        pt_W = T_body_i(1:3,4); ... % Punkt direkt im KS-Ursprung
        hdl=plot3(pt_W(1), pt_W(2), pt_W(3), 'x', 'MarkerSize', 15, 'Color', color);
%       'FaceColor', color, 'FaceAlpha', 0.2);
      else
        error('Fall %d nicht definiert', collbodies.type(j));
      end
      if cbtype == 1
        set(hdl, 'DisplayName', sprintf('CollBody%d', j));
      else
        set(hdl, 'DisplayName', sprintf('InstSpcBody%d', j));
      end
    end
  end
end
%% Segmente, also Verbindungen der einzelnen Gelenke zeichnen
if ~s.only_bodies && length(intersect(s.mode, [3 4 5]))==length(s.mode) || ...
  any(s.mode == 1) && s.straight
    % hybride Roboter: Die Koordinatentransformationen sind eventuell nicht
    % einzeln bestimmbar. TODO: Prüfung für hybride Roboter
    for i = 1:Rob.NJ
      hdl_link_i = []; hdl_link_im1 = [];
      j = v(i)+1; % Körper-Index (Vorgänger)
      T1 = T_c_W(:,:,j);
      T2 = T_c_W(:,:,i+1);
      if any(s.mode == 1) && s.straight
        % Als Linie zeichnen
        plot3([T1(1,4),T2(1,4)],[T1(2,4),T2(2,4)],[T1(3,4),T2(3,4)], ...
          'LineWidth',4,'Color','k')
      elseif all(s.mode == 3) && s.straight
        plot3([T1(1,4),T2(1,4)],[T1(2,4),T2(2,4)],[T1(3,4),T2(3,4)], ...
          'LineWidth',1,'Color','k')
      elseif all(s.mode == 4)
        if Rob.MDH.sigma(i) == 0 % Drehgelenk
          % Als Zylinder entsprechend der Entwurfsparameter zeichnen
          % Bei Drehgelenken wird das aktuelle Gelenk direkt mit dem
          % vorherigen verbunden
          if Rob.DesPar.seg_type(i) == 1
            hdl_link_i(end+1) = drawCylinder([T1(1:3,4)', T2(1:3,4)', Rob.DesPar.seg_par(i,2)/2], ...
              'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[0 1 0], ...
              'EdgeAlpha', 0.1);  %#ok<AGROW>
          else
            error('Segmenttyp ist nicht definiert');
          end
        else % Schubgelenk
          r_W_Gi_offsetkorr = T2(1:3,1:3)*[0;0;Rob.DesPar.joint_offset(i)];
          % Bei Schubgelenken wird die Konstruktionsart des Schubgelenkes
          % berücksichtigt.
          % Transformation zu Anfangs- und Endlage des Schubgelenkes
          T_qmin = T2 * transl([0;0;-q_JV(i)+Rob.qlim(i,1)]); % Lage von T2, wenn q=qmin wäre
          T_qmax = T2 * transl([0;0;-q_JV(i)+Rob.qlim(i,2)]); % Lage von T2, wenn q=qmax wäre
          if abs(Rob.qlim(i,1)) < abs(Rob.qlim(i,2))
            % Untere Grenze ist betragsmäßig kleiner als obere Grenze.
            % Normaler Fall mit positiven Koordinaten. Wähle qmin als nahen
            % und qmax als fernen Punkt von Führungsschiene/Hubzylinder
            T_qdist = T_qmax;
            T_qprox = T_qmin;
          else
            % Negative Koordinaten. Untere Grenze hat betragsgrößeren (aber
            % negativen) Wert und ist deshalb weiter vom vorherigen Gelenk
            % weg. Intuitiver ist das Vertauschen der beiden Grenzen für
            % Plot.
            T_qdist = T_qmin;
            T_qprox = T_qmax;
          end
          % Debug:
%           trplot(T_qmin, 'frame', sprintf('qmin%d', i), 'rgb', 'length', 0.2);
%           trplot(T_qmax, 'frame', sprintf('qmax%d', i), 'rgb', 'length', 0.2);
          if Rob.DesPar.joint_type(i) ==  4 % Schubgelenk ist Linearführung
            % Zeichne Linearführung als Quader. TODO: Benutze T_qprox/T_qdist
            cubpar_c = (T_qmin(1:3,4)+T_qmax(1:3,4))/2-r_W_Gi_offsetkorr; % Mittelpunkt des Quaders
            cubpar_l = [Rob.DesPar.seg_par(i,2)*0.25*[1;1];Rob.qlim(i,2)-Rob.qlim(i,1)]; % Dimension des Quaders
            cubpar_a = rotation3dToEulerAngles(T_qmin(1:3,1:3))'; % Orientierung des Quaders
            drawCuboid([cubpar_c', cubpar_l', cubpar_a'], 'FaceColor', 'b', 'FaceAlpha', 0.3);
            % Normales Segment zum Start der Linearführung
            if Rob.DesPar.seg_type(i) == 1
              if Rob.qlim(i,1)*Rob.qlim(i,2)< 0 % Führung liegt auf der Höhe
                % Als senkrechte Verbindung unter Benutzung des a-Parameters der DH-Notation
                hdl_link_i(end+1) = drawCylinder([T1(1:3,4)', (eye(3,4)*T1*[Rob.MDH.a(i);0;0;1])', Rob.DesPar.seg_par(i,2)/2], ...
                  'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[0 1 0], ...
                  'EdgeAlpha', 0.1);  %#ok<AGROW>
                if Rob.DesPar.joint_offset(i) ~= 0
                  % TODO: Für diesen Fall noch nicht ausreichend getestet
                  hdl_link_im1(end+1) = drawCylinder([T2(1:3,4)', ...
                    T_qmax(1:3,4)'-r_W_Gi_offsetkorr', Rob.DesPar.seg_par(i,2)/2], ...
                    'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[0 1 0], ...
                    'EdgeAlpha', 0.1);  %#ok<AGROW>
                end
              else
                % Keine senkrechte Verbindung (Schiene fängt woanders an)
                if Rob.qlim(i,2) < 0 % Schiene liegt komplett "links" vom KS. Gehe zum Endpunkt
                  hdl_link_i(end+1) = drawCylinder([T1(1:3,4)', T_qmax(1:3,4)', Rob.DesPar.seg_par(i,2)/2], ...
                    'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[0 1 0], ...
                    'EdgeAlpha', 0.1);  %#ok<AGROW>
                  if Rob.DesPar.joint_offset(i) ~= 0
                    % TODO: Für diesen Fall noch nicht ausreichend getestet
                    hdl_link_im1(end+1) = drawCylinder([T2(1:3,4)', ...
                      T_qmax(1:3,4)'-r_W_Gi_offsetkorr', Rob.DesPar.seg_par(i,2)/2], ...
                      'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[0 1 0], ...
                      'EdgeAlpha', 0.1);  %#ok<AGROW>
                  end
                else % Schiene liegt "rechts" vom KS. Gehe zum Startpunkt
                  hdl_link_i(end+1) = drawCylinder([T1(1:3,4)', ...
                    T_qmin(1:3,4)'-r_W_Gi_offsetkorr', Rob.DesPar.seg_par(i,2)/2], ...
                    'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[0 1 0], ...
                    'EdgeAlpha', 0.1);  %#ok<AGROW>
                  % Zusätzliches Segment nach dem Schubgelenk (mit Offset)
                  if Rob.DesPar.joint_offset(i) ~= 0
                    % Das Offset-Segment fängt in der Mitte des durch den
                    % Quader symbolisierten Gelenks an (damit konstant lang)
                    hdl_link_im1(end+1) = drawCylinder([T2(1:3,4)', ...
                      T2(1:3,4)'-r_W_Gi_offsetkorr', Rob.DesPar.seg_par(i,2)/2], ...
                      'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[0 1 0], ...
                      'EdgeAlpha', 0.1);  %#ok<AGROW>
                  end
                end
              end
            end
          else % Schubgelenk ist Hubzylinder
            if Rob.DesPar.joint_offset(i) ~= 0
              warning('Ser.DesPar.joint_offset noch nicht für diesen Fall implementiert');
            end
            % Großer Zylinder, muss so lang sein wie maximaler Hub.
            T_grozylstart = T_qprox * transl([0;0;-sign(q_JV(i))*(Rob.qlim(i,2)-Rob.qlim(i,1))]);
            hdl_link_im1(end+1) = drawCylinder([T_grozylstart(1:3,4)', T_qprox(1:3,4)', Rob.DesPar.seg_par(i,2)/2], ...
              'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[1 1 0], ...
              'EdgeAlpha', 0.1);  %#ok<AGROW>
            % Kleinerer Zylinder, der herauskommt (aber so lang ist, wie
            % der maximale Hub)
            T_klezylstart = T2 * transl([0;0;-sign(q_JV(i))*(Rob.qlim(i,2)-Rob.qlim(i,1))]);
            hdl_link_i(end+1) = drawCylinder([T_klezylstart(1:3,4)', T2(1:3,4)', Rob.DesPar.seg_par(i,2)/3], ...
              'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[1 0 0], ...
              'EdgeAlpha', 0.1);  %#ok<AGROW>
            % Normales Segment zum Start des Hubzylinders.
            if Rob.DesPar.seg_type(i) == 1
              if Rob.qlim(i,2) > 2*Rob.qlim(i,1) && Rob.qlim(i,2) > 0 % Zylinder liegt auf der Höhe des KS
                % Als senkrechte Verbindung unter Benutzung des a-Parameters der DH-Notation
                hdl_link_im1(end+1) = drawCylinder([T1(1:3,4)', (eye(3,4)*T1*[Rob.MDH.a(i);0;0;1])', Rob.DesPar.seg_par(i,2)/2], ...
                  'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[0 1 0], ...
                  'EdgeAlpha', 0.1);  %#ok<AGROW>
              else
                % Keine senkrechte Verbindung (Zylinder fängt woanders an)
                if Rob.qlim(i,2) < 0 % Großer Zylinder liegt komplett "links" (von x-Achse schauend)
                  hdl_link_im1(end+1) = drawCylinder([T1(1:3,4)', T_qprox(1:3,4)', Rob.DesPar.seg_par(i,2)/2], ...
                    'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[0 1 0], ...
                    'EdgeAlpha', 0.1);  %#ok<AGROW>
                else % Großer Zylinder liegt komplett "rechts"
                  hdl_link_im1(end+1) = drawCylinder([T1(1:3,4)', T_grozylstart(1:3,4)', Rob.DesPar.seg_par(i,2)/2], ...
                    'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[0 1 0], ...
                    'EdgeAlpha', 0.1);  %#ok<AGROW>
                end
              end
            end
          end
        end
      end
      % Benennung der gezeichneten 3D-Körper. Es werden die Vorgänger- 
      % Segmente und die vom aktuellen Gelenk bewegten Teile getrennt.
      for kk = 1:length(hdl_link_im1) % Vorgänger
        set(hdl_link_im1(kk), 'DisplayName', sprintf('Link_%d', i-1));
      end
      for kk = 1:length(hdl_link_i) % vom Gelenk bewegt
        set(hdl_link_i(kk), 'DisplayName', sprintf('Link_%d', i));
      end
    end % for i = 1:Rob.NJ
end
if ~s.only_bodies && ~s.straight && isempty(intersect(s.mode,[2 4 5]))
    for i = 1:Rob.NJ
      j = v(i)+1;
      % MDH-Transformation in einzelnen Schritten nachrechnen, damit diese
      % geplottet werden können.
      T_k = NaN(4,4,3);
      T_k(:,:,1) = T_c_W(:,:,j);
      T_k(:,:,2) = T_k(:,:,1) * trotz(Rob.MDH.beta(i)) * transl([0;0;Rob.MDH.b(i)]);
      T_k(:,:,3) = T_k(:,:,2) * trotx(Rob.MDH.alpha(i)) * transl([Rob.MDH.a(i);0;0]);
      if Rob.MDH.sigma(i) == 0 % Drehgelenk
        theta = q_JV(i) + Rob.MDH.offset(i);
        d = Rob.MDH.d(i);
      elseif Rob.MDH.sigma(i) == 1 % Schubgelenk
        theta = Rob.MDH.theta(i);
        d = q_JV(i) + Rob.MDH.offset(i);
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
        if any(s.mode == 1)
          % Strichmodell: Dicke Linie zeichnen
          plot3([T1(1,4),T2(1,4)],[T1(2,4),T2(2,4)],[T1(3,4),T2(3,4)], ...
            'LineWidth',4,'Color','k');
        elseif any(s.mode == 3)
          % Trägheitsellipsen: Dünnere Linie zeichnen
          plot3([T1(1,4),T2(1,4)],[T1(2,4),T2(2,4)],[T1(3,4),T2(3,4)], ...
            'LineWidth',1,'Color','k');
        end
      end
    end
end

%% CAD-Modelle zeichnen
if any(s.mode == 2) && ~isempty(Rob.CADstruct)
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
T_W_N = T_c_W(:,:,Rob.I_EElink+1);
T_W_E = T_W_N*Rob.T_N_E;
if ~s.only_bodies
  if any(s.mode == 1)
    plot3([T_W_N(1,4),T_W_E(1,4)],[T_W_N(2,4),T_W_E(2,4)],[T_W_N(3,4),T_W_E(3,4)], ...
      'LineWidth',4,'Color','k')
  elseif any(s.mode == 3)
    plot3([T_W_N(1,4),T_W_E(1,4)],[T_W_N(2,4),T_W_E(2,4)],[T_W_N(3,4),T_W_E(3,4)], ...
      'LineWidth',1,'Color','k')  
  elseif any(s.mode == 4)
    % Als Zylinder entsprechend der Entwurfsparameter zeichnen (falls >0)
    if Rob.DesPar.seg_type(i+1) == 1
      if norm(T_W_N(1:3,4)'-T_W_E(1:3,4)')>1e-9
        hdl = drawCylinder([T_W_N(1:3,4)', T_W_E(1:3,4)', Rob.DesPar.seg_par(end,2)/2], ...
          'open', 'FaceAlpha', 0.3, 'FaceColor', 'k', 'edgeColor', 0.7*[0 1 0], ...
          'EdgeAlpha', 0.1);
        set(hdl, 'DisplayName', sprintf('Link_%d', Rob.I_EElink));
      end
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
  trplot(T_c_W(:,:,i), 'frame', sprintf('%d',i-1), 'rgb', 'length', s.length_frames)
end
if any(s.ks == Rob.NJ+2)
  trplot(T_W_E, 'frame', 'E', 'rgb', 'length', s.length_frames)
end

%% Formatierung
axis equal
