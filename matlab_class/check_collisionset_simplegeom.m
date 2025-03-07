% Kollisionsprüfung eines Roboters mit einfachen Ersatzgeometrien.
% Zusätzlich schnelle Prüfung mit Axis Aligned Bounding Boxes (AABB)
% Die Geometrien basieren nur auf Gelenkpunkten ohne Transformationsmatrix
% (daher die Bezeichnung "simple geometry").
% 
% Eingabe:
% cb (collbodies)
%   Struktur der M Kollisionskörper des Roboters mit Feldern:
%   link [M X 2 uint16]
%     Erster Eintrag: Nummer des zugehörigen Starrkörpers des Roboters.
%     Zweiter Eintrag: Zweiter zugehöriger Starrkörper. Normalfall: Vor-
%     gänger-Indizes für alle N Koordinatensysteme des Roboters.
%     Basis des Roboters hat den Eintrag 0. Dann fortlaufend nummeriert.
%     Auch Verbindung von KS-Ursprüngen nicht aufeinanderfolgender Segmente
%     des Roboter möglich (z.B. für Gestell und Plattform von PKM).
%   type [M X 1 uint8]
%     Art des Kollisionskörpers. Nummern konsistent mit Variable collbodies
%     in Klasse SerRob. Allerdings nur wenige Nummern implementiert
%     Alle anderen Typen erfordern umfangreichere Eingaben und haben
%     höheren Rechenaufwand. Bei den einfachen Körpern ist keine Eingabe
%     von Koordinaten als Parameter notwendig, da die Körper direkt die
%     in JP gegebenen KS-Ursprünge entsprechend cb.link verbinden.
%     *  1 Quader (10 Parameter: Aufpunkt, 2 Kantenvektoren, Länge 3. Kante)
%          (erfordert vollständiges Körper-KS. Aktuell nicht möglich)
%     *  2 Zylinder (7 Parameter: Punkt 1, Punkt 2, Radius)
%          (erfordert vollständiges Körper-KS. Aktuell nicht möglich)
%     *  3 Kapsel (7 Parameter: Punkt 1, Punkt 2, Radius)
%          (erfordert vollständiges Körper-KS. Aktuell nicht möglich)
%     *  4 Kugel (4 Parameter: Mittelpunkt, Radius)
%          (erfordert vollständiges Körper-KS. Aktuell nicht möglich)
%     *  5 Zylinder als schräge DH-Verbindung (1 Parameter: Radius)
%     *  6 Kapsel als schräge DH-Verbindung (1 Parameter: Radius)
%     *  7 Zylinder als gewinkelte DH-Verbindung (entlang a- und d-Parameter)
%          (Platzhalter; nicht implementiert)
%     *  8 Kapsel als gewinkelte DH-Verbindung (1 Parameter: Radius)
%          (Platzhalter; nicht implementiert)
%     *  9 Punkt am DH-KS-Ursprung (0 Parameter)
%     * 10 Quader im Basis-KS. Entspricht einem Störobjekt in der Umgebung
%          (Im Gegensatz zu 1 sind keine vollständigen Körper-KS notwendig)
%     * 11 Quader im Basis-KS. Exakt entlang der Achsen ausgerichtet.
%          Dadurch sehr einfache Prüfung mit AABB-Verfahren möglich. TODO.
%     * 12 Zylinder im Basis-KS. Störobjekt in der Umgebung
%          (Gegensatz zu 2)
%     * 13 Kapsel im Basis-KS. Störobjekt in der Umgebung oder Ersatz für
%           Führung eines Schubgelenks (Gegensatz zu 3)
%     * 14 Punkt im Basis-KS (3 Parameter) (im Gegensatz zu 9). Kann einem
%          Kollisionsobjekt an der Roboterbasis entsprechen
%     * 15 Kugel im Basis-KS (4 Parameter) (im Gegensatz zu 4). Kann einem
%          Kollisionsobjekt an der Roboterbasis entsprechen
%     * 16 Kugel im Körper-KS (1 Parameter)
%   params [M x 10 double]
%     Parameter für die Kollisionskörper. Je nachdem wie viele Parameter
%     die obigen Kollisionsgeometrien haben, werden Spalten mit NaN aufgefüllt
% cc (collchecks) [NC x 2 uint8]
%   Zu prüfende NC Paare von Kollisionskörpern (jew. in den Spalten)
%   Der Wert in den Zeilen ist ein index auf die M Einträge in collbodies
% JP [NT x 3*N]
%   Zeitreihe mit NT Zeitschritten von Gelenkpositionen des Roboters.
%   Die einzelnen KS des Roboters werden jeweils mit der Ursprungsposition
%   angegeben. Entspricht den Indizes in cb.link.
%   Der erste Eintrag entspricht einer nicht angetriebenen Roboterbasis
% Set
%   Struktur mit Einstellungen zur Arbeitsweise der Funktion
%   collsearch: Suche Kollisionen (true) oder Nicht-Kollisionen
% 
% Ausgabe:
% coll [NT x NC logical]
%   false=Keine Kollision
%   true=Kollision
% dist [NT x NC double]
%   Durchdringungstiefe der Kollisionskörper im Fall eine Kollision.
%   Absoluter Wert
% dist_rel [NT x NC double]
%   Durchdringungstiefe der Kollisionskörper im Fall eine Kollision.
%   Relativer Wert (1 ist maximal mögliche Durchdringung der Körper)
% p [NC x 6 x NT double]
%   Koordinaten der kürzesten Verbindung zwischen den Objekten
%   Erst Körper 1 aus `cc`, dann Körper 2

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [coll, dist, dist_rel, p] = check_collisionset_simplegeom(cb, cc, JP, Set)
%#codegen
%$cgargs {struct('link', coder.newtype('uint16',[inf,2]),
%$cgargs 'type', coder.newtype('uint8',[inf,1]),
%$cgargs 'params', coder.newtype('double',[inf,10])),
%$cgargs coder.newtype('uint8',[inf,2]),
%$cgargs coder.newtype('double',[inf,inf]), struct(
%$cgargs 'collsearch', true)}

  %% Initialisierung
  % Arbeitsmodus der Funktion. Bei true werden Kollisionen gesucht. Bei
  % erwiesener Nicht-Kollision wird abgebrochen. Bei false wird geprüft, ob
  % es keine Kollision gibt (z.B. zur Bauraumüberprüfung)
  collsearch = Set.collsearch;

  %% Kollisionen prüfen
  p = NaN(size(cc,1), 6, size(JP, 1));
  coll = false(size(JP, 1), size(cc,1));
  dist = NaN(size(JP, 1), size(cc,1));
  dist_rel = NaN(size(JP, 1), size(cc,1));
  for i = 1:size(cc,1) % Alle vorgemerkten Prüfungen durchgehen
    %% Fall der Kollisionsprüfung festlegen (Vereinfachung in innerster Schleife)
    if any(cb.type(cc(i,1)) == [6 13]) && any(cb.type(cc(i,2)) == [6,13])
      collcase = uint8(1); % Kapsel+Kapsel (egal ob in Basis- oder Körper-KS)
    elseif any(cb.type(cc(i,1)) == [9,14]) && cb.type(cc(i,2)) == 10
      collcase = uint8(2); % Punkt+Quader
    elseif any(cb.type(cc(i,1)) == [9,14]) && cb.type(cc(i,2)) == 12
      collcase = uint8(3); % Punkt+Zylinder
    elseif any(cb.type(cc(i,1)) == [9,14]) && cb.type(cc(i,2)) == 13
      collcase = uint8(4); % Punkt+Kapsel
    elseif any(cb.type(cc(i,1)) == [6,13]) && any(cb.type(cc(i,2)) == [16,15])
      collcase = uint8(5); % Kapsel+Kugel (egal ob in Basis- oder Körper-KS)
    elseif any(cb.type(cc(i,2)) == [6,13]) && any(cb.type(cc(i,1)) == [16,15])
      collcase = uint8(6); % Kugel+Kapsel (egal ob in Basis- oder Körper-KS)
    elseif any(cb.type(cc(i,2)) == [15,16]) && any(cb.type(cc(i,1)) == [15,16])
      collcase = uint8(7); % Kugel+Kugel (egal ob in Basis- oder Körper-KS)
    else
      error('Fall %d vs %d nicht definiert', cb.type(cc(i,1)), cb.type(cc(i,2)));
    end
      
    %% Parameter für Kollisionsobjekt
    b_param = [cb.params(cc(i,1),:); ...
               cb.params(cc(i,2),:)];
    %% Generiere die Koordinaten
    % Nummern der Körper herausfinden (Notation: 0=Basis)
    b_ilink = [cb.link(cc(i,1),1); ... % Körper 1
               cb.link(cc(i,2),1)];    % Körper 2
    % Indizes der beteiligten Punkte bestimmen
    % (+1 für 1-Indizierung in Matlab)
    % erste Spalte: Segment, zweite Spalte: Vorgänger-Segment
    % Zeilen sind die beiden an der Kollision beteiligten Körper.
    b_idx_pt = [1+b_ilink(1), 1+cb.link(cc(i,1),2); ... % Körper 1
                1+b_ilink(2), 1+cb.link(cc(i,2),2)];    % Körper 2
    for j = 1:size(JP, 1) % alle Zeitschritte durchgehen
      % Bestimme Parameter der AABB und des Kollisionsobjektes für beide an
      % der Kollision beteiligte Geometrien
      [b1_aabb, b1_cbparam] = get_cbparam(cb.type(cc(i,1)), b_idx_pt(1,:), ...
        JP(j,:), b_param(1,:));
      [b2_aabb, b2_cbparam] = get_cbparam(cb.type(cc(i,2)), b_idx_pt(2,:), ...
        JP(j,:), b_param(2,:));
      %% Berechne grobe Prüfung mit AABB
      coll_aabb = separating_axes_check(b1_aabb, b2_aabb);
      if ~coll_aabb
        % Nach Prüfung mit AABB-Verfahren ist keine Kollision möglich.
        % Das ist die notwendige Bedingung für eine Kollision.
        coll(j,i) = 0;
        if collsearch % Wenn Kollisionen gesucht sind, ist keine Rechnung mehr notwendig
          continue % Für Debuggen: Auskommentieren, trotzdem weiterrechnen
        end
      end
      %% Feinprüfung der verbliebenen Kollisionskörper
      % Exakte geometrische Prüfung
      switch collcase
        case 1 % Kapsel+Kapsel
          [di, coll_geom, pkol, d_min] = collision_capsule_capsule(b1_cbparam, b2_cbparam);
        case 2 % Punkt+Quader
          [di, coll_geom, pkol_box] = collision_box_point(b2_cbparam, b1_cbparam);
          % maximale Eindringung des Punktes in Zylinder: Ganz in der Mitte
          % (nur Annäherung, da AABB größer als Quader sein kann, bei Rotation)
          d_min = -max(diff(b2_aabb))/2;
          pkol = [b1_cbparam(1:3);pkol_box]; % In Funktion Koordinaten auf dem Quader
        case 3 % Punkt+Zylinder
          [di, coll_geom, pkol_cyl, d_min] = collision_cylinder_point(b2_cbparam, b1_cbparam);
          pkol = [b1_cbparam(1:3); pkol_cyl]; % In Funktion Koordinaten auf dem Zylinder
        case 4 % Punkt+Kapsel
          % Interpretiere den Punkt als Kugel mit Radius Null und nehme die
          % existierende Funktion für Kapsel+Kugel.
          [di, coll_geom, pkol, d_min] = collision_capsule_sphere(b2_cbparam, [b1_cbparam,0.0]);
          % Ausgabe pkol:
        case 5 % Kapsel+Kugel
          [di, coll_geom, pkol, d_min] = collision_capsule_sphere(b1_cbparam, b2_cbparam);
          pkol = flipud(pkol); %  In Funktion erst Kugel, dann Kapsel
        case 6 % Kugel+Kapsel
          [di, coll_geom, pkol, d_min] = collision_capsule_sphere(b2_cbparam, b1_cbparam);
        case 7 % Kugel+Kugel
          [di, coll_geom, pkol, d_min] = collision_sphere_sphere(b1_cbparam, b2_cbparam);
        otherwise
          error('Fall nicht definiert. Dieser Fehler darf gar nicht auftreten');
      end
      % Debug:
%       if ~coll_aabb && coll_geom
%         error('i=%d. Kollisionspruefung zwischen AABB und Geometrie stimmt nicht', i);
%       end
      coll(j,i) = coll_geom;
      p(i, :, j) = pkol([1 3 5 2 4 6]);
      if coll_geom
        dist(j,i) = di;
        dist_rel(j,i) = di/d_min;
      elseif ~collsearch
        % Es gibt keine Kollision. Wenn nach Objekten gesucht wird, die
        % nicht ineinander liegen, ist auch deren Abstand interessant
        % d_min ist immer negativ. dist ist positiv (da keine Kollision)
        dist(j,i) = di;
        dist_rel(j,i) = -di/d_min;
      end
      continue
      %% Debug: Situation zeichnen
      figure(); clf; view(3); grid on; axis equal; hold on; %#ok<UNRCH>
      if collcase == 1
        drawCapsule(b1_cbparam,'FaceColor', 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineStyle', ':');
        drawCapsule(b2_cbparam,'FaceColor', 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineStyle', '--');
      elseif collcase == 2
        plot3(b1_cbparam(1), b1_cbparam(2), b1_cbparam(3), 'bx', 'MarkerSize', 10);
        q_W = eye(3,4)*[b2_cbparam(1:3)';1]; u1_W = b2_cbparam(4:6)'; 
        u2_W = b2_cbparam(7:9)'; u3_W = b2_cbparam(10:12);
        % Umrechnen in Format der plot-Funktion
        cubpar_c = q_W(:)+(u1_W(:)+u2_W(:)+u3_W(:))/2; % Mittelpunkt des Quaders
        cubpar_l = [norm(u1_W); norm(u2_W); norm(u3_W)]; % Dimension des Quaders
        cubpar_a = 180/pi*tr2rpy([u1_W(:)/norm(u1_W), u2_W(:)/norm(u2_W), u3_W(:)/norm(u3_W)],'zyx')'; % Orientierung des Quaders
        drawCuboid([cubpar_c', cubpar_l', cubpar_a'], 'FaceColor', 'r', 'FaceAlpha', 0.1);
      elseif collcase == 5
        drawCapsule(b1_cbparam,'FaceColor', 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineStyle', ':');
        drawSphere(b2_cbparam,'FaceColor', 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineStyle', '--');
      elseif collcase == 6
        drawCapsule(b2_cbparam,'FaceColor', 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineStyle', ':');
        drawSphere(b1_cbparam,'FaceColor', 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineStyle', '--');
      end
      plot3(pkol(:,1), pkol(:,2), pkol(:,3), '-kx', 'MarkerSize', 5, 'LineWidth', 3);
    end
  end
end

function collision = separating_axes_check(A, B)
  % Prüft zwei AABBs von Objekten auf Überschneidung
  % Für jede Raumachse gilt: Ist die obere Grenze von Objekt 1 größer als
  % die untere Grenze von Objekt 2 und die untere Grenze von Objekt 1
  % kleiner als die obere Grenze von Objekt 2, liegt eine
  % Intervallüberschneidung vor. Liegt auf allen Achsen eine
  % Intervallüberschneidung vor, kollidieren die AABBs und können in die
  % Kandidatenpaarliste ür die nahe Phase aufgenommen werden.
  %
  % Quelle: BA Dwayne Steinke, 2018-03
  % 
  % Eingabe:
  % A: AABB von Objekt A: [[xmin; xmax], [ymin; ymax], [zmin; zmax]]
  % B: AABB von Objekt B.
  %
  % Ausgabe:
  % collision: true, falls Kollision möglich ist; sonst false
  
  collision = false;
  % Teste nacheinander die x-, y- und z-Achse
  if A(2,1)<B(1,1) || A(1,1)>B(2,1)
    return % keine Überschneidung. Abbruch.
  end
  if A(2,2)<B(1,2) || A(1,2)>B(2,2)
    return
  end
  if A(2,3)<B(1,3) || A(1,3)>B(2,3)
    return
  end
  % Überschneidung auf allen Achsen, da bis hierher gekommen.
  collision = true;
end

function [aabbdata, cb_param] = get_cbparam(type, b_idx_pt, JP_i, b_param)
  % Input:
  %   type: Nummer des Typs des Kollisionsobjektes
  %   b_idx_pt: 1: Index des Punktes für Körper-KS des Kollisionskörpers.
  %             2: Index des Vorgänger-KS-Ursprungs in JP_i
  %   JP_i: Zeile der Eingabevariable JP von oben
  %   b_param
  % Output:
  %   aabbdata; AABB von Objekt: [[xmin; xmax], [ymin; ymax], [zmin; zmax]]
  %   cb_param; Parameter für Kollisionskörper. Je nach Typ
  switch type
    case 6 % Kapsel bezogen auf Vorgänger-KS-Ursprung
      b_pts = JP_i(1,[3*(b_idx_pt(1,1)-1)+1:3*b_idx_pt(1,1), ...
                      3*(b_idx_pt(1,2)-1)+1:3*b_idx_pt(1,2)]); % bezogen auf Basis-KS
      % Nehme Endpunkte des Zylinders der Kapsel und erweitere um den Radius
      % der Halbkugeln an den Enden
      aabbdata = sort([b_pts(1:3); b_pts(4:6)])+[-repmat(b_param(1),1,3);+repmat(b_param(1),1,3)];
      % Endpunkte der Zylinder. Radius der Halbkugeln
      cb_param = [b_pts,b_param(1)];
    case 9 % Punkt des KS-Ursprungs
      b_pt = JP_i(1,3*(b_idx_pt(1,1)-1)+1:3*b_idx_pt(1,1)); % bezogen auf Basis-KS
      aabbdata = [b_pt;b_pt]; % Bounding-Box ist auch nur ein Punkt
      % Parameter sind die Punktkoordinaten
      cb_param = b_pt;
    case 10 % Quader im Basis-KS (frei drehbar)
      % Vektorielle Darstellung: Aufpunkt und Kantenvektoren
      q = b_param(1:3); u1 = b_param(4:6); u2 = b_param(7:9);
      % letzte Kante per Definition senkrecht auf anderen beiden.
      u3 = cross(u1,u2); u3 = u3/norm(u3)*b_param(10);
      % Ecken des Quaders
      corners = [q; q+u1; q+u2; q+u1+u2; q+u3; q+u1+u3; q+u2+u3; q+u1+u2+u3];
      % Begrenzende AABB durch Minimal- und Maximalwerte
      [xyzmin, xyzmax] = bounds(corners);
      aabbdata = [xyzmin; xyzmax];
      % Endpunkte der Zylinder. Radius der Halbkugeln
      cb_param = [q, u1, u2, u3];
    case 12 % Zylinder im Basis-KS
      % Interpretiere Zylinder als Kapsel, da bei schräger Anordnung nicht
      % definiert ist, wo die Kante liegt. Dadurch ist die AABB zu groß
      aabbdata = sort([b_param(1:3); b_param(4:6)])+...
                 [-repmat(b_param(7),1,3);+repmat(b_param(7),1,3)];
      cb_param = b_param(1:7);
    case 13 % Kapsel im Basis-KS
      aabbdata = sort([b_param(1:3); b_param(4:6)])+...
                 [-repmat(b_param(7),1,3);+repmat(b_param(7),1,3)];
      cb_param = b_param(1:7);
    case 14 % Punkt im Basis-KS
      b_pt = b_param(1:3); % bezogen auf Basis-KS
      aabbdata = [b_pt;b_pt]; % Bounding-Box ist auch nur ein Punkt
      % Parameter sind die Punktkoordinaten
      cb_param = b_pt;
    case 15 % Kugel im Basis-KS 
      c = b_param(1:3); % Mittelpunkt bezogen auf Basis-KS
      r = b_param(4); % Radius
      aabbdata = [c-[1,1,1]*r;c+[1,1,1]*r]; % unten links und oben rechts zum Aufspannen der Box
      cb_param = b_param(1:4);
    case 16 % Kugel in KS-Ursprung
      c = JP_i(1,3*(b_idx_pt(1,1)-1)+1:3*b_idx_pt(1,1)); % bezogen auf Basis-KS
      r = b_param(1); % Radius
      aabbdata = [c-[1,1,1]*r;c+[1,1,1]*r]; % unten links und oben rechts zum Aufspannen der Box
      cb_param = [c,r];
    otherwise
      error('Fall %d fuer Kollisionskoerper nicht definiert', type);
  end
  % Debug-Prüfungen. Können später auskommentiert werden.
  if any(size(aabbdata)~=[2 3])
    error('Falsche Dimension der Ausgabe');
  end
  if any(aabbdata(1,:)>aabbdata(2,:)) % min (1. Zeile) > max (2. Zeile)
    error('Die AABB-Parameter sind nicht richtig sortiert');
  end
end
