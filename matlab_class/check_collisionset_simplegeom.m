% Kollisionsprüfung eines Roboters mit einfachen Ersatzgeometrien.
% Zusätzlich schnelle Prüfung mit Axis Aligned Bounding Boxes (AABB)
% Die Geometrien basieren nur auf Gelenkpunkten ohne Transformationsmatrix
% (daher die Bezeichnung "simple geometry").
% 
% Eingabe:
% v [N x 1 uint8]
%   Vorgänger-Indizes für alle N bewegten Koordinatensysteme des Roboters.
%   Das Gelenk, das einen Körper bewegt, bildet eine Relativbewegung mit
%   dem aktuellen Körper (Zeile in v) und dem durch den Wert in v
%   angegebenen Vorgänger
% collbodies
%   Struktur der M Kollisionskörper des Roboters mit Feldern:
%   link [M X 1 uint8]
%     Nummer des zugehörigen Starrkörpers des Roboters
%   type [M X 1 uint8]
%     Art des Kollisionskörpers. Nummern konsistent mit Variable collbodies
%     in Klasse SerRob. Allerdings nur Nummer 6 implementiert (Kapsel)
%     Alle anderen Typen erfordern umfangreichere Eingaben und haben
%     höheren Rechenaufwand
%   params [M x 1 double]
%     Parameter für die Kollisionskörper
% selfcollchecks [NC x 2 uint8]
%   Zu prüfende NC Paare von Kollisionskörpern (jew. in den Spalten)
%   Der Wert in den Zeilen ist ein index auf die M Einträge in collbodies
% JP [NT x 3*(N+1)]
%   Zeitreihe mit NT Zeitschritten von Gelenkpositionen des Roboters.
%   Die einzelnen KS des Roboters werden jeweils mit der Ursprungsposition
%   angegeben. Entspricht den Zeilen der Eingangsgröße v.
%   Der erste Eintrag entspricht einer nicht angetriebenen Roboterbasis
% 
% Ausgabe:
% coll [NT x NC logical]
%   false=Keine Kollision
%   true=Kollision
% colldepth_rel [NT x NC double]
%   Durchdringungstiefe der Kollisionskörper im Fall eine Kollision.
%   Relativer Wert (1 ist maximal mögliche Durchdringung der Körper)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [coll, colldepth] = check_collisionset_simplegeom(v, collbodies, selfcollchecks, JP)
%#codegen
%$cgargs {coder.newtype('uint8',[inf,1]), struct(
%$cgargs 'link', coder.newtype('uint8',[inf,1]),
%$cgargs 'type', coder.newtype('uint8',[inf,1]),
%$cgargs 'params', coder.newtype('double',[inf,1])),
%$cgargs coder.newtype('uint8',[inf,2]),
%$cgargs coder.newtype('double',[inf,inf])}

  %% Initialisierung
  if (length(v)+1)*3 ~= size(JP,2)
    error(['Jeder Zeile aus v (%dx1) muessen 3 Spalten in JP zugeordnet sein. ', ...
      'Zusaetzlich ein Eintrag fuer Basis. Ist: %dx%d, Soll: %dx%d.'], ...
      length(v), size(JP,1), size(JP,2), size(JP,1), 3*(length(v)+1));
  end

  %% Kollisionen prüfen
  coll = false(size(JP, 1), size(selfcollchecks,1));
  colldepth = NaN(size(JP, 1), size(selfcollchecks,1));
  for i = 1:size(selfcollchecks,1) % Alle vorgemerkten Prüfungen durchgehen
    %% Parameter für Kollisionsobjekt
    b1_r = collbodies.params(selfcollchecks(i,1),1); % aktuell nur Radius
    b2_r = collbodies.params(selfcollchecks(i,2),1);
    %% Generiere die Koordinaten
    % Nummern der Körper herausfinden (Notation: 0=Basis)
    b1_ilink = collbodies.link(selfcollchecks(i,1));
    b2_ilink = collbodies.link(selfcollchecks(i,2));
    % Koordinaten herausfinden (+1 für 1-Indizierung in Matlab)
    b1_i1_pt = 1+b1_ilink;
    b1_i2_pt = 1+v(b1_ilink); % Vorgänger-Segment
    b2_i1_pt = 1+b2_ilink;
    b2_i2_pt = 1+v(b2_ilink);
    for j = 1:size(JP, 1)
      b1_pts = JP(1,[3*(b1_i1_pt-1)+1:3*b1_i1_pt, ...
                     3*(b1_i2_pt-1)+1:3*b1_i2_pt]); % bezogen auf Basis-KS
      % Das gleiche für den Kollisionsgegner
      b2_pts = JP(1,[3*(b2_i1_pt-1)+1:3*b2_i1_pt, ...
                     3*(b2_i2_pt-1)+1:3*b2_i2_pt]); % bezogen auf Basis-KS
      % Debug: Prüfe Plausibilität. Sollte vorher schon geprüft sein.
%       if all(b1_pts(1:3)==b1_pts(4:6)) 
%         warning('Kollisionspaar %d/%d: Kapsel auf Segment %d hat keine Laenge.', i, size(selfcollchecks,1), b1_ilink);
%         return
%       end
%       if all(b2_pts(1:3)==b2_pts(4:6)) 
%         warning('Kollisionspaar %d/%d: Kapsel auf Segment  %d hat keine Laenge.', i, size(selfcollchecks,1), b2_ilink);
%         return
%       end
      %% Berechne schnelle Prüfung mit AABB
      % AABB von Objekt: [xmin xmax; ymin ymax; zmin zmax]
      % Nehme Endpunkte des Zylinders der Kapsel und erweitere um den Radius
      % der Halbkugeln an den Enden
      b1_aabb = sort([b1_pts(1:3); b1_pts(4:6)])+[-repmat(b1_r,1,3);+repmat(b1_r,1,3)];
      b2_aabb = sort([b2_pts(1:3); b2_pts(4:6)])+[-repmat(b2_r,1,3);+repmat(b2_r,1,3)];
      coll_aabb = separating_axes_check(b1_aabb, b2_aabb);
      if ~coll_aabb
        coll(j,i) = 0;
        continue % Debug: Auskommentieren, trotzdem weiterrechnen
      end
      %% Feinprüfung der verbliebenen Kollisionskörper
      % Exakte geometrische Prüfung
      [dist, coll_geom, ~, d_min] = collision_capsule_capsule([b1_pts,b1_r], [b2_pts,b2_r]);
      % Debug:
%       if ~coll_aabb && coll_geom
%         error('i=%d. Kollisionspruefung zwischen AABB und Geometrie stimmt nicht', i);
%       end
      coll(j,i) = coll_geom;
      if coll_geom
        colldepth(j,i) = dist/d_min;
      end
      continue
      %% Debug: Situation zeichnen
      figure(); clf; view(3); grid on; axis equal; %#ok<UNRCH>
      drawCapsule([b1_pts,b1_r],'FaceColor', 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineStyle', ':');
      drawCapsule([b2_pts,b2_r],'FaceColor', 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineStyle', '--');
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