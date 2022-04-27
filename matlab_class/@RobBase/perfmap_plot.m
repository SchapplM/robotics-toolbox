% Zeichne eine Redundanzkarte (Farbkarte über Trajektorie und Koordinate)
% 
% Eingabe:
% H_all [NI x NP x NK+4]
%   Diskretisierung der `NK` möglichen Kriterien für Raster mit Auflösung 
%   `NI x NP`. Aus perfmap_taskred_ik.
% phiz_range [NP x 1]
%   Diskretisierung der redundanten Koordinate in H_all
% s_ref
%   Diskretisierung der Interpolationsschritte in s_ref. Kann entweder Zeit oder
%   Bahnkoordinate sein, je nach Wert in Einstellung s.
% s_in [struct]
%   Einstellungen zum Zeichnen der Redundanzkarte. Felder:
%   .reference ['time', 'normalized']
%     Skalierung der waagerechten Achse. Wert 'normalized' für Bahnkoordinate s
%   .abort_thresh_h [1xNK]
%     Schwellwert für jedes Kriterium zur Begrenzung der Farben
%   .PM_limit [true, false]
%     Begrenze die Farben mit einem oberen Wert
%   .s_range_plot
%     Grenzen für die senkrechte Achse (redundante Koordinate)
%   .phiz_range_plot
%     Grenzen für die senkrechte Achse (redundante Koordinate)
%   .wn [NK x 1]
%     Gewichtung der Kriterien im Plot. Sollte mit Gewichtung in Trajektorie
%     übereinstimmen, damit Verhalten plausibel aus Karte abgeleitet werden kann
%   .markermindist [1x2]
%     Mindestabstand zweier Marker für Nebenbedingungen (1. waagerechter,
%     2. senkrechter Abstand in Plot (in Plot-Einheiten))
%   .extend_map [true, false]
%     Verdopple die Karte transparent nach unten und oben mit 2pi-Periodizität
%   .violation_markers {2xNV} cell
%     Marker für die Verletzung von Nebenbedingungen. Erster Eintrag (Zeile)
%     Nebenbedingung (qlim_hyp, jac_cond, ...), zweiter Eintrag Plot-Marker.
%   .xlabel, .ylabel
%     String for axis labels
% 
% Ausgabe:
% Hdl_all [struct]
%   Struktur mit diversen Handles zu den Objekten des Plots
%   .VM [NV x 1 handles]: Marker aus s.violation_markers
%   .surf: Eigentliche Redundanzkarte (Surface-Plot)
%   .cb: Color Bar
% s [struct]
%   Einstellungen die benutzt wurden (inklusive Standardwerten)
% PlotData [struct]
%   Details für die Berechnung des Plots. Felder, siehe Quelltext.
%   Kann benutzt werden, um Plot besser zu interpretieren oder zu beschreiben.
% 
% Siehe auch: cds_debug_taskred_perfmap, @RobBase/perfmap_taskred_ik
% 
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-02
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [Hdl_all, s, PlotData] = perfmap_plot(R, H_all, phiz_range, s_ref, s_in)
%% Initialisierung
Hdl_all = struct('surf', NaN, 'VM', NaN(6,1), 'cb', NaN);
s = struct( ...
  'reference', 'normalized', ...
  'abort_thresh_h', inf(R.idx_ik_length.hnpos, 1), ... % Schwellwert für Marker
  'PM_limit', true, ... % Farben begrenzen
  's_range_plot', NaN(1,2), ...
  'phiz_range_plot', NaN(1,2), ...
  'wn', NaN(R.idx_ik_length.hnpos, 1), ... % Gewichtung Leistungsmerkmale
  'markermindist', [0,0], ... % Minimaler Abstand zweier gleicher Marker in x- und y-Richtung
  'extend_map', true, ... % Karte erweitern
  'log', false, ... % Log-Skalierung der Farbe
  'xlabel', '#AUTO', ... % Mit Makro #AUTO wird die Beschriftung automatisch gesetzt
  'clabel', 'Performance criterion', ... % Beschriftung Farbskala
  'ylabel', 'Redundant coordinate in deg'); % Plot-Beschriftung
s.violation_markers = [{'qlim_hyp';'bx'}, {'jac_cond';'g*'}, {'ikjac_cond';'g^'}, ...
  {'coll_hyp'; 'co'}, {'instspc_hyp'; 'gv'}, {'invalid'; 'm+'}]; % NB-Marker
if nargin == 5
  for f = fields(s_in)'
    if isfield(s, f{1})
      s.(f{1}) = s_in.(f{1});
    else % Fall soll eigentlich nicht vorkommen. Daher Prüfung als zweites
      error('Feld %s aus s_in kann nicht übergeben werden', f{1});
    end
  end
end
if strcmp(s.reference, 'normalized') && strcmp(s.xlabel, '#AUTO')
  s.xlabel = 'Normalized trajectory progress';
elseif strcmp(s.reference, 'time') && strcmp(s.xlabel, '#AUTO')
  s.xlabel = 'Time in s';
end
assert(length(s.wn)==R.idx_ik_length.hnpos, 'Eingabe s.wn muss Länge von hnpos haben');
assert(length(s.abort_thresh_h)==R.idx_ik_length.hnpos, 'Eingabe s.abort_thresh_h muss Länge von hnpos haben');

if all(isnan(s.wn))
  s.wn(:) = 0;
  s.wn(R.idx_ikpos_hn.jac_cond) = 1; % nur Konditionszahl plotten
end
%% Farbkarte vorbereiten
if any(isnan(s.s_range_plot))
  % Keine Grenzen der Karte festgelegt. Alles zeichnen
  I_x = true(length(s_ref),1);
  s.s_range_plot = s_ref([1,end])';
else
  I_x = s_ref>=s.s_range_plot(1) & s_ref<=s.s_range_plot(2);
end
if any(isnan(s.phiz_range_plot))
  % Keine Grenzen der Karte festgelegt. Alles zeichnen
  I_y = true(length(phiz_range),1);
else
  I_y = phiz_range>=s.phiz_range_plot(1) & phiz_range<=s.phiz_range_plot(2);
end
[X_ext, Y_ext] = meshgrid(s_ref(I_x), 180/pi*phiz_range(I_y));
CC_ext = zeros(size(X_ext));
wn_plot = s.wn;
for iii = 1:length(wn_plot)
  if wn_plot(iii) == 0, continue; end
  CC_ext = CC_ext + wn_plot(iii) * H_all(I_x,I_y,iii)';
end
if all(CC_ext(:)==0 | isnan(CC_ext(:))) 
  CC_ext = H_all(I_x,I_y,end)'; % Nur Konditionszahl (direkt)
  title('Farbe nur illustrativ cond(J). Alle h=0');
end
%% Begrenzung der Farb-Werte berechnen
if s.PM_limit
  % Begrenze die Farb-Werte im Plot. Logik für Farben noch unfertig
  % Der Farbbereich für die interessanten Werte wird besser ausgenutzt.
  % Benutze Abbruch-Schwelle als Kriterium
  colorlimit = min(s.abort_thresh_h(s.abort_thresh_h<inf));
  if isempty(colorlimit)
    colorlimit = 1e3;
  end
  % Sättige alle Werte oberhalb einer Grenze
  condsat_limit = min(colorlimit-10, 1e4);
  assert(colorlimit > condsat_limit, 'colorlimit muss größer als condsat_limit sein');
  % Begrenze den Farbraum. Bezieht sich auf Werte oberhalb der Sättigung.
  % Diese werden unten logarithmisch behandelt.
  condsat_limit_rel = min(CC_ext(:)) + condsat_limit;
  if isinf(condsat_limit_rel)
    condsat_limit_rel = condsat_limit;
  end
  colorlimit_rel = min(CC_ext(:)) + colorlimit;
  if isinf(colorlimit_rel)
    colorlimit_rel = colorlimit;
  end
  assert(colorlimit_rel > condsat_limit_rel, 'colorlimit_rel muss größer als condsat_limit_rel sein');
  % Begrenze die Farbwerte (siehe oben, Beschreibung von colorlimit)
  I_colorlim = CC_ext>=colorlimit_rel;
  CC_ext(I_colorlim) = colorlimit_rel;
  I_exc = CC_ext >= condsat_limit_rel;
  CC_ext(I_exc) = condsat_limit_rel+10*log10(CC_ext(I_exc)/condsat_limit_rel);
else
  I_exc = false(size(CC_ext));
  I_colorlim = false(size(CC_ext));
  condsat_limit_rel = NaN;
  colorlimit_rel = NaN;
end
%% Zeichne die Farbkarte
Hdl_all.surf = surf(X_ext,Y_ext,zeros(size(X_ext)),CC_ext, 'EdgeColor', 'none');
xlim(minmax2(s_ref(I_x)'));
ylim(180/pi*minmax2(phiz_range(I_y)'));
colors_map = flipud(hot(1024)); % white to dark red.
% Falls starke Singularitäten oder Grenzverletzungen vorliegen, 
% wird diesdurch eine neue Farbe (Schwarz) hervorgehoben. Die Farbe
%  ist nicht so wichtig. Es werden noch Marker darüber gezeichnet.
if any(I_colorlim(:))
  numcolors_sat = 1; % add black for worst value. Do not show magenta in legend (1 value not visible)
  colors_map = [colors_map; repmat([0 0 0]/255, numcolors_sat,1)];
end
colormap(colors_map);
if max(CC_ext(:)) / min(CC_ext(:)) > 100 % Log-Skalierung wenn Größenordnung stark unterschiedlich
  set(gca,'ColorScale','log');
end
Hdl_all.cb = colorbar();
%% Formatieren
ylabel(Hdl_all.cb, s.clabel, 'Rotation', 90, 'interpreter', 'none');
xlabel(s.xlabel);
ylabel(s.ylabel);
if s.log
  set(gca, 'ColorScale', 'log');
end
%% Replizieren der Daten des Bildes über berechneten Bereich hinaus.
% (Ist nicht ganz korrekt z.B. bei Gelenkwinkel-Kriterium)
if s.extend_map
  for ysign = [-1, +1]
    Y_ext2 = Y_ext + ysign*360;
    % Wähle nur die Indizes aus, die nicht mit den bisherigen Daten
    % überlappen. Alle Spalten von Y_ext sind identisch (grid)
    if ysign < 0
      Iy2 = Y_ext2(:,1)<=min(Y_ext(:,1));
    else
      Iy2 = Y_ext2(:,1)>=max(Y_ext(:,1));
    end
    % Erneuter Plot (Transparent)
    surf(X_ext(Iy2,:),Y_ext2(Iy2,:),zeros(size(X_ext(Iy2,:))), ...
      CC_ext(Iy2,:), 'EdgeColor', 'none', 'FaceAlpha', 0.5);
  end
end

%% Zusätzliche Marker für Nebenbedingungsverletzungen setzen
Hdl_all.VM = NaN(6,1);
for kk = 1:size(s.violation_markers,2)
  % Bestimme Indizes für bestimmte Sonderfälle, wie Gelenküberschreitung,
  % Singularität, Kollision, Bauraumverletzung.
  % Mit den Farben sind diese Bereiche nicht eindeutig zu kennzeichnen, da
  % immer die summierte Zielfunktion gezeichnet wird
  if strcmp(s.violation_markers{1,kk}, 'invalid')
    I = isnan(H_all(I_x,I_y,1)');
  else
    Icrit = strcmp(fields(R.idx_ikpos_hn), s.violation_markers{1,kk});
    assert(sum(Icrit)==1, sprintf('Kriterium %s passt nicht', s.violation_markers{1,kk}));
    I = H_all(I_x,I_y,Icrit)' >= s.abort_thresh_h(Icrit);
  end
  if ~any(I(:))
    continue
  end
  x_i = X_ext(I);
  y_i = Y_ext(I);
  if any(s.markermindist) % Marker ausdünnen.
    II = select_plot_indices_downsample_nonuniform([x_i(:), y_i(:)], [], ...
      s.markermindist(1), s.markermindist(2));
  else % alle Marker nehmen
    II = true(length(x_i), 1);
  end
  Hdl_all.VM(kk) = plot(x_i(II), y_i(II), s.violation_markers{2,kk}, 'MarkerSize', 4);
end

PlotData = struct( 'I_exc', I_exc, 'I_colorlim', I_colorlim, ...
  'condsat_limit_rel', condsat_limit_rel, 'colorlimit_rel', colorlimit_rel);
