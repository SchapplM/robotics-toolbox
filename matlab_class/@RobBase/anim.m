% Animiere den Roboter über eine gegebenene Bahn im Gelenkraum
%
% Eingabe:
% Q: MxN
%   Matrix mit M Gelenkwinkelstellungen der N Gelenkwinkel.
% X: Mx6
%   Matrix mit M Plattformkoordinaten (leer lassen bei SerRob)
% s_anim:
%   Struktur für Einstellungen zur Animation mit Feldern
%   * gif_name: Pfad zu einer GIF-Datei (Endung .gif) zum speichern der Animation
%   * avi_name: Pfad zu einer AVI-Videodatei. Das Video wird mit 30fps
%     gespeichert mit den Standard-Einstellungen des VideoWriter.
%   * mp4_name: Pfad zu mp4-Videodatei. Das Video wird zuerst mit
%     VideoWriter als avi gespeichert und dann automatisch komprimiert
%     Falls PNG-Einzelbilder erzeugt werden, wird das Video daraus erzeugt.
%   * png_name: Pfad zu PNG-Bilddatei (als Muster mit %05d im Dateinamen).
%     Die Einzelbilder des Videos werden darin gespeichert. Optional.
%   * resolution: Auflösung der PNG-Bilder in dpi. Hiermit können Videos mit
%     höherer Auflösung als der Bildschirmauflösung generiert werden
% s_plot:
%   Struktur mit Einstellungen zum Plotten in der Animation mit
%   Feldern wie in SerRob.plot
% 
% Vorbereitung:
% Vor Starten der Animation muss ein Figure geöffnet werden und vorbereitet
% werden mit passenden Formateinstellungen
% 
% Siehe auch: SerRob/anim.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-12
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

function anim(Rob, Q, X, s_anim, s_plot)

if isempty(Q)
  error('RobBase/anim: Trajektorie Q ist leer');
end
if isa(X, 'struct')
  error('Falsches Eingabeformat für Argument X');
end
if any(isnan(Q(1,:))) || ~isempty(X) && any(isnan(X(1,:)))
  warning('Ungültige Trajektorie. Erster Zeitschritt ist NaN. Abbruch.');
  return
end
%% Initialisierung
s_std = struct('gif_name', [], 'avi_name', [], 'mp4_name', [], 'png_name', [], 'tmpdir', []);

if nargin < 4
  % Keine Einstellungen übergeben. Standard-Einstellungen
  s_anim = s_std;
end
if nargin < 5
  % Keine Einstellungen für plot bergeben. Standard-Einstellungen
  s_plot = struct('info', 'empty plot settings');
end

% Prüfe Felder der Einstellungs-Struktur
if ~isfield(s_anim, 'gif_name')
  s_anim.gif_name = s_std.gif_name;
elseif ~isempty(s_anim.gif_name) && ~strcmp(s_anim.gif_name(end-2:end), 'gif')
  error('GIF-Datei %s hat falsche Endung', s_anim.gif_name);
end
if ~isfield(s_anim, 'avi_name')
  s_anim.avi_name = s_std.avi_name;
elseif ~isempty(s_anim.avi_name) && ~strcmp(s_anim.avi_name(end-2:end), 'avi')
  error('AVI-Datei %s hat falsche Endung', s_anim.avi_name);
end
if ~isfield(s_anim, 'mp4_name')
  s_anim.mp4_name = s_std.mp4_name;
elseif ~isempty(s_anim.mp4_name) && ~strcmp(s_anim.mp4_name(end-2:end), 'mp4')
  error('MP4-Datei %s hat falsche Endung', s_anim.mp4_name);
end
if ~isfield(s_anim, 'png_name')
  s_anim.png_name = s_std.png_name;
elseif ~isempty(s_anim.png_name) && (~contains(s_anim.png_name, '%') || ...
    ~strcmp(s_anim.mp4_name(end-2:end), 'png'))
  error('PNG-Dateimuster %s hat falsche Endung oder enthält kein %%d-Muster', s_anim.png_name);
end
if ~isfield(s_anim, 'tmpdir')
  s_anim.tmpdir = [];
end

if isempty(s_anim.avi_name) && ~isempty(s_anim.mp4_name)
  % Die AVI-Datei wird nur temporär erstellt und am Ende wieder gelöscht
  avi_only_temp = true;
  % Setze einen neuen Namen für die AVI-Datei
  [f,d] = fileparts(s_anim.mp4_name);
  if isempty(s_anim.tmpdir)
    % kein spezielles temporäres Verzeichnis gefordert. Lege am Ort der mp4 an
    s_anim.avi_name = fullfile(f, [d, '.avi']);
  else
    % Benutze vorgegebenes Verzeichnis (Platz sparen)
    s_anim.avi_name = fullfile(s_anim.tmpdir, [d, '.avi']);
  end
else
  % Beide Dateien sollen erhalten bleiben oder nur AVI gefordert.
  avi_only_temp = false;
end

if ~isempty(s_anim.avi_name)
  % Schalter dafür, dass mit Matlab-VideoWriter ein AVI erzeugt wird.
  video_in_frameloop = true;
else
  video_in_frameloop = false;
end

if isfield(s_anim, 'resolution')
  % PNG-Auflösung gegeben. Damit implizit Erstellung von PNG-Dateien
  % gefordert. Aus Matlab heraus wird kein Video erzeugt
  video_in_frameloop = false;
  % Belege die Einstellungsvariablen zu Speicherorten
  if isempty(s_anim.png_name)
    tdir = tmpDirFcn();
    s_anim.png_name = fullfile(tdir,'frame_%05d.png');
  end
end

drawnow;
pause(1);

%% Beginne Animation
% Kind-Objekte des Figures merken, die bis jetzt vorliegen (z.B.
% eingezeichnete Trajektorien). Diese sollen bei der Animation erhalten
% bleiben.
children_keeplist = get(gca, 'children');
% Nehme aktuelle Grenzen des Plots als Grundlage und erweitere die Grenzen
% schon jetzt, falls der Roboter sich heraus bewegt.
xyzminmax = [get(gca, 'xlim'); get(gca, 'ylim'); get(gca, 'zlim')];
% Zeichne den Roboter in der Startpose. Damit können basisfeste Objekte am
% Roboter, wie bspw. Führungsschienen von Schubgelenken bereits hier die
% Begrenzung der Achsen beeinflussen. Nehme nicht einfach direkt die
% Plot-Grenzen hiernach, da bei automatischer Einstellung der Grenzen diese
% oft zu groß sind.
if isa(Rob, 'ParRob'),     Rob.plot(Q(1,:)',X(1,:)',s_plot);
elseif isa(Rob, 'SerRob'), Rob.plot(Q(1,:)',s_plot);
else, error('Klasse nicht definiert');
end
baseobjectsxyz = repmat(Rob.T_W_0(1:3,4),1,2);
for c = get(gca, 'children')'
  % Größe von Basis-Objekten bestimmen
  if isfield(get(c), 'DisplayName') && ~isempty(get(c, 'DisplayName'))
    if contains(get(c, 'DisplayName'), 'Link_0')
      for ixyz = 1:3
        xdata = get(c, [char(119+ixyz),'Data']);
        baseobjectsxyz(ixyz,:) = minmax2([xdata(:)', baseobjectsxyz(ixyz,:)]);
      end
    end
  end
  % Objekt wieder löschen (Plot diente nur zur Bestimmung der Größe).
  % Bezieht sich nur auf die Objekte aus dem obigen Plot-Befehl
  inkeeplist = false;
  for ckl = children_keeplist'
    if c == ckl, inkeeplist = true; end
  end
  if inkeeplist == false, delete(c); end
end
xyzminmax = minmax2([xyzminmax, baseobjectsxyz]);

% Prüfe Bereich des Plots
for i=1:size(Q,1)
  if isa(Rob, 'ParRob')
    [~,Tc_W] = Rob.fkine(Q(i,:)', X(i,:)');
    xyzminmax_i = [minmax2(squeeze(Tc_W(1,4,:))');
                   minmax2(squeeze(Tc_W(2,4,:))');
                   minmax2(squeeze(Tc_W(3,4,:))')];
  elseif isa(Rob, 'SerRob')
    [~,Tc_W] = Rob.fkine(Q(i,:)');
    [~,Tc_WE] = Rob.fkineEE(Q(i,:)');
    xyzminmax_i = [minmax2([squeeze(Tc_W(1,4,:))',Tc_WE(1,4)]);
                   minmax2([squeeze(Tc_W(2,4,:))',Tc_WE(2,4)]);
                   minmax2([squeeze(Tc_W(3,4,:))',Tc_WE(3,4)])];
  else
    error('Klasse nicht definiert');
  end
  xyzminmax = minmax2([xyzminmax_i, xyzminmax]);
end
% Grenzen um 10% der Spannweite aufweiten (nur x und y)
xyzw = abs(xyzminmax(:,2)-xyzminmax(:,1));
for i_xyz = 1:2
  xyzminmax(i_xyz,1) = xyzminmax(i_xyz,1)-0.05*xyzw(i_xyz);
  xyzminmax(i_xyz,2) = xyzminmax(i_xyz,2)+0.05*xyzw(i_xyz);
end
xyzw = abs(xyzminmax(:,2)-xyzminmax(:,1)); % Spannweite hat sich geändert
% Grenzen größer Null setzen (gleich Null bei planaren Systemen)
for i_xyz = 1:3
  if xyzw(i_xyz) < 1e-3
    xyzminmax(i_xyz,:) = [-0.2, 0.2]+mean(xyzminmax(i_xyz,:));
  end
end
% Achsgrenzen setzen (nochmal mit 10% mehr Spannweite)
for i_xyz = 1:3
  xyzminmax(i_xyz,1) = xyzminmax(i_xyz,1)-0.05*xyzw(i_xyz);
  xyzminmax(i_xyz,2) = xyzminmax(i_xyz,2)+0.05*xyzw(i_xyz);
end

xlim(xyzminmax(1,:));ylim(xyzminmax(2,:));zlim(xyzminmax(3,:));
[view1_save,view2_save] = view();
if video_in_frameloop
  v = VideoWriter(s_anim.avi_name, 'Uncompressed AVI');
  open(v)
end
%% Trajektorie durchgehen und Roboter für jeden Zeitpunkt neu zeichnen
i_break = NaN;
for i=1:size(Q,1)
  % Plot-Elemente des vorherigen Zeitpunktes entfernen, nur zu behaltende
  % Elemente nicht löschen.
  for c = get(gca, 'children')'
    inkeeplist = false;
    for ckl = children_keeplist'
      if c == ckl
        inkeeplist = true;
      end
    end
    if inkeeplist == false
      delete(c);
    end
  end
  hold on;
  xlim(xyzminmax(1,:));ylim(xyzminmax(2,:));zlim(xyzminmax(3,:));
  view(view1_save,view2_save);
  q = Q(i,:)';
  if any(isnan(q))
    warning('Stoppe Animation bei Zeitschritt %d/%d. q wird NaN.', i, size(Q,1)); 
    i_break = i-1; break; 
  end
  if isa(Rob, 'ParRob')
    x = X(i,:)';
    Rob.plot(q,x,s_plot);
  elseif isa(Rob, 'SerRob')
    Rob.plot(q,s_plot);
  else
    error('Klasse nicht definiert');
  end
  % Zurücksetzen der Achsbegrenzungen auf den ersten Wert
  xlim(xyzminmax(1,:));ylim(xyzminmax(2,:));zlim(xyzminmax(3,:));
  
  grid on;
  if ~isempty(s_anim.gif_name) || video_in_frameloop
    f=getframe(gcf);
    if ~isempty(s_anim.avi_name)
      % Zuschneiden auf Mod32 für anschließende Kompression (egal ob hier
      % direkt nach mp4 komprimiert wird oder vielleicht erst später)
      res_tmp = size(f.cdata);
      res_crop = floor(res_tmp(1:2)/32)*32;
      crop_begin = ceil((res_tmp(1:2)-res_crop(1:2))/2);
      crop_end = floor((res_tmp(1:2)-res_crop(1:2))/2);
      f.cdata = f.cdata((1+crop_begin(1)):(res_tmp(1)-crop_end(1)), ...
                        (1+crop_begin(2)):(res_tmp(2)-crop_end(2)), :);
    end
    % Prüfe nochmal die Auflösung
    if i == 1 % Initialisierung:
      % Speichere Auflösung ab
      res_1 = size(f.cdata);
    else
      if any(size(f.cdata)~=res_1)
        warning(['Auflösung des aktuellen Bildes (%d x %d) passt nicht zu ', ...
          'der des ersten (%d x %d)'], f.cdata(1), f.cdata(2), res_1(1), res_1(1));
        % Wahrscheinlich Durch Benutzer händisch verändert oder Fehler
        i_break = i-1;
        break;
      end
    end
  end
  
  % Create a colormap for the first frame. For the rest of the frames,
  % use the same colormap
  if ~isempty(s_anim.gif_name) % schreibe GIF-Bild
    mov = uint8(zeros(size(f.cdata, 1), size(f.cdata, 2), 1, 1));
    [mov(:,:, 1, 1), map] = rgb2ind(f.cdata, 256, 'nodither');
    if i == 1 % Initialisierung
      imwrite(mov,map,s_anim.gif_name, 'DelayTime', 0, 'LoopCount', inf)
    else
      imwrite(mov, map, s_anim.gif_name, 'DelayTime', 0, 'WriteMode','append'); % Vermeide riesige temporäre Variablen
    end
  end
  if video_in_frameloop
     writeVideo(v,f); % Schreibe Frame des Videos
  end
  if isfield(s_anim, 'resolution')
    % Speichere Einzelbild als hochauflösendes PNG. Interpretation des
    % Backslash durch sprintf bei Windows-Pfaden vermeiden
    tmpimage_png = sprintf(strrep(s_anim.png_name,'\','\\'), i); % Nummer hochzählen
    print(tmpimage_png,'-dpng','-r300','-opengl');
%     % Befehl exportgraphics geht nicht (schneidet Ränder variabel ab)
%     % exportgraphics(gcf,fullfile(tdir,sprintf('frame_%05d.png',i)),'Resolution',300);
  end
  drawnow();
end
%% Create GIF and AVI files
if video_in_frameloop
   close(v);
end
%% Erzeuge AVI aus png-Einzelbildern
if isfield(s_anim, 'resolution') && ...
    (~isempty(s_anim.mp4_name) || ~isempty(s_anim.avi_name))
  % Speichere alle PNG-Dateien als Video: h264-Codec mit verlustfreier
  % Speicherung. Dateiformat .avi, damit das nachfolgende
  % Kompressionsskript damit direkt funktioniert.
  avsettings = '-c:v libx264 -qp 0';
  cmd = sprintf('ffmpeg -y -f image2 -r 30 -i "%s" %s "%s" -loglevel 0', ...
    s_anim.png_name, avsettings, s_anim.avi_name);
  res = system(cmd);
  if res == 0
    % Erfolgreich AVI erstellt. Lösche PNG-Einzelbilder
    d = dir(fullfile(fileparts(s_anim.png_name), '*.png'));
    for i = 1:length(d)
      delete(fullfile(d(i).folder, d(i).name));
    end
  else
    warning('Fehler beim Zusammenfassen der PNG-Dateien aus %s', fileparts(s_anim.png_name));
  end
end
%% Komprimiere die AVI-Datei
if ~isempty(s_anim.mp4_name)
  compress_video_file(s_anim.avi_name, avi_only_temp, 1);
  % Verschiebe die erzeugte mp4-Datei an den gewünschten Zielort
  [~,d] = fileparts(s_anim.avi_name);
  if ~isempty(s_anim.tmpdir)
    movefile(fullfile(s_anim.tmpdir, [d, '.mp4']), s_anim.mp4_name);
  end
end
