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
%% Initialisierung
s_std = struct('gif_name', [], 'avi_name', [], 'mp4_name', []);

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

if isempty(s_anim.avi_name) && ~isempty(s_anim.mp4_name)
  % Die AVI-Datei wird nur temporär erstellt und am Ende wieder gelöscht
  avi_only_temp = true;
  % Setze einen neuen Namen für die AVI-Datei
  [f,d] = fileparts(s_anim.mp4_name);
  s_anim.avi_name = fullfile(f, [d, '.avi']);
else
  % Beide Dateien sollen erhalten bleiben oder nur AVI gefordert.
  avi_only_temp = false;
end

drawnow;
pause(1);

%% Beginne Animation
% Prüfe Bereich des Plots
for i=1:size(Q,1)
  if isa(Rob, 'ParRob')
    [~,Tc_W] = Rob.fkine(Q(i,:)', X(i,:)');
    xminmax_i = minmax2(squeeze(Tc_W(1,4,:))');
    yminmax_i = minmax2(squeeze(Tc_W(2,4,:))');
    zminmax_i = minmax2(squeeze(Tc_W(3,4,:))');
  elseif isa(Rob, 'SerRob')
    [~,Tc_W] = Rob.fkine(Q(i,:)');
    [~,Tc_WE] = Rob.fkineEE(Q(i,:)');
    xminmax_i = minmax2([squeeze(Tc_W(1,4,:))',Tc_WE(1,4)]);
    yminmax_i = minmax2([squeeze(Tc_W(2,4,:))',Tc_WE(2,4)]);
    zminmax_i = minmax2([squeeze(Tc_W(3,4,:))',Tc_WE(3,4)]);
  else
    error('Klasse nicht definiert');
  end

  if i == 1
    xminmax = xminmax_i;
    yminmax = yminmax_i;
    zminmax = zminmax_i;
  else
    xminmax = minmax2([xminmax_i, xminmax]);
    yminmax = minmax2([yminmax_i, yminmax]);
    zminmax = minmax2([zminmax_i, zminmax]);
  end
end
% Grenzen um 10% der Spannweite aufweiten
xw = abs(xminmax(2)-xminmax(1));
xminmax(1) = xminmax(1)-0.05*xw;
xminmax(2) = xminmax(2)+0.05*xw;
yw = abs(yminmax(2)-yminmax(1));
yminmax(1) = yminmax(1)-0.05*yw;
yminmax(2) = yminmax(2)+0.05*yw;
zw = abs(zminmax(2)-zminmax(1));
% Grenzen größer Null setzen (gleich Null bei planaren Systemen)
if xw < 1e-3
  xminmax = [-0.2, 0.2]+mean(xminmax);
end
if yw < 1e-3
  yminmax = [-0.2, 0.2]+mean(yminmax);
end
if zw < 1e-3
  zminmax = [-0.2, 0.2]+mean(zminmax);
end
% Achsgrenzen setzen
xminmax(1) = xminmax(1)-0.05*xw;
xminmax(2) = xminmax(2)+0.05*xw;
yminmax(1) = yminmax(1)-0.05*yw;
yminmax(2) = yminmax(2)+0.05*yw;
zminmax(1) = zminmax(1)-0.05*zw;
zminmax(2) = zminmax(2)+0.05*zw;

xlim(xminmax);ylim(yminmax);zlim(zminmax);
% Kind-Objekte des Figures merken, die bis jetzt vorliegen (z.B.
% eingezeichnete Trajektorien). Diese sollen bei der Animation erhalten
% bleiben.
children_keeplist = get(gca, 'children');

if ~isempty(s_anim.avi_name)
  v = VideoWriter(s_anim.avi_name, 'Uncompressed AVI');
  open(v)
end
%% Trajektorie durchgehen und Roboter für jeden Zeitpunkt neu zeichnen
i_break = NaN;
for i=1:size(Q,1)
  % Plot-Elemente des vorherigen Zeitpunktes entfernen, nur zu behaltende
  % Elemente nicht löschen.
  if i > 1
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
    xlim(xminmax);ylim(yminmax);zlim(zminmax);
    view(view1_save,view2_save);
  end
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

  xlim(xminmax);ylim(yminmax);zlim(zminmax);
  
  grid on;
  if ~isempty(s_anim.gif_name) || ~isempty(s_anim.avi_name)
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
        warning('Auflösung des aktuellen Bildes (%d x %d) passt nicht zu der des ersten (%d x %d)', ...
          res_1(1), res_1(1), f.cdata(1), f.cdata(2));
        % Wahrscheinlich Durch Benutzer händisch verändert oder Fehler
        i_break = i-1;
        break;
      end
    end
  end
  
  % Create a colormap for the first frame. For the rest of the frames,
  % use the same colormap
  if ~isempty(s_anim.gif_name) % schreibe GIF-Bild
    if i == 1 % Initialisierung
      mov = uint8(zeros(size(f.cdata, 1), size(f.cdata, 2), 1, size(Q,1)));
      [mov(:,:,1, i), map] = rgb2ind(f.cdata, 256, 'nodither');
    else
      mov(:,:,1, i) = rgb2ind(f.cdata, map, 'nodither');
    end
  end
  if ~isempty(s_anim.avi_name)
     writeVideo(v,f); % Schreibe Frame des Videos
  end
  if i == 1
    [view1_save,view2_save] = view();
  end
  drawnow();
end
% Verkürzen des Videos bei Fehler
if ~isempty(s_anim.gif_name) && ~isnan(i_break)
  mov = mov(:,:,1,1:i_break);
end
%% Create GIF and AVI files
if ~isempty(s_anim.gif_name)
  imwrite(mov,map,s_anim.gif_name, 'DelayTime', 0, 'LoopCount', inf)
end
if ~isempty(s_anim.avi_name)
   close(v);
end
%% Komprimiere die AVI-Datei
if ~isempty(s_anim.mp4_name)
  compress_video_file(s_anim.avi_name, 1, avi_only_temp);
end
