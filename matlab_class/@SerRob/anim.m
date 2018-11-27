% Animiere den Roboter über eine gegebenene Bahn im Gelenkraum
%
% Eingabe:
% Q: MxN
%   Matrix mit M Gelenkwinkelstellungen der N Gelenkwinkel.
% s_anim:
%   Struktur für Einstellungen zur Animation mit Feldern
%   * gif_name: Pfad zu einer GIF-Datei zum speichern der Animation
% s_plot:
%   Struktur mit Einstellungen zum Plotten in der Animation mit
%   Feldern wie in SerRob.plot
% 
% Vorbereitung:
% Vor Starten der Animation muss ein Figure geöffnet werden und vorbereitet
% werden mit passenden Formateinstellungen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Universität Hannover

function anim(Rob, Q, s_anim, s_plot)

if isempty(Q)
  error('SerRob/anim: Trajektorie Q ist leer');
end

%% Initialisierung
s_std = struct('gif_name', []);

if nargin < 3
  % Keine Einstellungen übergeben. Standard-Einstellungen
  s_anim = s_std;
end
if nargin < 4
  % Keine Einstellungen üfür plot bergeben. Standard-Einstellungen
  s_plot = struct('info', 'empty plot settings');
end

% Prüfe Felder der Einstellungs-Struktur
if ~isfield(s_anim, 'gif_name'),s_anim.gif_name = s_std.gif_name; end

drawnow;
pause(1);

%% Beginne Animation
% Prüfe Bereich des Plots
for i=1:size(Q,1)
  Tc = Rob.fkine(Q(i,:)');
  xminmax_i = minmax(squeeze(Tc(1,4,:))');
  yminmax_i = minmax(squeeze(Tc(2,4,:))');
  zminmax_i = minmax(squeeze(Tc(3,4,:))');
  if i == 1
    xminmax = xminmax_i;
    yminmax = yminmax_i;
    zminmax = zminmax_i;
  else
    xminmax = minmax([xminmax_i, xminmax]);
    yminmax = minmax([yminmax_i, yminmax]);
    zminmax = minmax([zminmax_i, zminmax]);
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
if all(zminmax == 0)
  zminmax = [-0.1, 0.1];
end
zminmax(1) = zminmax(1)-0.05*zw;
zminmax(2) = zminmax(2)+0.05*zw;

xlim(xminmax);ylim(yminmax);zlim(zminmax);
children_keeplist = get(gca, 'children');
for i=1:size(Q,1)

  if i > 1
%     clf;
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

%     xlim(xlim_save);
%     ylim(ylim_save);
%     zlim(zlim_save);
    xlim(xminmax);ylim(yminmax);zlim(zminmax);
    view(view1_save,view2_save);
  end
  q = Q(i,:)';

  Rob.plot(q,s_plot);
  
  xlim(xminmax);ylim(yminmax);zlim(zminmax);
  
  grid on;

  f=getframe(gcf);

  % Create a colormap for the first frame. For the rest of the frames,
  % use the same colormap
  if ~isempty(s_anim.gif_name)
    if i == 1
      % Initialisierung
      mov = uint8(zeros(size(f.cdata, 1), size(f.cdata, 2), 1, size(Q,1)));
      [mov(:,:,1, i), map] = rgb2ind(f.cdata, 256, 'nodither');
    else
      mov(:,:,1, i) = rgb2ind(f.cdata, map, 'nodither');
    end
  end
  if i == 1
%     xlim_save = xlim();
%     ylim_save = ylim();
%     zlim_save = zlim();
    [view1_save,view2_save] = view();
  end
end

%% Create animated GIF
if ~isempty(s_anim.gif_name)
  imwrite(mov,map,s_anim.gif_name, 'DelayTime', 0, 'LoopCount', inf)
end
 