% Beispielskript für die Benutzung der Roboterklasse SerRob anhand eines
% 7-Achsigen-Beispielroboters mit Modellparametern von Kuka LBR 4+
% 
% Ablauf:
% * Zeichne den Roboter als CAD-Modell

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-11
% (C) Institut für mechatronische Systeme, Universität Hannover

clear
clc

%% Init
if isempty(which('serroblib_path_init.m'))
  warning('Repo mit Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end

rob_path = fileparts(which('robotics_toolbox_path_init.m'));
resdir = fullfile(rob_path, 'examples_tests', 'results');

% Typ des seriellen Roboters auswählen (LBR-Typ)
SName='S7RRRRRRR1';
% Modellparameter auswählen (hinterlegt aus Datenblatt)
RName='S7RRRRRRR1_LWR4P';

% Instanz der Roboterklasse erstellen
serroblib_update_template_functions({SName}, false);
RS = serroblib_create_robot_class(SName, RName);

%% CAD-Modell plotten
s_plot = struct( 'ks', [1:RS.NJ, RS.NJ+2], 'mode', 2);
q = pi/180*[45, -60, 0, 60, 0, -60, 0]';
figure(2);clf;
hold on;
grid on;
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
view(3);
cadhdl=RS.plot( q, s_plot );

%% Bewegung visualisieren
% Beispielbewegung Gelenkraum
QE = [[  0,   0, 0,  0, 0,  0,  0]; ...
      [-45, -60, 0, 60, 0, -60, 0]] * pi/180;
[Q,QD,QDD,t] = traj_trapez2_multipoint(QE, 1, 1e-1, 1e-2, 1e-3, 0.25);

s_anim = struct( 'mp4_name', fullfile(resdir,'LBR4p_Traj.mp4')); % als mp4 speichern
figure(3);clf;
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
set(gcf, 'color','w'); % Wichtig für Zuschneiden Schneiden des Videos
view(3);
title('Beispiel-Trajektorie LBR4+');
RS.anim( Q(1:20:end,:), [], s_anim, s_plot);