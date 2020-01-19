% PKM-Steifigkeitsmodell mit 6UPS-Beispiel testen
% 
% Siehe auch: ParRob_class_example_6UPS.m

% Masterarbeit Yuqi ZHAO, zhaoyuqi.nicolas@gmail.com, 2019-12
% Betreuer: Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für mechatronische Systeme, Universität Hannover

clear
clc

%% Definitionen, Benutzereingaben
% Robotermodell entweder aus PKM-Bibliothek oder nur aus
% Seriell-Roboter-Bibliothek laden. Stellt keinen Unterschied dar.
use_parrob = false;

rob_path = fileparts(which('robotics_toolbox_path_init.m'));
respath = fullfile(rob_path, 'examples_tests', 'results');

%% Klasse für PKM erstellen (basierend auf serieller Beinkette)
if isempty(which('serroblib_path_init.m'))
  warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end
if ~use_parrob
  % Typ des seriellen Roboters auswählen (S6RRPRRRV3 = UPS)
  SName='S6RRPRRR14V3';
  % Instanz der Roboterklasse erstellen
  RS = serroblib_create_robot_class(SName);
  RS.fill_fcn_handles(true, true);
  % RS.mex_dep(true)
  RP = ParRob('P6RRPRRR14V3G1P1A1');
  RP.create_symmetric_robot(6, RS, 0.5, 0.2);
  RP.initialize();
  % Schubgelenke sind aktuiert
  I_qa = false(36,1);
  I_qa(3:6:36) = true;
  RP.update_actuation(I_qa);
  % Benutze PKM-Bibliothek für gespeicherte Funktionen
  if ~isempty(which('parroblib_path_init.m'))
    parroblib_addtopath({'P6RRPRRR14V3G1P1A1'});
  end
  RP.fill_fcn_handles();
end
%% Alternativ: Klasse für PKM erstellen (basierend auf PKM-Bibliothek)
if use_parrob
  if isempty(which('parroblib_path_init.m'))
    warning('Repo mit parallelen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
    return
  end
  RP = parroblib_create_robot_class('P6RRPRRR14V3G1P1A1', 0.5, 0.2);
  RP.fill_fcn_handles(true, true);
end

%% Grenzen für die Gelenkpositionen setzen
% Dadurch wird die Schrittweite bei der inversen Kinematik begrenzt (auf 5%
% der Spannbreite der Gelenkgrenzen) und die Konfiguration klappt nicht um.
for i = 1:RP.NLEG
  % Begrenze die Winkel der Kugel- und Kardangelenke auf +/- 360°
  RP.Leg(i).qlim = repmat([-2*pi, 2*pi], RP.Leg(i).NQJ, 1);
  % Begrenze die Länge der Schubgelenke
  RP.Leg(i).qlim(3,:) = [0.4, 0.7];
end
qlim_pkm = cat(1, RP.Leg.qlim);
%% Startpose bestimmen
% Mittelstellung im Arbeitsraum
X = [ [0;0;0.4]; [0;0;0]*pi/180 ];
for i = 1:10 % Mehrere Versuche für "gute" Pose
  % Startwerte für numerische IK
  [q0, Phis] = RP.invkin_ser(X, rand(36,1));
  q0(RP.I_qa) = 0.5; % mit Schubaktor größer Null anfangen (damit Konfiguration nicht umklappt)

  % Inverse Kinematik auf zwei Arten berechnen
  [q1, Phi] = RP.invkin1(X, q0);
  if any(abs(Phi) > 1e-8)
    error('Inverse Kinematik konnte in Startpose nicht berechnet werden');
  end
  if any(q1(RP.I_qa) < 0)
    warning('Start-Konfiguration ist umgeklappt mit Methode 1.');
  end

  [q, Phis] = RP.invkin_ser(X, q0);
  if any(abs(Phis) > 1e-6)
    error('Inverse Kinematik (für jedes Bein einzeln) konnte in Startpose nicht berechnet werden');
  end
  if any(q(RP.I_qa) < 0)
    warning('Versuch %d: Start-Konfiguration ist umgeklappt mit Methode Seriell. Erneuter Versuch.', i);
    if i == 10
      return
    else
      continue;
    end
  else
    break;
  end
end

%% Zwangsbedingungen in Startpose testen
Phi1=RP.constr1(q, X);
Phit1=RP.constr1_trans(q, X);
Phir1=RP.constr1_rot(q, X);
if any(abs(Phi1) > 1e-6)
  error('ZB in Startpose ungleich Null');
end

%% Gelenktypen eintragen (zur Erkennung auf Kugel-/Kardangelenk)
for i = 1:RP.NLEG
  RP.Leg(i).DesPar.joint_type = [2; 6; 5; 3; 6; 6];
end

%% Kartesische Trajektorie
% Fläche durch Spirale rastern: Trajektorie erstellen
X0 = [ [0;0;0.5]; [0;0;0]*pi/180 ];
dl=0.04; 
XL = X0';
r1=5*pi/180;
% Trajektorie
for k = 1:3%10
  k = k+1; %#ok<FXSET>
  if rem(k,2) == 0
    XL(2*k-2,:) = XL(2*k-3,:)+[[(k-1)*dl,0,0], [-(k-1)*r1,0,0]];
    XL(2*k-1,:) = XL(2*k-2,:)+[[0,-(k-1)*dl,0], [0,(k-1)*r1,0]];
  else
    XL(2*k-2,:) = XL(2*k-3,:)+[[-(k-1)*dl,0,0], [(k-1)*r1,0,0]];
    XL(2*k-1,:) = XL(2*k-2,:)+[[0,(k-1)*dl,0], [0,-(k-1)*r1,0]];
  end
end
[X_t,XD_t,XDD_t,t] = traj_trapez2_multipoint(XL, 1, 0.1, 0.01, 1e-3, 1e-1);

% Inverse Kinematik berechnen
q0 = q; % Lösung der IK von oben als Startwert
t0 = tic();
% IK-Einstellungen: Sehr lockere Toleranzen, damit es schneller geht
s = struct('Phit_tol', 1e-4, 'Phir_tol', 0.1*pi/180);
[q1, Phi_num1] = RP.invkin1(X_t(1,:)', q0, s);
if any(abs(Phi_num1) > 1e-3)
  error('IK konvergiert nicht');
end
fprintf('Inverse Kinematik für Trajektorie berechnen: %d Bahnpunkte\n', length(t));
[Q_t, ~, ~, Phi_t] = RP.invkin_traj(X_t, XD_t, XDD_t, t, q1, s);
if any(any(abs(Phi_t(:,RP.I_constr_t_red)) > s.Phit_tol)) || ...
   any(any(abs(Phi_t(:,RP.I_constr_r_red)) > s.Phir_tol))
   error('Fehler in Trajektorie zu groß. IK nicht berechenbar');
end
fprintf('%1.1fs nach Start. %d Punkte berechnet.\n', ...
  toc(t0), length(t));
save(fullfile(respath, 'ParRob_class_example_6UPS_stiffness_traj.mat'));
% Start hier: load(fullfile(respath,'ParRob_class_example_6UPS_stiffness_traj.mat'));

%% Steifigkeitsmodell: Parameter
d_seg = 0.050;
e_seg = 0.010;
m_sum = 18; % Masse einer Beinkette
% Parameter in Robotermodell eintragen für neue Funktion
for i = 1:RP.NLEG
  % Setze Schubgelenke als Hubzylinder
  RP.Leg(i).DesPar.joint_type(RP.I_qa((RP.I1J_LEG(i):RP.I2J_LEG(i)))) = 5;
  % Setze Segmente als Hohlzylinder
  RP.Leg(i).DesPar.seg_par = repmat([e_seg, d_seg], 7,1);
  % Gleichmäßige Verteilung der Masse
  RP.Leg(i).update_dynpar1([NaN;repmat(m_sum/(RP.NLEG*RP.Leg(i).NJ), RP.Leg(i).NJ,1)]);
  RP.Leg(i).islegchain = true;
end
RP.DesPar.platform_par(2) = 5e-3;
%% Steifigkeit des Roboters für Fläche berechnen
II = (1:10:size(Q_t,1))'; % gröbere Rasterung für Rechenzeit
Kx_ges = NaN(6,6,length(II)); % Summe
i = 0;
for s = II'
  i = i + 1;
  qs = Q_t(s,:)';
  Kx_ges(:,:,i)  = RP.stiffness(qs);
end

%% Ergebnisse zeichnen
for k = 1:3
  figure(3+k);clf;hold on; view(3);
  % Auswahl der Daten für Steifigkeits-Bild
  Elem = squeeze(Kx_ges(k,k,:));
  [xi,yi] = meshgrid(X_t(II,1), X_t(II,2));
  Elemi = griddata(X_t(II,1),X_t(II,2),Elem,xi,yi,'natural');
  surf(xi,yi,1e-6*Elemi,'FaceAlpha',0.5,'EdgeColor','none');
  xlabel('x in m');
  ylabel('y in m');
  zlabel(sprintf('K%s%s in N/µm', char(119+k), char(119+k)));
  title(sprintf('Translationsteifigkeit entlang der %s-Achse', char(119+k)));
  grid on; grid minor;
end


% Zeichne den Roboter
figure(8); clf; hold on; grid on;
xlabel('x in m');
ylabel('y in m');
zlabel('z in m'); view(3);
s_plot = struct( 'ks_legs', [], 'straight', 0, 'mode', 4);
RP.plot( q, X, s_plot );
title(sprintf('Roboter mit Steifigkeits-Trajektorie'));
plot3(X_t(:,1), X_t(:,2), X_t(:,3));
grid on; grid minor;
