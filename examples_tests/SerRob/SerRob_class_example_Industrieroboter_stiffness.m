% Steifigkeitsmodell für serielle Roboter mit Industrieroboter-Bsp testen

% Masterarbeit Yuqi ZHAO, zhaoyuqi.nicolas@gmail.com, 2019-12
% Betreuer: Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc

%% Init
if isempty(which('serroblib_path_init.m'))
  warning('Repo mit Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
end

% Typ des seriellen Roboters auswählen (Industrieroboter KR 30-3)
SName='S6RRRRRR10V2';
% Modellparameter auswählen (hinterlegt aus Datenblatt)
RName='S6RRRRRR10V2_KUKA1';

% Instanz der Roboterklasse erstellen
RS = serroblib_create_robot_class(SName, RName);
% Test-Einstellungen generieren
TSS = RS.gen_testsettings(true, false);
RS.I_EE = [1 1 1 1 1 1];
RS.update_EE([], [0;pi;0]); % EE drehen, damit Grundstellung nicht Euler-Singularität ist
% Funktionen kompilieren
RS.fill_fcn_handles(true, true);

%% Kartesische Trajektorie
% Fläche durch Spirale rastern: Trajektorie erstellen
q0 = RS.qref+[0;-25;-50;0;-50;0]*pi/180;
T_E = RS.fkineEE(q0);
x0 = [T_E(1:3,4); r2eul(T_E(1:3,1:3), RS.phiconv_W_E)];
% Start in Grundstellung
k=0; XE = x0';
% Fahrt in Startstellung
k=k+1; XE(k+1,:) = XE(k,:);
% Trajektorie
dl=0.05;
for k = 1:21
  k = k + 1; %#ok<FXSET>
  if rem(k,2) == 0
    XE(2*k-2,:) = XE(2*k-3,:)+[(k-1)*dl,0,0, 0,0,0];
    XE(2*k-1,:) = XE(2*k-2,:)+[0,-(k-1)*dl,0, 0,0,0];
  else
    XE(2*k-2,:) = XE(2*k-3,:)+[-(k-1)*dl,0,0, 0,0,0];
    XE(2*k-1,:) = XE(2*k-2,:)+[0,(k-1)*dl,0, 0,0,0];
  end
end
[X,XD,XDD,T] = traj_trapez2_multipoint(XE, 1, 1e-1, 1e-2, 1e-3, 0.25);

% Inverse Kinematik berechnen
[Q, QD, QDD, PHI] = RS.invkin2_traj(X,XD,XDD,T,q0,struct('n_max', 100, 'Phit_tol', 1e-6, 'Phir_tol', 1e-6 ));

% IK-Ergebnis testen
if any(abs(PHI(:)) > 1e-6)
  error('IK stimmt nicht');
end
%% Steifigkeitsmodell: Parameter
d_seg = 0.20; % Durchmesser
e_seg = 0.02; % Wandstärle der Segmente
% Parameter in Robotermodell eintragen für neue Funktion
RS.DesPar.seg_par = repmat([e_seg, d_seg], RS.NL,1);

gW = 400; % Gesamtmasse des Roboters (für Skalierung der Gelenke).
% Masse gleichmäßig auf Segmente aufteilen
RS.update_dynpar1([NaN;repmat(gW/RS.NJ, RS.NJ,1)]);
RS.islegchain = false;
II = (1:30:size(Q,1))'; % gröbere Rasterung für Rechenzeit

%% Steifigkeit des Roboters für Fläche berechnen
Kx_ges = NaN(6,6,length(II));
KxJ_ges = NaN(6,6,length(II));
KxL_ges = NaN(6,6,length(II));
i = 0;
for s = II'
  i = i + 1;
  [Kx_ges(:,:,i),~,NxJ_i,NxL_ges_i] = RS.stiffness(Q(s,:)');
  KxJ_ges(:,:,i) = inv(NxJ_i);
  KxL_ges(:,:,i) = inv(NxL_ges_i);
end

%% Ergebnisse zeichnen
for k = 1:3
  figure(3+k);clf;hold on; view(3);
  % Auswahl der Daten für Steifigkeits-Bild
  Elem = squeeze(Kx_ges(k,k,:));
  [xi,yi] = meshgrid(X(II,1), X(II,2));
  Elemi = griddata(X(II,1),X(II,2),Elem,xi,yi,'natural');
  surf(xi,yi,1e-6*Elemi,'FaceAlpha',0.5,'EdgeColor','none');
  xlabel('x in m');
  ylabel('y in m');
  zlabel(sprintf('K%s%s in N/µm', char(119+k), char(119+k)));
  title(sprintf('Translationsteifigkeit entlang der %s-Achse', char(119+k)));
  grid on;
end

% Zeichne den Roboter
s_plot = struct( 'ks', [1:RS.NJ, RS.NJ+2], 'straight', 1, 'mode', 4);
figure(8);clf;
hold on;
grid on;
xlabel('x in m');
ylabel('y in m');
zlabel('z in m');
view(3);
RS.plot( q0, s_plot );
title(sprintf('Roboter mit Steifigkeits-Trajektorie'));
plot3(X(:,1), X(:,2), X(:,3));

%% Debug-Ergebnisse zeichnen
for k = 1:3
  figure(13+k);clf;hold on; view(3);
  % Auswahl der Daten für Steifigkeits-Bild
  Elem = squeeze(KxJ_ges(k,k,:));
  [xi,yi] = meshgrid(X(II,1), X(II,2));
  Elemi = griddata(X(II,1),X(II,2),Elem,xi,yi,'natural');
  surf(xi,yi,1e-6*Elemi,'FaceAlpha',0.5,'EdgeColor','none');
  xlabel('x in m');
  ylabel('y in m');
  zlabel(sprintf('K%s%s in N/µm', char(119+k), char(119+k)));
  title(sprintf('Translationsteifigkeit der Gelenke (%s-Achse)', char(119+k)));
  grid on;
  
  figure(23+k);clf;hold on; view(3);
  % Auswahl der Daten für Steifigkeits-Bild
  Elem = squeeze(KxL_ges(k,k,:));
  [xi,yi] = meshgrid(X(II,1), X(II,2));
  Elemi = griddata(X(II,1),X(II,2),Elem,xi,yi,'natural');
  surf(xi,yi,1e-6*Elemi,'FaceAlpha',0.5,'EdgeColor','none');
  xlabel('x in m');
  ylabel('y in m');
  zlabel(sprintf('K%s%s in N/µm', char(119+k), char(119+k)));
  title(sprintf('Translationsteifigkeit der Segmente (%s-Achse)', char(119+k)));
  grid on;
end