% Simulation zweier Punktmassen im Raum verbunden mit einer 3D-Feder
% Dient als Testmodell für 3D-Federn (Energiekonsistenz, Simulink-Modellierung)
% 
% Startet Simulink-Modell: rotation_controllers_energy_test.mdl
% 
% Aufbau/mechanisches Modell:
% * Jede Masse hat ein Basis-KS (B1/B2) und einen Schwerpunkt (C1,C2)
% * Die Massen sind an den Punkten F1/F2 miteinander über die Feder verbunden
% * Die Feder wird durch unterschiedliche kartesische Controller simuliert
% 
% Testfälle:
% [x] Steifigkeit und Dämpfung Null (ohne Kopplung): Gesamtenergie bleibt konstant
% [x] Nur translatorische Steifigkeit, Keine Dämpfung: Gesamtenergie bleibt konstant
% [x] Nur translatorische Steifigkeit und Dämpfung: Gesamtenergie monoton fallend
% [x] Nur rotatorische Steifigkeit, Keine Dämpfung: Gesamtenergie bleibt konstant
%     * Der Rotationsfehler darf nicht exakt pi/2 werden (wegen
%       Singularität) und pi nicht übersteigen (wegen Energiekonsistenz)! 
% [x] Nur rotatorische Steifigkeit und Dämpfung: Gesamtenergie monoton fallend
% [x] Beliebige Steifigkeit, Keine Dämpfung: Gesamtenergie bleibt konstant
% [x] Beliebige Steifigkeit und Dämpfung: Gesamtenergie monoton fallend
% 
% Auswahl der Feder-Modellierung mit Variable `CARTCTRL`:
% * Alternative Euler-Winkel (siehe [Natale2003]) -> obiges nur hierfür getestet
% * Quaternionen-Fehler (siehe [Natale2003])
% 
% Literatur
% [Natale2003] Interaction Control of Robot Manipulators:
%              Six-Degrees-of-Freedom Tasks. Springer, 2003

% Moritz Schappler, schappler@irt.uni-hannover.de, 2017-11
% (C) Institut für Regelungstechnik, Universität Hannover

clear
clc

mex_script_dependencies(mfilename('fullpath'))
%% Simuliertes dynamisches System
% Trägheitstensor um den Schwerpunkt im Körper-KS (B)
% Zum Testen der Rotation um nur eine Achse: Deviationsmomente Null setzen
I_B1_C1 = inertiavector2matrix([2, 3, 12,0,-3,-0.2]);
I_B2_C2 = inertiavector2matrix([4, 8, 15,-1,-2,-0.5]);
if any(eig(I_B2_C2) < 0) ||  any(eig(I_B1_C1) < 0)
  error('Gewählter Trägheitstensor ist nicht positiv definit');
end
% Masse
m1 = 1;
m2 = 1.5;

% Beliebige Anfangsposition und -geschwindigkeit der _Basis_
r_W_W_B1_t0 = [7;9;11];
r_W_W_B2_t0 = [7;9;11];
rD_W_W_B1_t0 = [3;1;-7];
rD_W_W_B2_t0 = [-6;-2;8];

% beliebige Anfangs-Winkelgeschwindigkeit (aber nicht zu groß, damit
% Winkelfehler in Feder pi/2 nicht übersteigt)
omega_W_B1_t0 = [1;5;5]*0.1;
omega_W_B2_t0 = [7;8;9]*0.1;

% beliebige Anfangsorientierung
R_W_B1_t0 = rotx(pi/3)*roty(pi/6)*rotz(pi/4);
R_W_B2_t0 = rotz(1*pi/180)*roty(2*pi/180)*rotz(2*pi/180);

% beliebige Schwerpunktskoordinaten (im Körper-KS B)
r_B1_B1_C1 = [.5, .5, .6]';
r_B2_B2_C2 = [.16, .23, .42]';

% Koordinaten zum Federfußpunkt (Annahme: Nicht im Schwerpunkt)
r_B1_B1_F1 = [0.15, -0.2, 0.1]';
r_B2_B2_F2 = [0.44, 0.12, -0.08]';

% Gravitation
g_W = [0;0;-9.81];

r_B1_C1_F1 = -r_B1_B1_C1+r_B1_B1_F1;
r_B2_C2_F2 = -r_B2_B2_C2+r_B2_B2_F2;

% Anfangsgeschwindigkeit der Basis so, dass Schwerpunkt keine
% Geschwindigkeit hat (zum Testen der reinen Rotation)
% rD_W_W_B1_t0 = -cross(omega_W_B1_t0, R_W_B1_t0*r_B1_B1_C1);
% rD_W_W_B2_t0 = -cross(omega_W_B2_t0, R_W_B2_t0*r_B2_B2_C2);
% rD_W_W_C1_t0_test = rD_W_W_B1_t0 + cross(omega_W_B1_t0, R_W_B1_t0*r_B1_B1_C1);
% rD_W_W_C2_t0_test = rD_W_W_B2_t0 + cross(omega_W_B2_t0, R_W_B2_t0*r_B2_B2_C2);

%% Auswahl der Feder
CART_CONTROLLER_EULER=Simulink.Variant('CARTCTRL==1');
CART_CONTROLLER_QUATERNION=Simulink.Variant('CARTCTRL==2');
CARTCTRL=1; % Quaternion-Controller

%% Orientierungsregler-Parameter
if CARTCTRL == 1 % Alt-Euler-Regler-Parameter
  k_P_p = 100*ones(3,1); % Steifigkeit Translatorisch (Welt-KS)
  k_P_r = 30*ones(3,1); % Steifigkeit Rotatorisch (entlang der mitgedrehten Euler-Winkel von KS1 zu KS2)
  k_D_p = 50*ones(3,1)*0;% Dämpfung Translatorisch (Welt-KS)
  k_D_r = 10*ones(3,1)*0; % Dämpfung rotatorisch (Welt-KS)
else % Quaternion-Regler-Parameter
  % TODO: Eigenschaften von oben noch nicht getestet
  k_P_p = 100*ones(3,1); % Steifigkeit Translatorisch (Welt-KS)
  k_P_r = 30*ones(3,1); % Steifigkeit Rotatorisch (Quaternion-Fehler um Drehachse)
  k_D_p = 50*ones(3,1)*0;% Dämpfung Translatorisch (Welt-KS)
  k_D_r = 20*ones(3,1)*0; % Dämpfung rotatorisch (Welt-KS)
end

%% Simulation starten
t_End = 10;
sl_Modellname = 'rotation_controllers_energy_test';
simOut = sim(sl_Modellname, 'StopTime', sprintf('%d', t_End), ...
  'SimulationMode', 'normal'); % normal
sl = get_simulink_outputs(simOut, sl_Modellname);

%% Nachbearbeitung
nt = length(sl.t);
E_F_transl = NaN(nt,1); % Energie in der Feder
E_F_rot = NaN(nt,1); % Energie in der Feder
% Unterschiedliche Winkeldarstellungen des Rotationsfehlers
R_B1_B2_ges = NaN(3,3,nt);
rpy_B1_B2_ges = NaN(nt,3);
angvec_B1_B2_ges = NaN(nt,4);
quat_B1_B2_ges = NaN(nt,4);
for i = 1:nt
  r_W_W_B1_i = sl.xq_Masse1(i,1:3)';
  r_W_W_B2_i = sl.xq_Masse2(i,1:3)';
  R_W_B1_i = quat2r_mex(sl.xq_Masse1(i,4:7));
  R_W_B2_i = quat2r_mex(sl.xq_Masse2(i,4:7));
  r_W_W_F1_i = r_W_W_B1_i + R_W_B1_i*r_B1_B1_F1;
  r_W_W_F2_i = r_W_W_B2_i + R_W_B2_i*r_B2_B2_F2;
  
  R_B1_B2_ges(:,:,i) = R_W_B1_i' * R_W_B2_i;
  rpy_B1_B2_ges(i,:) = r2rpy_mex(R_B1_B2_ges(:,:,i));
  quat_B1_B2_ges(i,:) = r2quat_mex(R_B1_B2_ges(:,:,i));
  try %#ok<TRYNC>
    % `try` notwendig, da nicht immer die Umwandlung funktioniert (Eigenwerte
    % der Rotationsmatrix stimmen nicht)
    [theta,n] = r2angvec_mex(R_B1_B2_ges(:,:,i));
    angvec_B1_B2_ges(i,:) = [theta;n(:)];
  end
  Delta_W_F_i = r_W_W_F2_i-r_W_W_F1_i;
  E_F_transl(i) = sum(0.5 * k_P_p .* Delta_W_F_i.^2);
  
  % Federenergie. Siehe Implementierung in MatlabFcn
  if CARTCTRL == 1 % Euler-RPY
    R_B1_B2 = R_W_B1_i' * R_W_B2_i;
    phi_B1_B2 = r2rpy(R_B1_B2)';
    T = angvelotrans_rpy(phi_B1_B2(1), phi_B1_B2(2), phi_B1_B2(3)); % [Natale2003], (2.34)
    Te = R_W_B1_i*T; % [Natale2003], (2.31)
    mu_ce = (Te') \ (k_P_r .* (phi_B1_B2)); % [Natale2003], (2.32)
    E_F_rot(i) = sum(0.5 * k_P_r .* phi_B1_B2.^2);
  elseif CARTCTRL == 2 % Quaternionen
    Q_d = Quaternion(sl.xq_Masse1(i,4:7));
    Q_e = Quaternion(sl.xq_Masse2(i,4:7));
    % Quaternion-Orientierungsfehler: [Natale2003], (2.53)
    Q_de = (Q_e.inv) * Q_d;
    q_de = double(Q_de)';
    % [Natale2003], (2.56)
    R_e = quat2r(sl.xq_Masse2(i,4:7));
    M_P = k_P_r .* (R_e * q_de(2:4));
    E_F_rot(i) = 0.5 * q_de(2:4)' * M_P; % Nicht validiert
  end
end


sl.rpy_Masse1 = NaN(length(sl.t), 3);
sl.rpy_Masse2 = NaN(length(sl.t), 3);
sl.R_Masse1 = NaN(3,3,length(sl.t));
sl.R_Masse2 = NaN(3,3,length(sl.t));
for i = 1:length(sl.t)
  sl.rpy_Masse1(i,:) = r2rpy( quat2r(sl.xq_Masse1(i,4:7)) );
  sl.rpy_Masse2(i,:) = r2rpy( quat2r(sl.xq_Masse2(i,4:7)) );
  
  sl.R_Masse1(:,:,i) = quat2r_mex(sl.xq_Masse1(i,4:7));
  sl.R_Masse2(:,:,i) = quat2r_mex(sl.xq_Masse2(i,4:7));
end

%% Auswerten
% Position
figure(1);clf;
set(1,'Name','r_B','NumberTitle','off');
clf;
for i = 1:3
  subplot(3,1,i);
  hold all;
  plot(sl.t, sl.xq_Masse1(:,i));
  plot(sl.t, sl.xq_Masse2(:,i));
  legend({'Masse1', 'Masse2'});
  ylabel(sprintf('Position %s', char(119+i)));
  grid on;
end

% Geschwindigkeit (transl)
figure(2);clf;
set(2,'Name','v_B','NumberTitle','off');
clf;
for i = 1:3
  subplot(3,1,i);
  hold all;
  plot(sl.t, sl.V_Masse1(:,i));
  plot(sl.t, sl.V_Masse2(:,i));
  legend({'Masse1', 'Masse2'});
  ylabel(sprintf('Geschw. %s', char(119+i)));
  grid on;
end

% Orientierung (RPY)
figure(3);
set(3,'Name','Ori_RPY','NumberTitle','off');
clf;
for i = 1:3
  subplot(3,1,i);
  hold all;
  plot(sl.t, 180/pi*sl.rpy_Masse1(:,i));
  plot(sl.t, 180/pi*sl.rpy_Masse2(:,i));
  legend({'Masse1', 'Masse2'});
  ylabel(sprintf('RPY-Winkel %s [deg]', char(119+i)));
  grid on;
end
linkxaxes

% Winkelgeschwindigkeit
figure(4);clf;
set(4,'Name','omega','NumberTitle','off');
clf;
for i = 1:3
  subplot(3,1,i);
  hold all;
  plot(sl.t, sl.V_Masse1(:,3+i));
  plot(sl.t, sl.V_Masse2(:,3+i));
  legend({'Masse1', 'Masse2'});
  ylabel(sprintf('Winkelgeschw. %s', char(119+i)));
  grid on;
end
linkxaxes

% Beschleunigung (transl)
figure(5);clf;
set(5,'Name','a_B','NumberTitle','off');
clf;
for i = 1:3
  subplot(3,1,i);
  hold all;
  plot(sl.t, sl.A_Masse1(:,i));
  plot(sl.t, sl.A_Masse2(:,i));
  plot(sl.t([1, end]), g_W(i)*[1;1], '--');
  legend({'Masse1', 'Masse2', 'g'});
  ylabel(sprintf('Beschl. %s', char(119+i)));
  grid on;
end
linkxaxes

% Winkelbeschleunigung
figure(6);clf;
set(6,'Name','omegaD','NumberTitle','off');
clf;
for i = 1:3
  subplot(3,1,i);
  hold all;
  plot(sl.t, sl.A_Masse1(:,3+i));
  plot(sl.t, sl.A_Masse2(:,3+i));
  legend({'Masse1', 'Masse2'});
  ylabel(sprintf('Winkelbeschl. %s', char(119+i)));
  grid on;
end
linkxaxes

% Kraft/Moment
figure(7);
set(7,'Name','F','NumberTitle','off');
clf;
for i = 1:3
  for j = 1:2
    subplot(3,2,sprc2no(3,2,i,j));
    hold all;
    plot(sl.t, sl.F(:,i+3*(j-1)));
    if j == 1
      ylabel(sprintf('Kraft %s', char(119+i)));
    else
      ylabel(sprintf('Moment %s', char(119+i)));
    end
    grid on;
  end
end
linkxaxes

% Orientierung (Quaternion)
figure(8);clf;
set(8,'Name','Ori_Quat','NumberTitle','off');
for i = 1:4
  subplot(2,2,i);
  hold all;
  plot(sl.t, sl.xq_Masse1(:,3+i));
  plot(sl.t, sl.xq_Masse2(:,3+i));
  legend({'Masse1', 'Masse2'});
  ylabel(sprintf('Quaternion Element %d', i));
  grid on;
end
linkxaxes

% 3D-Plot der Trajektorie
figure(9);clf;
set(9,'Name','Traj_B_3d','NumberTitle','off');
hold all;
plot3(sl.xq_Masse1(:,1), sl.xq_Masse1(:,2), sl.xq_Masse1(:,3));
plot3(sl.xq_Masse2(:,1), sl.xq_Masse2(:,2), sl.xq_Masse2(:,3));
legend({'Masse1', 'Masse2'});
xlabel('x');ylabel('y');zlabel('z');
grid on;view(3)

figure(17);clf;
set(17,'Name','Traj_F_3d','NumberTitle','off');
hold all;
plot3(sl.r_W_W_F1(:,1), sl.r_W_W_F1(:,2), sl.r_W_W_F1(:,3));
plot3(sl.r_W_W_F2(:,1), sl.r_W_W_F2(:,2), sl.r_W_W_F2(:,3));
legend({'Masse1', 'Masse2'});
xlabel('x');ylabel('y');zlabel('z');
grid on;view(3)

% Orientierung (RotMat)
figure(10);
set(10,'Name','Ori_RotMat','NumberTitle','off');
clf;
for i = 1:3
  for j = 1:3
    subplot(3,3,sprc2no(3,3,i,j));
    hold all;
    plot(sl.t, squeeze(sl.R_Masse1(i,j,:)));
    plot(sl.t, squeeze(sl.R_Masse2(i,j,:)));
    legend({'Masse1', 'Masse2'});
    ylabel(sprintf('R %d,%d', i,j));
    grid on;
  end
end
linkxaxes

% Energie
E_ges = sl.Energie_Masse1(:,4)+sl.Energie_Masse2(:,4)+E_F_transl+E_F_rot;
E_ges_transl = sl.Energie_Masse1(:,1)+sl.Energie_Masse1(:,3)+...
               sl.Energie_Masse2(:,1)+sl.Energie_Masse2(:,3)+sl.Energie_Feder(:,1);
E_ges_rot = sl.Energie_Masse1(:,2)+...
            sl.Energie_Masse2(:,2)+sl.Energie_Feder(:,2);
figure(11); clf;
set(11,'Name','Energie','NumberTitle','off');
subplot(4,1,1);hold on;
plot(sl.t, sl.Energie_Masse1);
legend({'Kin,Transl','Kin,Rot', 'Pot', 'Gesamt1'}); grid on;
subplot(4,1,2);hold on;
plot(sl.t, sl.Energie_Masse2);
legend({'Kin,Transl','Kin,Rot', 'Pot', 'Gesamt2'}); grid on;
subplot(4,1,3);hold on;
plot(sl.t, sl.Energie_Feder);
plot(sl.t, E_F_transl, '--');
plot(sl.t, E_F_rot, '--');
legend({'Transl', 'Rot', 'Gesamt Feder', 'transl,Test', 'rot,Test'}); grid on;
subplot(4,1,4);hold on;
plot(sl.t, E_ges);
plot(sl.t, E_ges_transl, '--');
plot(sl.t, E_ges_rot, '--');
legend({'Gesamt', 'Gesamt,Transl', 'Gesamt,Rot'}); grid on;
linkxaxes

% Debug: Schwerpunktskoordinaten
figure(12);clf;
set(12,'Name','rC, rDC','NumberTitle','off');
for i = 1:3
  subplot(3,2,sprc2no(3,2,i,1)); hold on;
  plot(sl.t, sl.r_W_W_C1(:,i));
  plot(sl.t, sl.r_W_W_C2(:,i));
  legend({'C1', 'C2'});
  ylabel(sprintf('r C %s', char(119+i))); grid on;
  
  subplot(3,2,sprc2no(3,2,i,2)); hold on;
  plot(sl.t, sl.rD_W_W_C1(:,i));
  plot(sl.t, sl.rD_W_W_C2(:,i));
  legend({'C1', 'C2'});
  ylabel(sprintf('rD C %s', char(119+i)));grid on;
end
linkxaxes

% Debug: Federfußpunktkoordinaten
figure(13);clf;
set(13,'Name','rF, rDF','NumberTitle','off');
for i = 1:3
  subplot(3,2,sprc2no(3,2,i,1)); hold on;
  plot(sl.t, sl.r_W_W_F1(:,i));
  plot(sl.t, sl.r_W_W_F2(:,i));
  legend({'F1', 'F2'});
  ylabel(sprintf('r F %s', char(119+i))); grid on;
  
  subplot(3,2,sprc2no(3,2,i,2)); hold on;
  plot(sl.t, sl.rD_W_W_F1(:,i));
  plot(sl.t, sl.rD_W_W_F2(:,i));
  legend({'F1', 'F2'});
  ylabel(sprintf('rD F %s', char(119+i)));grid on;
end
linkxaxes

% Debug: Energie-Analyse
E_steig = NaN(nt,1);
E_steig(1) = 0;
E_Max_ges = NaN(nt,1);
E_Min_ges = NaN(nt,1);
E_Max_i = E_ges(1);
E_Min_i = E_ges(1);
for i = 2:nt
  E_Max_i = max([E_ges(i),E_Max_i]);
  E_Min_i = min([E_ges(i),E_Min_i]);
  E_Max_ges(i) = E_Max_i;
  E_Min_ges(i) = E_Min_i;
  E_steig(i) = E_ges(i) - E_Min_i; % Differenz der aktuellen Energie zum bisherigen Energie-Tiefststand
end
figure(14);clf;
set(14,'Name','E:Analyse','NumberTitle','off');
subplot(2,1,1); hold on;
plot(sl.t, E_ges);
plot(sl.t, E_Min_ges);
plot(sl.t, E_Max_ges);
legend({'Ges', 'Min', 'Max'});
subplot(2,1,2);
plot(sl.t, E_steig);
title('Übersteigung der aktuellen Energie über das bisherige Minimum');
linkxaxes

fprintf('Energiedifferenz: %1.5e\n', E_ges(end)-E_ges(1));
fprintf('Maximale Energiesteigung: %1.5e\n', max(E_steig));

% Debug: Winkel-Fehler
figure(15);clf;
set(15,'Name','Delta R,RPY','NumberTitle','off');
for i = 1:3
  for j = 1:4
    subplot(3,4,sprc2no(3,4,i,j));
    if j < 4
      plot(sl.t, squeeze(R_B1_B2_ges(i,j,:)));
      ylabel(sprintf('R %d %d', i,j));
    else
      plot(sl.t, 180/pi*rpy_B1_B2_ges(:,i));
      ylabel(sprintf('RPY %d [deg]',i));
    end
    grid on
  end
end
linkxaxes

figure(16);clf;
set(16,'Name','Delta AngVec,Quat','NumberTitle','off');
for i = 1:2
  for j = 1:4
    subplot(2,4,sprc2no(2,4,i,j));hold on;
    if i == 1
      plot(sl.t, angvec_B1_B2_ges(:,j));
      ylabel(sprintf('angvec %d', j));
    elseif i == 2
      % Quaternion-Darstellung
      plot(sl.t, quat_B1_B2_ges(:,j));

      % Quaternion-Darstellung ausgerechnet aus Achse-Winkel-Darstellung
      if j == 1
        y = cos(angvec_B1_B2_ges(:,1)/2);
      else
        y = angvec_B1_B2_ges(:,j);
        y = y .* sin(angvec_B1_B2_ges(:,1)/2);
      end
      plot(sl.t, y, '--');
      legend({'Quat', 'Quat,test'});
      ylabel(sprintf('quat %d', j));
    end
    grid on
  end
end
linkxaxes

%% Ende
return
dockall

%% Debug
t_i = 0;
i = 1;
f_i = sl.F(i,1:3)';
m_i = sl.F(i,4:6)';
% Rechne Drallsatz nach
R_W_B1 = sl.R_Masse1(:,:,i);
r_W_F1 = sl.r_W_W_F1(i,:)';
r_W_C1 = sl.r_W_W_C1(i,:)';
r_W_C1_F1 = -r_W_C1+r_W_F1;
I_W_C1 = R_W_B1 * I_B1_C1 * R_W_B1';
w_i = sl.V_Masse1(i,4:6)';
wD_i = sl.A_Masse1(i,4:6)';
wD_test = I_W_C1 \ (m_i - skew(w_i)*I_W_C1*w_i + cross(r_W_C1_F1, f_i));
if any(abs(wD_i-wD_test) > 1e-10)
  error('Simulierte und nachgerechnete Drehbeschleunigung stimmen nicht überein');
end