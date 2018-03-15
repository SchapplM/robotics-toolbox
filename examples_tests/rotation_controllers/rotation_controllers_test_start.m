% Regelung einer freien Masse im Raum mit einem kartesischen Regler
% 
% Fahre eine Solltrajektorie mit verschiedenen kartesischen Reglern ab.
% Testet die Regler aus lib_cartesian_controllers.mdl
% 
% Siehe: rigid_body_dynamics_test.m
% 
% Ergebnisse:
% |Dynamik und Regler für Translation und Rotation entkoppelt. Getrennte Betrachtung
% -Translation:
% |* Nur P-Regler: Dauerschwingung
% |* PD-Regler: Annäherung an Soll-Trajektorie von Anfangswerten an
% |
% -Rotation:
% |* Nur P-Regler: Dauerschwingung der Winkel, bei gegebenem Schwerpunkt
%    auch Dauerschwingung des betrachteten Punktes
% |* PD-Regler: Annäherung an Trajektorie

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-10
% (c) Institut für Regelungstechnik, Universität Hannover

clear
clc

% Auswahl der Trajektorie
% 1: Würfel (nur kartesische Trajektorie)
% 2: Schrauben-Bewegung (Rotations- und Translations-Trajektorie)
usr_TrajNr = 2;

%% Simuliertes dynamisches System
% Trägheitstensor um den Schwerpunkt im Körper-KS (B)
I_B_C = inertiavector2matrix([4, 8, 15,-1,-2,-0.5]);
if any(eig(I_B_C) < 0)
  error('Gewählter Trägheitstensor ist nicht positiv definit');
end
% Masse
m = 1.5;

% Beliebige Anfangsposition und -geschwindigkeit der _Basis_
r0 = [7;9;11];
rD0 = [-6;-2;8];

% beliebige Anfangs-Winkelgeschwindigkeit
omega0_t0 = [7;8;9];

% beliebige Anfangsorientierung
R_W_B_t0 = rotx(pi/3)*roty(pi/6)*rotz(pi/4);

% beliebige Schwerpunktskoordinaten (im Körper-KS B)
r_B_B_C = [.16, .23, .42]';

% Gravitation: Führt zu bleibender Abweichung in z-Achse
g = [0;0;-9.81];

%% Alt-Euler-Regler-Parameter
k_P_p_C1 = 10*ones(3,1);
k_P_r_C1 = 30*ones(3,1);
k_D_p_C1 = 10*ones(3,1);
k_D_r_C1 = 10*ones(3,1);

%% Quaternionen-Regler-Parameter
k_P_p_C2 = 10*ones(3,1);
k_P_r_C2 = 30*ones(3,1);
k_D_p_C2 = 10*ones(3,1);
k_D_r_C2 = 10*ones(3,1);

%% Trajektorien
% Für Position und Orientierung

if usr_TrajNr == 1
  
  % Den Punkt in einem Würfel bewegen
  dxyz = [0   0   0; ...
          0.5 0   0; ... % nach vorne
          0   0.8 0; ... % nach links
          -0.5 0  0; ... % nach hinten
          0  -0.8 0; ... % nach rechts
          0  0    0.7]; ... % nach oben
  dxyz = [dxyz; dxyz(1:end-1,:)];  % nochmal

  % Eckpunkte berechnen
  P_eck = NaN(size(dxyz,1), 3);
  P_eck(1,1:3) = r0;
  for i = 2:size(dxyz,1)
    P_eck(i,:) = P_eck(i-1,:) + dxyz(i,:);
  end

  ori_quat = r2quat(eye(3)); % t2r(T_t0)

  % Eckpunkte mit Trajektorien verbinden
  vmax=0.5;
  T2=500e-3;
  T3=100e-3;
  T_Abt=1e-3;
  T_pause = 0.1;
  [R,RD,RDD,t] = traj_trapez2_multipoint(P_eck, ...
    vmax, T2, T3, T_Abt, T_pause);

  nt = length(t);
  XQ_soll = [R, repmat(ori_quat, nt,1)];
  XD_soll = [RD, zeros(nt,3)];
  XDD_soll = [RDD, zeros(nt,3)];
  
elseif usr_TrajNr == 2
  % Trajektorie für ein Zwischen-Koordinatensystem D (zwischen Welt W und
  % Basis B)
  % Siehe cone_rotation_trajectory_example.m
  traj_settings = struct( ...
    'delta_theta', 30*pi/180, ...
    'arclength', 2*pi*5, ... % 5 Umdrehungen
    'cone_axis', [0;0;1], ...
    'start_axis', [1;0;0], ...
    'omega_max', [0.5 1 0.3],...
    'T2', ones(1,3)*50e-3, ...
    'T3', ones(1,3)*5e-3, ...
    'Ts', 1e-3, ...
    'T_pause_hin', 0.2, ...
    'T_pause_rueck', 0.3);
  traj_W_D_rot = traj_rot_cone_angaxis(traj_settings);
  % Verschiebung des Ursprungs des Basis-KS vom Zwischen-KS D aus gesehen
  r_D_D_B = [0;0;3];
  % Das KS B ist parallel zum KS D (aber verschoben)
  R_D_B = eye(3);
  omega_D_D_B = zeros(3,1);
  
  % Das KS D startet im KS W und bewegt sich nach oben (während es sich
  % verdreht): Translations-Trajektorie
  [traj_W_D_R,TrajD_RD,TrajD_RDD,TrajD_t] = traj_trapez2_multipoint([0,0,0; 0,0,100], ...
    0.2, 0.05, 0.01, 1e-3, 0);
  traj_W_D_transl = struct('t', TrajD_t, 'r', traj_W_D_R, 'v', TrajD_RD);
  
  t = traj_W_D_rot.t;
  nt = length(t);
  % Trajektorie der Basis
  XQ_soll = NaN(nt,7);
  XD_soll = NaN(nt,6);
  XDD_soll = NaN(nt,6);
  for i = 1:nt
    r_W_W_D_i = traj_W_D_transl.r(i,:)';
    rD_W_W_D_i = traj_W_D_transl.v(i,:)'; % Verschiebung des Ursprungs O_D
    
    R_W_D_i = traj_W_D_rot.R(:,:,i); % Drehung des Koordinatensystems D
    r_W_D_B_i = R_W_D_i * r_D_D_B;
    r_W_W_B_i = r_W_W_D_i + r_W_D_B_i; % Position des Ursprungs O_B
    XQ_soll(i,1:3) = r_W_W_B_i;
    XQ_soll(i,4:7) = r2quat(R_W_D_i*R_D_B);
    
    omega_W_W_D_i = traj_W_D_rot.omega(i,:)';
    omega_W_W_B_i = omega_W_W_D_i + R_W_D_i*omega_D_D_B;
    rD_W_W_B_i = rD_W_W_D_i + cross(omega_W_W_B_i, r_W_D_B_i);
    
    XD_soll(i,1:3) = rD_W_W_B_i;
    XD_soll(i,4:6) = omega_W_W_D_i;
  end
else
  error('Benutzer-Trajektorie nicht definiert');
end

simin_xq_d = struct('time', t, ...
    'signals', struct('values', XQ_soll, 'dimensions', 7), ...
    'name', 'xq_soll');
simin_xD_d = struct('time', t, ...
    'signals', struct('values', XD_soll, 'dimensions', 6), ...
    'name', 'xD_soll');

%% Simulation starten
t_End = t(end)+1;
sl_Modellname = 'rotation_controllers_test';
simOut = sim(sl_Modellname, 'StopTime', sprintf('%d', t_End), ...
  'SimulationMode', 'normal'); % normal
sl = get_simulink_outputs(simOut, sl_Modellname);

%% Nachbearbeitung
sl.rpy_ctrlEul = NaN(length(sl.t), 3);
sl.rpy_ctrlquat = NaN(length(sl.t), 3);
sl.R_ctrlEul = NaN(3,3,length(sl.t));
sl.R_ctrlquat = NaN(3,3,length(sl.t));
for i = 1:length(sl.t)
  sl.rpy_ctrlEul(i,:) = r2rpy( quat2r(sl.xq_ctrlEul(i,4:7)') );
  sl.rpy_ctrlquat(i,:) = r2rpy( quat2r(sl.xq_ctrlquat(i,4:7)') );
  
  sl.R_ctrlEul(:,:,i) = quat2r_mex(sl.xq_ctrlEul(i,4:7)');
  sl.R_ctrlquat(:,:,i) = quat2r_mex(sl.xq_ctrlquat(i,4:7)');
end

R_soll = NaN(3,3,nt);
RPY_soll = NaN(nt,3);
for i = 1:nt
  RPY_soll(i,:) = r2rpy( quat2r( XQ_soll(i,4:7)' ) );
  R_soll(:,:,i) = quat2r_mex(XQ_soll(i,4:7)');
end
%% Auswerten
% Position
figure(1);
set(1,'Name','p','NumberTitle','off');
clf;
for i = 1:3
  subplot(3,1,i);
  hold all;
  plot(sl.t, sl.xq_ctrlEul(:,i));
  plot(sl.t, sl.xq_ctrlquat(:,i));
  plot(t, XQ_soll(:,i), '--');
  legend({'Sim-CtrlEuler', 'Sim-CtrlQuat', 'Soll'});
  ylabel(sprintf('Position %s', char(119+i)));
  grid on;
end

% Geschwindigkeit (transl)
figure(2);
set(2,'Name','v','NumberTitle','off');
clf;
for i = 1:3
  subplot(3,1,i);
  hold all;
  plot(sl.t, sl.xD_ctrlEul(:,i));
  plot(sl.t, sl.xD_ctrlquat(:,i));
  plot(t, XD_soll(:,i), '--');
  legend({'Sim-CtrlEuler', 'Sim-CtrlQuat', 'Soll'});
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
  plot(sl.t, sl.rpy_ctrlEul(:,i));
  plot(sl.t, sl.rpy_ctrlquat(:,i));
  plot(t, RPY_soll(:,i), '--');
  legend({'Sim-CtrlEuler', 'Sim-CtrlQuat', 'Soll'});
  ylabel(sprintf('RPY-Winkel %s', char(119+i)));
  grid on;
end

% Winkelgeschwindigkeit
figure(4);
set(4,'Name','omega','NumberTitle','off');
clf;
for i = 1:3
  subplot(3,1,i);
  hold all;
  plot(sl.t, sl.xD_ctrlEul(:,3+i));
  plot(sl.t, sl.xD_ctrlquat(:,3+i));
  plot(t, XD_soll(:,3+i), '--');
  legend({'Sim-CtrlEuler', 'Sim-CtrlQuat', 'Soll'});
  ylabel(sprintf('Winkelgeschw. %s', char(119+i)));
  grid on;
end

% Beschleunigung (transl)
figure(5);
set(5,'Name','a','NumberTitle','off');
clf;
for i = 1:3
  subplot(3,1,i);
  hold all;
  plot(sl.t, sl.xDD_ctrlEul(:,i));
  plot(sl.t, sl.xDD_ctrlquat(:,i));
  plot(t, XDD_soll(:,i), '--');
  legend({'Sim-CtrlEuler', 'Sim-CtrlQuat', 'Soll'});
  ylabel(sprintf('Beschl. %s', char(119+i)));
  grid on;
end


% Winkelbeschleunigung
figure(6);
set(6,'Name','omegaD','NumberTitle','off');
clf;
for i = 1:3
  subplot(3,1,i);
  hold all;
  plot(sl.t, sl.xDD_ctrlEul(:,3+i));
  plot(sl.t, sl.xDD_ctrlquat(:,3+i));
  plot(t, XD_soll(:,3+i), '--');
  legend({'Sim-CtrlEuler', 'Sim-CtrlQuat', 'Soll'});
  ylabel(sprintf('Winkelbeschl. %s', char(119+i)));
  grid on;
end

% Kraft/Moment
figure(7);
set(7,'Name','F','NumberTitle','off');
clf;
for i = 1:3
  for j = 1:2
    subplot(3,2,sprc2no(3,2,i,j));
    hold all;
    plot(sl.t, sl.F_ctrlEul(:,i+3*(j-1)));
    plot(sl.t, sl.F_ctrlquat(:,i+3*(j-1)));
    legend({'Sim-CtrlEuler', 'Sim-CtrlQuat'});
    if j == 1
      ylabel(sprintf('Kraft %s', char(119+i)));
    else
      ylabel(sprintf('Moment %s', char(119+i)));
    end
    grid on;
  end
end


% Orientierung (Quaternion)
figure(8);clf;
set(8,'Name','Ori_Quat','NumberTitle','off');
for i = 1:4
  subplot(2,2,i);
  hold all;
  plot(sl.t, sl.xq_ctrlEul(:,3+i));
  plot(sl.t, sl.xq_ctrlquat(:,3+i));
  plot(t, XQ_soll(:,3+i), '--');
  legend({'Sim-CtrlEuler', 'Sim-CtrlQuat', 'Soll'});
  ylabel(sprintf('Quaternion Element %d', i));
  grid on;
end

% 3D-Plot der Trajektorie
figure(9);clf;
set(9,'Name','Traj_3d','NumberTitle','off');
hold all;
plot3(sl.xq_ctrlEul(:,1), sl.xq_ctrlEul(:,2), sl.xq_ctrlEul(:,3));
plot3(sl.xq_ctrlquat(:,1), sl.xq_ctrlquat(:,2), sl.xq_ctrlquat(:,3));
plot3(XQ_soll(:,1), XQ_soll(:,2), XQ_soll(:,3), '--');
legend({'Sim-CtrlEuler', 'Sim-CtrlQuat', 'Soll'});
grid on;


% Orientierung (RotMat)
figure(10);
set(10,'Name','Ori_RotMat','NumberTitle','off');
clf;
for i = 1:3
  for j = 1:3
    subplot(3,3,sprc2no(3,3,i,j));
    hold all;
    plot(sl.t, squeeze(sl.R_ctrlEul(i,j,:)));
    plot(sl.t, squeeze(sl.R_ctrlquat(i,j,:)));
    plot(t, squeeze(R_soll(i,j,:)), '--');
    legend({'Sim-CtrlEuler', 'Sim-CtrlQuat', 'Soll'});
    ylabel(sprintf('R %d,%d', i,j));
    grid on;
  end
end

dockall