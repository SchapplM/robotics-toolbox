% Roboterklasse für 6PUS-PKM testen
% TODO: Plot korrigieren

% Zilin He (Masterarbeit bei Moritz Schappler), 2019-06
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc

rob_path = fileparts(which('robotics_toolbox_path_init.m'));
respath = fullfile(rob_path, 'examples_tests', 'results');
%% Definitionen, Benutzereingaben

RS = serroblib_create_robot_class('S6PRRRRR6V2');
RS.fill_fcn_handles(false);
% ParRob-Klasse für PKM erstellen
if ~isempty(which('parroblib_path_init.m'))
  parroblib_addtopath({'P6PRRRRR6V2G8P1A1'});
end
RP = ParRob('P6PRRRRR6V2G8P1A1');
RP.create_symmetric_robot(6, RS, 1.0, 0.3);
RP.initialize();

% Basis KS einstellen: Gruppenweise für bessere Konditionierung der Jacobi
RP.align_base_coupling(8, [0.8;0.3;pi/3]);

% Plattform-Konfiguration verändern
% Mit einer Kreisförmigen Plattformkoppelpunktanordnung ist die PKM
% nicht singulär (Jacobi der direkten Kinematik), wenn die Basis
% Pyramidenförmig ist. Daher normale Kreis-Anordnung der Plattform
RP.align_platform_coupling(1, 0.2);

% Kinematikparameter einstellen
pkin_6_PUS = zeros(length(RP.Leg(1).pkin),1); % Namen, siehe RP.Leg(1).pkin_names
pkin_6_PUS(1) = 0.2; % a2
pkin_6_PUS(10) = -pi/2; % theta1: So drehen, dass a2 nach oben zeigt
pkin_6_PUS(3) = 0.5; % a4
pkin_6_PUS(4:5) = pi/2; % alpha2, alpha3  
for i = 1:RP.NLEG
  RP.Leg(i).update_mdh(pkin_6_PUS);
end

% EE-KS mit Null-Rotation vorbelegen
for i = 1:RP.NLEG
  RP.Leg(i).update_EE(zeros(3,1),zeros(3,1));
end

RP.fill_fcn_handles();

%% Grenzen für die Gelenkpositionen setzen
% Dadurch wird die Schrittweite bei der inversen Kinematik begrenzt (auf 5%
% der Spannbreite der Gelenkgrenzen) und die Konfiguration klappt nicht um.
for i = 1:RP.NLEG
  % Begrenze die Winkel der Kugel- und Kardangelenke auf +/- 360°
  RP.Leg(i).qlim = repmat([-2*pi, 2*pi], RP.Leg(i).NQJ, 1);
  % Begrenze die Länge der Schubgelenke
  RP.Leg(i).qlim(1,:) = [0.2, 0.7];
end

%% Startpose bestimmen
% Mittelstellung im Arbeitsraum
X = [ [0;0;0.7]; [10;10;5]*pi/180 ];
q0 = rand(36,1); % Startwerte für numerische IK
q0(RP.I_qa) = 0.6; % mit Schubaktor größer Null anfangen (damit Konfiguration nicht umklappt)

% Inverse Kinematik berechnen (so dass Schubgelenk nicht umklappt)
for i = 1:10
  ik_umgeklappt = false;
  [q, Phi] = RP.invkin_ser(X, q0);
  q_a = q(RP.I_qa);
  for j = 1:RP.NLEG
    if q_a(j) > 0.7
      ik_umgeklappt = true;
      q0(RP.I1J_LEG(j):RP.I2J_LEG(j)) = rand(RP.Leg(j).NJ,1);
    else
      % Diese Beinkette hat funktioniert. Passe IK-Anfangswerte an.
      q0(RP.I1J_LEG(j):RP.I2J_LEG(j)) = q(RP.I1J_LEG(j):RP.I2J_LEG(j));
    end
  end
  if ~ik_umgeklappt
    break;
  end
end
if ik_umgeklappt
  warning('Konnte keine IK ohne Umklappen der Schubachse berechnen');
end
if any(abs(Phi) > 1e-6)
  error('Inverse Kinematik (für jedes Bein einzeln) konnte in Startpose nicht berechnet werden');
end
fprintf('Antriebskoordinaten in Startstellung:\n');
disp(q(RP.I_qa)');
%% Zwangsbedingungen in Startpose (nochmal) testen
Phi1=RP.constr1(q, X);
Phit1=RP.constr1_trans(q, X);
Phir1=RP.constr1_rot(q, X);
if any(abs(Phi1) > 1e-6)
  error('ZB in Startpose ungleich Null');
end

%% Entwurfsparameter festlegen (Hauptsächlich für Plot)
for i = 1:RP.NLEG
  % Setze Schubgelenke als Linearachsen mit Führungsschiene
  RP.Leg(i).DesPar.joint_type(RP.I_qa((RP.I1J_LEG(i):RP.I2J_LEG(i)))) = 4;
  % Setze Segmente als Hohlzylinder mit Durchmesser 100mm
  RP.Leg(i).DesPar.seg_par=repmat([5e-3,100e-3],RP.Leg(i).NL,1);
  % Markiere Kardan- und Kugelgelenk (zum Plotten)
  RP.Leg(i).DesPar.joint_type(2:3) = 2;
  RP.Leg(i).DesPar.joint_type(4:6) = 3;
end
RP.DesPar.platform_par(end) = 5e-3; % Dicke der Plattform (als Kreisscheibe)

%% Roboter in Startpose plotten
figure(1); clf; hold on; grid on; % Bild als Kinematik-Skizze
xlabel('x in m');ylabel('y in m');zlabel('z in m'); view(3);
s_plot = struct( 'ks_legs', [RP.I1L_LEG+3;RP.I1L_LEG+4;RP.I2L_LEG], 'straight', 0); %RP.I1L_LEG; RP.I1L_LEG+1;RP.I1L_LEG+2;RP.I1L_LEG+3;RP.I1L_LEG+4; RP.I2L_LEG]
RP.plot( q, X, s_plot );
title('6PUS in Startkonfiguration als Kinematik-Skizze');

figure(2); clf; hold on; grid on; % Bild der Entwurfsparameter
xlabel('x in m');ylabel('y in m');zlabel('z in m'); view(3);
s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0, 'mode', 4);
RP.plot( q, X, s_plot );
title('6PUS in Startkonfiguration mit Ersatzkörpern');

%% Jacobi-Matrizen auswerten
G_q = RP.constr1grad_q(q, X);
G_x = RP.constr1grad_x(q, X);

% Aufteilung der Ableitung nach den Gelenken in Gelenkklassen 
% * aktiv/unabhängig (a),
% * passiv+schnitt/abhängig (d)
G_a = G_q(:,RP.I_qa);
G_d = G_q(:,RP.I_qd);
% Jacobi-Matrix zur Berechnung der abhängigen Gelenke und EE-Koordinaten
G_dx = [G_d, G_x];

fprintf('%s: Rang der vollständigen Jacobi der inversen Kinematik: %d/%d (Kondition %1.1f)\n', ...
  RP.mdlname, rank(G_q), RP.NJ, cond(G_q));
fprintf('%s: Rang der vollständigen Jacobi der direkten Kinematik: %d/%d (Kondition %1.1e)\n', ...
  RP.mdlname, rank(G_dx), sum(RP.I_EE)+sum(RP.I_qd), cond(G_dx));
fprintf('%s: Rang der Jacobi der aktiven Gelenke: %d/%d\n', ...
  RP.mdlname, rank(G_a), sum(RP.I_EE));

Jinv_num_voll = -inv(G_q) * G_x;
Jinv_num = Jinv_num_voll(RP.I_qa,:);
fprintf('%s: Rang der inversen PKM-Jacobi: %d/%d (Kondition %1.1e)\n', ...
  RP.mdlname, rank(Jinv_num, 1e-6), sum(RP.I_qa), cond(Jinv_num));

% Inverse Jacobi-Matrix aus symbolischer Berechnung (mit Funktion aus HybrDyn)
if ~isempty(which('parroblib_path_init.m'))
  Jinv_sym = RP.jacobi_qa_x(q, X);
  test_Jinv = Jinv_sym - Jinv_num;
  if max(abs(test_Jinv(:))) > 1e-10
    error('Inverse Jacobi-Matrix stimmt nicht zwischen numerischer und symbolischer Berechnung überein');
  else
    fprintf('Die inverse Jacobi-Matrix stimmt zwischen symbolischer und numerischer Berechnung überein\n');
  end
end
