% Roboterklasse für Hexa-PKM testen
% Quelle:
% Pierrot: Towards a fully-parallel 6 DOF robot for high-speed applications

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-02
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc
if isempty(which('serroblib_path_init.m')) || isempty(which('parroblib_path_init.m'))
  warning('Repos mit seriellen und parallelen Robotermodellen sind nicht im Pfad. Beispiel nicht ausführbar.');
  return
end

%% Klasse für PKM erstellen (basierend auf PKM-Bibliothek)
if isempty(which('parroblib_path_init.m'))
  warning('Repo mit parallelen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end
serroblib_update_template_functions({'S6RRRRRR10V3'});
parroblib_update_template_functions({'P6RRRRRR10V3G6P4'});
RP = parroblib_create_robot_class('P6RRRRRR10V3G6P4A1', [0.2; 0.1], [0.2; 0.1]);
RP.fill_fcn_handles(true, true); % keine mex-Funktionen, einfache Rechnung

pkin = NaN(length(RP.Leg(1).pkin_names),1);
pkin(strcmp(RP.Leg(1).pkin_names, 'd1')) = 0;
pkin(strcmp(RP.Leg(1).pkin_names, 'a2')) = 0.3;
pkin(strcmp(RP.Leg(1).pkin_names, 'd2')) = 0;
pkin(strcmp(RP.Leg(1).pkin_names, 'alpha2')) = 0;
pkin(strcmp(RP.Leg(1).pkin_names, 'a3')) = 0; % für Kardan-Gelenk
pkin(strcmp(RP.Leg(1).pkin_names, 'd3')) = 0;
pkin(strcmp(RP.Leg(1).pkin_names, 'alpha3')) = pi/2;
pkin(strcmp(RP.Leg(1).pkin_names, 'a4')) = 0.4;
pkin(strcmp(RP.Leg(1).pkin_names, 'alpha4')) = 0;
pkin(strcmp(RP.Leg(1).pkin_names, 'd4')) = 0;
pkin(strcmp(RP.Leg(1).pkin_names, 'd6')) = 0;
pkin(strcmp(RP.Leg(1).pkin_names, 'a6')) = 0;
for i = 1:RP.NLEG
  RP.Leg(i).update_mdh(pkin);
end

% Markiere Kardan- und Kugelgelenk (zum Plotten)
for i = 1:RP.NLEG
  RP.Leg(i).DesPar.joint_type(2:3) = 2; % Kardan
  RP.Leg(i).DesPar.joint_type(4:6) = 3; % Kugel
end
%% Grenzen für die Gelenkpositionen setzen
% Dadurch wird die Schrittweite bei der inversen Kinematik begrenzt (auf 5%
% der Spannbreite der Gelenkgrenzen) und die Konfiguration klappt nicht um.
for i = 1:RP.NLEG
  % Begrenze die Winkel der Kugel- und Kardangelenke auf +/- 360°
  RP.Leg(i).qlim = repmat([-2*pi, 2*pi], RP.Leg(i).NQJ, 1);
end
qlim_pkm = cat(1, RP.Leg.qlim);
%% Startpose bestimmen
% Mittelstellung im Arbeitsraum
X = [ [0.15;0.05;0.3]; [10;-10;5]*pi/180 ];
q0 = qlim_pkm(:,1)+rand(36,1).*(qlim_pkm(:,2)-qlim_pkm(:,1));
q0(1:6:end) = 60*pi/180; % Erstes Gelenk sollte nach außen zeigen
q0(2:6:end) = -120*pi/180; % damit Außenstellung gelingt
q0(3:6:end) = 0;
q0(4:6:end) = 0;
q0(5:6:end) = 0;
q0(6:6:end) = 0;
[q, Phis, ~, Stats] = RP.invkin_ser(X, q0);
%% Zwangsbedingungen in Startpose testen
Phi1=RP.constr1(q, X);
Phit1=RP.constr1_trans(q, X);
Phir1=RP.constr1_rot(q, X);

%% Roboter in Startpose plotten
figure(1); clf; hold on; grid on; % Bild als Kinematik-Skizze
xlabel('x in m');ylabel('y in m');zlabel('z in m'); view(3);
s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
RP.plot( q, X, s_plot );
title('6RUS (Hexa) in Startkonfiguration als Kinematik-Skizze');

if any(abs(Phi1) > 1e-6)
  error('ZB in Startpose ungleich Null');
end

%% Jacobi-Matrizen auswerten
G_q = RP.constr1grad_q(q, X);
G_x = RP.constr1grad_x(q, X);
G_a = G_q(:,RP.I_qa);  % aktiv/unabhängig (a),
G_d = G_q(:,RP.I_qd); % passiv+schnitt/abhängig (d)
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
Jinv_sym = RP.jacobi_qa_x(q, X);
test_Jinv = Jinv_sym - Jinv_num;
if max(abs(test_Jinv(:))) > 1e-10
  error('Inverse Jacobi-Matrix stimmt nicht zwischen numerischer und symbolischer Berechnung überein');
else
  fprintf('Die inverse Jacobi-Matrix stimmt zwischen symbolischer und numerischer Berechnung überein\n');
end
