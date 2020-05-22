% Teste die in der ParRob und RobBase gespeicherten
% Koordinatentransformationen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

%% Initialisierung des 6UPS-Roboters
% Kinematik ist eigentlich nicht wichtig für folgende Tests
if isempty(which('parroblib_path_init.m'))
  warning('Repo mit parallelen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end
RP = parroblib_create_robot_class('P6RRPRRR14V3G1P4A1', 0.5, 0.2);
RP.fill_fcn_handles(true, true);
RP.align_platform_coupling(4, [0.2;0.1]);

%% Beispiel-Trajektorie
X0 = [ [0;0;0.5]; [0;0;0]*pi/180 ];
% Trajektorie mit beliebigen Bewegungen der Plattform
XL = [X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.3]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.3, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [0.3, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.2,-0.1, 0.3], [0.3, 0.2, 0.1]]; ...
      X0'+1*[[-0.1, 0.2,-0.1], [0.5,-0.2,-0.2]]; ...
      X0'+1*[[ 0.2, 0.3, 0.2], [0.2, 0.1, 0.3]]];
XL = [XL; XL(1,:)]; % Rückfahrt zurück zum Startpunkt.
[X_t,XD_t,XDD_t,T] = traj_trapez2_multipoint(XL, 1, 0.1, 0.01, 1e-3, 1e-1);

%% Teste Umrechnung zwischen Plattform- und EE-Koordinaten
X_E = X_t;
XD_E = XD_t;
XDD_E = XDD_t;
[X_P, XD_P, XDD_P] = RP.xE2xP_traj(X_E, XD_E, XDD_E);
% Neuen EE festlegen
RP.update_EE(rand(3,1),rand(3,1));
% Geschwindigkeit des neuen EE ausrechnen
[X_E2, XD_E2, XDD_E2] = RP.xP2xE_traj(X_P, XD_P, XDD_P);
% Zurückrechnen auf Plattform
[X_P2, XD_P2, XDD_P2] = RP.xE2xP_traj(X_E2, XD_E2, XDD_E2);
% Prüfen, ob durch Hin- und Herrechnen ein Fehler passiert ist
Test=[X_P;XD_P;XDD_P]-[X_P2;XD_P2;XDD_P2];
assert(all(abs(Test(:))<1e-10), 'Umrechnung Plattform-EE mit xP2xE / xE2xP stimmt nicht.');

%% Teste Umrechnung der Trajektorie zwischen Basis und Welt
Traj_W = struct('T', T, 'X', X_t, 'XD', XD_t, 'XDD', XDD_t);
for i = 1:2 % zwei Fälle durchgehen: mit/ohne Rotation
  % Beliebige Transformation der Basis einstellen
  if i == 1
    RP.update_base(rand(3,1), rand(3,1));
  else
    RP.update_base(rand(3,1), zeros(3,1)); % Keine Rotation der Basis
  end
  T_W_B1 = RP.T_W_0;
  % Trajektorien-Eckpunkte in Basis-KS umrechnen
  Traj_B1 = RP.rotate_traj(Traj_W);
  % Inverse Rechnung anstellen. Transformiere Welt-KS
  T_W_B2 = invtr(T_W_B1);
  RP.update_base(T_W_B2(1:3,4), r2eul(T_W_B2(1:3,1:3), RP.phiconv_W_0));
  Traj_B2 = RP.rotate_traj(Traj_B1);
  % Testen: B2 ist eigentlich identisch mit W. Trajektorie muss nach
  % zweifacher Transformation wieder die ursprüngliche sein
  test_X = Traj_W.X - Traj_B2.X;
  assert(all(abs(test_X(:))<1e-10), 'Fehler bei Umrechnung der Positions-Traj.');
  test_XD = Traj_W.XD - Traj_B2.XD;
  assert(all(abs(test_XD(:))<1e-10), 'Fehler bei Umrechnung der Geschw.-Traj.');
  test_XDD = Traj_W.XDD - Traj_B2.XDD;
  assert(all(abs(test_XDD(:))<1e-10), 'Fehler bei Umrechnung der Beschl.-Traj.');
end