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
RP = parroblib_create_robot_class('P6RRPRRR14V3G1P4A1', '', 0.5, 0.2);
RP.fill_fcn_handles(false);
RP.align_platform_coupling(4, [0.2;0.1]);

%% Beispiel-Trajektorie
X0 = [ [0;0;0.5]; [0;0;0]*pi/180 ];
% Trajektorie mit beliebigen Bewegungen der Plattform
XL = [X0'+1*[[ 0.0, 0.0, 0.0], [ 0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.1, 0.0, 0.0], [ 0.0, 0.0, 0.0]]; ...
      X0'+1*[[-0.1, 0.0, 0.0], [ 0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.1, 0.0], [ 0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0,-0.1, 0.0], [ 0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.1], [ 0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0,-0.1], [ 0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.1, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [-0.1, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.0, 0.1, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.0,-0.1, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.0, 0.0, 0.1]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.0, 0.0,-0.1]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.0, 0.0, 0.3]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.0, 0.3, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.3, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [ 0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.2,-0.1, 0.3], [ 0.3, 0.2, 0.1]]; ...
      X0'+1*[[-0.1, 0.2,-0.1], [ 0.5,-0.2,-0.2]]; ...
      X0'+1*[[ 0.2, 0.3, 0.2], [ 0.2, 0.1, 0.3]]];
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
fprintf('Klassen-Methode xP2xE_traj und xE2xP_traj in sich getestet\n');
%% Teste Umrechnung der Trajektorie zwischen Basis und Welt
Traj_W = struct('T', T, 'X', X_t, 'XD', XD_t, 'XDD', XDD_t);
for i = 0:4 % drei Fälle durchgehen: mit/ohne Rotation und 180° um x
  fprintf('Prüfe Transformation der Trajektorie, Fall %d\n', i);
  % Beliebige Transformation der Basis einstellen
  if i == 0
    % Keine Transformation durchführen
    RP.update_base(zeros(3,1), zeros(3,1));
  elseif i == 1
    RP.update_base(rand(3,1), rand(3,1));
  elseif i == 2
    RP.update_base(rand(3,1), zeros(3,1)); % Keine Rotation der Basis
  elseif i == 3
    RP.update_base(rand(3,1), [pi;0;0]); % Nur Drehung um x (Deckenmontage)
  elseif i == 4
    RP.update_base(rand(3,1), [0;0;pi/2]); % Nur Drehung um z
  else
    error('Fall nicht definiert');
  end
  T_W_B1 = RP.T_W_0;
  % Trajektorien-Eckpunkte in Basis-KS umrechnen
  Traj_B1 = RP.transform_traj(Traj_W);
  % De-Normalisieren, damit besser gegen Integration von XD vergleichbar
  Traj_B1.X(:,4:6) = denormalize_angle_traj(Traj_B1.X(:,4:6));
  % Inverse Rechnung anstellen. Rechenweg 1.
  Traj_B2_v1 = RP.transform_traj(Traj_B1, false); % über Argument umschalten
  % Rechenweg 2: Transformiere Welt-KS (alte Implementierung)
  T_W_B2 = invtr(T_W_B1);
  RP.update_base(T_W_B2(1:3,4), r2eul(T_W_B2(1:3,1:3), RP.phiconv_W_0));
  Traj_B2_v2 = RP.transform_traj(Traj_B1);
  % Vergleiche beide Rechenwege
  test_X_v12 = Traj_B2_v1.X - Traj_B2_v2.X;
  test_XD_v12 = Traj_B2_v1.XD - Traj_B2_v2.XD;
  test_XDD_v12 = Traj_B2_v1.XDD - Traj_B2_v2.XDD;
  assert(all(abs(test_X_v12(:))<1e-10), 'Fehler bei Umrechnung der Positions-Traj_B1.');
  assert(all(abs(test_XD_v12(:))<1e-10), 'Fehler bei Umrechnung der Geschw.-Traj_B1.');
  assert(all(abs(test_XDD_v12(:))<1e-10), 'Fehler bei Umrechnung der Beschl.-Traj_B1.');
  % Testen: B2 ist eigentlich identisch mit W. Trajektorie muss nach
  % zweifacher Transformation wieder die ursprüngliche sein
  test_X = Traj_W.X - Traj_B2_v2.X;
  assert(all(abs(test_X(:))<1e-10), 'Fehler bei Umrechnung der Positions-Traj_B1.');
  test_XD = Traj_W.XD - Traj_B2_v2.XD;
  assert(all(abs(test_XD(:))<1e-10), 'Fehler bei Umrechnung der Geschw.-Traj_B1.');
  test_XDD = Traj_W.XDD - Traj_B2_v2.XDD;
  assert(all(abs(test_XDD(:))<1e-10), 'Fehler bei Umrechnung der Beschl.-Traj_B1.');
  % Prüfe auch die Konsistenz der Trajektorie mit Integration. Bei Euler-
  % Winkeln muss die Trajektorie auch integrierbar sein
  X_numint = repmat(Traj_B1.X(1,:),size(Traj_B1.X,1),1)+cumtrapz(Traj_B1.T, Traj_B1.XD);
  XD_numint = repmat(Traj_B1.XD(1,:),size(Traj_B1.XD,1),1)+cumtrapz(Traj_B1.T, Traj_B1.XDD);
  corrX = diag(corr(X_numint, Traj_B1.X));
  corrX(all(abs(X_numint-Traj_B1.X)<1e-6)) = 1;
  assert(all(corrX>0.98), 'Trajektorie ist nicht konsistent (X-XD)');
  corrXD = diag(corr(XD_numint, Traj_B1.XD));
  corrXD(all(abs(Traj_B1.XD)<1e-3)) = 1;
  assert(all(corrXD>0.98), 'Trajektorie ist nicht konsistent (XD-XDD)');
  continue
  % Debug (für den Fehlerfall bei der Integration+Korrelation)
  figure(1);clf; %#ok<UNRCH>
  for rr = 1:6
    if rr <4, l = 'trans'; else, l = 'rot'; end
    subplot(3,6,sprc2no(3,6,1,rr)); hold on;
    plot(Traj_B1.T, X_numint(:,rr), '-');
    plot(Traj_B1.T, Traj_B1.X(:,rr), '-');
    if rr==6, legend({'int(xD)', 'x'}); end
    grid on; ylabel(sprintf('x %d (%s)', rr, l));
    subplot(3,6,sprc2no(3,6,2,rr)); hold on;
    plot(Traj_B1.T, XD_numint(:,rr), '-');
    plot(Traj_B1.T, Traj_B1.XD(:,rr), '-');
    grid on; ylabel(sprintf('xD %d (%s)', rr, l));
    subplot(3,6,sprc2no(3,6,3,rr)); hold on;
    plot(Traj_B1.T, Traj_B1.XDD(:,rr), '-');
    grid on; ylabel(sprintf('xDD %d (%s)', rr, l));
  end
  linkxaxes
end
fprintf('Klassen-Methode transform_traj in sich getestet\n');