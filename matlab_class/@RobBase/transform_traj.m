% Transformiere die EE-Trajektorie ins Basis-KS des Roboters
% Die Roboter-Funktionen sind alle mit X in Basis-Koordinaten definiert.
% Für eine vorgegebene Aufgabe ist es aber sinnvoller, diese unabhängig vom
% Roboter im Welt-KS zu definieren.
% 
% Eingabe:
% R [class]
%   Handle zum Klassen-Objekt
% Traj_W [struct]
%   Roboter-Trajektorie (EE) bezogen auf Welt-KS
%   Felder: X [Nx6], XD [Nx6], XDD [Nx6]. X sind Positionen und Euler-Winkel
% direction_W_0 (Optional)
%   Richtung der Transformation. Bei true Ein-/Ausgabe wie dokumentiert.
%   Bei false: Eingabe ist im Basis-KS und Ausgabe ist im Welt-KS.
% 
% Ausgabe:
% Traj_0 [struct]
%   Roboter-Trajektorie (EE) bezogen auf Basis-KS des Roboters
%   Felder: Siehe Eingabe.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function Traj_0 = transform_traj(R, Traj_W, direction_W_0)
Traj_0 = Traj_W;

%% Initialisierung
X_W = Traj_W.X;
calc_vel = false;
if isfield(Traj_W, 'XD')
  calc_vel = true;
  XD_W = Traj_W.XD;
end
calc_acc = false;
if isfield(Traj_W, 'XDD')
  calc_acc = true;
  XDD_W = Traj_W.XDD;
  if ~calc_vel
    error('Berechnung der Beschleunigung ohne Berechnung der Geschw. nicht möglich');
  end
end
if nargin < 3
  direction_W_0 = true;
end
if direction_W_0
  T_W_0 = R.T_W_0;
else
  T_W_0 = invtr(R.T_W_0);
end
R_W_0 = T_W_0(1:3,1:3);
R_0_W = R_W_0';

%% Testen und Fallunterscheidung
% Prüfe, ob reine Verschiebung ohne Drehung
test_norot = all(all(abs(R_W_0 - eye(3)) < 1e-12));
% Teste ob nur Drehung nach unten und Verschiebung
test_rotx_xyz = all(all(abs(R_W_0 - [1,0,0;0,-1,0;0,0,-1]) < 1e-12)) & ...
            R.phiconv_W_0 == 2; % Formel unten geht nur mit Euler-XYZ-Winkeln
if ~test_norot && ~test_rotx_xyz
  %% Allgemeiner Fall: Beliebige Drehung
  X_0 = NaN(size(X_W));
  if calc_vel
    XD_0 = NaN(size(XD_W));
  end
  if calc_acc
    XDD_0 = NaN(size(XDD_W));
  end
  for i = 1:size(X_W,1) % Gehe alle Zeitschritte durch
    % Rotiere alle Orientierungsdarstellungen und Geschwindigkeiten
    T_W_Pi = R.x2t(X_W(i,:)');
    R_W_Pi = T_W_Pi(1:3,1:3);
    R_0_Pi = R_0_W*R_W_Pi;
    
    % Translation: r_0_Pi = R_0_W*(r_W_0_W + r_W_W_Pi);
    X_0(i,1:3) = R_0_W*(-T_W_0(1:3,4)+X_W(i,1:3)');
    % Rotation: Euler-Winkel aus neuer Rotationsmatrix berechnen
    X_0(i,4:6) = r2eul(R_0_Pi, R.phiconv_W_E);
    if calc_vel
      % Berechne Winkelgeschwindigkeit im KS W
      T_phiW = euljac_mex(X_W(i,4:6)', R.phiconv_W_E);
      omega_W_Pi = T_phiW*XD_W(i,4:6)';
      % Umrechnung auf KS 0
      V_W_Pi = [XD_W(i,1:3)'; omega_W_Pi];
      V_0_Pi = rotate_wrench(V_W_Pi, R_0_W);
      % Umrechnung auf Euler-Geschwindigkeit in KS 0
      T_phi0 = euljac_mex(X_0(i,4:6)', R.phiconv_W_E);
      phiD0i = T_phi0\V_0_Pi(4:6);
      XD_0(i,:) = [V_0_Pi(1:3); phiD0i];
    end
    if calc_acc
      % Berechne Winkelbeschleunigung im KS W
      TD_phiW = euljacD_mex(X_W(i,4:6)', XD_W(i,4:6)', R.phiconv_W_E);
      omegaD_W_Pi = T_phiW*XDD_W(i,4:6)' + TD_phiW*XD_W(i,4:6)';
      % Umrechnung auf KS 0
      VD_W_Pi = [XDD_W(i,1:3)'; omegaD_W_Pi];
      VD_0_Pi = rotate_wrench(VD_W_Pi, R_0_W);
      % Umrechnung auf Euler-Beschleunigung in KS 0
      TD_phi0 = euljacD_mex(X_0(i,4:6)', XD_0(i,4:6)', R.phiconv_W_E);
      phiDD0i = T_phi0\(VD_0_Pi(4:6)-TD_phi0*XD_0(i,4:6)');
      XDD_0(i,:) = [VD_0_Pi(1:3); phiDD0i];
    end
  end
elseif test_norot
  %% Sonderfall: Keine Rotation
  % Rechnung: r_0_P = r_W_P - r_W_0; (falls R_W_0 = 1)
  X_0 = X_W - repmat([T_W_0(1:3,4); zeros(3,1)]', size(X_W,1),1);
  % Keine Neuberechnung der Geschwindigkeit/Beschl- notwendig.
  % Daher auch keine erneute Speicherung
  calc_vel = false;
  calc_acc = false;
else % test_rotx_xyz
  %% Sonderfall: Nur Translation und Rotation mit 180° um x-Achse
  % Rechnung: r_0_P = R_0_W * r_W_0_P = R_0_W * (-r_W_0 + r_W_P);
  r_W_0_P = - repmat([T_W_0(1:3,4); zeros(3,1)]', size(X_W,1),1) + X_W;
  % Rotation um 180°: Richtungen umdrehen; r_0_P = R_0_W * r_W_0_P
  r_0_0_P = [r_W_0_P(:,1),-r_W_0_P(:,2:3)];
  % Euler-Winkel: Die Drehung der x-Achse um 180° wird wieder korrigiert.
  phi_0_P = [wrapToPi(X_W(:,4)+pi),X_W(:,5:6)];
  X_0 = [r_0_0_P, phi_0_P];
  if calc_vel
    % Translatorische Geschwindigkeit: y/z umkehren
    rD_0_0_P = [XD_W(:,1),-XD_W(:,2:3)];
    % Euler-Winkel-Zeitableitung bleibt gleich (pi hinzuaddiert)
    phiD_0_P = XD_W(:,4:6);
    XD_0 = [rD_0_0_P, phiD_0_P];
  end
  if calc_acc
    % Gleiche Vorgehensweise wie bei Geschwindigkeit
    rDD_0_0_P = [XDD_W(:,1),-XDD_W(:,2:3)];
    phiDD_0_P = XDD_W(:,4:6);
    XDD_0 = [rDD_0_0_P, phiDD_0_P];
  end
end

Traj_0.X = X_0;
if calc_vel
  Traj_0.XD = XD_0;
end
if calc_acc
  Traj_0.XDD = XDD_0;
end