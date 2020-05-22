% Rotiere die EE-Trajektorie ins Basis-KS des Roboters
% Die Roboter-Funktionen sind alle mit X in Basis-Koordinaten definiert.
% Für eine vorgegebene Aufgabe ist es aber sinnvoller, diese unabhängig vom
% Roboter im Welt-KS zu definieren.
% 
% Eingabe:
% R [class]
%   Handle zum Klassen-Objekt
% Traj_W [struct]
%   Roboter-Trajektorie (EE) bezogen auf Welt-KS
%   Felder: X [Nx6], XD [Nx6], XDD [Nx6]. X sind Euler-Winkel
% 
% Ausgabe:
% Traj_0 [struct]
%   Roboter-Trajektorie (EE) bezogen auf Basis-KS des Roboters
%   Felder: Siehe Eingabe.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function Traj_0 = rotate_traj(R, Traj_W)
Traj_0 = Traj_W;

% Trajektorie verschieben (nur Position. Geschwindigkeit egal)
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

R_W_0 = R.T_W_0(1:3,1:3);
R_0_W = R_W_0';
test = R_W_0-eye(3); % Bei Rotation der Basis muss die Trajektorie auch gedreht werden
if max(abs(test(:))) > 1e-10
  X_0 = NaN(size(XD_W));
  if calc_vel
    XD_0 = NaN(size(XD_W));
  end
  if calc_vel
    XDD_0 = NaN(size(XDD_W));
  end
  for i = 1:size(X_W,1) % Gehe alle Zeitschritte durch
    % Rotiere alle Orientierungsdarstellungen und Geschwindigkeiten
    T_W_Pi = R.x2t(X_W(i,:)');
    R_W_Pi = T_W_Pi(1:3,1:3);
    R_0_Pi = R_W_0*R_W_Pi;
    
    % Translation: r_0_Pi = R_0_W*(r_W_0_W + r_W_W_Pi);
    X_0(i,1:3) = R_0_W*(-R.T_W_0(1:3,4)+X_W(i,1:3)');
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
else
  % Rechnung: r_0_P = r_W_P - r_W_0; (falls R_W_0 = 1)
  X_0 = X_W - repmat([R.T_W_0(1:3,4); zeros(3,1)]', size(X_W,1),1);
  % Keine Neuberechnung der Geschwindigkeit/Beschl- notwendig.
  % Daher auch keine erneute Speicherung
  calc_vel = false;
  calc_acc = false;
end

Traj_0.X = X_0;
if calc_vel
  Traj_0.XD = XD_0;
end
if calc_acc
  Traj_0.XDD = XDD_0;
end