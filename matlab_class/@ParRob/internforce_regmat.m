% Interne Schnittkräfte der PKM basierend auf inverser Dynamik
% 
% Eingabe:
% q, qD, qDD
%   Gelenkkoordinaten aller Gelenke aller Beinketten der PKM
% xP, xDP, xDDP
%   Plattform-Koordinaten (nicht: Endeffektor), Plattform-Geschwindigkeit,
%   Plattform-Beschleunigung (Rotationskomponente sind Euler-Winkel)
% JinvP
%   Inverse Jacobi-Matrix für alle Gelenke
%   (bezogen auf Plattform-Koordinaten; siehe jacobi_qa_x)
% tau_add_mult
%   Multiplikator für zusätzliche Gelenkmomente (in den Gelenkkoordinaten
%   der Beinketten). Werden alle Einträge auf "1" gesetzt, wird der Regressor
%   später direkt mit den Gelenkmomenten multipliziert. Für Reibkräfte führt bspw.
%   das Einsetzen von qD oder sign(qD) zu einer Linearität mit den Reibparameter
%   (bei viskosem oder Coulombschen Reibmodell).
% 
% Ausgabe:
% w_B [6*NLEG x NDP]
%   Schnittkräfte und -momente in allen Plattform-Koppelpunkten
% w_all_linkframe [6*NLEGJ*NLEG x NDP]
%   Alle Schnittkräfte in allen Gelenken der PKM. Ausgedrückt im Körper-KS
% w_all_baseframe
%   ... ausgedrückt im Basis-KS der PKM.
% 
% Quellen:
% Aufzeichnungen Schappler, 9.5.19
% [KhalilGue2004] Inverse and direct dynamic modeling of Gough-Stewart robots
%
% Siehe auch: ParRob/internforce (dort gleicher Rechenweg. Hier nur
% spaltenweise Berechnung für Regressor-Matrix)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-05
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [w_B_reg, w_all_linkframe_reg, w_all_baseframe_reg] = internforce_regmat(RP, q, qD, qDD, xP, xDP, xDDP, JinvP, tau_add_mult)

% Antriebskraft-Regressor bestimmen
if nargin < 9
  % Antriebskräfte notwendig zur Kompensation der inversen Dynamik.
  % Nehme an, dass diese Antriebskräfte immer gestellt werden (Grund-
  % annahme bei der inversen Dynamik). Entspricht Eingabe der passenden 
  % Antriebsmomente in ParRob/internforce.
  [~, Fa_reg] = RP.invdyn2_actjoint(q, qD, qDD, xP, xDP, xDDP, JinvP);
else
  % Regressor für die Antriebsmomente notwendig zur Kompensation der
  % Gelenkmomente. Entspricht der Annahme, dass die zur Kompensation notwendigen
  % Antriebsmomente gestellt werden.
  [~, Fa_reg] = RP.jointtorque_actjoint(q,xP,zeros(length(tau_add_mult),1),JinvP);
  % Skaliere die Regressor-Matrix mit dem gegebenen Vektor. Dadurch kann z.B.
  % die lineare Abhängigkeit von Reibparametern abgebildet werden.
  Fa_reg = repmat(tau_add_mult', size(Fa_reg,1), 1) .* Fa_reg;
end
% Ausgabevariablen initialisieren. Je nach Modus andere Spaltenzahl der 
% Regressormatrix (Inertialparameter oder Gelenkmomente)
w_B_reg = NaN(6*RP.NLEG, size(Fa_reg,2));
w_all_linkframe_reg = NaN(6*RP.Leg(1).NL*RP.NLEG, size(Fa_reg,2)); % TODO: Aktuell nur symmetrische PKM
w_all_baseframe_reg = w_all_linkframe_reg;

% Berechne die Schnittkräfte. Siehe Aufzeichnungen Schappler, 9.5.19
for j = 1:RP.NLEG % Für alle Beinketten
  % Initialisierung der aktuellen Beinkette
  q_j = q(RP.I1J_LEG(j):RP.I2J_LEG(j));
  qD_j = qD(RP.I1J_LEG(j):RP.I2J_LEG(j));
  qDD_j = qDD(RP.I1J_LEG(j):RP.I2J_LEG(j));
  % Schnittkräfte in der Beinkette aufgrund der internen Kräfte
  I_joints = (RP.Leg(j).MDH.sigma==1) .* (3+(3:3:3*RP.Leg(j).NJ)') + ... % erste Einträge entsprechen Schnittkräften und damit Schubgelenken (z-Komponente)
             (RP.Leg(j).MDH.sigma==0) .* (6+3*RP.Leg(j).NJ+(3:3:3*RP.Leg(j).NJ)'); % 
  if nargin < 9
    % Regressor-Matrix für Schnittkräfte der Beinkette. Erst alle Kräfte,
    % dann alle Momente. Inkl. Basis. Siehe SerRob/internforce.
    [~, W_j_l_int_regleg] = RP.Leg(j).internforce(q_j, qD_j, qDD_j);
    % Regressor-Einträge der Beinkette umrechnen auf PKM-Regressor
    W_j_l_int_reg = [W_j_l_int_regleg(:,11:end), zeros(6*RP.Leg(1).NL,sum(RP.I_platform_dynpar))];
    % Gelenkmomente aufgrund interner Dynamik
    tau_j_reg = W_j_l_int_reg(I_joints,:);
  else
    % Einfachster Fall: Setze überall eine "1", wo die Schnittkraft direkt
    % dem Gelenk entspricht. Dann ergibt der Regressor multipliziert mit dem
    % Gelenkmoment-Vektor wieder die richtige Schnittkraft. Benutze den
    % Multiplikationsfaktor aus der Eingabe.
    tau_j_reg = zeros(RP.Leg(j).NJ, length(tau_add_mult));
    for k = 1:RP.Leg(j).NJ
      tau_j_reg(k, RP.I1J_LEG(j)+k-1) = tau_add_mult(RP.I1J_LEG(j)+k-1);
    end
  end
  % Antriebsmomente dieses Beins (passive sind Null)
  % Anzahl der Spalten bei Dynamik: RP.Leg(j).NJ*10+sum(RP.I_platform_dynpar); erste Spalten für Parameter der Beinkette, letzte für die der Plattform
  % Anzahl der Spalten bei Gelenkmoment: RP.NJ
  tau_m_j_reg = zeros( RP.Leg(j).NQJ, size(Fa_reg,2) );
  tau_m_j_reg(RP.I_qa(RP.I1J_LEG(j):RP.I2J_LEG(j)),:) = Fa_reg(j,:); % TODO: Aktuell nur ein Antrieb pro Bein

  R_0_0j = RP.Leg(j).T_W_0(1:3,1:3); % Rotation PKM-Basis - Beinkette-Basis

  % Bein-Jacobi-Matrix für Koppelpunkt. Im PKM-Basis-KS
  J_j_0 = [R_0_0j, zeros(3,3); zeros(3,3), R_0_0j] * RP.Leg(j).jacobig(q_j);
  % Kraft, mit der die Plattform auf die Beinketten drückt (PKM-Basis-KS)
  % Die externe Kraft verursacht zusammen mit den Antriebskräften das
  % Bewegungsverhalten (q,qD,qDD) der Beinkette
  % [KhalilGue2004], Gl. 20-21; dort steht das f_ext auf der anderen Seite.
  % Daher andere Vorzeichen
  FB_j_0_reg = (J_j_0') \ (tau_j_reg - tau_m_j_reg); % Anwendung für jede Regressor-Spalte einzeln
  w_B_reg(1+(j-1)*6:j*6,:) = FB_j_0_reg;
  
  % Berechne die Schnittkräfte im Bein.
  % Modell-Annahme: Bein ist freistehende serielle Kette, an der vorne die
  % Schnittkraft mit der Plattform als externe Kraft angreift.
  % Kraft in Bein-Basis-KS umrechnen
  Jg_C = RP.Leg(j).jacobig_cutforce(q_j, RP.Leg(j).NL-1, zeros(3,1));
  FB_j_0j_reg = [R_0_0j'*FB_j_0_reg(1:3,:); R_0_0j'*FB_j_0_reg(4:6,:)]; % Spaltenweise für gesamten Regressor (z.B. alle Dynamikparameter)
  % Schnittkräfte des Beins im Bein-KS berechnen ("l"=Linkframe)
  W_j_l_ext_reg_tmp = Jg_C'*FB_j_0j_reg; % Reihenfolge: Alle Kraft-Momenten-Vektoren aller Beinkette
  % Umrechnung der Reihenfolge von Kräften und Momenten ins Ausgabeformat.
  % Anzahl der Spalten bei Dynamik: length(RP.DynPar.ipv_n1s)
  f_j_l_ext_reg = NaN(3*RP.Leg(1).NL, size(FB_j_0j_reg,2));
  m_j_l_ext_reg = f_j_l_ext_reg;
  % Erst alle Kräfte ...
  f_j_l_ext_reg(1:3:end,:) = W_j_l_ext_reg_tmp(1:6:end,:);
  f_j_l_ext_reg(2:3:end,:) = W_j_l_ext_reg_tmp(2:6:end,:);
  f_j_l_ext_reg(3:3:end,:) = W_j_l_ext_reg_tmp(3:6:end,:);
  % ... dann alle Momente
  m_j_l_ext_reg(1:3:end,:) = W_j_l_ext_reg_tmp(4:6:end,:);
  m_j_l_ext_reg(2:3:end,:) = W_j_l_ext_reg_tmp(5:6:end,:);
  m_j_l_ext_reg(3:3:end,:) = W_j_l_ext_reg_tmp(6:6:end,:);
  W_j_l_ext_reg = [f_j_l_ext_reg;m_j_l_ext_reg];
  % Schnittkräfte aufgrund der internen Kräfte `W_j_l_int_reg` oben berechnet
  % Gesamte Schnittkräfte: Differenz entspricht Schnittkraft im Gelenk.
  % Je nach Gelenktyp in der Struktur aufgefangen oder in Richtung des
  % Gelenk-FG. Weitere Erklärung siehe ParRob/internforce.
  if nargin < 9
    % Regressor der inversen Dynamik (Massenträgheit, Coriolis, Gravitation
    % als innere Kräfte)
    W_j_l_reg = -W_j_l_ext_reg + W_j_l_int_reg;
  else
    % Nur Betrachtung der zusätzlichen Gelenkmoment als externe Kraft.
    % Gleiche Formel wie im anderen Fall, aber W_j_l_int_reg=0.
    W_j_l_reg = -W_j_l_ext_reg;
  end
  % Ergebnisse für diese Beinkette eintragen
  sblock = size(W_j_l_reg,1);
  w_all_linkframe_reg(1+sblock*(j-1):sblock*j,:) = W_j_l_reg;
  
  % Schnittkräfte wieder in Basis-KS zurückrechnen (sind vorher im
  % Körper-KS)
  if nargout < 3
    continue
  end
  Tc_j_0j = RP.Leg(j).fkine(q_j);
  W_j_0_reg = NaN(size(W_j_l_reg));
  for k = 1:RP.Leg(j).NL % Alle Segmente durchgehen
    R_0j_l = Tc_j_0j(1:3,1:3,k);
    % Kräfte rotieren (alle Regressor-Spalten gleichzeitig)
    W_j_0_reg((k-1)*3+1:3*k,:) = R_0_0j*R_0j_l*W_j_l_reg((k-1)*3+1:3*k,:);
    % Momente rotieren (zweiter Teil der Matrix-Zeilen)
    W_j_0_reg(3*RP.Leg(j).NL+((k-1)*3+1:3*k),:) = R_0_0j*R_0j_l*W_j_l_reg(3*RP.Leg(j).NL+((k-1)*3+1:3*k),:);
  end
  % Ergebnisse für diese Beinkette eintragen
  w_all_baseframe_reg(1+sblock*(j-1):sblock*j,:) = W_j_0_reg;
end
