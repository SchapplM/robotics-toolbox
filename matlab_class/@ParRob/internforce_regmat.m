% Interne Schnittkräfte der PKM basierend auf inverser Dynamik
% 
% Eingabe:
% q, qD, qDD
%   Gelenkkoordinaten aller Gelenke aller Beinketten der PKM
% tauA
%   Antriebskräfte der PKM (nur bezogen auf aktuierte Gelenke)
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
%
% Siehe auch: ParRob/internforce (dort gleicher Rechenweg. Hier nur
% spaltenweise Berechnung für Regressor-Matrix)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-05
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [w_B_reg, w_all_linkframe_reg, w_all_baseframe_reg] = internforce_regmat(RP, q, qD, qDD, xE, xDE, xDDE, Jinv)

% Antriebskraft-Regressor bestimmen
[~, Fa_reg] = RP.invdyn2_actjoint(q, qD, qDD, xE, xDE, xDDE, Jinv);
% Ausgabevariablen initialisieren
w_B_reg = NaN(6*RP.NLEG, length(RP.DynPar.ipv_n1s));
w_all_linkframe_reg = NaN(6*RP.Leg(1).NL*RP.NLEG, length(RP.DynPar.ipv_n1s)); % TODO: Aktuell nur symmetrische PKM

% Berechne die Schnittkräfte. Siehe Aufzeichnungen Schappler, 9.5.19
for j = 1:RP.NLEG % Für alle Beinketten
  % Initialisierung der aktuellen Beinkette
  q_j = q(RP.I1J_LEG(j):RP.I2J_LEG(j));
  qD_j = qD(RP.I1J_LEG(j):RP.I2J_LEG(j));
  qDD_j = qDD(RP.I1J_LEG(j):RP.I2J_LEG(j));
  % Schnittkräfte in der Beinkette aufgrund der internen Kräfte
  [~, W_j_l_int_regleg] = RP.Leg(j).internforce(q_j, qD_j, qDD_j);
  % Regressor-Einträge der Beinkette umrechnen auf PKM-Regressor
  W_j_l_int_reg = [W_j_l_int_regleg(:,11:end), zeros(6*RP.Leg(1).NL,length(RP.I_platform_dynpar))];
  
  I_joints = (RP.Leg(j).MDH.sigma==1) .* (3+(3:3:3*RP.Leg(j).NJ)') + ... % erste Einträge entsprechen Schnittkräften und damit Schubgelenken (z-Komponente)
             (RP.Leg(j).MDH.sigma==0) .* (6+3*RP.Leg(j).NJ+(3:3:3*RP.Leg(j).NJ)'); % 
  % Gelenkmomente aufgrund interner Dynamik
  tau_j_reg = W_j_l_int_reg(I_joints,:);
  % Antriebsmomente dieses Beins (passive sind Null)
  tau_m_j_reg = zeros( RP.Leg(j).NQJ, RP.Leg(j).NJ*10+length(RP.I_platform_dynpar) ); % erste Spalten für Parameter der Beinkette, letzte für die der Plattform
  tau_m_j_reg(RP.I_qa(RP.I1J_LEG(j):RP.I2J_LEG(j)),:) = Fa_reg(j,:); % TODO: Aktuell nur ein Antrieb pro Bein
  R_0_0j = RP.Leg(j).T_W_0(1:3,1:3); % Rotation PKM-Basis - Beinkette-Basis

  % Bein-Jacobi-Matrix für Koppelpunkt. Im PKM-Basis-KS
  J_j_0 = [R_0_0j, zeros(3,3); zeros(3,3), R_0_0j] * RP.Leg(1).jacobig(q_j);
  % Kraft, mit der die Plattform auf die Beinketten drückt (PKM-Basis-KS)
  % Die externe Kraft verursacht zusammen mit den Antriebskräften das
  % Bewegungsverhalten (q,qD,qDD) der Beinkette
  FB_j_0_reg = (J_j_0') \ (tau_j_reg - tau_m_j_reg); % Anwendung für jede Regressor-Spalte einzeln
  w_B_reg(1+(j-1)*6:j*6,:) = FB_j_0_reg;
  
  % Berechne die Schnittkräfte im Bein.
  % Modell-Annahme: Bein ist freistehende serielle Kette, an der vorne die
  % Schnittkraft mit der Plattform als externe Kraft angreift.
  % Kraft in Bein-Basis-KS umrechnen
  W_j_l_ext_reg = NaN(size(W_j_l_int_reg));
  Jg_C = RP.Leg(j).jacobig_cutforce(q_j, RP.Leg(j).NL-1, zeros(3,1));
  for i = 1:size(FB_j_0_reg, 2)
    F_B_j_0j_reg_i = rotate_wrench(FB_j_0_reg(:,i), R_0_0j');
    % Schnittkräfte des Beins im Bein-KS berechnen ("l"=Linkframe)
    W_j_l_ext_reg_i = reshape(Jg_C'*F_B_j_0j_reg_i, 6, RP.Leg(j).NL);
    f = W_j_l_ext_reg_i(1:3,:);
    m = W_j_l_ext_reg_i(4:6,:);
    W_j_l_ext_reg(:,i) = [f(:); m(:)];
  end

  % Schnittkräfte aufgrund der internen Kräfte `W_j_l_int_reg` oben berechnet
  % Gesamte Schnittkräfte: Differenz entspricht Schnittkraft im Gelenk.
  % Je nach Gelenktyp in der Struktur aufgefangen oder in Richtung des
  % Gelenk-FG
  W_j_l_reg = W_j_l_ext_reg - W_j_l_int_reg;
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
