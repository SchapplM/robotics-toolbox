% Interne Schnittkräfte der PKM basierend auf inverser Dynamik
% 
% Eingabe:
% q, qD, qDD
%   Gelenkkoordinaten aller Gelenke aller Beinketten der PKM
% tauA
%   Antriebskräfte der PKM (nur bezogen auf aktuierte Gelenke)
% tau_add
%   Zusätzliche Gelenkmomente der PKM (bezogen auf alle Gelenkkoordinaten).
%   Enthält z.B. Reibung oder Gelenkfedern. Wenn gesetzt, wird die inverse
%   Dynamik (Massenträgheit, Coriolis, Gravitation) nicht berechnet.
%   Die Eingabe tau_A sollte so gesetzt werden, dass sie die Gelenkmomente 
%   kompensieren (siehe ParRob/jointtorque_actjoint).
%   Definition entgegengesetzt zu tau_m: MqDD + c + g = tau_m - tau_add
%   (verallgemeinerte Gleichung; tau_m entspricht tauA).
% 
% Ausgabe:
% w_B [6xNLEG]
%   Schnittkräfte und -momente in allen Plattform-Koppelpunkten
% w_all_linkframe [6xNLEGJxNLEG]
%   Alle Schnittkräfte in allen Gelenken der PKM. Ausgedrückt im Körper-KS
% w_all_baseframe
%   ... ausgedrückt im Basis-KS der PKM.
% 
% Quellen:
% Aufzeichnungen Schappler, 9.5.19
% [KhalilGue2004] Inverse and direct dynamic modeling of Gough-Stewart robots

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-05
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [w_B, w_all_linkframe, w_all_baseframe] = internforce(RP, q, qD, qDD, tauA, tau_add)

% Ausgabevariablen initialisieren
w_B = NaN(6, RP.NLEG);
w_all_linkframe = NaN(6, RP.Leg(1).NL, RP.NLEG); % TODO: Aktuell nur symmetrische PKM
w_all_baseframe = w_all_linkframe;

% Berechne die Schnittkräfte. Siehe Aufzeichnungen Schappler, 9.5.19
for j = 1:RP.NLEG % Für alle Beinketten
  % Initialisierung der aktuellen Beinkette
  q_j = q(RP.I1J_LEG(j):RP.I2J_LEG(j));
  qD_j = qD(RP.I1J_LEG(j):RP.I2J_LEG(j));
  qDD_j = qDD(RP.I1J_LEG(j):RP.I2J_LEG(j));
  if nargin < 6
    % Schnittkräfte in der Beinkette aufgrund der internen Kräfte
    % der inversen Dynamik (Massenträgheit, Coriolis, Gravitation)
    W_j_l_int = RP.Leg(j).internforce(q_j, qD_j, qDD_j);
    % Gelenkmomente aufgrund interner Dynamik
    tau_j = (RP.Leg(j).MDH.sigma==0) .* W_j_l_int(6, 2:end)' + ...
            (RP.Leg(j).MDH.sigma==1) .* W_j_l_int(3, 2:end)';
  else
    % Zähle die zusätzlich gegebenen Gelenkmomente für die Berechnung der
    % Schnittkräfte und ignoriere die inverse Dynamik. Diese Momente sind
    % bspw. Reibkräfte (in allen Gelenken) oder Federmomente.
    % Keine internen Schnittkräfte; W_j_l_int daher Null und nicht benutzt.
    tau_j = tau_add(RP.I1J_LEG(j):RP.I2J_LEG(j));
  end
  % Antriebsmomente dieses Beins (passive sind Null)
  tau_m_j = zeros(RP.Leg(j).NQJ,1);
  tau_m_j(RP.I_qa(RP.I1J_LEG(j):RP.I2J_LEG(j))) = tauA(j); % TODO: Aktuell nur ein Antrieb pro Bein
  R_0_0j = RP.Leg(j).T_W_0(1:3,1:3); % Rotation PKM-Basis - Beinkette-Basis

  % Bein-Jacobi-Matrix für Koppelpunkt. Im PKM-Basis-KS
  J_j_0 = [R_0_0j, zeros(3,3); zeros(3,3), R_0_0j] * RP.Leg(j).jacobig(q_j);
  % Kraft, mit der die Plattform auf die Beinketten drückt (PKM-Basis-KS)
  % Die externe Kraft verursacht zusammen mit den Antriebskräften das
  % Bewegungsverhalten (q,qD,qDD) der Beinkette
  % [KhalilGue2004], Gl. 20-21; dort steht das f_ext auf der anderen Seite.
  % Daher andere Vorzeichen
  FB_j_0 = (J_j_0') \ (tau_j - tau_m_j);
  w_B(:,j) = FB_j_0;
  
  % Berechne die Schnittkräfte im Bein.
  % Modell-Annahme: Bein ist freistehende serielle Kette, an der vorne die
  % Schnittkraft mit der Plattform als externe Kraft angreift.
  % Kraft in Bein-Basis-KS umrechnen
  F_B_j_0j = rotate_wrench(FB_j_0, R_0_0j');
  % Schnittkräfte des Beins im Bein-KS berechnen ("l"=Linkframe)
  W_j_l_ext = RP.Leg(j).internforce_ext(q_j, F_B_j_0j, RP.Leg(j).NL-1, zeros(3,1));
  % Schnittkräfte aufgrund der internen Kräfte `W_j_l_int` oben berechnet

  % Gesamte Schnittkräfte: Differenz entspricht Schnittkraft im Gelenk.
  % Je nach Gelenktyp in der Struktur aufgefangen oder in Richtung des
  % Gelenk-FG
  if nargin < 6
    % Schnittkräfte durch die Dynamik der Beinkette selbst, Antriebskräfte
    % sowie die Kopplung durch die geschlossene kinematische Ketten.
    W_j_l = -W_j_l_ext + W_j_l_int;
  else
    % Anderer Berechnungsmodus: Zähle nur die Schnittkräfte durch die Gelenk-
    % momente, die vorgegeben sind. Kein Zählen als interne Dynamik. Damit
    % bspw. auch Schnittmomente in passiven Drehgelenken möglich.
    % Gleiche Formel wie im anderen Fall, aber W_j_l_int=0.
    W_j_l = -W_j_l_ext;
  end
  w_all_linkframe(:,:,j) = W_j_l;
  
  % Schnittkräfte wieder in Basis-KS zurückrechnen (sind vorher im
  % Körper-KS)
  if nargout < 3
    continue
  end
  Tc_j_0j = RP.Leg(j).fkine(q_j);
  W_j_0 = NaN(size(W_j_l));
  for k = 1:size(W_j_l,2)
    R_0j_l = Tc_j_0j(1:3,1:3,k);
    W_j_0(:,k) = rotate_wrench(W_j_l(:,k), R_0_0j*R_0j_l);
  end
  w_all_baseframe(:,:,j) = W_j_0;
end
