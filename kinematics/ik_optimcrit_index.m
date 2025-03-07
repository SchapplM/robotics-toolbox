% Indizes für die Variablen für Kriterien der Nullraumbewegung.
% Dient zur Vermeidung hart kodierter Indizes im Quelltext
% 
% Eingabe:
% Type
%   Typ des Roboters. 0/1=SerRob, 2=ParRob
% 
% Ausgabe:
% idx_ikpos_wn
%   Gewichtungsfaktoren in der Einzelpunkt-IK (Eingabe-Variable `wn`)
% idx_ikpos_hn
%   Zielfunktionen in der Einzelpunkt-IK (Ausgabe-Variable `h`)
%   Index 1 in der Funktion ist die Summe. 1 hier ist also dort 2.
% idx_iktraj_wnP
%   Gewichtungsfaktoren der P-Verstärkungen in der Trajektorien-IK
%   bezogen auf die Beschleunigung (in der Eingabe-Variable `wn`)
% idx_iktraj_wnD
%   Gewichtungsfaktoren der D-Verstärkungen in der Trajektorien-IK
%   bezogen auf die Beschleunigung. Ist also eine P-Verstärkung auf die 
%   Geschwindigkeit (auch in der Eingabe-Variable `wn`)
% idx_iktraj_hn
%   Zielfunktionen in der Trajektorien-IK
% idx_ik_length
%   Struktur mit Einträgen für die Länge der jeweils indizierten Vektoren.
% 
% Siehe auch: SerRob/invkin2, SerRob/invkin2_traj, ParRob/invkin3, 
%             ParRob/invkin4, ParRob/invkin_traj, ParRob/invkin2_traj
% (dort sind die Indizes im IK-Algorithmus verwendet)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [idx_ikpos_wn, idx_ikpos_hn, idx_iktraj_wnP, idx_iktraj_wnD, ...
  idx_iktraj_hn, idx_ik_length] = ik_optimcrit_index(Type)
idx_ik_length = struct('wnpos', NaN, 'wntraj', NaN, 'hnpos', NaN, 'hntraj', NaN);
% Anzahl der Elemente in indiziertem Vektor
if Type < 2 % SerRob
  idx_ik_length.wnpos = 13;
else % ParRob
  idx_ik_length.wnpos = 11;
end
idx_ikpos_wn = struct( ...
  'qlim_par', 1, ...
  'qlim_hyp', 2, ...
  'ikjac_cond', 3, ...
  'jac_cond', 4, ...
  'coll_hyp', 5, ...
  'instspc_hyp', 6, ...
  'xlim_par', 7, ...
  'xlim_hyp', 8, ...
  'coll_par', 9, ...
  'instspc_par', 10, ...
  'poserr_ee', 11);
if Type < 2 % SerRob
  % Für ParRob noch nicht implementiert, für SerRob nur in PTP-IK.
  idx_ikpos_wn.xlim_trans_par = 12;
  idx_ikpos_wn.xlim_trans_hyp = 13;
end
% Gleiche Reihenfolge in Ausgabe-Variable h
idx_ik_length.hnpos = idx_ik_length.wnpos;
idx_ikpos_hn = idx_ikpos_wn;
% Trajektorien-Indizes, aufgeteilt in P- und D-Anteil
idx_ik_length.wntraj = 25;
idx_iktraj_wnP = struct( ...
  'qlim_par', 1, ...
  'qlim_hyp', 2, ...
  'qDlim_par', 3, ...
  'qDlim_hyp', 4, ...
  'ikjac_cond', 5, ...
  'jac_cond', 6, ...
  'coll_hyp', 11, ...
  'instspc_hyp', 13, ...
  'xlim_par', 15, ...
  'xlim_hyp', 17, ...
  'xDlim_par', 19, ...
  'coll_par', 20, ...
  'instspc_par', 22, ...
  'poserr_ee', 24);
idx_iktraj_wnD = struct( ...
  'qlim_par', 7, ...
  'qlim_hyp', 8, ...
  'ikjac_cond', 9, ...
  'jac_cond', 10, ...
  'coll_hyp', 12, ...
  'instspc_hyp', 14, ...
  'xlim_par', 16, ...
  'xlim_hyp', 18, ...
  'coll_par', 21, ...
  'instspc_par', 23, ...
  'poserr_ee', 25);
idx_ik_length.hntraj = 14;
idx_iktraj_hn = struct( ...
  'qlim_par', 1, ...
  'qlim_hyp', 2, ...
  'qDlim_par', 3, ...
  'qDlim_hyp', 4, ...
  'ikjac_cond', 5, ...
  'jac_cond', 6, ...
  'coll_hyp', 7, ...
  'instspc_hyp', 8, ...
  'xlim_par', 9, ...
  'xlim_hyp', 10, ...
  'xDlim_par', 11, ...
  'coll_par', 12, ...
  'instspc_par', 13, ...
  'poserr_ee', 14);
