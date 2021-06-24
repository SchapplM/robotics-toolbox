% Direkte Kinematik für alle KS bzw. Gelenkpositionen der PKM, die für
% Kollisionen notwendig sind
% 
% Eingabe:
% q
%   Gelenkkoordinaten aller Beinkette (inkl. Koppel-Gelenke)
% 
% Ausgabe:
% Tc_stack_PKM [3*N x 4] (N ist die Anzahl der benutzten Körper-KS)
%   Gestapelte Transformationsmatrizen der PKM. Im Basis-KS. 
%   Entspricht mit Abwandlung der Anordnung wie in fkine: 
%   * PKM-Basis
%   * Für jede Beinkette: Basis und alle bewegten Körper-KS. Ohne
%     virtuelles EE-KS
%   * Kein Plattform-KS
% JointPos_all
%   gestapelte Positionen aller Gelenke der PKM (als Zeilenvektor)
%   (Letzte Spalte von Tc_stack_PKM)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-06
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [Tc_stack_PKM, JointPos_all] = fkine_coll(Rob, q)

Tc_stack_PKM = NaN((Rob.NL-1+Rob.NLEG)*3,4); % siehe fkine_legs; dort aber leicht anders
% Basis-KS. Trägt keine Information. Dient nur zum einfacheren Zugriff auf
% die Variable und zur Angleichung an Darstellung im Welt-KS.
Tc_stack_PKM(1:3,1:4) = eye(3,4); % Basis-KS im Basis-KS.
out3_ind1 = 3; % Zeilenzähler für obige Variable (drei Zeilen stehen schon)
for i = 1:Rob.NLEG
  [~, ~, Tc_stack_0i] = Rob.Leg(i).fkine(q(Rob.I1J_LEG(i):Rob.I2J_LEG(i)));
  T_0_0i = Rob.Leg(i).T_W_0;
  % Umrechnung auf PKM-Basis-KS. Nehme nur die KS, die auch einem Körper
  % zugeordnet sind. In Tc_stack_0i bei hybriden Systemen teilw. mehr.
  Tc_stack_0 = NaN(3*(Rob.Leg(i).NL),4);
  for kk = 1:Rob.Leg(i).NL
    Tc_stack_k = Tc_stack_0i(3*(kk-1)+1:kk*3,1:4);
    T_0_kk = T_0_0i * [Tc_stack_k;0 0 0 1];
    Tc_stack_0((kk-1)*3+1:kk*3,1:4) = T_0_kk(1:3,:);
  end
  % Eintragen in Ergebnis-Variable
  Tc_stack_PKM(out3_ind1+(1:3*Rob.Leg(i).NL),:) = Tc_stack_0;
  out3_ind1 = out3_ind1 + 3*Rob.Leg(i).NL;
end

JointPos_all = Tc_stack_PKM(:,4)';