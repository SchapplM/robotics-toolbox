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
%   * Plattform-KS
% JointPos_all
%   gestapelte Positionen aller Gelenke der PKM (als Zeilenvektor)
%   (Letzte Spalte von Tc_stack_PKM)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-06
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [Tc_stack_PKM, JointPos_all] = fkine_coll(Rob, q)

Tc_stack_PKM = NaN((Rob.NL+Rob.NLEG)*3,4); % siehe fkine_legs; dort aber leicht anders
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
% Plattform-KS aus den Daten der ersten Beinkette eintragen
T_0_N1 = [Tc_stack_PKM(3+(3*Rob.Leg(1).NL-2:3*Rob.Leg(1).NL),:); [0 0 0 1]];
T_0_E1 = T_0_N1 * Rob.Leg(1).T_N_E;
% Bei einigen 2T1R-PKM zusätzliche Trafo N-E für Beinkette
R_P_B1 = eulxyz2r(Rob.phi_P_B_all(:,1));
r_P_P_B1 = Rob.r_P_B_all(:,1);
T_P_B1 = [[R_P_B1, r_P_P_B1]; [0 0 0 1]];
T_0_P_via1 = T_0_E1 * invtr(T_P_B1);
Tc_stack_PKM(end-2:end,:) = T_0_P_via1(1:3,:);
% Positionen in separate Ausgabe extrahieren
JointPos_all = Tc_stack_PKM(:,4)';