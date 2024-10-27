% Prüfe, welche Kinematik-Parameter kinematisch relevant sind
% Je nach geforderter Aufgabe, gibt es Kinematikparameter in den
% Beinketten, die keinen Einfluss auf die Kinematik in Aufgabenrichtung
% haben
% z.B. planarer RRR mit MDH-Verschiebung außerhalb der Ebene
%
% Eingabe:
% I_EE [6x1]
%   Marker für EE-FG, ob diese belegt sind oder nicht.
% 
% Ausgabe:
% I_rel
%   Vektor mit Einträgen, die die Parameter in pkin beschreiben.
%   1=Parameter hat Einfluss auf die Kinematik; 0=kein Einfluss.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Universität Hannover

function [I_rel] = get_relevant_pkin(Rob, I_EE)

I_rel = false(length(Rob.pkin),1);
% zufällige Gelenkwinkel
q = rand(Rob.NQJ,1);
% Parameter festlegen
pkin = Rob.pkin;
pkin(isnan(pkin)) = rand(sum(isnan(pkin)),1);
for i = 1:length(Rob.pkin)
  % Bestimme alle Transformationsmatrizen zum Vergleich der Auswirkung des
  % Parameters (nicht nur auf EE-Transformation bezogen)
  Tc_all_orig = Rob.fkine_vp(q, pkin);
  % Zufälliges Variieren des Parameters
  for j = 1:3 % drei Versuche zum Ausprobieren
    pkin_test = pkin;
    pkin_test(i) = pkin_test(i) + rand(1,1);
    % Vergleich, ob Parameter Einfluss hatte
    Tc_all_test = Rob.fkine_vp(q, pkin_test);
    % Vergleich für alle Körper-KS (Annahme: Wenn eines unterschiedlich
    % ist, sind Kollisionskörper oder Übertragungsverhältnisse anders und
    % der Parameter hat praktisch eine Auswirkung).
    for k = 1:size(Tc_all_test,3)
      Test = Rob.t2x(invtr(Tc_all_orig(:,:,k))*Tc_all_test(:,:,k));
      % Prüfe nur den Einfluss in Richtung der Aufgabe. Ist etwas ungenau
      % bei Anwendung auf 3T2R-Aufgaben, aber müsste trotzdem funktionieren
      if any(abs(Test(I_EE)) > 1e-6)
        % Parameter hat Einfluss
        I_rel(i) = true;
        break;
      end
    end
    if I_rel(i), break; end
  end
end