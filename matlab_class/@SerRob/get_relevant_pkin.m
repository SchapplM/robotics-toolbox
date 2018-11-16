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
q = rand(Rob.NJ,1);
for i = 1:length(Rob.pkin)
  % Zufälliges Variieren des Parameters

  T_EE_orig = Rob.fkineEE(q);
  x_orig = [T_EE_orig(1:3,4); r2eul(T_EE_orig(1:3,1:3), Rob.phiconv_W_E)];
  for j = 1:3 % drei Versuche zum Ausprobieren
    pkin_test = Rob.pkin;
    pkin_test(i) = pkin_test(i) + rand(1,1);
    % Vergleich, ob Parameter Einfluss hatte
    T_EE_test = Rob.fkineEE_vp(q, pkin_test);
    x_test = [T_EE_test(1:3,4); r2eul(T_EE_test(1:3,1:3), Rob.phiconv_W_E)];
    Test = x_orig - x_test; % TODO: Differenz von Euler-Winkeln unglücklich
    if any(abs(Test(I_EE)) > 1e-6)
      % Parameter hat Einfluss
      I_rel(i) = true;
      break;
    end
  end
end