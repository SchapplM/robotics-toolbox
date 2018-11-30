% Güte-Funktion zur Optimierung der Kinematikparameter
% Wird von opt_pkin benutzt.
% 
% Eingabe:
% X_traj
%   Trajektorie von Endeffektor-Posen, die der Roboter abfahren soll
% p
%   Zu prüfender Satz Kinematik-Parameter. Die Parameter sind nur
%   diejenigen aus pkin, die mit der Maske aus p_mask angesprochen sind.
% p_mask
%   Maske für Parameter aus pkin, die durch Optimierung verändert werden
% w
%   Gewichtung der einzelnen Zielfunktionen
% 
% Ausgabe:
% fval
%   Zielfunktionswert für zu prüfenden Parameter

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Universität Hannover

function fval = opt_pkin_fitness(RS, X_traj, p, p_mask, w)

p_akt = RS.pkin;
p_neu = p_akt;
p_neu(p_mask) = p;
RS.update_mdh(p_neu);

q0 = rand(RS.NJ,1);

fval_ges = NaN(size(X_traj,1), 1);
F_ges = NaN(size(X_traj,1), 7);
for i = 1:size(X_traj,1)
  xE_soll = X_traj(i,:)';
  if i > 1
    q0 = q;
  end
  % IK berechnen. TODO: Methode 1 wird hier nur vorgegeben, damit
  % Kompatibel zu Auswahl der EE-FG mit I_EE
  [q, Phi] = RS.invkin(xE_soll, q0, struct('constr_m', 1));
  % leichte Verletzung der ZB abrunden (damit sie für die Gütefunktion
  % keine Rolle spielen)
  Phi(abs(Phi)<1e-6) = 0;
  
  % Gütekriterien ausrechnen
  % TODO: Richtige Betrachtung der ZB-Arten
%   F_ges(i,1) = norm(Phi(1:3));
%   F_ges(i,2) = norm(Phi(4:6));
  F_ges(i,3) = norm(Phi);
  
  % Rotationsgelenke
  qR = q(RS.MDH.sigma==0);
  qT = q(RS.MDH.sigma==1);
  F_ges(i,4) = norm(qT);
  F_ges(i,5) = norm(qR);
  
  % Längen der Geometrieparameter
  types = RS.get_pkin_parameter_type(); % TODO: Als Klasseneigenschaft
  pt = p_neu(types == 2 | types == 4 | types == 6);
  pr = p_neu(types == 1 | types == 3 | types == 5);
  
  F_ges(i,6) = norm(pt);
  F_ges(i,7) = norm(pr);
  fval_ges(i) = 0;
  for j = 1:7
    if w(j) ~= 0
      fval_ges(i) = fval_ges(i) + w(j) * F_ges(i,j);
    end
  end
end

fval = norm(fval_ges);