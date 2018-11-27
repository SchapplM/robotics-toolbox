% Optimierungs-Funktion für die Kinematikparameter
% Verändert die Kinematik-Parameter des Roboters, damit dieser eine
% Trajektorie besser abfahren kann. Benutzt PSO-Optimierung.
% 
% Eingabe:
% X_traj
%   Trajektorie von Endeffektor-Posen, die der Roboter abfahren soll
% p_mask
%   Maske für Parameter aus pkin, die durch Optimierung verändert werden
% w
%   Gewichtung der einzelnen Zielfunktionen
% 
% Ausgabe:
% p_neu
%   Neue optimale Kinematikparameter für die Trajektorie
% fval_neu
%   Funktionswert der Gütefunktion für neue Parameter
% fval_alt
%   Funktionswert für alte Parameter

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Universität Hannover

function [p_neu, fval_neu, fval_alt] = opt_pkin(RS, X_traj, p_mask, w)

if nargin < 3
  p_mask = true(length(RS.pkin), 1);
end
if nargin < 4
  w = [zeros(2,1); ...
    1e3; ...
    1; ... % q Translatorisch
    0; ... % q Rotatorisch
    1; ... % Parameter Translatorisch
    0];    % ...       Rotatorisch
end


% Charakteristische Länge des Roboters und der Aufgabe
ntraj = size(X_traj,1);
X_traj0 = repmat(RS.T_W_0(1:3,4)', ntraj, 1) - X_traj(:,1:3);
% TODO: Besser implementieren. Evtl `rms`-Befehl
DistX0 = sqrt( X_traj0(:,1).^2 + X_traj0(:,2).^2 + X_traj0(:,3).^2 );
LC = DistX0;
% TODO: Typ der Parameter in Klasseneigenschaft auslagern
types = RS.get_pkin_parameter_type();
pmin = NaN(size(RS.pkin));
pmax = pmin;
for i = 1:length(pmin)
  % Winkel-Parameter maximal 180°
  if types(i) == 1 || types(i) == 3 || types(i) == 5
    pmin(i) = -pi;
    pmax(i) = pi;
  elseif types(i) == 2 || types(i) == 4 || types(i) == 6
    % Maximale Länge der einzelnen Segmente
    pmax(i) = LC *1.5;
    pmin(i) = -pmax(i);
  else
    % offset. TODO: Fallunterscheidung sigma
  end
end

% Grenzen festlegen (nur für die tatsächlich optimierten Parameter)
lb = pmin(p_mask);
ub = pmax(p_mask);

% Optionen für Globale Optimierung festlegen
options = optimoptions('particleswarm');
options.PlotFcns={@pswplotbestf};
NumIndividuals = 20;
MaxIter = 5;
options.Display='off';
options.MaxIter = MaxIter; %70 100 % in GeneralConfig
options.StallIterLimit = 2; % 30 % 20 % 30 % defined: 40 % in GeneralConfig
options.SwarmSize = NumIndividuals; % 40, 100 % defined: 50 % in GeneralConfig
options.ObjectiveLimit = 0.1*max(w(1:3)); % Das reicht für lösbare IK (Haupt-Kriterium)
options.PlotFcn = {};
nvars = sum(p_mask);

% Gütefunktion definieren und testweise ausführen
ff = @RS.opt_pkin_fitness;
fval_alt = ff(X_traj, RS.pkin(p_mask), p_mask, w);
fitnessfcn=@(p)ff(X_traj, p, p_mask, w);

% Optimierung der Kinematikparameter mit PSO durchführen
[pm_neu,fval_neu,~, ~] = particleswarm(fitnessfcn,nvars,lb, ub, options);

% Ergebnis anpassen und ausgeben.
p_neu = RS.pkin;
p_neu(p_mask) = pm_neu;
fitnessfcn(pm_neu);


