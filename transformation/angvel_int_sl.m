% Integration von Winkelgeschwindigkeiten berechnen
% Nutze ein Simulink-Modell
% 
% Eingabe:
% t_in [ni x 1]
%   Zeitreihe als Eingabe in das Simulink-Modell (streng monoton steigende
%   Zeitpunkte. Müssen aber nicht mit Schrittweite Ts vorliegen)
% omega [ni x 3]
%   Zeitreihe von Winkelgeschwindigkeiten, die integriert werden
%   Koordinatensystem, in dem die Vektoren definiert sind kann gewählt
%   werden.
% Ts [1x1]
%   Abtastrate des Simulink-Modells (nicht zwangsläufig Abtastrate der
%   Eingabedaten)
% R_W_B_t0 [3x3]
%   Anfangswert für die Rotationsmatrix
% frame [1x1] logical
%   Kennzeichnung, in welchem Koordinatensystem die Winkelgeschwindigkeit
%   angegeben wird.
%   true: Welt-KS; z.B. für gegebene Winkelgeschwindigkeiten eines
%         Roboter-Endeffektors [Standard-Einstellung]
%   false: Körper-KS (mitgedrehtes KS); z.B. für Drehratensensor-Rohdaten
% 
% Ausgabe:
% R_int [3 x 3 x no]
%   Zeitreihe von Rotationsmatrizen, die aus aufintegrierten
%   Winkelgeschwindigkeiten berechnet werden.
% t_out [no x 1]
%   Zeitwerte zu den Werten in R_int
%   Der Anfangszeitpunkt ist identisch mit dem aus t_in

% Moritz Schappler, schappler@irt.uni-hannover.de, 2017-03
% (c) Institut für Regelungstechnik, Universität Hannover

function [R_int, t_out] = angvel_int_sl(t_in, omega, Ts, R_W_B_t0, frame)

if nargin < 5
  frame = true;
end

% Quaternion-Methode. Hier ist eine Normierung möglich und Rundungsfehler werden eliminiert
if frame == true
  sl_Modellname = 'angvel_int_quat';
else
  sl_Modellname = 'angvel_body_int_quat';
end
options=simset('FixedStep',sprintf('%e',Ts), 'Solver', 'ode4', ...
  'ReturnWorkspaceOutputs','on', ... % Ausgabe als Signalstruktur
  'SrcWorkspace', 'current'); % Variablennamen des Funktions-Workspace benutzen
R_t0 = R_W_B_t0; %#ok<NASGU>
simOut = sim(sl_Modellname, t_in(end)-t_in(1),options, [t_in-t_in(1), omega]);
sl = get_simulink_outputs(simOut, sl_Modellname);
R_int = sl.R_W_B;
t_out = sl.t + t_in(1);