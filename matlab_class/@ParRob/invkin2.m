% Inverse Kinematik fÃŒr allgemeinen Roboter
% Variante 2:
% Implementierung mit Führungs-Beinkette und Folge-Beinketten 
% 
% Numerische Berechnung mit Inverser Jacobi-Matrix der inversen Kinematik.
% Dadurch Berechnung aller Gelenkwinkel aller Beine auf einmal
% 
% Eingabe:
% xE_soll [6x1]
%   Endeffektorpose des Roboters bezÃŒglich des Basis-KS (Soll)
% q0 [Nx1]
%   Startkonfiguration: Alle Gelenkwinkel aller serieller Beinketten der PKM
% s
%   Struktur mit Eingabedaten. Felder, siehe Quelltext.
% 
% Ausgabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM als LÃ¶sung der IK
% Phi
%   Kinematische Zwangsbedingungen fÃŒr die LÃ¶sung. Bei korrekter Berechnung
%   muss dieser Wert Null sein.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut fÃŒr Mechatronische Systeme, UniversitÃ€t Hannover

function [q, Phi] = invkin2(Rob, xE_soll, q0, s)

%% Initialisierung
assert(isreal(xE_soll) && all(size(xE_soll) == [6 1]), ...
  'ParRob/invkin1: xE_soll muss 6x1 sein');
assert(isreal(q0) && all(size(q0) == [Rob.NJ 1]), ...
  'ParRob/invkin1: q0 muss %dx1 sein', Rob.NJ);
s_std = struct('task_red', false, ...
               'n_min', 0, ... % Minimale Anzahl Iterationen
               'n_max', 1000, ... % Maximale Anzahl Iterationen
               'Phit_tol', 1e-8, ... % Toleranz fÃŒr translatorischen Fehler
               'Phir_tol', 1e-8); % Toleranz fÃŒr rotatorischen Fehler
if nargin < 4
  % Keine Einstellungen ÃŒbergeben. Standard-Einstellungen
  s = s_std;
end
% PrÃŒfe Felder der Einstellungs-Struktur und setze Standard-Werte, falls
% Eingabe nicht gesetzt
for f = fields(s_std)'
  if ~isfield(s, f{1})
    s.(f{1}) = s_std.(f{1});
  end
end

% Variablen aus Einstellungsstruktur holen
task_red = false;
n_min = s.n_min;
n_max = s.n_max;
Phit_tol = s.Phit_tol;
Phir_tol = s.Phir_tol;
%%
if task_red == false
  I_constr_t_red = Rob.I_constr_t_red;
  I_constr_r_red = Rob.I_constr_r_red;
else
  nPhit = 3;
  nPhir = 2;
  nPhi = nPhit + nPhir;  
  Rob.I_constr_t_red = zeros(nPhit*Rob.NLEG,1);
  Rob.I_constr_r_red = zeros(nPhir*Rob.NLEG,1);
  I_constr_t_red = Rob.I_constr_t_red;
  I_constr_r_red = Rob.I_constr_r_red;
  % Indizes bestimmen
  for i = 1:Rob.NLEG
     Rob.I_constr_t_red(nPhit*(i-1)+1:nPhit*i) = (i-1)*nPhi+1:(i)*nPhi-nPhir;
     Rob.I_constr_r_red(nPhir*(i-1)+1:nPhir*i) = (i-1)*nPhi+1+nPhit:(i)*nPhi;
     I_constr_t_red = Rob.I_constr_t_red;
     I_constr_r_red = Rob.I_constr_r_red;     
  end
end
%% Definitionen
% Variablen zum Speichern der Zwischenergebnisse
q1 = q0;
sigma_PKM = Rob.MDH.sigma; % Marker fÃŒr Dreh-/Schubgelenk
K = 1*ones(Rob.NJ,1);
K(sigma_PKM==1) = K(sigma_PKM==1) / 5; % VerstÃ€rkung fÃŒr Schubgelenke kleiner

%I_IK = 1:6;
if task_red
  xE_soll(6) = 0; % Dieser Wert hat keinen Einfluss auf die Berechnung, darf aber aufgrund der Implementierung nicht NaN sein.
  % Indizes zur Auswahl der berÃŒcksichtigten ZB-Komponenten
%  I_IK = [1 2 3 5 6]; % Nehme an, dass immer die vierte Komponente des Fehlers weggelassen wird
elseif any(~Rob.I_EE)
  % Falls EE-FG nicht gefordert sind: Streiche die entsprechenden Zeilen in
  % den ZB und der Jacobi
%  I_IK = find(Rob.I_EE);
  task_red = true; % TODO: Eigener Marker hierfÃŒr
end
%% Iterative LÃ¶sung der IK
for jj = 2:n_max

  % Gesamt-Jacobi bilden (reduziert um nicht betrachtete EE-Koordinaten)
  Jik_voll=Rob.constr2grad_q(q1, xE_soll);
  Jik = Jik_voll(:,:);

  % Grad der Nicht-ErfÃŒllung der Zwangsbedingungen (Fehler)
  Phi = Rob.constr2(q1, xE_soll);
  
%% Aufgabenredundanz
 % if task_red
  % Aufgabenredundanz. Lasse den letzten Rotations-FG wegfallen
 %   Jik = Jik(I_IK,:);
 %   Phi = Phi(I_IK);
 % end
  
%% Iterationsschritt mit Inverser Jacobi
  delta_q = Jik \ (-Phi);

  % Inkrement der Gelenkwinkel
  q2 = q1 + K.*delta_q;
  q1 = q2;
  q1(sigma_PKM==0) = normalize_angle(q1(sigma_PKM==0)); % nur Winkel normalisieren
  
  if jj > n_min ... % Mindestzahl Iterationen erfÃŒllt
      && max(abs(Phi(I_constr_t_red))) < Phit_tol && max(abs(Phi(I_constr_r_red))) < Phir_tol % Haupt-Bedingung ist erfÃŒllt
    break;
  end
  
  if all(abs(Phi) < 1e-10)
    break;
  end
end
q = q1;
