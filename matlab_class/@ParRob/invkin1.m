% Inverse Kinematik für allgemeinen Roboter
% % Variante 1:
% * Translation vom Basis- zum Koppelpunkt-KS
% * Absolute Rotation ausgedrückt bspw. in XYZ-Euler-Winkeln
%   (statt XYZ wird die Konvention aus `phiconv_W_E` genommen)
% * Rotationsfehler ausgedrückt genauso bspw. in XYZ-Euler-Winkeln
% 
% Numerische Berechnung mit Inverser Jacobi-Matrix der inversen Kinematik.
% Dadurch Berechnung aller Gelenkwinkel aller Beine auf einmal
% 
% Eingabe:
% xE_soll [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS (Soll)
% q0 [Nx1]
%   Startkonfiguration: Alle Gelenkwinkel aller serieller Beinketten der PKM
% s
%   Struktur mit Eingabedaten. Felder, siehe Quelltext.
% 
% Ausgabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM als Lösung der IK
% Phi
%   Kinematische Zwangsbedingungen für die Lösung. Bei korrekter Berechnung
%   muss dieser Wert Null sein.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [q, Phi] = invkin1(Rob, xE_soll, q0, s)

%% Initialisierung
assert(isreal(xE_soll) && all(size(xE_soll) == [6 1]), ...
  'ParRob/invkin1: xE_soll muss 6x1 sein');
assert(isreal(q0) && all(size(q0) == [Rob.NJ 1]), ...
  'ParRob/invkin1: q0 muss %dx1 sein', Rob.NJ);
s_std = struct('n_min', 0, ... % Minimale Anzahl Iterationen
               'n_max', 1000, ... % Maximale Anzahl Iterationen
               'Phit_tol', 1e-8, ... % Toleranz für translatorischen Fehler
               'Phir_tol', 1e-8); % Toleranz für rotatorischen Fehler
if nargin < 4
  % Keine Einstellungen übergeben. Standard-Einstellungen
  s = s_std;
end
% Prüfe Felder der Einstellungs-Struktur und setze Standard-Werte, falls
% Eingabe nicht gesetzt
for f = fields(s_std)'
  if ~isfield(s, f{1})
    s.(f{1}) = s_std.(f{1});
  end
end

% Variablen aus Einstellungsstruktur holen
n_min = s.n_min;
n_max = s.n_max;
Phit_tol = s.Phit_tol;
Phir_tol = s.Phir_tol;
I_constr_t_red = Rob.I_constr_t_red;
I_constr_r_red = Rob.I_constr_r_red;
%% Definitionen
% Variablen zum Speichern der Zwischenergebnisse
q1 = q0;
sigma_PKM = Rob.MDH.sigma; % Marker für Dreh-/Schubgelenk
K = 0.1*ones(Rob.NJ,1);
K(sigma_PKM==1) = K(sigma_PKM==1) / 5; % Verstärkung für Schubgelenke kleiner
%% Iterative Lösung der IK
for jj = 2:n_max

  % Gesamt-Jacobi bilden (reduziert um nicht betrachtete EE-Koordinaten)
  Jik_voll=Rob.constr1grad_q(q1, xE_soll);
  Jik = Jik_voll(:,:);

  % Grad der Nicht-Erfüllung der Zwangsbedingungen (Fehler)
  Phi = Rob.constr1(q1, xE_soll);
  
  % Iterationsschritt mit Inverser Jacobi
  delta_q = Jik \ (-Phi);

  % Inkrement der Gelenkwinkel
  q2 = q1 + K.*delta_q;
  q1 = q2;
  q1(sigma_PKM==0) = normalize_angle(q1(sigma_PKM==0)); % nur Winkel normalisieren
  
  if jj > n_min ... % Mindestzahl Iterationen erfüllt
      && max(abs(Phi(I_constr_t_red))) < Phit_tol && max(abs(Phi(I_constr_r_red))) < Phir_tol % Haupt-Bedingung ist erfüllt
    break;
  end
  
  if all(abs(Phi) < 1e-10)
    break;
  end
end
q = q1;
