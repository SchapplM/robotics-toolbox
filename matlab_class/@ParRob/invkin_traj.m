% Inverse Kinematik für allgemeinen Roboter (Komplette Trajektorie)
% Allgemeine, stark parametrierbare Funktion zum Aufruf mit allen möglichen
% Einstellungen
% Iterative Lösung der inversen Kinematik mit inverser Jacobi-Matrix
% Zusätzlich Nutzung der differentiellen Kinematik für schnellere Konvergenz
% 
% Eingabe:
% XE
%   Trajektorie von EE-Lagen (Sollwerte)
% XDE
%   Trajektorie von EE-Geschwindigkeiten (Sollwerte)
%   (Die Orientierung wird durch Euler-Winkel-Zeitableitung dargestellt)
% XDDE
%   Trajektorie von EE-Beschleunigungen (Sollwerte)
%   Orientierung bezogen auf Euler-Winkel
% T
%   Zeitbasis der Trajektorie (Alle Zeit-Stützstellen)
% q0
%   Anfangs-Gelenkwinkel für Algorithmus
% s
%   Struktur mit Eingabedaten. Felder, siehe Quelltext.
% 
% Ausgabe:
% Q
%   Trajektorie von Gelenkpositionen (Lösung der IK)
% QD
%   Trajektorie von Gelenkgeschwindigkeiten
% QDD
%   Trajektorie von Gelenkbeschleunigungen
% 
% Siehe auch: SerRob/invkin_traj

% TODO: Nullraumbewegung mit Nebenbedingung
% TODO: Erfolg der IK prüfen

% Quelle:
% [2] Aufzeichnungen Schappler vom 11.12.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Q, QD, QDD, Phi] = invkin_traj(Rob, X, XD, XDD, T, q0, s)

s_std = struct( ...
  'I_EE', Rob.I_EE_Task, ... % FG für die IK
  'mode_IK', 1, ...  % 1=Seriell, 2=PKM
  'debug', false); % Zusätzliche Ausgabe
if nargin < 7
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
dof_3T2R = false;
mode_IK = s.mode_IK;
I_EE = Rob.I_EE;
if all(s.I_EE == logical([1 1 1 1 1 0]))
  dof_3T2R = true;
  I_EE = s.I_EE;
end
nt = length(T);


Q = NaN(nt, Rob.NJ);
QD = Q;
QDD = Q;
Phi = NaN(nt, length(Rob.I_constr_t_red)+length(Rob.I_constr_r_red));

qk0 = q0;
for k = 1:nt
  tic();
  x_k = X(k,:)';
  xD_k = XD(k,:)';
  xDD_k = XDD(k,:)';
  
  if mode_IK == 2
    % 3T2R-Funktion
    [q_k, Phi_k] = Rob.invkin3(x_k, qk0, s);
  else
     % Aufruf der Einzel-Beinketten-Funktion (etwas schneller, falls mit mex)
    [q_k, Phi_k] = Rob.invkin_ser(x_k, qk0, s);
  end
  % Gelenk-Geschwindigkeit berechnen
  if ~dof_3T2R
    Phi_q = Rob.constr1grad_q(q_k, x_k);
    Phi_x = Rob.constr1grad_x(q_k, x_k);
    J_x_inv = -Phi_q \ Phi_x;
  else
    % Nehme vollständige ZB-Gradienten (2. Ausgabe) und wähle Komponenten
    % hier aus. Reduzierte ZB sind noch nicht vollständig implementiert für
    % Systeme mit Beinketten mit fünf Gelenken.
    [~,Phi_q] = Rob.constr3grad_q(q_k, x_k);
    [~,Phi_x] = Rob.constr3grad_x(q_k, x_k);
    I = Rob.I_constr_red;
    J_x_inv = -Phi_q(I,:) \ Phi_x(I,1:5);
  end

  qD_k = J_x_inv * xD_k(I_EE);
  
  % Gelenk-Beschleunigung berechnen
  % TODO: Das ist noch falsch. Jacobi-Zeitableitung fehlt nocht
  qDD_k = J_x_inv * xDD_k(I_EE);

  % Aus Geschwindigkeit berechneter neuer Winkel für den nächsten Zeitschritt
  % Taylor-Reihe bis 2. Ordnung für Position (Siehe [2])
  if k < nt
    dt = T(k+1)-T(k);
    qk0 = q_k + qD_k*dt + 0.5*qDD_k*dt^2;
  end
  
  % Ergebnisse speichern
  Q(k,:) = q_k;
  QD(k,:) = qD_k;
  QDD(k,:) = qDD_k;
  Phi(k,:) = Phi_k;
  if s.debug
    if max(abs(Phi_k)) > 1e-3
      warning('Phi zu groß');
      break;
    end
    fprintf('Iteration %d/%d (%1.1f%%). Zeit %1.4f. Geschätzte Restzeit: %1.1fmin\n',...
      k, nt, 100*k/nt, toc(),(nt-k)*toc()/60);
  end
end