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

nt = length(T);
I_EE = Rob.I_EE;

Q = NaN(nt, Rob.NJ);
QD = Q;
QDD = Q;
Phi = NaN(nt, Rob.NLEG*sum(I_EE)); % TODO: Robuste Dimensionsinitialisierung


qk0 = q0;
for k = 1:nt
  x_k = X(k,:)';
  xD_k = XD(k,:)';
  xDD_k = XDD(k,:)';
  
  % TODO: IK-Methode hier frei wählbar machen über Einstellungen
  [q_k, Phi_k] = Rob.invkin1(x_k, qk0, s);

  % Gelenk-Geschwindigkeit berechnen
  Phi_q = Rob.constr1grad_q(q_k, x_k);
  Phi_x = Rob.constr1grad_x(q_k, x_k);
  J_x_inv = -Phi_q \ Phi_x;
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
end