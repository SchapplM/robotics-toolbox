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
% Siehe auch: ParRob/invkin_traj

% TODO: Nullraumbewegung mit Nebenbedingung
% TODO: Erfolg der IK prüfen
% TODO: Zur Berechnung der Jacobi-Matrizen + Zeitableitung werden viele
% Operationen doppelt in der Roboterklasse durchgeführt.

% Quelle:
% [1] Aufzeichnungen Schappler vom 28.11.2018
% [2] Aufzeichnungen Schappler vom 11.12.2018

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Q, QD, QDD, PHI] = invkin_traj(Rob, X, XD, XDD, T, q0, s)

if nargin < 7
  s = struct('constr_m', 2); % Muss für Übergabe an IK-Funktion definiert sein.
end

I_EE = Rob.I_EE;

Q = NaN(length(T), Rob.NQJ);
QD = Q;
QDD = Q;
PHI = NaN(length(T), sum(Rob.I_EE));

nt = length(T);
qk0 = q0;
for k = 1:nt
  [q_k, Phi_k] = Rob.invkin(X(k,:)',qk0, s);
  
  % Gelenk-Geschwindigkeit berechnen (Siehe [1]).
  J_x = Rob.jacobia(q_k);
  qD_k = J_x(I_EE,:) \ XD(k,I_EE)';
  
  % Gelenk-Beschleunigung berechnen
  JD_x = Rob.jacobiaD(q_k, qD_k);
  qDD_k = J_x(I_EE,:) \ (XDD(k,I_EE)' - JD_x(I_EE,:)*qD_k);
  
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
  PHI(k,:) = Phi_k;
end