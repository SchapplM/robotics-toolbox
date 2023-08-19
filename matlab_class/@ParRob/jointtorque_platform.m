% Einfluss von Gelenkmomenten in den Beinketten auf die PKM-Dynamik
% (bezogen auf Endeffektor-Plattform). Einfluss z.B. durch Reibung oder in
% Gelenken befestigten Federn.
%
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% xP [6x1]
%   Plattform-Pose des Roboters bezüglich des Basis-KS (Nicht: EE-KS)
% tau [N x NTau]
%   Gelenkmomente in den Gelenken der Beinketten. Entweder eine Spalte (NTau=1)
%   oder mehrere zur effizienten Berechnung mehrerer Kräfte für eine Pose.
% Jinv [N x Nx] (optional zur Rechenersparnis)
%   Vollständige Inverse Jacobi-Matrix der PKM (bezogen auf alle N aktiven
%   und passiven Gelenke und die bewegliche Plattform-Koordinaten xP).
%   Nicht bezogen auf die EE-Koordinaten (nicht identisch zu Matrix aus InvKin)
% 
% Ausgabe:
% taured_Fs
%   Aus Gelenkmomenten resultierende PKM-Kräfte (bezogen auf EE-Koordinaten
%   der PKM; Kräfte und Momente in kartesischen Koordinaten, bezogen auf Basis-KS)
% taured_Fs_reg
%   Regressormatrix der PKM-Kräfte. Wird mit Gelenkmomenten multipliziert.
%   Die PKM-Kräfte sind linear in den Kräften. Entspricht der Jacobi-Matrix
%   Muss zusätzlich nachverarbeitet werden, falls diese Matrix linear z.B.
%   in den Reibparametern sein soll.

% Quelle:
% [DT09] Do Thanh, T. et al: On the inverse dynamics problem of general
% parallel robots, ICM 2009

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [taured_Fs, taured_Fs_reg] = jointtorque_platform(Rob, q, xP, tau, Jinv)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/jointtorque_platform: q muss %dx1 sein', Rob.NJ);
assert(isreal(xP) && all(size(xP) == [6 1]), ...
  'ParRob/jointtorque_platform: xP muss 6x1 sein');
assert(isreal(tau) && all(size(tau,1) == [Rob.NJ]), ...
  'ParRob/jointtorque_platform: tau muss %dxNTau sein', Rob.NJ);
if nargin == 5
  assert(isreal(Jinv) && all(size(Jinv) == [Rob.NJ sum(Rob.I_EE)]), ...
    'ParRob/jointtorque_platform: Jinv muss %dx%d sein', Rob.NJ, sum(Rob.I_EE));
end

NLEG = Rob.NLEG;
NJ = Rob.NJ;

if Rob.issym
  error('Nicht implementiert')
end

%% Projektionsmatrizen
if nargin < 5
  % Gradientenmatrix der vollständigen Zwangsbedingungen
  G_q = Rob.constr1grad_q(q, xP, true);
  G_x = Rob.constr1grad_x(q, xP, true);
  % Inverse Jacobi-Matrix
  Jinv = - G_q \ G_x; % Siehe: ParRob/jacobi_qa_x
end

%% Berechnung der Projektion
% Projektion aller Terme der Subsysteme [DT09]/(23)
% Die Momente liegen noch bezogen auf die Euler-Winkel vor (entsprechend der
% EE-Koordinaten)
taured_Fx = Jinv'* tau;

% Umrechnung der Momente auf kartesische Koordinaten (Basis-KS des
% Roboters)
Tw = euljac(xP(4:6), Rob.phiconv_W_E);
H = [eye(3), zeros(3,3); zeros(3,3), Tw];
taured_Fs = H(Rob.I_EE,Rob.I_EE)' \  taured_Fx;

if nargout == 2
  taured_Fs_reg = H(Rob.I_EE,Rob.I_EE)' \  Jinv';
end
