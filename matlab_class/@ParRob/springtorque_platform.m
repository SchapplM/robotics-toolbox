% Einfluss von in Gelenken angebrachten Federn auf die PKM-Dynamik
% (bezogen auf Endeffektor-Plattform)
%
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% xP [6x1]
%   Plattform-Pose des Roboters bezüglich des Basis-KS (Nicht: EE-KS)
% Jinv [N x Nx] (optional zur Rechenersparnis)
%   Vollständige Inverse Jacobi-Matrix der PKM (bezogen auf alle N aktiven
%   und passiven Gelenke und die bewegliche Plattform-Koordinaten xP).
%   Nicht bezogen auf die EE-Koordinaten (nicht identisch zu Matrix aus InvKin)
% 
% Ausgabe:
% taured_Fs
%   Federkräfte (bezogen auf EE-Koordinaten der PKM; Kräfte und
%   Momente in kartesischen Koordinaten, bezogen auf Basis-KS)

% Quelle:
% [DT09] Do Thanh, T. et al: On the inverse dynamics problem of general
% parallel robots, ICM 2009

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function taured_Fs = springtorque_platform(Rob, q, xP, Jinv)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/springtorque_platform: q muss %dx1 sein', Rob.NJ);
assert(isreal(xP) && all(size(xP) == [6 1]), ...
  'ParRob/springtorque_platform: xP muss 6x1 sein');
if nargin == 4
  assert(isreal(Jinv) && all(size(Jinv) == [Rob.NJ sum(Rob.I_EE)]), ...
    'ParRob/springtorque_platform: Jinv muss %dx%d sein', Rob.NJ, sum(Rob.I_EE));
end

NLEG = Rob.NLEG;
NJ = Rob.NJ;

if Rob.issym
  error('Nicht implementiert')
end
% Variable zum Speichern der vollständigen Gravitations-Kräfte (Subsysteme)
TAU_full = NaN(NJ,1);

%% Projektionsmatrizen
if nargin < 4
  % Gradientenmatrix der vollständigen Zwangsbedingungen
  G_q = Rob.constr1grad_q(q, xP, true);
  G_x = Rob.constr1grad_x(q, xP, true);
  % Inverse Jacobi-Matrix
  Jinv = - G_q \ G_x; % Siehe: ParRob/jacobi_qa_x
end

%% Berechnung der Projektion
% Gravitations-Komponente aller Beinketten berechnen [DT09]/(6)
ii = 1;
for i = 1:NLEG
  q_i = q(Rob.I1J_LEG(i):Rob.I2J_LEG(i));
  tauq_leg = Rob.Leg(i).springtorque(q_i);
  TAU_full(ii:Rob.Leg(i).NJ+ii-1) = tauq_leg;
  ii = ii + Rob.Leg(i).NJ;
end
% Projektion aller Terme der Subsysteme [DT09]/(23)
% Die Momente liegen noch bezogen auf die Euler-Winkel vor (entsprechend der
% EE-Koordinaten)
taured_Fx = Jinv'* TAU_full;

% Umrechnung der Momente auf kartesische Koordinaten (Basis-KS des
% Roboters)
Tw = euljac(xP(4:6), Rob.phiconv_W_E);
H = [eye(3), zeros(3,3); zeros(3,3), Tw];
taured_Fs = H(Rob.I_EE,Rob.I_EE)' \  taured_Fx;

