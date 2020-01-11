% Berechnung der PKM-Massenmatrix (bezogen auf Plattform-Koordinaten x)
%
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS (nicht:
%   Plattform-Pose xP)
% Jinv [N x Nx] (optional zur Rechenersparnis)
%   Vollständige Inverse Jacobi-Matrix der PKM (bezogen auf alle N aktiven
%   und passiven Gelenke und die bewegliche EE-Koordinaten xE)
% M_full (optional zur Rechenersparnis)
%   Vollständige Massenmatrix der PKM (für alle Subsysteme, Kräfte und
%   Beschleunigungen in x-Koordinaten)
%   Siehe auch: ParRob/inertia2_platform_full
% M_full_reg (optional zur Rechenersparnis)
%   Regressormatrix zur zweiten Ausgabe (M_full). Matrix wird 3D gestapelt.
%   (Dimension 1 und 2 sind die ursprüngliche Massenmatrix, Dim. 3 sind die
%   Dynamikparameter 
%   Siehe auch: ParRob/inertia2_platform_full
% 
% Ausgabe:
% Mred_Fs
%   Massenmatrix (bezogen auf EE-Koordinaten der PKM)
%   Wird die Matrix mit Plattform-Beschleunigungen (x-Koordinaten, keine
%   Winkelbeschleunigung) multipliziert, entstehen kartesische Kräfte und
%   Momente im Basis-KS)
% Mredvec_Fs_reg
%   Regressormatrix zur ersten Ausgabe (Mred_xD). Matrix wird dafür 2D
%   gestapelt (Zeilen sind Massenmatrix-Elemente, Spalten sind Parameter).
%   Das zweite Ausgabeargument ist nur zulässig, wenn DynPar.mode=4 ist.

% Quelle:
% [DT09] Do Thanh, T. et al: On the inverse dynamics problem of general
% parallel robots, ICM 2009
% [Sriram2019_S876] Sriram, R.: Implementation of a Structurally Independant
% Dynamics Modelling of Parallel Kinematic Machines based on Full Kinematic
% Constraints (Studienarbeit bei Moritz Schappler, 2019).

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Mred_Fs, Mredvec_Fs_reg] = inertia2_platform(Rob, q, xE, Jinv, M_full, M_full_reg)

%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/inertia2_platform: q muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/inertia2_platform: xE muss 6x1 sein');
if nargin == 4
  assert(isreal(Jinv) && all(size(Jinv) == [Rob.NJ sum(Rob.I_EE)]), ...
    'ParRob/inertia2_platform: Jinv muss %dx%d sein', Rob.NJ, sum(Rob.I_EE));
end

NLEG = Rob.NLEG;

if Rob.issym
  error('Nicht implementiert')
end

%% Projektionsmatrizen
% Gradientenmatrix der vollständigen Zwangsbedingungen
if nargin < 4
  G_q = Rob.constr1grad_q(q, xE);
  G_x = Rob.constr1grad_x(q, xE);
  % Inverse Jacobi-Matrix
  Jinv = - G_q \ G_x; % Siehe: ParRob/jacobi_qa_x
end
K1 = eye ((NLEG+1)*NLEG); % Reihenfolge der Koordinaten (erst Beine, dann Plattform), [DT09]/(9)
R1 = K1  * [Jinv; eye(NLEG)]; % Projektionsmatrix, [DT09]/(15)

%% Massenmatrix des vollständigen Systems (alle Subsysteme)
% Ausgelagert in andere Funktion, da Term auch für Coriolis-Kräfte benötigt
if nargin < 5 && nargout == 1
  M_full = Rob.inertia2_platform_full(q, xE, Jinv);
elseif nargin < 6 && nargout == 2
  [M_full, M_full_reg] = Rob.inertia2_platform_full(q, xE, Jinv);
end

%% Projektion auf Subsysteme
% Projektion aller Terme der Subsysteme [DT09]/(23)
% Die Momente (Zeilen der Massenmatrix) liegen noch bezogen auf die
% Euler-Winkel vor (entsprechend der EE-Koordinaten)
Mred_Fx = R1' * M_full * R1;
if nargout == 2
  % Matrix-Operation für Massenmatrix mit der Regressor-Matrix
  % nachvollziehen
  Mred_sD_reg = NaN(size(Mred_Fx,1), size(Mred_Fx,2), size(M_full_reg,3));
  for jj = 1:size(M_full_reg,3)
    Mred_sD_reg(:,:,jj) = R1' * M_full_reg(:,:,jj) * R1;
  end
end
% Umrechnung der Momente auf kartesische Koordinaten (Basis-KS des
% Roboters)
Tw = euljac(xE(4:6), Rob.phiconv_W_E);
H = [eye(3), zeros(3,3); zeros(3,3), Tw];
Mred_Fs = H(Rob.I_EE,Rob.I_EE)'\ Mred_Fx;
if nargout == 2
  % Matrix-Operation für Massenmatrix mit der Regressor-Matrix
  % nachvollziehen
  Mred_Fs_reg = NaN(size(Mred_Fs,1), size(Mred_Fs,2), size(M_full_reg,3));
  for jj = 1:size(M_full_reg,3)
    Mred_Fs_reg(:,:,jj) = H(Rob.I_EE,Rob.I_EE)'\ Mred_sD_reg(:,:,jj);
  end
  % Regressor-Matrix wieder als Vektor schreiben
  Mredvec_Fs_reg = reshape(Mred_Fs_reg, size(Mred_Fs_reg,1)*size(Mred_Fs_reg,2), size(M_full_reg,3));
end