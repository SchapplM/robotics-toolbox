% Direkte Kinematik für alle KS der PKM
% Aufruf mit allen Gelenkwinkeln (aktiv, passiv, Schnitt) und
% Plattformkoordinaten. Es sind daher alle Koordinaten vorhanden und es
% müssen daraus nur noch direkt die KS berechnet werden (quasi trivial)
% 
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
% 
% Ausgabe:
% Tc_ges
%   Alle Transformationsmatrizen. Ausgedrückt im Basis-KS
%   1: Basis
%   2-...: Bein 1 (Basis, alle Körper-KS, Schnitt-KS)
%   ...
%   ...: Bein N (Basis, alle Körper-KS, Schnitt-KS)
%   ...: N Schnitt-KS Plattform-Seite
%   Vorletzter: Plattform-KS
%   Letzter: EE-KS
% Tc_ges_W
%   Matrizen bezogen auf Welt-KS

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Tc_ges, Tc_ges_W] = fkine(Rob, q, xE)

Tc_Lges = Rob.fkine_legs(q);
Tc_Pges = Rob.fkine_platform(xE);
N_KS_L = Rob.NLEG + Rob.NJ + Rob.NLEG; % Basis-KS für jede Beinkette, alle Körper-KS des Beins und "End-Effektor" der Beine (zusätzliches Schnitt-KS)
N_KS_P = Rob.NLEG + 2; % Plattform-Koppel-KS, Plattform-KS und EE-KS
N_KS_ges = 1 + N_KS_L + N_KS_P;

if isa(Tc_Lges, 'double') && isa(Tc_Pges, 'double')
  Tc_ges = NaN(4,4,N_KS_ges);
else
  Tc_ges = sym('xx', [4,4,N_KS_ges]);
end
Tc_ges(:,:,1) = eye(4);
Tc_ges(:,:,2:(N_KS_L+1)) = Tc_Lges;
Tc_ges(:,:,(N_KS_L+2):(N_KS_L+1+N_KS_P)) = Tc_Pges;

if nargout == 2
  T_W_0 = Rob.T_W_0;
  Tc_ges_W = NaN(4,4,size(Tc_ges,3));
  for i = 1:size(Tc_ges,3)
    Tc_ges_W(:,:,i) = T_W_0*Tc_ges_W(:,:,i);
  end
end