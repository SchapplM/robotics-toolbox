% Direkte Kinematik für Plattform-KS der PKM
% Die KS der Plattform-Koppelpunkte werden direkt aus den EE-Koordinaten
% berechnet
% 
% Eingabe:
% xE [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS
%   (nicht: Plattform-Pose)
% 
% Ausgabe:
% Tc_Pges
%   Transformationsmatrizen zu allen KS der Plattform. Ausgedrückt im Basis-KS
%   1...NLEG: Schnitt-KS Plattform-Seite für jedes Bein
%   NLEG+1: Plattform-KS
%   NLEG+2: EE-KS
% Tc_Pges_W
%   Transformationen bezogen auf Welt-KS

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Tc_Pges, Tc_Pges_W] = fkine_platform(Rob, xE)
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/fkine_platform: xE muss 6x1 sein');
%% Initialisierung
if ~Rob.issym
  Tc_Pges = NaN(4,4,Rob.NLEG+2);
else
  Tc_Pges = sym('xx', [4,4,Rob.NLEG+2]);
end

%% Berechnung der Kinematik
r_0_0_E = xE(1:3);
R_0_E = eul2r(xE(4:6), Rob.phiconv_W_E);
T_0_E = transl(r_0_0_E)*r2t(R_0_E);
T_0_P = T_0_E * invtr(Rob.T_P_E);
R_0_P = T_0_P(1:3,1:3);
r_0_0_P = T_0_P(1:3,4);

r_P_P_B_ges = Rob.r_P_B_all;

for iLeg = 1:Rob.NLEG

  r_P_P_Bi = r_P_P_B_ges(:,iLeg);
  r_0_P_Bi = R_0_P * r_P_P_Bi;
  r_0_0_Bi = r_0_0_P + r_0_P_Bi;
  
  Tc_Pges(:,:,iLeg) = rt2tr(R_0_P, r_0_0_Bi);
end

Tc_Pges(:,:,Rob.NLEG+1) = T_0_P;
Tc_Pges(:,:,Rob.NLEG+2) = T_0_E;

%% Transformation ins Welt-KS
if nargout == 2
  Tc_Pges_W = NaN(4,4,size(Tc_Pges,3));
  for i = 1:size(Tc_Pges,3)
    Tc_Pges_W(:,:,i) = Rob.T_W_0 * Tc_Pges(:,:,i);
  end
end