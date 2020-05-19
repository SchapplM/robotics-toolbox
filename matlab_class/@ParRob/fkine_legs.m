% Direkte Kinematik für Bein-KS der PKM
% Die direkte Kinematik wird mit allen Beingelenkwinkeln aufgerufen, also
% nicht nur den aktiven Gelenkwinkeln. Dadurch ist die Lösung die direkte
% Kinematik der seriellen Beinketten
% 
% Eingabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM
% 
% Ausgabe:
% Tc_Lges
%   Alle Transformationsmatrizen für die Beinketten. Ausgedrückt im
%   Basis-KS
%   1...N1+2: Bein 1 (Basis, N1 bewegte Körper-KS, Schnitt-KS)
%   ... Weiter für die weiteren Beine mit N2,N3,... jew, Anzahl Bein-Gelenke
% Tc_Lges_W
%   Transformationen bezogen auf Welt-KS

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Tc_Lges, Tc_Lges_W] = fkine_legs(Rob, q)

if ~Rob.issym
  Tc_Lges = NaN(4,4,Rob.NJ+Rob.NLEG+Rob.NLEG);
else
  Tc_Lges = sym('xx', [4,4,Rob.NJ+Rob.NLEG+Rob.NLEG]);
end

J1 = 0;
J2 = 0;
for iLeg = 1:Rob.NLEG
  IJ_i = Rob.I1J_LEG(iLeg):Rob.I2J_LEG(iLeg);
  qs = q(IJ_i); % Gelenkwinkel dieser Kette
  
  % Fußpunktkoordinaten
  r_0_0_Ai = Rob.Leg(iLeg).r_W_0;
  phi_0_Ai = Rob.Leg(iLeg).phi_W_0;
  R_0_0i = eul2r(phi_0_Ai, Rob.Leg(iLeg).phiconv_W_0);
  T_0_0i = transl(r_0_0_Ai) * r2t(R_0_0i);
  % Alle Körper-KS der Beinkette
  Tc_0i = Rob.Leg(iLeg).fkine(qs);
  Tc_0 = Tc_0i(:,:,1:Rob.Leg(iLeg).NL); % Nehme nur die KS die zu in NL gezählten Körpern gehören
  for i = 1:size(Tc_0,3)
    Tc_0(:,:,i) = T_0_0i * Tc_0i(:,:,i);
  end
  % Zusätzliches KS für virtuellen Endeffektor der Beinkette
  Tc_0(:,:,end+1) = Tc_0(:,:,Rob.Leg(iLeg).I_EElink+1) * Rob.Leg(iLeg).T_N_E; %#ok<AGROW>
  % In Ausgabevariable eintragen
  J1 = J2+1;
  J2 = J1+Rob.Leg(iLeg).NL-1+1;
  Tc_Lges(:,:,J1:J2) = Tc_0(:,:,[1:Rob.Leg(iLeg).NL, end]);
end

%% Transformation ins Welt-KS
if nargout == 2
  Tc_Lges_W = NaN(4,4,size(Tc_Lges,3));
  for i = 1:size(Tc_Lges,3)
    Tc_Lges_W(:,:,i) = Rob.T_W_0 * Tc_Lges(:,:,i);
  end
end