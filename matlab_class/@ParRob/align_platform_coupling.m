% Funktion zur Anordnung der Plattform-Koppelpunkte
% Durch die Aktualisierung können die Koppelpunkte parametrisiert werden
% 
% Eingabe:
% Rob
%   Roboter-Klasse
% method
%   Nummer der Methode (siehe Quelltext)
% param
%   Parameter für diese Methode (siehe Quelltext)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Universität Hannover

function align_platform_coupling(Rob, method, param)

NLEG = Rob.NLEG;

% Position der Plattform-Koppelpunkte Bi im Plattform-KS
if ~Rob.issym
  r_P_P_Bi_ges = NaN(3,NLEG);
else
  r_P_P_Bi_ges = sym('xx', [3,NLEG]);
  r_P_P_Bi_ges(:)=0;
end

% Methode 1: Symmetrische Anordnung im Kreis (entspricht N-Eck bei
% Verbindung der Punkte)
% Parameter: Abstand zum Mittelpunkt
if method == 1
  d_PB = param(1);
  
  for i = 1:NLEG
    if Rob.issym
      beta_i = 2*sym('pi')/NLEG*(i-1);
    else
      beta_i = 2*pi/NLEG*(i-1);
    end
    r_P_P_Bi_ges(:,i) = rotz(beta_i)*[d_PB;0;0];

    Rob.Leg(i).update_EE();
  end
  if Rob.issym
    % Annahme über symbolische Variablen, damit die späteren assert-Befehle
    % keinen Fehler aufwerfen
    warning('off', 'symbolic:sym:sym:AssumptionsOnConstantsIgnored');
    assume(r_P_P_Bi_ges, 'real');
    warning('on', 'symbolic:sym:sym:AssumptionsOnConstantsIgnored');
  end
  Rob.r_P_B_all = r_P_P_Bi_ges;
end