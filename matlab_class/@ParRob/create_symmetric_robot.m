% Initialisierung der Roboterklasse mit Erstellung einer symmetrischen PKM
% 
% Eingabe:
% NLEG
%   Anzahl Beine
% SerRob
%   Klasseninstanz eines Beins
% d_0A
%   Durchmesser des Kreises, auf dem die Basis-Koppelpunkte liegen
% d_PB
%   Durchmesser des Kreises der Plattform-Koppelpunkte
% 
% Ausgabe:
% Neue Klasseninstanz der PKM

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Rob = create_symmetric_robot(Rob, NLEG, SerRob, d_0A, d_PB)

if ~isempty( SerRob )
  for i = 1:NLEG
    if i == 1
      Rob.Leg = SerRob;
    else
      Rob.Leg(i) = SerRob.copy();
    end
  end
  Rob.Leg = Rob.Leg(:);
end

Rob.NLEG = NLEG;

% Fußpunkte und Plattform-Koppelpunkte gleichmäßig im Kreis anlegen
Rob.align_base_coupling(1, d_0A);
Rob.align_platform_coupling(1, d_PB);
