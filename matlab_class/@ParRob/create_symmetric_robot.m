% Initialisierung der Roboterklasse mit Erstellung einer symmetrischen PKM
% 
% Eingabe:
% NLEG
%   Anzahl Beine
% SerRob
%   Klasseninstanz eines Beins
% r_0A
%   Radius des Kreises, auf dem die Basis-Koppelpunkte liegen
% r_PB
%   Radius des Kreises der Plattform-Koppelpunkte

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

function create_symmetric_robot(Rob, NLEG, SerRob, r_0A, r_PB)

if ~isempty( SerRob )
  for i = 1:NLEG
    if i == 1
      Rob.Leg = SerRob;
    else
      Rob.Leg(i) = SerRob.copy();
    end
  end
  Rob.Leg = Rob.Leg(:);
  % Eigenschaft der seriellen Kette als PKM-Beinkette eintragen
  Rob.Leg(i).islegchain = true;
end

Rob.NLEG = NLEG;

% Fußpunkte und Plattform-Koppelpunkte gleichmäßig im Kreis anlegen
if nargin > 3 && ~isempty(r_0A)
  Rob.align_base_coupling(1, r_0A);
end
if nargin > 4 && ~isempty(r_PB)
  Rob.align_platform_coupling(1, r_PB);
end
