% Trage die technischen Gelenke in die Roboterklasse ein.
% 
% Eingabe:
% techjointstr
%   Zeichenfolge mit technischen Gelenken (R,P,U,S)
%   gelesen von links (erstes Gelenk) nach rechts

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function set_techjoints(R, techjointstr)
assert(isa(techjointstr, 'char'), 'Eingabe techjointstr muss String sein');
numtechjoints = length(techjointstr);
i_coord = 0; % Zähler über Gelenke im Modell
for i = 1:numtechjoints % über technische Gelenke (können mehrwertig sein)
  % Der Gelenktyp wird als Zeichenkette von links nach rechts gelesen
  joint_i = techjointstr(i);
  switch joint_i
    case 'R'
      i_coord = i_coord + 1;
      R.DesPar.joint_type(i_coord) = 0; % Drehgelenk
    case 'P'
      i_coord = i_coord + 1;
      R.DesPar.joint_type(i_coord) = 1; % Schubgelenk (allgemein)
    case 'C'
      i_coord = i_coord + 2;
      % Dreh-Schubgelenk noch nicht implementiert. Setze einfach direkt die
      % MDH-Parameter. Keine Auswirkung.
      R.DesPar.joint_type(i_coord-1:i_coord) = R.MDH.sigma(i_coord-1:i_coord);
      warning('Gelenk Typ C noch nicht implementiert');
    case 'U'
      i_coord = i_coord + 2;
      R.DesPar.joint_type(i_coord-1:i_coord) = 2; % Kardan
    case 'S'
      i_coord = i_coord + 3;
      R.DesPar.joint_type(i_coord-2:i_coord) = 3; % Kugel
    otherwise
      error('Fall %s nicht definiert', joint_i);
  end
end
if i_coord ~= R.NJ
  error(['Die Anzahl der eingetragenen technischen Gelenke %d passt nicht ', ...
    'zur Gelenkzahl %d des Roboters']);
end