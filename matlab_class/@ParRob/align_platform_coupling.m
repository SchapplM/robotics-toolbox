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
Rob.DesPar.platform_method = uint8(method);
Rob.DesPar.platform_par = [param;0]; % Der letzte Parameter ist die Stärke der Platte des Ersatzmodells
% Methode 1: Symmetrische Anordnung im Kreis (entspricht N-Eck bei
% Verbindung der Punkte)
% Parameter: Abstand zum Mittelpunkt (Radius der Plattform)
if method == 1
  n_pf_par = 1;
  r_PB = param(1);
  
  for i = 1:NLEG
    if Rob.issym
      beta_i = 2*sym('pi')/NLEG*(i-1);
    else
      beta_i = 2*pi/NLEG*(i-1);
    end
    r_P_P_Bi_ges(:,i) = rotz(beta_i)*[r_PB;0;0];

    Rob.Leg(i).update_EE();
  end
  if Rob.issym
    % Annahme über symbolische Variablen, damit die späteren assert-Befehle
    % keinen Fehler aufwerfen
    warning('off', 'symbolic:sym:sym:AssumptionsOnConstantsIgnored');
    assume(r_P_P_Bi_ges, 'real');
    warning('on', 'symbolic:sym:sym:AssumptionsOnConstantsIgnored');
  end
elseif method == 3
  n_pf_par = 2;
  % Methode 3: Paarweise angeordnet, z-Achse nach oben
  if Rob.NLEG ~= 6
    error('Methode %d ist nur für 6 Beine definiert', method);
  end
  r_PB = param(1); % Radius des Mittelpunktes der einzelnen Kreispaare
  d_Paar = param(2); % Abstand der Gelenkpaare voneinander
  for i = 1:3 % Paare von Koppelpunkten durchgehen
    rM = rotz(2*pi/3*(i-1))*[r_PB;0;0]; % Mittelpunkt des Punktepaares
    for j = 1:2 % Beide Punkte des Paares durchgehen
      r_P_P_Bi_ges(:,2*(i-1)+j) = rM + rotz(2*pi/3*(i-1))*[0;d_Paar/2*(-1)^j;0];
    end
  end
else
  error('Methode nicht implementiert');
end
if length(param) ~= n_pf_par
  error('Anzahl der eingegebenen Parameter stimmte gar nicht');
end
Rob.r_P_B_all = r_P_P_Bi_ges;