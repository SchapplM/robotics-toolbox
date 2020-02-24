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
phi_P_B_all = NaN(3,NLEG);

Rob.DesPar.platform_method = uint8(method);
% Parameter in Klasse abspeichern. Der letzte Parameter ist die Stärke der
% Platte des Ersatzmodells.
if length(Rob.DesPar.platform_par) == length(param)
  Rob.DesPar.platform_par = [param; 0];
else
  Rob.DesPar.platform_par(1:length(param)) = param; 
end
% Methode 1: Symmetrische Anordnung im Kreis (entspricht N-Eck bei
% Verbindung der Punkte)
% Methode 2: X-Achse nach oben, Z-Achse tangential 
% Methode 3: X-Achse nach oben, Z-Achse zu Mitte
% Parameter: Abstand zum Mittelpunkt (Radius der Plattform)
if method <= 3
  n_pf_par = 1;
  r_PB = param(1);
  
  for i = 1:NLEG
    if Rob.issym
      beta_i = 2*sym('pi')/NLEG*(i-1);
    else
      beta_i = 2*pi/NLEG*(i-1);
    end
    r_P_P_Bi_ges(:,i) = rotz(beta_i)*[r_PB;0;0];
    
    if method == 1
      phi_P_Bi = [0*pi/180;0*pi/180;pi/2+beta_i];
    elseif method == 2
      phi_P_Bi = [-pi/2;-beta_i;-pi/2];
    elseif method == 3
      phi_P_Bi = [-pi/2;-pi/2-beta_i;0*pi/180];
    end
    Rob.Leg(i).update_EE();
    phi_P_B_all(:,i) =  phi_P_Bi;
  end
  if Rob.issym
    % Annahme über symbolische Variablen, damit die späteren assert-Befehle
    % keinen Fehler aufwerfen
    warning('off', 'symbolic:sym:sym:AssumptionsOnConstantsIgnored');
    assume(r_P_P_Bi_ges, 'real');
    warning('on', 'symbolic:sym:sym:AssumptionsOnConstantsIgnored');
  end
elseif method > 3 && method <= 6
  n_pf_par = 2;
  % Methode 3: Paarweise angeordnet, z-Achse nach oben
  if Rob.NLEG ~= 6
    error('Methode %d ist nur für 6 Beine definiert', method);
  end
  r_PB = param(1); % Radius des Mittelpunktes der einzelnen Kreispaare
  d_Paar = param(2); % Abstand der Gelenkpaare voneinander
  for i = 1:3 % Paare von Koppelpunkten durchgehen
    rM = rotz(2*pi/3*(i-1))*[r_PB;0;0]; % Mittelpunkt des Punktepaares
    beta_i = 2*pi/3*(i-1);
    for j = 1:2 % Beide Punkte des Paares durchgehen
      r_P_P_Bi_ges(:,2*(i-1)+j) = rM + rotz(2*pi/3*(i-1))*[0;d_Paar/2*(-1)^j;0];
      
      if method == 4
        phi_P_Bi = [0*pi/180;0*pi/180;pi/2+beta_i];
      elseif method == 5
        phi_P_Bi = [-pi/2;-beta_i;-pi/2];
      elseif method == 6
        phi_P_Bi = [-pi/2;-pi/2-beta_i;0*pi/180];
      end
      Rob.Leg(j+2*(i-1)).update_EE();
      phi_P_B_all(:,j+2*(i-1)) =  phi_P_Bi;
    end
  end
else
  error('Methode nicht implementiert');
end
if length(param) ~= n_pf_par
  error('Anzahl der eingegebenen Parameter stimmte gar nicht');
end
if Rob.DesPar.base_method ~= 1 || any(Rob.Leg(1).I_EE > Rob.I_EE)
  for i = 1:NLEG
    Rob.Leg(i).I_EE = true(1,6);
  end
  Rob.update_EE_FG(Rob.I_EE,Rob.I_EE,repmat(true(1,6), Rob.NLEG,1));
end
Rob.r_P_B_all = r_P_P_Bi_ges;
Rob.phi_P_B_all = phi_P_B_all; % TODO: Das was in phi_Ni_Ei steht jetzt hier rein. Prüfen.