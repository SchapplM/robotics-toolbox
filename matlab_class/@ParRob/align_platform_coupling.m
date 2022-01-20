% Funktion zur Anordnung der Plattform-Koppelpunkte
% Durch die Aktualisierung können die Koppelpunkte parametrisiert werden
% 
% Eingabe:
% Rob
%   Roboter-Klasse
% method
%   Nummer der Methode (siehe Quelltext)
%   1-3: Kreisförmig. Gelenk-Achse Normal (1), Tangential (2), Radial (3)
%   4-6: Paarweise. Gelenk-Achse Normal (4), Tangential (5), Radial (6)
%   7: Nicht implementiert (TODO: Passend zur Kinematik bei n-FG-Ketten für
%      n-FG-PKM
%   8: Alle Gelenkachsen parallel und in Plattform-Ebene (für 3T2R)
% param
%   Parameter für diese Methode (siehe Quelltext)
%   Alle Methoden:
%     1: Radius
%   Methoden 4-6:
%     1: Abstand von der Mitte zur Linie zwischen den Gelenkpaaren
%     2: Abstand der Gelenke eines Paares
%   Methode 8:
%     2: Winkel der Gelenkachsen gegen die x-Achse der Plattform

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
% Parameter in Klasse abspeichern. Der letzte Parameter in der
% Klassenvariable ist die Stärke der Platte des Ersatzmodells.
if length(Rob.DesPar.platform_par) == length(param)
  Rob.DesPar.platform_par = [param; 0];
else
  Rob.DesPar.platform_par(1:length(param)) = param; 
end
if method <= 3
  % Methode 1 bis 3: Symmetrische Anordnung der Plattform-Koppelgelenke im
  % Kreis (entspricht N-Eck bei Verbindung der Punkte). Das letzte Gelenk
  % der Beinkette ist entsprechend der Z-Achse ausgerichtet:
  % Methode 1: Z-Achse nach oben, X-Achse tangential
  % Methode 2: Z-Achse tangential, X-Achse nach oben
  % Methode 3: Z-Achse zur Mitte,  X-Achse nach oben
  % Parameter: Abstand der Gelenke zum Mittelpunkt (Radius der Plattform)
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
  % Methode 4-6: Plattform-Koppelgelenke paarweise angeordnet (nur für
  % Hexapod-Strukturen). Ansonsten analog zu Methode 1-3
  % Methode 4: Z-Achse nach oben, X-Achse tangential
  % Methode 5: Z-Achse tangential, X-Achse nach oben
  % Methode 6: Z-Achse zur Mitte, X-Achse tangential
  % Parameter: (1) Abstand von Mitte zur Linie der Gelenkpaar
  % (näherungsweise Radius der Plattform, (2) Abstand der Gelenke eines Paares
  n_pf_par = 2;
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
elseif method == 8
  % Methode 8: Alle Gelenkachsen parallel angeordnet
  n_pf_par = 2;
  r_PB = param(1);
  if length(param) == 1 % Geänderte Implementierung, Abwärtskompatibilität
    param = [param; 0];
    warning(['Eingabeparameter param muss für Methode P8 2x1 sein. ', ...
      'Zweiter, fehlender Eintrag auf 0 gesetzt.']);
  end
  psi_offset = param(2);
  for i = 1:NLEG
    if Rob.issym
      beta_i = 2*sym('pi')/NLEG*(i-1);
    else
      beta_i = 2*pi/NLEG*(i-1);
    end
    % Kreisförmig außen an Plattform
    r_P_P_Bi_ges(:,i) = rotz(beta_i)*[r_PB;0;0];
    % Zunächst y-Drehung mit pi/2 so, dass Gelenkachse in die Ebene zeigt
    % und die xB-Achse nach unten. Dann Offset-Drehung um xB-Achse zur Ver-
    % drehung der Gelenkachse in der Ebene
    phi_P_Bi = r2eulxyz(roty(pi/2)*rotx(psi_offset));
    phi_P_B_all(:,i) =  phi_P_Bi;
  end   
else
  error('Plattform-Koppelgelenk-Methode %d nicht implementiert', method);
end
if length(param) ~= n_pf_par
  error('Anzahl der eingegebenen Parameter (%d) stimmt nicht für die Methode %d (erwartet: %d)', ...
    length(param), method, n_pf_par);
end
Rob.r_P_B_all = r_P_P_Bi_ges;
Rob.phi_P_B_all = phi_P_B_all;
