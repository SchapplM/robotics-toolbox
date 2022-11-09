% Funktion zur Anordnung der Gestell-Koppelpunkte
% Durch die Aktualisierung können die Koppelpunkte parametrisiert werden
% 
% Eingabe:
% Rob
%   Roboter-Klasse
% method
%   Nummer der Methode (siehe Quelltext)
%   1-3: Kreisförmig. Gelenk-Achse Normal (1), Tangential (2), Radial (3)
%   4: Kegel (z-Achse radial und nach oben)
%   5-8: Paarweise, sonst wie 1-4
%   9: Alle Gelenkachsen parallel angeordnet (in der Ebene)
%   10: parallel senkrecht zur Ebene
% param
%   Parameter für diese Methode (siehe Quelltext)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Universität Hannover

function align_base_coupling(Rob, method, param)

NLEG = Rob.NLEG;
if ~Rob.issym
  r_0_0_Ai_ges = NaN(3,NLEG);
  phi_0_Ai_ges = NaN(3,NLEG);
else
  r_0_0_Ai_ges = sym('xx', [3,NLEG]);
  r_0_0_Ai_ges(:)=0;
  phi_0_Ai_ges = sym('xx', [3,NLEG]);
  phi_0_Ai_ges(:)=0;
end
Rob.DesPar.base_method = uint8(method);
Rob.DesPar.base_par = param;
if method <= 3 % Kreisförmig
  n_base_par = 1;
  % Methode 1: Symmetrische Anordnung im Kreis (entspricht N-Eck bei
  % Verbindung der Punkte)
  % Methode 2: Kreis, x nach oben, z tangential
  % Methode 3: Zentral (planar), z zeigen in die Mitte, x tangential
  % Parameter: Abstand (Radius des Plattform-Kreises)
  r_0A = param(1);
  for i = 1:NLEG
    if Rob.issym
      beta_i = 2*sym('pi')/NLEG*(i-1);
    else
      beta_i = 2*pi/NLEG*(i-1);
    end
    r_0_0_Ai_ges(:,i) = rotz(beta_i)*[r_0A;0;0];
    % Basis-KS Einstellung
    if method == 1
      if Rob.issym
        phi_0_Ai_ges(:,i) = [0*sym('pi')/180;0*sym('pi')/180;sym('pi')/2+beta_i];
      else
        phi_0_Ai_ges(:,i) = [0*pi/180;0*pi/180;pi/2+beta_i];
      end
    elseif method == 2
      if Rob.issym
        phi_0_Ai_ges(:,i) = [-sym('pi')/2;-beta_i;-sym('pi')/2];
      else
        phi_0_Ai_ges(:,i) = [-pi/2;-beta_i;-pi/2];
      end
    elseif method == 3
      if Rob.issym
        phi_0_Ai_ges(:,i) = [-sym('pi')/2;-sym('pi')/2-beta_i;0*sym('pi')/180];
      else
        phi_0_Ai_ges(:,i) = [-pi/2;-pi/2-beta_i;0*pi/180];
      end   
    end
    if Rob.issym
      % TODO: Bessere Methode zur Unterdrückung der Warnung (tritt auf,
      % wenn 0 oder Pi mit Annahmen belegt werden soll
      warning('off', 'symbolic:sym:sym:AssumptionsOnConstantsIgnored');
      assume(r_0_0_Ai_ges(:,i), 'real');
      assume(phi_0_Ai_ges(:,i), 'real');
      warning('on', 'symbolic:sym:sym:AssumptionsOnConstantsIgnored');
    end
    Rob.Leg(i).r_W_0 = r_0_0_Ai_ges(:,i);
    Rob.Leg(i).phi_W_0 = phi_0_Ai_ges(:,i); % Standard-Konvention XYZ lassen

    % Transformationsmatrizen aus den geänderten Euler-Winkeln generieren
    Rob.Leg(i).update_base();
  end
elseif method == 4
  n_base_par = 2;  
  % Methode 4: Zentral (Kegel); aktuell pi/4 Steigungswinkel
  % Parameter: Abstand (Radius des Plattform-Kreises)
  r_0A = param(1);
  alpha_cone = param(2);
  for i = 1:NLEG
    if Rob.issym
      beta_i = 2*sym('pi')/NLEG*(i-1);
    else
      beta_i = 2*pi/NLEG*(i-1);
    end
    r_0_0_Ai_ges(:,i) = rotz(beta_i)*[r_0A;0;0];
    % Basis-KS Einstellung
    if Rob.issym
      phi_0_Ai_ges(:,i) = [0*sym('pi')/180;0*sym('pi')/180;sym('pi')/2+beta_i];
    else
      phi_0_Ai_ges(:,i) = r2eulxyz(rotz(pi/2+beta_i)*rotx(-alpha_cone));
    end  
    if Rob.issym
      % TODO: Bessere Methode zur Unterdrückung der Warnung (tritt auf,
      % wenn 0 oder Pi mit Annahmen belegt werden soll
      warning('off', 'symbolic:sym:sym:AssumptionsOnConstantsIgnored');
      assume(r_0_0_Ai_ges(:,i), 'real');
      assume(phi_0_Ai_ges(:,i), 'real');
      warning('on', 'symbolic:sym:sym:AssumptionsOnConstantsIgnored');
    end
    Rob.Leg(i).r_W_0 = r_0_0_Ai_ges(:,i);
    Rob.Leg(i).phi_W_0 = phi_0_Ai_ges(:,i); % Standard-Konvention XYZ lassen

    % Transformationsmatrizen aus den geänderten Euler-Winkeln generieren
    Rob.Leg(i).update_base();
  end
elseif method > 4 && method <= 8 % Paarweise angeordnet
  % Methode 5: Wie Methode 1, aber paarweise
  % Methode 6: Wie Methode 2, aber paarweise
  % Methode 7: Wie Methode 3, aber paarweise
  % Methode 8: Wie Methode 4, aber paarweise
  % Parameter aus Eingabe extrahieren
  if method ~= 8
    n_base_par = 2;
  else
    n_base_par = 3;
  alpha_pyr = param(3); % Steigung der Pyramide
  end
  r_PA = param(1); % Radius des Mittelpunktes der einzelnen Kreispaare
  d_Paar = param(2); % Abstand der Gelenkpaare voneinander
  % Parameter: Abstand der Grundlinien vom Mittelpunkt, Abstand der
  % Paarweisen Fußpunkte, Steigung
  if Rob.NLEG ~= 6
    error('Methode %d ist nur für 6 Beine definiert', method);
  end
  for i = 1:3 % Paare von Koppelpunkten durchgehen
    rM = rotz(2*pi/3*(i-1))*[r_PA;0;0]; % Mittelpunkt des Punktepaares
    for j = 1:2 % Beide Punkte des Paares durchgehen
      k_leg = 2*(i-1)+j;
      % Vektor zum Mittelpunkt des Punktepaares und dann auseinander
      r_0_0_Ai_ges(:,k_leg) = rM + rotz(2*pi/3*(i-1))*[0;d_Paar/2*(-1)^j;0];
      % Die drei Paare werden 120° gedreht und dann um die x-Achse in die
      % Mitte zeigend
      if method == 5
        % z-Achse nach oben
        phi_0_Ai_ges(:,k_leg) = [0*pi/180;0*pi/180;pi/2+2*pi/3*(i-1)];
      elseif method == 6
        % x-Achse nach oben, z-Achse tangential
        phi_0_Ai_ges(:,k_leg) = [-pi/2;-2*pi/3*(i-1);-pi/2];
      elseif method == 7
        % z-Achse zentral in die Mitte (paarweise parallel); x-Achse
        % tangential
        phi_0_Ai_ges(:,k_leg) = [-pi/2;-pi/2-2*pi/3*(i-1);0*pi/180];
      elseif method == 8
        % Paarweise angeordnet als Pyramide (Tetraeder)
        phi_0_Ai_ges(:,k_leg) = r2eulxyz(rotz(pi/2+2*(i-1)*pi/3)*rotx(-alpha_pyr));
      end
      % Parameter in einzelne Beinketten eintragen
      Rob.Leg(k_leg).r_W_0 = r_0_0_Ai_ges(:,k_leg);
      Rob.Leg(k_leg).phi_W_0 = phi_0_Ai_ges(:,k_leg);
      Rob.Leg(k_leg).update_base();
    end
  end
elseif any(method == [9 10])
  % Methode 9: Alle Gelenkachsen parallel angeordnet (in der Ebene)
  % Methode 10: parallel senkrecht zur Ebene
  n_base_par = 1;
  r_0A = param(1);
  for i = 1:NLEG
    if Rob.issym
      beta_i = 2*sym('pi')/NLEG*(i-1);
    else
      beta_i = 2*pi/NLEG*(i-1);
    end
    r_0_0_Ai_ges(:,i) = rotz(beta_i)*[r_0A;0;0];
    if Rob.issym
      phi_0_Ai_ges(:,i) = [0*sym('pi')/180;0*sym('pi')/180;sym('pi')/2];
    else
      phi_0_Ai_ges(:,i) = [pi/2;0*pi/180; pi/2];
    end
    if method == 10 % genauso ausgerichtet wie das Basis-KS
      phi_0_Ai_ges(:) = 0;
    end
    
    if Rob.issym
      % TODO: Bessere Methode zur Unterdrückung der Warnung (tritt auf,
      % wenn 0 oder Pi mit Annahmen belegt werden soll
      warning('off', 'symbolic:sym:sym:AssumptionsOnConstantsIgnored');
      assume(r_0_0_Ai_ges(:,i), 'real');
      assume(phi_0_Ai_ges(:,i), 'real');
      warning('on', 'symbolic:sym:sym:AssumptionsOnConstantsIgnored');
    end
    Rob.Leg(i).r_W_0 = r_0_0_Ai_ges(:,i);
    Rob.Leg(i).phi_W_0 = phi_0_Ai_ges(:,i); % Standard-Konvention XYZ lassen

    % Transformationsmatrizen aus den geänderten Euler-Winkeln generieren
    Rob.Leg(i).update_base();
  end
else
  error('Methode nicht implementiert');
end
if length(param) ~= n_base_par
  error('Anzahl der eingegebenen Parameter stimmte gar nicht');
end
Rob.r_0_A_all = r_0_0_Ai_ges;