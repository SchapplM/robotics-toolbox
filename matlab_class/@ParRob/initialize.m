% Initialisiere Klassenvariablen für die PKM
% Diese Funktion wird nach dem Konstruktor ausgeführt

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Universität Hannover

function R = initialize(R)

NLEG = R.NLEG;

% Gelenkvariablen-Indizes der einzelnen Beinketten (dient zur leichteren
% Indizierung)
I2=0;
I1J_LEG = zeros(NLEG,1);
I2J_LEG = zeros(NLEG,1);
for i = 1:NLEG
  NJ_Leg_i = R.Leg(i).NJ;
  I1 = I2+1;
  I2 = I1+NJ_Leg_i-1;
  
  I1J_LEG(i) = I1;
  I2J_LEG(i) = I2;
end

R.I1J_LEG = I1J_LEG;
R.I2J_LEG = I2J_LEG;
NJ = I2J_LEG(end);
R.NJ = NJ;

% Körper-KS-Indizes der einzelnen Beinketten (bezogen auf die Ausgabe von
% fkine)
I2=0;
I1L_LEG = zeros(NLEG,1);
I2L_LEG = zeros(NLEG,1);
for i = 1:NLEG
  NL_Leg_i = R.Leg(i).NL;
  I1 = I2+1;
  I2 = I1+NL_Leg_i;
  
  I1L_LEG(i) = I1;
  I2L_LEG(i) = I2;
end

R.I1L_LEG = I1L_LEG;
R.I2L_LEG = I2L_LEG;

% MDH-Parameter sigma für ganzen Roboter
sigma_PKM = zeros(NJ,1);
for i = 1:NLEG
  sigma_PKM(I1J_LEG(i):I2J_LEG(i)) = R.Leg(i).MDH.sigma;
end
R.MDH.sigma = sigma_PKM;

% aktive und passive Gelenke festlegen
% Annahme: Immer erstes Gelenk der Beinkette aktiv
mu_PKM = zeros(NJ,1);
for i = 1:NLEG
  mu_PKM(I1J_LEG(i):I2J_LEG(i)) = [1; zeros(R.Leg(i).NJ-1,1)];
end
R.update_actuation(mu_PKM)

% Marker für gewählte EE-FG
R.I_EE = true(1,6); % Initialisierung mit allen Freiheitsgraden (räumliche PKM). Muss logical sein, damit Binär-Indizes.