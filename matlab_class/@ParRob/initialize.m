% Initialisiere Klassenvariablen für die PKM
% Diese Funktion wird nach dem Konstruktor ausgeführt

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Universität Hannover

function initialize(R)

NLEG = R.NLEG;

% Gelenkvariablen-Indizes der einzelnen Beinketten (dient zur leichteren
% Indizierung). Damit wird Anfang und Ende der Gelenkkoordinaten einer
% Beinkette im Vektor aller Gelenkkoordinaten der PKM gefunden.
I2=0;
I1J_LEG = zeros(NLEG,1); % Indizes des Anfangs für alle Beinketten
I2J_LEG = zeros(NLEG,1); % Indizes der letzten Koordinate
for i = 1:NLEG
  % Benutze Minimalkoordinaten (NQJ) der Beinkette. Keine Berücksichtigung
  % der passiven Gelenke hybrider Beinketten (in `NJ`)
  NQJ_Leg_i = R.Leg(i).NQJ; 
  I1 = I2+1;
  I2 = I1+NQJ_Leg_i-1;
  
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
I1L_LEG = zeros(NLEG,1); % Start-Index der Starrkörper der Beinkette in Starrkörpern der PKM
I2L_LEG = zeros(NLEG,1); % End-Index
R.NL = 1; % Zähler für Starrkörper der PKM. Basis entspricht 1
for i = 1:NLEG
  NL_Leg_i = R.Leg(i).NL;
  I1 = I2+1;
  I2 = I1+NL_Leg_i;
  
  I1L_LEG(i) = I1;
  I2L_LEG(i) = I2;
  
  R.NL = R.NL + NL_Leg_i-1; % Anzahl der bewegten Körper der Beinkette
end
R.NL = R.NL + 1;
R.I1L_LEG = I1L_LEG;
R.I2L_LEG = I2L_LEG;

% phi_P_B_all initialisieren
R.phi_P_B_all = zeros(3,NLEG);

% MDH-Parameter sigma für ganzen Roboter: Anzeige für Art der einzelnen
% Gelenke der Beinketten der PKM (bezieht sich nur auf Minimalkoordinaten
% der Beingelenke)
sigma_PKM = zeros(NJ,1);
for i = 1:NLEG
  % Marker für Dreh-/Schubgelenk (in den Minimalkoordinaten der Beinkette)
  sigmaJ = R.Leg(i).MDH.sigma(R.Leg(i).MDH.mu>=1);
  sigma_PKM(I1J_LEG(i):I2J_LEG(i)) = sigmaJ;
end
R.MDH.sigma = sigma_PKM;

% aktive und passive Gelenke festlegen
% Annahme: Immer erstes Gelenk der Beinkette aktiv
mu_PKM = zeros(NJ,1);
for i = 1:NLEG
  mu_PKM(I1J_LEG(i):I2J_LEG(i)) = [1; zeros(R.Leg(i).NQJ-1,1)];
end
R.update_actuation(mu_PKM)

% Marker für gewählte EE-FG
R.update_EE_FG(R.I_EE);

% Robotereigenschaften auslesen und in Klasse abspeichern
structkinpar_hdl = eval(sprintf('@%s_structural_kinematic_parameters', R.mdlname));

% Anzahl der relevanten Beingelenke in den (symmetrischen) Beinketten
% Diese Variable wird von PKM-Funktionen aus der HybrDyn-Toolbox benötigt
if ~isempty( which(sprintf('%s_structural_kinematic_parameters.m', R.mdlname)) )
  R.NQJ_LEG_bc = structkinpar_hdl();
else
  % Die PKM-Funktion ...structural_kinematic_parameters existiert nicht.
  % Daher wird die Variable sowieso nicht benötigt.
  % Setze auf plausiblen Wert (die meisten PKM haben 3 Gelenke, die die
  % Position der Koppelpunkte beeinflussen)
  if all(R.I_EE == [1 1 0 0 0 1])
    % Im planaren Fall nur 2 Gelenke zur Beeinflussung der 2D-Koppelpunkt-
    % position sinnvoll.
    R.NQJ_LEG_bc = 2;
  else
    % Im räumlichen Fall 3 Gelenkkoordinaten sinnvoll.
    R.NQJ_LEG_bc = 3;
  end
end

% Dynamik-Parameter initialisieren: Die Anzahl entspricht der Anzahl der
% für die Dynamik relevanten Beingelenke und eins für die Plattform
% Annahme: Es sind prinzipiell Dynamikparameter für alle Segmente der
% Beinkette möglich, auch für virtuelle Segmente in und hinter
% Kugelgelenken (Koppelgelenken).
NL_symPKM = R.Leg(1).NL-1+1;
R.DynPar = struct('mges',   NaN(NL_symPKM,1), ... % Massen
                  'rSges',  NaN(NL_symPKM,3), 'Icges', NaN(NL_symPKM,6), ... % Baryzentrische Parameter
                  'mrSges', NaN(NL_symPKM,3), 'Ifges', NaN(NL_symPKM,6), ... % Inertial-Parameter
                  'mpv_n1s', [], ...  % Minimalparameter-Dynamikvektor für numerische Dynamikberechnung
                  'ipv_n1s', [], ...  % Inertialparameter-Vektor für numerische Berechnung der Schnittkräfte
                  'mpv_sym', [], ...  % Minimalparameter-Dynamikvektor für symbolische Berechnung
                  'mode', 2); % Standard-Modus: Inertialparameter
