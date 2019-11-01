% Inverse Kinematik für allgemeinen parallelen Roboter
% Aufruf der Funktion für serielle Roboter. Dadurch Berechnung der IK
% für alle Beine unabhängig und nicht gleichzeitig.
%
% Eingabe:
% xE_soll [6x1]
%   Endeffektorpose des Roboters bezüglich des Basis-KS (Soll)
% q0 [Nx1]
%   Startkonfiguration: Alle Gelenkwinkel aller serieller Beinketten der PKM
% s
%   Struktur mit Eingabedaten. Felder, siehe Quelltext. Identisch mit
%   Feldern aus SerRob/invkin.md
%
% Ausgabe:
% q [Nx1]
%   Alle Gelenkwinkel aller serieller Beinketten der PKM als Lösung der IK
% Phi
%   Erfüllung der kinematischen Zwangsbedingungen in der
%   Gelenkkonfiguration q. Ist bei Erfolg ein Null-Vektor
%
% Siehe auch: SerRob/invkin.m
%
% TODO: Wenn die IK für die Beinkette funktioniert, aber dies keiner
% gültigen IK für die PKM entspricht, wird ein Fehler aufgeworfen.
% Das müsste noch systematischer abgefangen werden.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function tau_plf = invdyn2_platform (Rob,q,qD,xE,xDE)
m = Rob.DynPar.mges(end);
mrSges = Rob.DynPar.mrSges(end,:);
Ifges = Rob.DynPar.Ifges(end,:);
NLEG = Rob.NLEG;
NJ = Rob.NJ;
phi = xE(4:6);
g = Rob.gravity;
%% Initialisierung
assert(isreal(q) && all(size(q) == [Rob.NJ 1]), ...
  'ParRob/invdyn: q muss %dx1 sein',Rob.NJ);
assert(isreal(qD) && all(size(qD) == [Rob.NJ 1]), ...
  'ParRob/invdyn: qD muss %dx1 sein', Rob.NJ);
assert(isreal(xE) && all(size(xE) == [6 1]), ...
  'ParRob/constr1grad_q: xE muss 6x1 sein');
assert(isreal(xDE) && all(size(xDE) == [6 1]), ...
  'ParRob/constr1grad_q: xDE muss 6x1 sein');
% s_std = struct( ...
%              'n_min', 0, ... % Minimale Anzahl Iterationen
%              'n_max', 1000, ... % Maximale Anzahl Iterationen
%              'Phit_tol', 1e-8, ... % Toleranz für translatorischen Fehler
%              'Phir_tol', 1e-8, ... % Toleranz für rotatorischen Fehler
%              'reci', false, ... % Keine reziproken Winkel für ZB-Def.
%              'retry_limit', 100); % Anzahl der Neuversuche
% if nargin < 4
%   % Keine Einstellungen übergeben. Standard-Einstellungen
%   s = s_std;
% end
% Prüfe Felder der Einstellungs-Struktur und setze Standard-Werte, falls
% Eingabe nicht gesetzt
% for f = fields(s_std)'
%   if ~isfield(s, f{1})
%     s.(f{1}) = s_std.(f{1});
%   end

%% Aufruf der Unterfunktion
Cred = Rob.coriolisvec2_platform(q ,qD,xE, xDE) ;
gred = Rob.gravload2_platform(q ,xE, g) ;
%M_plf = Rob.inertia_platform_2(q,xE) ;
Mred = inertia2_platform(Rob,q ,xE) ;
Mred = Mred.Mred
xDD = ones( NLEG,1)

Tw = eulxyzjac(phi);
H = [eye(3), zeros(3,3); zeros(3,3), Tw];    % on changing the xyz the value of Mcomp also changes .
%H1 = [eye(3), zeros(3,3); zeros(3,3), Tw];  


%% Start
%tau_plf = NaN(NLEG);
 G_q = Rob.constr1grad_q(q, xE);
 G_x = Rob.constr1grad_x(q, xE);
 J =   - G_q \ G_x;
 K   = eye ((NLEG+1)*NLEG  );
 R   = K  * [ J',eye(NLEG)']' ;
% if NLEG  == 6 

  taured=  Mred*xDD  + Cred + gred 
  
  JT   = transpose(R)*eye((NLEG+1)*NLEG,NLEG) ;
  tau_plf = JT * taured ;
  
% else  NLEG == 3
%   
% 
%   
%   taured=  Mred*xDD + Cred + gred ;
% 
% end


tau_plf = struct( 'Mred', Mred, ...
  'gred', gred,....
  'Cred', Cred ) ;
