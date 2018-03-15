% Tangentiale Reaktionskraft nach dem LuGre Reibmodell
% 
% Eingabe:
% v_FC [nx1]
%   Relativgeschwindigkeit des Kontaktpunktes
%   Es können eine (n=1) oder zwei (n=2) Komponenten der Geschwindigkeit
%   angegeben werden
% F_FCn [1x1]
%   Normalkraft am Kontaktpunkt (z.B. aus Hunt-Crossley-Modell)
% s [nx1]
%   Schlupf am Kontaktpunkt.
%   Für den Schlupf wird eine Dynamik angenommen. Die Zeitableitung wird
%   hier berechnet und ausgegeben. Die Integration muss in der aufrufenden
%   Funktion erfolgen
% LuGreParam [7x1]
%   Parameter des LuGre-Kontaktmodells (siehe unten)
% kv_arctan [1x1] double
%   Faktor zur Annäherung der sgn-Funktion durch einen arctan
%   sgn(v) = 2/pi*arctan(kv * v)
%   Für kv=Inf: sgn-Funktion (Standardeinstellung)
% 
% Ausgabe:
% F_t [nx1]
%   Tangentialkraft aus dem LuGre-Modell
% sD [nx1]
%   Zeitableitung des Schlupfes

% Quelle:
% [AstromWit2008] K.J. Astrom and C. Canudas-de-Wit: Revisiting the LuGre
% friction model (2008)
% [HaddadinKriAlb2015XX] Haddadin, Sami and Krieger, Kai and Albu-Schäffer,
% Alin: Exploiting Elastic Energy Storage for "Blind" Cyclic Manipulation:
% Modeling, Stability Analysis, Control and Experiments for Dribbling (2015)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-04
% (c) Institut für Regelungstechnik, Universität Hannover

function [F_t, sD] = LuGre_tangential_friction_model_func(v_FC, F_FCn, s, LuGreParam, kv_arctan)

%% Init
assert(isa(F_FCn,'double') && isreal(F_FCn) && all(size(F_FCn) == [1 1]), ...
  'LuGre_tangential_friction_model_func: F_FCn muss [1x1] double sein');
if nargin == 4
  kv_arctan = Inf; % Standardmäßig mit dieser Methode, falls nicht anders definiert.
end
assert(isa(LuGreParam,'double') && isreal(LuGreParam) && all(size(LuGreParam) == [7 1]), ...
  'LuGre_tangential_friction_model_func: LuGreParam muss [7x1] double sein');

% Parameter: Siehe [AstromWit2008], [HaddadinKriAlb2015XX]
sigma0 = LuGreParam(1); % Steifigkeit
sigma1 = LuGreParam(2); % Mikro-Dämpfung
sigma2 = LuGreParam(3); % Makro-Dämpfung (viskos)
muC = LuGreParam(4);    % Coulomb-Reibung (Faktor im Verhältnis zur Normalkraft)
muS = LuGreParam(5);    % Haftreibung (Faktor)
vs = LuGreParam(6);     % Stribeck-Geschwindigkeit
alpha = LuGreParam(7);  % Exponent für Stribeck-Funktion

%% Berechnung

% [HaddadinKriAlb2015XX] Gl. (10)
% [AstromWit2008] Gl. (4)
g = muC + (muS - muC) * exp( -abs(v_FC/vs).^alpha );

% [HaddadinKriAlb2015XX] Gl. (8)
sD  = abs(v_FC) - (sigma0*abs(v_FC) ./ g) .* s;
% [AstromWit2008] Gl. (1), TODO: Ergebnis hiermit noch falsch/unplausibel
% sD  = v_FC - (sigma0*abs(v_FC) ./ g) .* s;

% Signum-Funktion berechnen oder annähern
if isinf(kv_arctan)
  sgn_vFC = sign(v_FC);
else
  % Implementierung mit sgn gibt numerische Probleme in
  % Vorwärtsdynamiksimulationen (Nulldurchgang) und Konsistenz der
  % Integratoren in ode4. Problem wird durch atan-Näherung behoben.
  % Dafür besteht bei konstanter externer Kraft eine konstante Geschwindigkeit
  sgn_vFC = 2/pi*atan(kv_arctan*v_FC);
end
% [HaddadinKriAlb2015XX] Gl. (9)
F_t = -sgn_vFC .* (sigma0*s + sigma1*sD + sigma2*abs(v_FC))*abs(F_FCn);
% [AstromWit2008] Gl. (2): TODO: Ergebnis ist hiermit noch falsch
% F_t = ( -sgn_vFC .* sigma0.*s - sigma1.*sD - sigma2.*v_FC ) * abs(F_FCn);
