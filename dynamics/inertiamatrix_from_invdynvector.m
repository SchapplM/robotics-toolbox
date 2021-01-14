% Berechne die Massenmatrix durch mehrfachen Aufruf der Inversdynamik
% 
% Eingabe:
% fcnhandle
%   Handle zur Funktion, die die inverse Dynamik (als Vektor der
%   Gelenkmomente) berechnet. Die folgenden Eingaben in diese Funktion sind
%   die Eingaben für die Funktion in fcnhandle (werden durchgeschleift).
% q [nx1]
%   Gelenkpositionen
% qD [nx1]
%   Gelenkgeschwindigkeiten. Eingabe wird nicht beachtet
% qDD [nx1]
%   Gelenkbeschleunigungen. Eingabe wird nicht beachtet.
% Weitere Eingaben (z.B. Kinematik- und Dynamikparameter, Gravitation).
% 
% Ausgabe:
% M
%   nxn Massenmatrix

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function M = inertiamatrix_from_invdynvector(varargin)
% Eingaben auslesen
invdynfcn = varargin{1}; % Variable fcnhandle
n = length(varargin{2}); % Variable q
% Massenmatrix durch Aufruf der Inversdynamik mit Einheits-Beschleunigung
% zusammenstellen
M = NaN(n,n);
for i = 1:n
  qDD_test = zeros(n,1);
  qDD_test(i) = 1;
  varargin{3} = zeros(n,1);
  varargin{4} = qDD_test;
  tau_i = invdynfcn(varargin{2:end});
  M(:,i) = tau_i;
end
  