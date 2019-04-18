% Generiere (zufällige) Einstellungen für den Test des Roboters
% Die Zufallswerte können als Eingabewerte für die Funktionen verwendet
% werden
% 
% Eingabe:
% setDyn [true/false]
%   Schalter für das Neu-Schreiben der Dynamikparameter in die
%   Roboterklasse
% setKin [true/false]
%   Schalter für das Neu-Schreiben der Kinematikparameter

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-11
% (C) Institut für mechatronische Systeme, Universität Hannover

function TSS = gen_testsettings(RS, setDyn, setKin)

if nargin < 2
  setDyn = false;
end
if nargin < 3
  setKin = false;
end

NL = RS.NL;
NQJ = RS.NQJ;
%% Zufällige Dynamikparameter generieren
rSges = rand(NL,3); % All center of mass coordinates in body frames
mges = rand(NL,1); % masses of all links (are positive due to rand() function)
Ic_pa = rand(NL,3); % inertia of all links around their center of mass in principal axes
Icges = NaN(NL,6); % inertia of all links around their center of mass in body frame
for i = 1:NL
  R_pa = eulxyz2r(rand(3,1)); % random principal axes
  % inertia tensor in body frame: make sure the eigenvalues are positive and the tensor is positive definite
  Icges(i,:) = inertiamatrix2vector(R_pa*diag(Ic_pa(i,:))*R_pa');
end

%% Zufällige Kinematik-Parameter generieren
if any(isnan(RS.pkin)) || setKin
  % nur generieren, wenn nicht schon gesetzt
  pkin = rand(length(RS.pkin),1);
end

%% Zufällige Gelenkwinkel-Konfigurationen generieren
n = 100;
% Gelenkwinkel und -zeitableitungen
if any(isnan(RS.qlim(:)))
  % Keine Grenzen vorher festgelegt, nehme willkürlich +/-Pi
  q_min = -pi*ones(NQJ,1);q_max = pi*ones(NQJ,1);
else
  q_min = RS.qlim(:,1);
  q_max = RS.qlim(:,2);
end
if any(isnan(RS.qDlim(:)))
  % nichts festgelegt, willkürliche Wahl
  qD_min = -pi*ones(NQJ,1);qD_max = pi*ones(NQJ,1);
else
  qD_min = RS.qDlim(:,1);
  qD_max = RS.qDlim(:,2);
end

% Zufällige Gelenkpositionen im zulässigen Bereich
Q = repmat(q_min',n,1) + rand(n,NQJ).*repmat(q_max'-q_min',n,1);
% Zufällige Gelenkgeschwindigkeiten im zulässigen Bereich
QD = repmat(qD_min',n,1) + rand(n,NQJ).*repmat(qD_max'-qD_min',n,1);
QD(1:NQJ,:)=eye(NQJ);
% Zufällige Beschleunigungen
QDD = (0.5-rand(n, NQJ))*pi;

%% Zufällige Gravitationsrichtungen
G = 7*2*(0.5-rand(n,3));

%% Struktur mit zufälligen Gelenkwinkelkonfigurationen
TSS = struct('type', 'Test Settings Structure');
% Zufällige Konfigurationen für Modultests
TSS.n = n;
TSS.Q = Q;
TSS.QD = QD;
TSS.QDD = QDD;
TSS.G = G;

%% Eigenschaften des Roboters auf Zufallswerte setzen
% Setze Kinematikparameter neu, falls gewünscht
if setKin
  RS.update_mdh(pkin);
end

% Setze Dynamikparameter auf oben zufällig generierte
if setDyn
  RS.update_dynpar1(mges, rSges, Icges);
end
