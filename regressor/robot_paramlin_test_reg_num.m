% Teste Minimalparameterform gegen numerische Berechnung nach [Gautier1990]
% Kann für fixed base und floating base Modell der symbolisch generierten
% Minimalparameterform aus der Maple-Toolbox benutzt werden.
% Nehme Symbolische Generierung (und damit Reihenfolge der
% Minimalparameter) nach [GautierKhalil1990]
% 
% Diese Funktion wird von der Testfunktion
% robot_varpar_fixbase_paramlin_test.m und
% robot_varpar_floatbase_paramlin_test.m aus der Maple-Toolbox aufgerufen.
% 
% Eingabe:
% plin_num_test_struct: Struktur mit allen benötigten Variablen zum Testen
% der Regressorform
% 
% Quellen:
% [Gautier1990] M. Gautier: Numerical calculation of the base inertial
% parameters of robots, ICRA 1990
% [Sousa2014] Sousa, C. D. and Cortesao, R.: Physical feasibility of robot
% base inertial parameter identification: A linear matrix inequality approach (2014)
% [GautierKhalil1990] Direct Calculation of Minimum Set of Inertial
% Parameters of Serial Robots 
% 
% Diese Funktion wird für generierten Code aus der Maple-Toolbox benötigt,
% liegt aber in der Robotik-Toolbox, damit die Maple-Toolbox nicht im Pfad
% sein muss, um die Testfunktionen auszuführen.

% Moritz Schappler, schappler@irt.uni-hannover.de, 2017-02
% (C) Institut für Regelungstechnik, Universität Hannover

function robot_paramlin_test_reg_num(plin_num_test_struct)

%% Initialisierung
% Variablen aus Eingabestruktur laden
PV2 = plin_num_test_struct.PV2;
W_g = plin_num_test_struct.W; % Informationsmatrix
P1sym = plin_num_test_struct.P1sym; % Permutationsmatrix für unabhängige Inertialparameter (aus symbolischer Berechnung, zum Vergleich)
P2sym = plin_num_test_struct.P2sym; % Permutationsmatrix für abhängige Inertialparameter (aus symbolischer Berechnung, zum Vergleich)
Ksym = plin_num_test_struct.Ksym; % Lineare Zuordnungsmatrix aller Inertialparameter (aus symbolischer Berechnung, zum Vergleich)
K_dsym = plin_num_test_struct.K_dsym; % Lineare Zuordnungsmatrix der abhängigen Inertialparameter (aus symbolischer Berechnung, zum Vergleich)
MPVsym = plin_num_test_struct.MPV; % Minimalparameter (aus symbolischer Berechnung, zum Vergleich)
NQ = plin_num_test_struct.NQ; % Anzahl der Freiheitsgrade (allgemein, damit für fixed und floating base funktionierend)
PV2_Names = plin_num_test_struct.PV2_Names;
if isfield(plin_num_test_struct, 'qr_tolerance')
  qr_tolerance = plin_num_test_struct.qr_tolerance;
else
  qr_tolerance = NaN;
end
n = size(W_g,1)/NQ; % Anzahl der Samples: Geht davon aus, dass ein Dynamik-Regressor übergeben wurde. Ist aber auch später egal.

% [Gautier1990] Gl. (30): Skalierung der Werte der Informationsmatrix
S = max(abs(W_g(:))) / min(abs(W_g(W_g~=0)));
W_F = sqrt(sum(W_g(:).^2)); % Gl. (31)
% TODO: Skalierung gem. [Gautier1990] Kap. 5-1-1

if size(W_g,1) < size(W_g,2)
  error(['Die Informationsmatrix muss mindestens so viele Zeilen wie Spalten ', ...
    'haben. Aktuell %dx%d'], size(W_g,1), size(W_g,2));
end

% Prüfe Rang der Matrix, [Gautier1990] Kap. 3-2
c = length(PV2);
b = rank(W_g);
if b < length(MPVsym)
  warning(['Die numerisch berechnete Informationsmatrix hat den Rang %d, der nicht ', ...
    'der Länge %d des symbolisch bestimmten Minimalparametervektors entspricht! ', ...
    'Die Berechnung des MPV ist womöglich fehlerhaft'], b, length(MPVsym));
elseif b > length(MPVsym)
  warning('Der MPV enthält zu wenige Einträge. Das ist eigentlich gar nicht möglich');
end

%% QR-Zerlegung und Permutation der Informationsmatrix
% QR-Zerlegung, [Gautier1990] Kap. 3-2
% [Q_g1,R_g1,P_g1] = qr(W_g); % Benennung g1: erste QR-Zerlegung nach Gautier
[Q_g1,R_g1] = qr(W_g); % Benennung g1: erste QR-Zerlegung nach Gautier

r = NQ*n;
if isnan(qr_tolerance)
  tau = r * eps(1) * max(abs(diag(R_g1))); % ist sehr kleine Zahl. [Gautier1990] Gl. (14)
else
  tau = qr_tolerance;
end
% Permutation nach [Gautier1990] Kap. 3-3
% Sortiere die Parameter damit gemäß [GautierKhalil1990]
R_g1(abs(R_g1(:))<tau) = 0;

P1 = zeros(c,b);
P2 = zeros(c,c-b);
% Abhängige Spalten finden
% "... The c-b diagonal elements Rii give the subscripts i of the columns W:,i to be deleted"
k_1 = 0;
k_2 = 0;
for i = 1:c
  if abs(R_g1(i,i)) == 0
    % Parameter muss eliminiert werden
    k_2 = k_2 + 1;
    P2(i,k_2) = 1;
  else
    % Parameter ist unabhängig
    k_1 = k_1 + 1;
    P1(i,k_1) = 1;
  end
end

P_g1 = [P1, P2];

% Prüfe P1,P2 (Validität der Permutationsmatrizen)
if ~all(sum(P1) == 1) || ~all(sum(P2) == 1) && ~all(sum(P_g1,2) == 1) 
  error('Permutationsmatrizen sind falsch');
end


%% Prüfe, ob die symbolische und numerische Berechnung der Permutation übereinstimmt
% Übereinstimmung der Aufteilung
Ibsym=any(P1sym'==1); % Welche Parametereinträge sind unabhängig
Ibnum=any(P1'==1);
Idsym=any(P2sym'==1); % Welche sind abhängig
Idnum=any(P2'==1);
if ~all(Ibsym==Ibnum) || ~all(Idsym==Idnum)
  % Unterschiedliche unabhängige Parametereinträge
  IIbsym_diff = find((Ibsym~=Ibnum).*Ibsym); % ... bezogen auf unabhängige nach symbolischer Herleitung
  IIbnum_diff = find((Ibsym~=Ibnum).*Ibnum); % ... bezogen auf numerische Herleitung
  if any(IIbnum_diff)
    fprintf('Dynamikparameter, die unabhängig im numerischem MPV [%dx1] vorkommen, aber nicht im symbolischen [%dx1]:\n', sum(Ibnum), sum(Ibsym))
    disp(PV2_Names(IIbnum_diff)');
  end
  if any(IIbsym_diff)
    fprintf('Dynamikparameter, die unabhängig im symbolischen MPV [%dx1] vorkommen, aber nicht im numerischen [%dx1]:\n', sum(Ibsym), sum(Ibnum))
    disp(PV2_Names(IIbsym_diff)');
  end
  % Unterschiedliche abhängige Einträge
  IIdsym_diff = find((Idsym~=Idnum).*Idsym); % ... bezogen auf symbolische Herleitung
  IIdnum_diff = find((Idsym~=Idnum).*Idnum); % ... bezogen auf numerische Herleitung
  if any(IIdnum_diff)
    fprintf('Dynamikparameter, die abhängig im numerischem MPV [%dx1] vorkommen, aber nicht im symbolischen [%dx1]:\n', sum(Ibnum), sum(Ibsym))
    disp(PV2_Names(IIdnum_diff)');
  end
  if any(IIdsym_diff)
    fprintf('Dynamikparameter, die abhängig im symbolischen MPV [%dx1] vorkommen, aber nicht im numerischen [%dx1]:\n', sum(Ibsym), sum(Ibnum))
    disp(PV2_Names(IIdsym_diff)');
  end
  error('Die Aufteilung der Inertialparameter in linear abhängige und unabhängige ist zwischen num. und sym. Berechnung unterschiedlich');
end

% Übereinstimmung der exakten Permutation
test1 = P1sym-P1; test2 = P2sym-P2;
if any(abs(test1(:)) > 1e-10) || any(abs(test2(:)) > 1e-10)
  warning('Exakte Permutation der Inertialparameter aus numerischer QR-Zerlegung stimmt nicht mit symbolischer Berechnung überein');
  % Hinweis zusammenstellen
  strb_sym = 'delta_b_sym=';
  strd_sym = 'delta_d_sym=';
  strb_num = 'delta_b_num=';
  strd_num = 'delta_d_num=';

  for i = 1:c
    if any(P1sym(i,:)==1), strb_sym = [strb_sym, PV2_Names{i}, ',']; end %#ok<AGROW>
    if any(P2sym(i,:)==1), strd_sym = [strd_sym, PV2_Names{i}, ',']; end %#ok<AGROW>
    if any(P1(i,:)==1), strb_num = [strb_num, PV2_Names{i}, ',']; end %#ok<AGROW>
    if any(P2(i,:)==1), strd_num = [strd_num, PV2_Names{i}, ',']; end %#ok<AGROW>
  end
  fprintf('Parametervektoren:\n%s\n%s\n%s\n%s\n', strb_sym,strb_num,strd_sym,strd_num);
end

%% Inertialparameter aufteilen
% [Gautier1990] Gl. (12)
tmp = Q_g1'*W_g*P_g1;

% [Gautier1990] Kap 3-2: Absteigender Betrag der Diagonalelemente wird
% erzeugt, wenn qr-Aufruf mit 3 Ausgabeargumenten erfolgt. Wir müssen aber
% unsere eigene Permutationsreihenfolge vorgeben

tmp(abs(tmp(:)) < tau) = 0;
R1 = tmp(1:b,1:b);
R2 = tmp(1:b,b+1:end);

% [Gautier1990], Gl. (15)
PX = P_g1' * PV2;
X1 = PX(1:b);
X2 = PX(b+1:end);

% [Gautier1990], Gl. (16)
WP = W_g*P_g1;
W1 = WP(:, 1:b);
W2 = WP(:,b+1:end);

% Prüfe Regruppierung von Parametern und Informationsmatrix
if any(abs( W_g*PV2 - (W1*X1 + W2*X2) )>1e-10)
  error('Permutation der Informationsmatrix und des Inertialparametervektors stimmt nicht');
end

% Dieser Test wird nicht benötigt, wenn die Permutation oben selbst
% durchgeführt wird.
% [Gautier1990], Gl. (17)
% [Q_g2,R_g2,P_g2] = qr(WP); % Zweite QR-Zerlegung nach Gautier
% R1_g2 = R_g2(1:b,1:b);
% R2_g2 = R_g2(1:b,b+1:end);
% if any(any( P_g2-eye(c)) )
%   error('Permutation in zweiter QR-Zerlegung. Darf nicht sein');
% end
% % Prüfe, ob Matrix R1,R2 gleich geblieben sind
% if any(abs(R1(:)-R1_g2(:))>1e-10) || any(abs(R2(:)-R2_g2(:))>1e-10)
%   error('Matrizen R1 oder R2 stimmen nicht in zweiter QR-Zerlegung');
% end

%% Minimalparameter bestimmen
% [Gautier1990], Gl. (18)
test = W2 - W1*(R1\R2);
if any(abs(test(:))>1e-10)
  warning('[Gautier1990], Gl. (18) stimmt nicht. Fehler: %1.3e', max(abs(test(:))));
end

% Minimalparametern nach Gautier1990
% [Gautier1990], Ende Kap 3-5

XB1 = X1 + R1 \ R2 * X2;

% Teste, ob Minimalparameterform nach [Gautier1990] funktioniert
P1 = P_g1(:,1:b);
P2 = P_g1(:,b+1:end);
Wb = W_g*P1; % [Sousa2014], Gl. (33),(34)
test = Wb*XB1 - W_g*PV2;% [Sousa2014], Gl. (35)
if any(abs(test(:)) > 1e-8)
  error(['Minimalparameterform nach [Gautier1990] stimmt nicht. Fehler: ', ...
    '%1.2e'], max(abs(test)));
end

% Prüfe MPV
if any(abs( XB1-MPVsym ) > 1e-8)
  error(['MPV aus num. und symb. Berechnung stimmen nicht überein. Fehler: ', ...
    '%1.3e'], max(abs(XB1-MPVsym)));
end
%% Transformationsmatrizen zwischen Inertial und Min.-Parametern prüfen
% Prüfe Zusammenhang zwischen Inertialparametern und Minimalparametern für
% symbolische und numerische Berechnung
K_g = (P1' + (R1 \ R2) * P2');
K_g( abs(K_g)<eps(max(abs(K_g(:)))) ) = 0;
% Prüfe K_g
if any(abs(K_g * PV2 - XB1) > 1e-10)
  error('Matrix K_g stimmt nicht');
end

K_d_num = R1 \ R2;
test = K_d_num - K_dsym;
if any(abs(test(:)) > 1e-10)
  error('Transformation der linear abhängigen Inertialparameter stimmt nicht überein');
end

test = K_g - Ksym;
if any(abs(test(:)) > 1e-10)
  error('Transformation der Inertialparameter stimmt nicht überein zwischen sym. und num.');
end

return
%% Debug-Ausgaben
% Falls oben Fehler auftreten (wenn der symbolische MPV nicht mit dem
% numerischen übereinstimmt), können diese Ausgaben zur Fehlersuche benutzt
% werden.

II2 = P_g1'*(1:c)'; % Indizes für die nach abhängig/unabhängig permutierte Parameter (numerisch bestimmt)
X1_Names = PV2_Names(II2(1:b));  % Namen der unabhängigen Parameter (numerisch bestimmt)
X2_Names = PV2_Names(II2(b+1:c));% ... abhängigen Parameter ...

tmp = R1 \ R2; % Matrix zur Auswahl der abhängigen Parameter in den numerischen MPV
fprintf('Inhalte des numerisch bestimmten MPV:\n');
for i = 1:b
  fprintf('%02d: ', i);
  fprintf('indep: ');
  fprintf( X1_Names{i}); % Unabhängiger Parameter dieses Eintrags
  fprintf(', ');
  fprintf('dep: ');
  % Gehe durch die abhängigen Parameter und prüfe, ob sie jeweils im MPV
  % vorkommen
  for j = 1:size(tmp,2)
    if any(abs(tmp(i,j))>1e-10)
      fprintf( X2_Names{j});
      if i ~= size(P1,2)
        fprintf(', ');
      end
     end
  end
  fprintf('\n');
end
