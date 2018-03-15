% Berechne die Beschleunigung aus der Vorwärtsdynamik mit einer
% LTL-zerlegten Massenmatrix
% 
% Eingabe:
% L [nxn]
%   Faktorisierte Massenmatrix (H = L' * L, H_fact)
% tau [nx1]
%   Beschleunigungsmoment der verallgemeinerten Koordinaten
% lambda [nx1]
%   Vorgänger-Indizes der Gelenke
%   Die Indizes müssen im Falle von Gelenken mit mehreren Freiheitsgraden
%   (z.B. 6-FD-Basis) nachbearbeitet werden. Siehe [Featherstone2008], S. 114
% 
% Ausgabe:
% qDD [nx1]
%   Beschleunigung der verallgemeinerten Koordinaten gemäß qDD = H\tau
% 
% Sources:
% [Featherstone2008] Roy Featherstone: Rigid Body Dynamics Algorithms (2008)
% [WikiCholesky] https://de.wikipedia.org/wiki/Cholesky-Zerlegung
% 
% Siehe:
% inertia_factorization_LTL.m

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-12
% (c) Institut für Regelungstechnik, Universität Hannover

function qDD = fdyn_solve_inertia_LTL(L, tau, lambda)

n = size(L,1);

%% Rücksubstitution zur Berechnung von qDD
% [WikiCholesky]#Formulierung_und_Anwendung
% Achtung: Hier ist H=L'*L und nicht A=G*G';

% [WikiCholesky]#Formulierung_und_Anwendung
% Vorwärtseinsetzen: Löse G*y=b mit G=L', y=tmp-Variable, b=tau
% [Featherstone2008], Table 6.5: x=L^(-T)*x
y = tau;
for i = n:-1:1
  y(i) = y(i)/L(i,i);
  j = lambda(i);
  while j > 0
    y(j) = y(j)-L(i,j)*y(i);
    j = lambda(j);
  end
end

% Teste Ergebnis
% if any(abs( L'*y - tau ) > 1e-10)
%   error('Vorwärtseinsetzen stimmt nicht');
% end

% [WikiCholesky]#Formulierung_und_Anwendung
% Rückwärtseinsetzen: Löse G'*x=y mit G'=L, y=tmp-Variable (s.o.), x=qDD
% [Featherstone2008], Table 6.5: x=L^(-1)*x
x = y;
for i = 1:n
  % fprintf('i=%d\n', i);
  % Durch die Struktur ist j<i -> Es wird immer auf Elemente j zugegriffen,
  % die vorher verarbeitet wurden, L(i,j) ist immer die untere linke
  % Dreiecksmatrix
  j = lambda(i);
  while j > 0
    x(i) = x(i)-L(i,j)*x(j);
    j = lambda(j);
  end
  x(i) = x(i) / L(i,i);
end

% Teste Ergebnis
% if any(abs( L*x - y ) > 1e-10)
%   error('Rückwärtseinsetzen stimmt nicht');
% end

qDD = x;