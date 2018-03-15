% Zerlege eine Massenmatrix mit der LTL-Zerlegung
% Nutze dazu die spärliche Besetzung (Sparsity) der Matrix
% Das Ergebnis kann genutzt werden, um eine Gleichung der Form H*qDD = tau
% zu lösen
% 
% Eingabe:
% H [nxn]
%   Massenmatrix
% lambda [nx1]
%   Vorgänger-Indizes der Gelenke
%   Die Indizes müssen im Falle von Gelenken mit mehreren Freiheitsgraden
%   (z.B. 6-FG-Basis) nachbearbeitet werden. Siehe [Featherstone2008], S. 114
% 
% Ausgabe
% H [nxn]
%   Matrix, die die Zerlegungs-Matrix L auf der unteren linken Hälfte hat
%   Siehe Algorithmus aus [Featherstone2008]

% Sources:
% [Featherstone2008] Roy Featherstone: Rigid Body Dynamics Algorithms (2008)
% 
% Siehe:
% fdyn_solve_inertia_LTL.m

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-12
% (c) Institut für Regelungstechnik, Universität Hannover

function H = inertia_factorization_LTL(H, lambda)

n = size(H,1);

%% Massenmatrix H faktorisieren
% Mit LTL Faktorisierung
% [Featherstone2008], Table 6.3
for k = n:-1:1
  H(k,k) = sqrt(H(k,k));
  i = lambda(k);
  while i > 0
    H(k,i) = H(k,i)/H(k,k);
    i = lambda(i);
  end
    
  i = lambda(k);
  while i > 0
    j = i;
    while j > 0
      H(i,j) = H(i,j) - H(k,i)*H(k,j);
      j = lambda(j);
    end
    i = lambda(i);
  end
end

% In H stehen jetzt die Faktorisierungsmatrizen
return