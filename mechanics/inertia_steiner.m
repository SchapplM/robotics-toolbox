% Berechnung des Massenträgheitsmoments für verschobene Koordinaten
% 
% Eingabe:
% J_C
%   Massenträgheitsmoment um den Punkt C
% a
%   Vektor vom (neuen) Punkt P zum (alten) Punkt C
% m
%   Masse des Körpers
% 
% Ausgabe:
% J_P
%   Massenträgheitsmoment um den Punkt P

% Quelle:
% [1] http://de.wikipedia.org/wiki/Steinerscher_Satz

% Moritz Schappler, schappler@irt.uni-hannover.de, 2014-07
% (c) Institut für Regelungstechnik, Universität Hannover

function J_P = inertia_steiner(J_C, a, m)
%#codegen
assert(isa(J_C, 'double') && all(size(J_C) == [3, 3]), ...
      'inertia_steiner: J_C = [3x3] double');
assert(isa(a, 'double') && all(size(a) == [3, 1]), ...
      'inertia_steiner: a = [3x1] double');
assert(isa(m, 'double') && all(size(m) == [1, 1]), ...
      'inertia_steiner: m = [1x1] double');

J_P = J_C + m * ...
    [a(2)^2+a(3)^2, -a(1)*a(2), -a(1)*a(3); ...
    -a(1)*a(2), a(1)^2+a(3)^2, -a(2)*a(3); ...
    -a(1)*a(3), -a(2)*a(3), a(1)^2+a(2)^2];