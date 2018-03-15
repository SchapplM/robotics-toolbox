function [theta_, n_] = tr2angvec(T)

%% Init
%#codegen
assert(isa(T,'double') && isreal(T) && all(size(T) == [4 4]), ...
  'tr2angvec: T = [4x4] double');

%% Calculation
R = T(1:3,1:3);
[theta_, n_] = r2angvec(R);