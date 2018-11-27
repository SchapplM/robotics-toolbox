% Eintragen der Funktions-Handles in die Roboterstruktur durch Verweis auf
% automatisch generierte Matlab-Funktionen, die als Dateien vorliegen
% müssen.
% 
% Eingabe:
% mex
%   Schalter zur Wahl von vorkompilierten Funktionen (schnellere Berechnung)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für mechatronische Systeme, Universität Hannover

function R = fill_fcn_handles(R, mex)
mdlname = R.mdlname;
for i = 1:length(R.all_fcn_hdl)
  ca = R.all_fcn_hdl{i};
  hdlname = ca{1};
  fcnname = ca{2};
  if nargin == 1 || mex == 0
    eval(sprintf('R.%s = @%s_%s;', hdlname, mdlname, fcnname));
  else
    eval(sprintf('R.%s = @%s_%s_mex;', hdlname, mdlname, fcnname));
  end
end