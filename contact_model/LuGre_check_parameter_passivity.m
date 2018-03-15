% Prüfe, ob die gewählte LuGre-Parameterkombination ein passives
% Systemverhalten erzeugt
% 
% Eingabe:
% sigma0, sigma1, sigma2, Fs, FC
%   LuGre-Parameter nach [AstromWit2008]
% zeta 
%   Modale Dämpfung (sinnvoll: 0.0 bis 1.0)
% m
%   Masse des Körpers, auf den die Reibung wirkt
% debug
%   Ausgabe zusätzlicher Informationen
% 
% Ausgabe:
% passiv
%   Ist 1, wenn die ursprünglichen Parameter die Passivitätsbedingungen
%   nach [AstromWit2008] nicht verletzen
% sigma2korr
%   Korrigiertes sigma2
% sigma1korr
%   Korrigiertes sigma1
% 
% Quelle:
% [AstromWit2008] K.J. Astrom and C. Canudas-de-Wit: Revisiting the LuGre
% friction model (2008)

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-12
% (c) Institut für Regelungstechnik, Universität Hannover

function [passiv, sigma2korr, sigma1korr] = LuGre_check_parameter_passivity( ...
  sigma0, sigma1, sigma2, Fs, FC, zeta, m, debug)

passiv = true;

% Prüfe [AstromWit2008], Gl. (8)
zeta_max = sigma2/(2*sqrt(sigma0*m)) * (FC/(Fs-FC)+1);
if zeta > zeta_max
  if debug
    warning('Passivität nach [AstromWit2008], Gl. (8) verletzt. zeta=%1.1f>%1.1f', zeta, zeta_max);
  end
  % Erhöhe sigma2 so, dass Grenzwert für zeta nicht mehr überschritten wird
  sigma2_alt = sigma2;
  sigma2 = sigma2_alt * zeta_max/zeta*1.1; % 10% obendrauf 
  if debug
    fprintf('Ändere LuGre-Parameter sigma2: %1.1f -> %1.1f\n',sigma2_alt, sigma2);
  end
  passiv = false;
end

% Prüfe [AstromWit2008], Gl. (7)
sigma2_max = sigma1*(Fs-FC)/FC;
if sigma2 <= sigma2_max
  if debug
    warning('Passivität nach [AstromWit2008], Gl. (7) verletzt. sigma2=%1.1f < %1.1f', ...
      sigma2, sigma2_max);
  end
  % Erhöhe sigma1 so, dass Maximalwert für sigma2 nicht mehr überschritten wird
  sigma1_alt = sigma1;
  sigma1 = sigma1_alt * (Fs-FC)/FC * 1.1; % 10% obendrauf
  if debug
    fprintf('Ändere LuGre-Parameter sigma1: %1.1f -> %1.1f\n',sigma1_alt, sigma1);
  end
  passiv = false;
end

sigma1korr = sigma1;
sigma2korr = sigma2;