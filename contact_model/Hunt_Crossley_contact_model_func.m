%% Hunt Crossley contact model
% 
% Kontakt zwischen einer Kugel und einer Ebene
% 
% Input:
% x [1x1]
%   Eindringtiefe z in [AzadFeatherstone2010].
% xD [1x1]
%   Eindringgeschwindigkeit
% R,h,alpha [3x1]
%   Hunt Crossley Parameters
%   R: Radius der Kugeln im Kontakt
%   h: Materialeigenschaft (E-Modul, Poisson-Zahl)
%     sigma aus [SunBai14], 1/E* (ohne Pi) aus [AzadFeatherstone2010]
%   alpha: Faktor für Größe der Dämpfungskraft (aus [AzadFeatherstone2010])
% 
% Output:
% F_s [1x1]
%   Joint Limit Spring Force
% F_d [1x1]
%   Joint Limit damping Force
% E_spring [1x1]
%   Energy stored in the spring
% 
% Source:
% [AzadFeatherstone2010] Morteza Azad and Roy Featherstone: Modeling the
% contact between a rolling sphere and a compliant ground plane (2010)
% [Miller2015] Niclas Miller: Modelling, Simulation and Control Design of
% an assistive Exoskeleton Arm (2015), Masterarbeit
% [SunBai14] Y. Sun Z.F. Bai, J. Chen. Effects of contact force model on dynamics charac-
% teristics of mechanical system with revolute clearance joints. Transactions of
% Mechanical Engineering, 38(M2):375-388, 2014.
 
% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-06
% (c) Institut fuer Regelungstechnik, Universitaet Hannover


function [F_s,F_d,E_spring]=Hunt_Crossley_contact_model_func(x,xD,R,h,alpha)

% Steifigkeit
% [AzadFeatherstone2010] Gl. (10)
k=(4/3)*1/h*sqrt(R/2);
F_s = sign(x)*k*abs(x)^(3/2);

% Dämpfung
% [AzadFeatherstone2010] Gl. (12)
F_d = 4*pi*R*alpha*abs(x)^(1/2)*xD;

% Energie
E_spring = 2/5 * k * abs(x)^(5/2);
