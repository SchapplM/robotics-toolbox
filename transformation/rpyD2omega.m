% Konvertiere die Zeitableitung der Rotationsdarstellung in RPY-Winkeln in
% Drehgeschwindigkeit
% 
% RPY-Winkel: R=rotx(alpha)*roty(beta)*rotz(gamma)
% 
% Eingabe:
% rpy [1x3]
%   RPY-Winkel alpha, beta, gamma
% rpyD [1x3]
%   Zeitableitung der RPY-Winkel alpha, beta, gamma
% Ausgabe:
% omega [1x3]
%   Drehgeschwindigkeit
% 
% Quelle:
% [ZupanSaj2011] Zupan, Eva and Saje, Miran: Integrating rotation from
% angular velocity 

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-09
% (c) Institut für Regelungstechnik, Universität Hannover

function omega = rpyD2omega(rpy, rpyD)

%% Init

assert(isa(rpy,'double') && all(size(rpy) == [3 1]) && isreal(rpy), ...
  'rpyD2omega: rpy angles have to be [3x1] double'); 
assert(isa(rpyD,'double') && all(size(rpyD) == [3 1]) && isreal(rpyD), ...
  'rpyD2omega: rpy angles time derivatives have to be [3x1] double'); 

alpha_s = rpy(1);
beta_s = rpy(2);
gamma_s = rpy(3);

alphaD_s = rpyD(1);
betaD_s = rpyD(2);
gammaD_s = rpyD(3);

%% Berechnung
% Quelle:
% maple_codegen/rotation_rpy_omega.mw
% maple_codegen/codeexport/omega_from_rpy.m

t15 = sin(alpha_s);
t17 = cos(alpha_s);
t18 = sin(beta_s);
t21 = t18 * alphaD_s + gammaD_s;
t19 = cos(beta_s);
t26 = betaD_s * t19;
t35 = t21 * t15 - t17 * t26;
t16 = cos(gamma_s);
t34 = t15 * t16;
t33 = t15 * t18;
t32 = t15 * t19;
t31 = t16 * t17;
t30 = t17 * t18;
t29 = t17 * t19;
t28 = betaD_s * t19 ^ 2;
t27 = betaD_s * t18;
t25 = t19 * alphaD_s;
t24 = t19 * gammaD_s;
t22 = t18 * gammaD_s + alphaD_s;
t20 = t22 * t17;
t14 = sin(gamma_s);
t12 = t15 * t14 - t16 * t30;
t11 = t14 * t30 + t34;
t10 = t14 * t17 + t16 * t33;
t9 = -t14 * t33 + t31;
t8 = -t14 * t24 - t16 * t27;
t7 = t14 * t27 - t16 * t24;
t6 = t15 * t27 - t17 * t25;
t5 = -t15 * t25 - t17 * t27;
t4 = t21 * t31 + (-t22 * t14 + t16 * t26) * t15;
t3 = -t22 * t34 + (-t15 * t26 - t21 * t17) * t14;
t2 = t14 * t20 + t35 * t16;
t1 = -t35 * t14 + t16 * t20;
omega_tilde = [(-t14 * t7 + t16 * t8 + t27) * t19 t10 * t8 - t15 * t28 + t7 * t9 t11 * t7 + t12 * t8 + t17 * t28; t6 * t18 + (-t14 * t3 + t16 * t4) * t19 t4 * t10 + t3 * t9 - t6 * t32 t3 * t11 + t4 * t12 + t6 * t29; t5 * t18 + (-t1 * t14 + t16 * t2) * t19 t1 * t9 + t2 * t10 - t5 * t32 t1 * t11 + t2 * t12 + t5 * t29;];

%% Nachbearbeitung
omega = [omega_tilde(3,2); omega_tilde(1,3); omega_tilde(2,1)];