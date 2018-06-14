% Konvertiere die zweite Zeitableitung der Rotationsdarstellung in RPY-Winkeln in
% Drehbeschleunigung
% 
% RPY-Winkel: R=rotx(alpha)*roty(beta)*rotz(gamma)
% 
% Eingabe:
% rpy [1x3]
%   RPY-Winkel alpha, beta, gamma. Entsprechen zusammen der Rotationsmatrix
%   R_A_B
% rpyD [1x3]
%   1. Zeitableitung der RPY-Winkel alpha, beta, gamma
% rpyDD [1x3]
%   2. Zeitableitung der RPY-Winkel alpha, beta, gamma
% Ausgabe:
% omegaD [1x3]
%   Drehbeschleunigung (omegaD_A_B) von KS_B ausgedrückt in KS_A
% 
% Quelle:
% [ZupanSaj2011] Zupan, Eva and Saje, Miran: Integrating rotation from
% angular velocity 

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-09
% (c) Institut für Regelungstechnik, Universität Hannover

function omegaD = rpyDD2omegaD(rpy, rpyD, rpyDD)

%% Init
%#codegen
%#cgargs {zeros(3,1),zeros(3,3),zeros(3,1)}
assert(all(size(rpy) == [3 1]) && isreal(rpy), ...
  'rpyDD2omegaD: rpy angles have to be [3x1] (double)'); 
assert(all(size(rpyD) == [3 1]) && isreal(rpyD), ...
  'rpyDD2omegaD: rpy angles time derivatives have to be [3x1] (double)'); 
assert(all(size(rpyDD) == [3 1]) && isreal(rpyDD), ...
  'rpyDD2omegaD: rpy angles second order time derivatives have to be [3x1] (double)'); 

alpha_s = rpy(1);
beta_s = rpy(2);
gamma_s = rpy(3);

alphaD_s = rpyD(1);
betaD_s = rpyD(2);
gammaD_s = rpyD(3);

alphaDD_s = rpyDD(1);
betaDD_s = rpyDD(2);
gammaDD_s = rpyDD(3);

%% Berechnung
% Quelle:
% maple_codegen/rotation_rpy_omega.mw
% maple_codegen/codeexport/omegaD_from_rpy.m

t60 = sin(alpha_s);
t62 = cos(alpha_s);
t63 = sin(beta_s);
t64 = cos(beta_s);
t65 = gammaD_s ^ 2;
t83 = (alphaD_s * gammaD_s);
t66 = alphaD_s ^ 2;
t67 = betaD_s ^ 2;
t92 = t66 + t67;
t70 = (t65 + t92) * t63 - t64 * betaDD_s + (2 * t83);
t84 = (betaD_s * alphaD_s);
t81 = 2 * t84;
t72 = t63 * alphaDD_s + t64 * t81 + gammaDD_s;
t107 = t72 * t60 + t70 * t62;
t78 = t63 * alphaD_s + gammaD_s;
t90 = betaD_s * t64;
t104 = t78 * t60 - t62 * t90;
t59 = sin(gamma_s);
t61 = cos(gamma_s);
t79 = t63 * gammaD_s + alphaD_s;
t77 = t79 * t62;
t42 = -t104 * t59 + t61 * t77;
t43 = t104 * t61 + t59 * t77;
t101 = t60 * t61;
t44 = -t79 * t101 + (-t60 * t90 - t78 * t62) * t59;
t98 = t62 * t61;
t45 = t78 * t98 + (-t79 * t59 + t61 * t90) * t60;
t88 = t64 * alphaD_s;
t91 = betaD_s * t63;
t50 = -t60 * t88 - t62 * t91;
t51 = t60 * t91 - t62 * t88;
t105 = t42 * t44 + t43 * t45 + t51 * t50;
t89 = t63 * betaDD_s;
t103 = t92 * t64 + t89;
t102 = t89 + (t65 + t67) * t64;
t100 = t60 * t63;
t99 = t60 * t64;
t97 = t62 * t63;
t96 = t62 * t64;
t95 = t67 * t63;
t87 = t64 * gammaD_s;
t86 = t64 * t95;
t80 = 0.2e1 * betaD_s * gammaD_s;
t76 = -0.2e1 * t63 * t83 - t65 - t66;
t74 = t63 * t81 - t64 * alphaDD_s;
t73 = t63 * t80 - t64 * gammaDD_s;
t71 = t63 * gammaDD_s + t64 * t80 + alphaDD_s;
t68 = t76 * t60 + t71 * t62;
t58 = t64 ^ 2;
t57 = t60 * t59 - t61 * t97;
t56 = t59 * t97 + t101;
t55 = t61 * t100 + t62 * t59;
t54 = -t59 * t100 + t98;
t53 = -t59 * t87 - t61 * t91;
t52 = t59 * t91 - t61 * t87;
t49 = t103 * t60 + t74 * t62;
t48 = -t103 * t62 + t74 * t60;
t47 = -t102 * t61 + t73 * t59;
t46 = t102 * t59 + t73 * t61;
t39 = (t76 * t59 + t72 * t61) * t62 + (-t71 * t59 - t61 * t70) * t60;
t38 = (-t71 * t60 + t76 * t62) * t61 + (t70 * t60 - t72 * t62) * t59;
t37 = t107 * t61 + t68 * t59;
t36 = -t107 * t59 + t68 * t61;
omegaD_tilde = [t67 * t58 + (-t95 + (t52 * t59 - t53 * t61) * betaD_s) * t63 + (t89 + (-t52 * gammaD_s + t47) * t61 + (-t53 * gammaD_s - t46) * t59) * t64 0.2e1 * t60 * t86 + t52 * t44 + t53 * t45 + t46 * t54 + t47 * t55 + (-t60 * betaDD_s - t62 * t84) * t58 -0.2e1 * t62 * t86 + t52 * t42 + t53 * t43 + t46 * t56 + t47 * t57 + (-t60 * t84 + t62 * betaDD_s) * t58; (t49 + (t44 * t59 - t45 * t61) * betaD_s) * t63 + (betaD_s * t51 + (-t44 * gammaD_s + t39) * t61 + (-t45 * gammaD_s - t38) * t59) * t64 t38 * t54 + t39 * t55 + t44 ^ 2 + t45 ^ 2 - t49 * t99 + t51 ^ 2 t38 * t56 + t39 * t57 + t49 * t96 + t105; (t48 + (t42 * t59 - t43 * t61) * betaD_s) * t63 + (betaD_s * t50 + (-t42 * gammaD_s + t37) * t61 + (-t43 * gammaD_s - t36) * t59) * t64 t36 * t54 + t37 * t55 - t48 * t99 + t105 t36 * t56 + t37 * t57 + t42 ^ 2 + t43 ^ 2 + t48 * t96 + t50 ^ 2;];

%% Nachbearbeitung
omegaD = [omegaD_tilde(3,2); omegaD_tilde(1,3); omegaD_tilde(2,1)];