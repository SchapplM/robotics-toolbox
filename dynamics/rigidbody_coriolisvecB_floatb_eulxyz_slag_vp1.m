% Calculate vector of centrifugal and coriolis load on the base for
% rigidbody
% Use Code from Maple symbolic Code Generation
% 
% Input:
% phi_base [3x1]
%   Base orientation in world frame. Expressed with XYZ-Euler angles
% r_base [3x1]
%   Base position in world frame
% xD_base [6x1]
%   time derivative of r_base and phi_base
% m_mdh [1x1]
%   mass of all robot links (including the base)
% rSges [1x3]
%   center of mass of all robot links (in body frames)
%   rows: links of the robot (starting with base)
%   columns: x-, y-, z-coordinates
% Icges [1x6]
%   inertia of all robot links about their respective center of mass, in body frames
%   rows: links of the robot (starting with base)
%   columns: xx, yy, zz, xy, xz, yz (see inertiavector2matrix.m)
% 
% Output:
% Fc [6x1]
%   base forces and torques required to compensate coriolis and centrifugal load

% Quelle: HybrDyn-Toolbox
% Datum: 2019-03-19 12:00
% Revision: 3380762d7a67e58abcb72423f3b6cbd7db453188 (2019-03-19)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Fc = rigidbody_coriolisvecB_floatb_eulxyz_slag_vp1(phi_base, xD_base, ...
  m, rSges, Icges)
%% Coder Information
%#codegen
%$cgargs {zeros(3,1),zeros(6,1),zeros(1,1),zeros(1,3),zeros(1,6)}
assert(isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  'rigidbody_coriolisvecB_floatb_eulxyz_slag_vp1: phi_base has to be [3x1] (double)');
assert(isreal(xD_base) && all(size(xD_base) == [6 1]), ...
  'rigidbody_coriolisvecB_floatb_eulxyz_slag_vp1: xD_base has to be [6x1] (double)');
assert(isreal(m) && all(size(m) == [1 1]), ...
  'rigidbody_coriolisvecB_floatb_eulxyz_slag_vp1: m has to be [1x1] (double)'); 
assert(isreal(rSges) && all(size(rSges) == [1,3]), ...
  'rigidbody_coriolisvecB_floatb_eulxyz_slag_vp1: rSges has to be [1x3] (double)');
assert(isreal(Icges) && all(size(Icges) == [1 6]), ...
  'rigidbody_coriolisvecB_floatb_eulxyz_slag_vp1: Icges has to be [1x6] (double)'); 

%% Symbolic Calculation
% From coriolisvec_base_floatb_eulxyz_par1_matlab.m
% OptimizationMode: 2
% StartTime: 2019-03-19 12:00:19
% EndTime: 2019-03-19 12:00:25
% DurationCPUTime: 4.09s
% Computational Cost: add. (5120->365), mult. (13718->544), div. (0->0), fcn. (13657->6), ass. (0->179)
t110 = sin(phi_base(1));
t112 = cos(phi_base(1));
t114 = cos(phi_base(2));
t109 = sin(phi_base(3));
t111 = cos(phi_base(3));
t113 = sin(phi_base(2));
t185 = t112 * t113;
t100 = -t110 * t109 + t111 * t185;
t184 = t112 * t114;
t96 = Icges(1,4) * t100;
t189 = t110 * t111;
t99 = t109 * t185 + t189;
t56 = -Icges(1,2) * t99 - Icges(1,6) * t184 + t96;
t95 = Icges(1,4) * t99;
t60 = -Icges(1,1) * t100 + Icges(1,5) * t184 + t95;
t158 = t100 * t60 + t99 * t56;
t187 = t110 * t114;
t188 = t110 * t113;
t98 = t109 * t112 + t111 * t188;
t201 = Icges(1,4) * t98;
t186 = t111 * t112;
t97 = -t109 * t188 + t186;
t55 = Icges(1,2) * t97 - Icges(1,6) * t187 + t201;
t94 = Icges(1,4) * t97;
t58 = Icges(1,1) * t98 - Icges(1,5) * t187 + t94;
t206 = t97 * t55 + t98 * t58;
t52 = Icges(1,5) * t98 + Icges(1,6) * t97 - Icges(1,3) * t187;
t54 = -Icges(1,5) * t100 + Icges(1,6) * t99 + Icges(1,3) * t184;
t236 = t158 + t206 + (-t110 * t52 - t112 * t54) * t114;
t176 = t114 * xD_base(6);
t181 = t112 * xD_base(5);
t106 = -t110 * t176 + t181;
t178 = t113 * xD_base(6);
t107 = xD_base(4) + t178;
t21 = -t100 * t58 + t52 * t184 + t99 * t55;
t143 = Icges(1,5) * t111 - Icges(1,6) * t109;
t87 = Icges(1,3) * t113 + t114 * t143;
t193 = Icges(1,4) * t111;
t145 = -Icges(1,2) * t109 + t193;
t89 = Icges(1,6) * t113 + t114 * t145;
t194 = Icges(1,4) * t109;
t147 = Icges(1,1) * t111 - t194;
t91 = Icges(1,5) * t113 + t114 * t147;
t36 = -t100 * t91 + t184 * t87 + t99 * t89;
t235 = -t106 * t21 - t107 * t36;
t182 = t110 * xD_base(5);
t105 = t112 * t176 + t182;
t66 = -t100 * rSges(1,1) + t99 * rSges(1,2) + rSges(1,3) * t184;
t157 = rSges(1,1) * t111 - rSges(1,2) * t109;
t93 = t113 * rSges(1,3) + t114 * t157;
t233 = -t105 * t93 + t107 * t66;
t153 = -t109 * t56 - t111 * t60;
t24 = t113 * t54 - t114 * t153;
t20 = -t187 * t54 - t97 * t56 + t98 * t60;
t137 = -xD_base(4) + t178;
t141 = xD_base(4) * t176;
t83 = -t110 * t141 - t137 * t181;
t219 = t83 / 0.2e1;
t22 = t184 * t54 - t158;
t228 = t22 * t219;
t82 = -t112 * t141 + t137 * t182;
t220 = t82 / 0.2e1;
t217 = t105 / 0.2e1;
t215 = t106 / 0.2e1;
t214 = -t107 / 0.2e1;
t213 = t107 / 0.2e1;
t148 = xD_base(5) * t176;
t133 = t148 / 0.2e1;
t35 = -t187 * t87 + t97 * t89 + t98 * t91;
t226 = t105 * t20 + t35 * t107;
t129 = t143 * t113;
t152 = t109 * t89 - t111 * t91;
t154 = t109 * t55 - t111 * t58;
t115 = t105 * (-t112 * t87 + t153) + t106 * (t87 * t110 + t154) + t107 * (Icges(1,3) * t114 - t129 + t152);
t225 = t115 * t114;
t136 = t113 * xD_base(4) + xD_base(6);
t179 = t114 * xD_base(5);
t224 = t110 * t136 - t112 * t179;
t177 = t114 * xD_base(4);
t170 = t110 * t177;
t180 = t113 * xD_base(5);
t172 = t112 * t180;
t121 = -t170 - t172;
t124 = t107 * t112;
t47 = -t109 * t224 + t111 * t124;
t48 = t109 * t124 + t111 * t224;
t31 = t48 * rSges(1,1) + t47 * rSges(1,2) + rSges(1,3) * t121;
t175 = xD_base(4) * t112;
t171 = t114 * t175;
t174 = t110 * t180;
t120 = -t171 + t174;
t49 = -t107 * t189 + (-t110 * t179 - t112 * t136) * t109;
t50 = t136 * t186 + (-t107 * t109 + t111 * t179) * t110;
t32 = t50 * rSges(1,1) + t49 * rSges(1,2) + rSges(1,3) * t120;
t64 = t98 * rSges(1,1) + t97 * rSges(1,2) - rSges(1,3) * t187;
t11 = -t105 * t32 + t106 * t31 - t64 * t83 + t66 * t82;
t132 = t157 * t113;
t156 = -rSges(1,1) * t109 - rSges(1,2) * t111;
t67 = -xD_base(5) * t132 + (rSges(1,3) * xD_base(5) + t156 * xD_base(6)) * t114;
t16 = -t106 * t67 + t107 * t32 + t148 * t64 - t82 * t93;
t33 = -t105 * t64 + t106 * t66 + xD_base(1);
t155 = t106 * t93 - t107 * t64;
t37 = -t155 + xD_base(3);
t38 = xD_base(2) - t233;
t223 = -t11 * t66 + t33 * (t64 * xD_base(4) - t31) - (t38 * xD_base(4) - t16) * t93 + t37 * t67;
t146 = -Icges(1,1) * t109 - t193;
t222 = t105 * (-Icges(1,1) * t99 - t56 - t96) + t106 * (-Icges(1,1) * t97 + t201 + t55) - t107 * (t146 * t114 - t89);
t144 = -Icges(1,2) * t111 - t194;
t119 = t105 * (Icges(1,2) * t100 + t60 + t95) + t106 * (-Icges(1,2) * t98 + t58 + t94) + t107 * (t144 * t114 + t91);
t142 = -Icges(1,5) * t109 - Icges(1,6) * t111;
t61 = -xD_base(5) * t129 + (Icges(1,3) * xD_base(5) + t142 * xD_base(6)) * t114;
t126 = -t114 * t61 + t180 * t87;
t130 = t145 * t113;
t62 = -xD_base(5) * t130 + (Icges(1,6) * xD_base(5) + t144 * xD_base(6)) * t114;
t131 = t147 * t113;
t63 = -xD_base(5) * t131 + (Icges(1,5) * xD_base(5) + t146 * xD_base(6)) * t114;
t13 = -t100 * t63 - t112 * t126 - t170 * t87 + t47 * t89 + t48 * t91 + t99 * t62;
t26 = Icges(1,5) * t50 + Icges(1,6) * t49 + Icges(1,3) * t120;
t127 = -t114 * t26 + t180 * t52;
t28 = Icges(1,4) * t50 + Icges(1,2) * t49 + Icges(1,6) * t120;
t30 = Icges(1,1) * t50 + Icges(1,4) * t49 + Icges(1,5) * t120;
t3 = -t100 * t30 - t112 * t127 - t170 * t52 + t99 * t28 + t47 * t55 + t48 * t58;
t25 = Icges(1,5) * t48 + Icges(1,6) * t47 + Icges(1,3) * t121;
t128 = -t114 * t25 + t180 * t54;
t27 = Icges(1,4) * t48 + Icges(1,2) * t47 + Icges(1,6) * t121;
t29 = Icges(1,1) * t48 + Icges(1,4) * t47 + Icges(1,5) * t121;
t4 = -t100 * t29 - t112 * t128 - t170 * t54 + t99 * t27 - t47 * t56 + t48 * t60;
t221 = t13 * t213 + t36 * t133 + t21 * t220 + t3 * t215 + t4 * t217 + t228;
t218 = -t105 / 0.2e1;
t216 = -t106 / 0.2e1;
t212 = t112 / 0.2e1;
t23 = t113 * t52 - t114 * t154;
t211 = t23 * t82;
t210 = t24 * t83;
t7 = (t154 * xD_base(5) + t26) * t113 + (t52 * xD_base(5) + (-t55 * xD_base(6) + t30) * t111 + (-t58 * xD_base(6) - t28) * t109) * t114;
t209 = t7 * t106;
t8 = (t153 * xD_base(5) + t25) * t113 + (t54 * xD_base(5) + (t56 * xD_base(6) + t29) * t111 + (-t60 * xD_base(6) - t27) * t109) * t114;
t208 = t8 * t105;
t18 = (t152 * xD_base(5) + t61) * t113 + (t87 * xD_base(5) + (-t89 * xD_base(6) + t63) * t111 + (-t91 * xD_base(6) - t62) * t109) * t114;
t40 = t113 * t87 - t114 * t152;
t207 = t18 * t107 + t40 * t148;
t169 = -t177 / 0.2e1;
t19 = -t187 * t52 + t206;
t6 = t110 * t128 - t171 * t54 + t97 * t27 + t98 * t29 - t49 * t56 + t50 * t60;
t168 = -t19 * xD_base(4) + t6;
t5 = t110 * t127 - t171 * t52 + t97 * t28 + t98 * t30 + t49 * t55 + t50 * t58;
t167 = t20 * xD_base(4) + t5;
t166 = -t21 * xD_base(4) + t4;
t165 = t22 * xD_base(4) + t3;
t164 = -t23 * xD_base(4) + t8;
t163 = t24 * xD_base(4) + t7;
t159 = t37 * t64 - t38 * t66;
t151 = t110 * t19 - t112 * t20;
t150 = t110 * t21 - t112 * t22;
t149 = t110 * t23 - t112 * t24;
t125 = t105 * t54 + t106 * t52 + t107 * t87;
t123 = t142 * t114 * t107 + t105 * (Icges(1,5) * t99 + Icges(1,6) * t100) + t106 * (Icges(1,5) * t97 - Icges(1,6) * t98);
t17 = t105 * t67 - t107 * t31 - t148 * t66 + t83 * t93;
t122 = t16 * t64 - t17 * t66 - t38 * t31 + t37 * t32;
t117 = t33 * (t110 * t66 + t112 * t64) + (-t37 * t110 - t38 * t112) * t93;
t116 = -t11 * t64 + t33 * (-t66 * xD_base(4) - t32) + t38 * t67 + (t37 * xD_base(4) + t17) * t93;
t104 = t156 * t114;
t92 = rSges(1,3) * t114 - t132;
t90 = Icges(1,5) * t114 - t131;
t88 = Icges(1,6) * t114 - t130;
t85 = t93 * t112;
t84 = t93 * t110;
t81 = t91 * t112;
t80 = t91 * t110;
t79 = t89 * t112;
t78 = t89 * t110;
t75 = rSges(1,1) * t99 + rSges(1,2) * t100;
t74 = rSges(1,1) * t97 - rSges(1,2) * t98;
t14 = t110 * t126 - t171 * t87 + t49 * t89 + t50 * t91 + t97 * t62 + t98 * t63;
t12 = t105 * t24 + t106 * t23 + t107 * t40;
t10 = t105 * t22 - t235;
t9 = t106 * t19 + t226;
t2 = t6 * t105 + t5 * t106 + t14 * t107 + t148 * t35 + t19 * t82 + t20 * t83;
t1 = [m(1) * t11; m(1) * t17; m(1) * t16; t209 / 0.2e1 + t211 / 0.2e1 + t208 / 0.2e1 + t210 / 0.2e1 + t35 * t220 + t14 * t215 + t36 * t219 + ((t22 + t236) * t106 + t226) * t218 + t207 + (t13 + t9) * t217 + ((-t19 + t236) * t105 + t10 + t235) * t216 + (-t38 * t155 + t233 * t37 + t122) * m(1); t2 * t212 - t12 * t176 / 0.2e1 + (((-t109 * t88 + t111 * t90 + t87) * t107 + t40 * xD_base(6) + (-t109 * t78 + t111 * t80 + t52) * t106 + (t109 * t79 - t111 * t81 + t54) * t105) * t114 + (t149 * xD_base(6) + t115) * t113) * t214 + ((t97 * t88 + t98 * t90) * t107 + (t97 * t78 + t98 * t80) * t106 + (-t97 * t79 - t98 * t81) * t105 + (t114 * t35 - t185 * t20) * xD_base(6)) * t216 + ((-t100 * t90 + t99 * t88) * t107 + (-t100 * t80 + t99 * t78) * t106 + (t100 * t81 - t99 * t79) * t105 + (t114 * t36 + t188 * t21) * xD_base(6)) * t218 + (t175 / 0.2e1 + t178 * t212) * t10 + (-t33 * (-t105 * t84 - t106 * t85) - t38 * (t105 * t92 + t107 * t85) - t37 * (-t106 * t92 + t107 * t84) - (t113 * t117 + t114 * t159) * xD_base(6)) * m(1) + (t23 * t133 + t163 * t213 + t19 * t220 + t167 * t215 + t21 * t219 + t165 * t217 + ((-t22 * xD_base(6) - t125) * t113 + t225) * t218 - t223 * m(1)) * t112 + (t24 * t133 + t164 * t213 + t20 * t220 + t168 * t215 + t221 + t228 + t166 * t217 + ((t19 * xD_base(6) + t125) * t113 - t225) * t216 + t9 * t214 + m(1) * t116) * t110; t12 * t179 / 0.2e1 + t113 * (t207 + t208 + t209 + t210 + t211) / 0.2e1 + (t40 * t113 - t114 * t149) * t133 + ((t149 * xD_base(5) + t18) * t113 + (-t110 * t163 + t112 * t164 + t40 * xD_base(5)) * t114) * t213 - t2 * t187 / 0.2e1 + (t35 * t113 - t114 * t151) * t220 + ((t151 * xD_base(5) + t14) * t113 + (-t110 * t167 + t112 * t168 + t35 * xD_base(5)) * t114) * t215 + t184 * t221 + (t36 * t113 - t114 * t150) * t219 + ((t150 * xD_base(5) + t13) * t113 + (-t110 * t165 + t112 * t166 + t36 * xD_base(5)) * t114) * t217 + (t123 * t113 + (-t109 * t119 - t111 * t222) * t114) * t214 + (t119 * t97 - t123 * t187 - t222 * t98) * t216 + (t100 * t222 + t119 * t99 + t123 * t184) * t218 + (t112 * t169 + t174 / 0.2e1) * t9 + (t110 * t169 - t172 / 0.2e1) * t10 + ((t117 * xD_base(5) + t122) * t113 + (t110 * t223 + t116 * t112 + t159 * xD_base(5)) * t114 - t33 * (-t105 * t74 + t106 * t75) - t38 * (t104 * t105 - t107 * t75) - t37 * (-t104 * t106 + t107 * t74)) * m(1);];
Fc  = t1(:);
