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
% mrSges [1x3]
%  first moment of all robot links (mass times center of mass in body frames)
%  rows: links of the robot (starting with base)
%  columns: x-, y-, z-coordinates
% Ifges [1x6]
%   inertia of all robot links about their respective body frame origins, in body frames
%   rows: links of the robot (starting with base)
%   columns: xx, yy, zz, xy, xz, yz (see inertial_parameters_convert_par1_par2.m)
% 
% Output:
% Fc [6x1]
%   base forces and torques required to compensate coriolis and centrifugal load

% Quelle: HybrDyn-Toolbox
% Datum: 2019-03-19 12:00
% Revision: 3380762d7a67e58abcb72423f3b6cbd7db453188 (2019-03-19)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Fc = rigidbody_coriolisvecB_floatb_eulxyz_slag_vp2(phi_base, xD_base, ...
  m, mrSges, Ifges)
%% Coder Information
%#codegen
%$cgargs {zeros(3,1),zeros(6,1),zeros(1,1),zeros(1,3),zeros(1,6)}
assert(isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  'rigidbody_coriolisvecB_floatb_eulxyz_slag_vp2: phi_base has to be [3x1] (double)');
assert(isreal(xD_base) && all(size(xD_base) == [6 1]), ...
  'rigidbody_coriolisvecB_floatb_eulxyz_slag_vp2: xD_base has to be [6x1] (double)');
assert(isreal(m) && all(size(m) == [1 1]), ...
  'rigidbody_coriolisvecB_floatb_eulxyz_slag_vp2: m has to be [1x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [1,3]), ...
  'rigidbody_coriolisvecB_floatb_eulxyz_slag_vp2: mrSges has to be [1x3] (double)');
assert(isreal(Ifges) && all(size(Ifges) == [1 6]), ...
  'rigidbody_coriolisvecB_floatb_eulxyz_slag_vp2: Ifges has to be [1x6] (double)'); 

%% Symbolic Calculation
% From coriolisvec_base_floatb_eulxyz_par2_matlab.m
% OptimizationMode: 2
% StartTime: 2019-03-19 12:00:19
% EndTime: 2019-03-19 12:00:22
% DurationCPUTime: 1.82s
% Computational Cost: add. (3514->264), mult. (10227->409), div. (0->0), fcn. (10320->6), ass. (0->133)
t66 = sin(phi_base(2));
t60 = t66 ^ 2;
t67 = cos(phi_base(2));
t61 = t67 ^ 2;
t108 = t60 - t61;
t62 = sin(phi_base(3));
t63 = sin(phi_base(1));
t64 = cos(phi_base(3));
t65 = cos(phi_base(1));
t115 = t64 * t65;
t118 = t63 * t62;
t52 = -t118 * t66 + t115;
t113 = t65 * t66;
t117 = t63 * t64;
t54 = t113 * t62 + t117;
t83 = t52 * t63 - t54 * t65;
t79 = t83 * t66;
t100 = t67 * xD_base(5);
t97 = t66 * xD_base(4);
t80 = xD_base(6) + t97;
t96 = t66 * xD_base(6);
t81 = xD_base(4) + t96;
t33 = -t81 * t117 + (-t100 * t63 - t65 * t80) * t62;
t89 = t54 * xD_base(4) + t33;
t147 = -t65 * t100 + t63 * t80;
t73 = t81 * t65;
t31 = -t147 * t62 + t64 * t73;
t91 = -t52 * xD_base(4) + t31;
t12 = (t108 * t62 + t79) * xD_base(5) + (-t63 * t89 - t64 * t96 + t65 * t91) * t67;
t15 = t63 * t91 + t65 * t89;
t101 = t66 * xD_base(5);
t94 = t67 * xD_base(6);
t70 = t101 * t62 - t64 * t94;
t7 = t12 * xD_base(6) + t15 * xD_base(5) + t70 * xD_base(4);
t139 = t7 / 0.2e1;
t114 = t64 * t66;
t53 = t114 * t63 + t62 * t65;
t55 = t113 * t64 - t118;
t82 = t53 * t63 + t55 * t65;
t78 = t82 * t66;
t34 = t80 * t115 + (t100 * t64 - t62 * t81) * t63;
t88 = t55 * xD_base(4) - t34;
t32 = t147 * t64 + t62 * t73;
t90 = -t53 * xD_base(4) + t32;
t13 = (-t108 * t64 + t78) * xD_base(5) + (-t62 * t96 + t63 * t88 + t65 * t90) * t67;
t16 = t63 * t90 - t65 * t88;
t71 = -t101 * t64 - t62 * t94;
t8 = t13 * xD_base(6) + t16 * xD_base(5) + t71 * xD_base(4);
t138 = t8 / 0.2e1;
t107 = 0.2e1 * xD_base(6);
t109 = t63 ^ 2 + t65 ^ 2;
t93 = t66 * (0.1e1 - t109);
t45 = t93 * t100;
t95 = t67 * xD_base(4);
t39 = t107 * t45 + t95 * xD_base(5);
t133 = t39 / 0.2e1;
t150 = t109 * t61;
t144 = (t108 + t150) * xD_base(6) + t97;
t27 = t144 * t62 + xD_base(6) * t79;
t149 = t15 - t27;
t28 = -t144 * t64 + xD_base(6) * t78;
t148 = t16 - t28;
t105 = t67 * xD_base(1);
t36 = -t105 * t62 + t52 * xD_base(2) + t54 * xD_base(3);
t37 = t105 * t64 + t53 * xD_base(2) - t55 * xD_base(3);
t51 = t60 + t150;
t48 = t51 * xD_base(6) + t97;
t119 = t48 * Ifges(1,3);
t41 = t53 * t65 - t55 * t63;
t104 = t41 * xD_base(5);
t74 = t82 - t114;
t30 = t74 * t67;
t23 = -t30 * xD_base(6) + t64 * t95 + t104;
t123 = t23 * Ifges(1,5);
t29 = (-t62 * t66 - t83) * t67;
t40 = t52 * t65 + t54 * t63;
t22 = t29 * xD_base(6) + t40 * xD_base(5) - t62 * t95;
t124 = t22 * Ifges(1,6);
t9 = t119 + t123 + t124;
t146 = t36 * mrSges(1,1) - t37 * mrSges(1,2) + t9 / 0.2e1;
t76 = t63 * xD_base(2) - t65 * xD_base(3);
t145 = -t66 * xD_base(1) + t67 * t76;
t142 = Ifges(1,5) * t138 + Ifges(1,6) * t139 + Ifges(1,3) * t133;
t143 = 0.2e1 * t45;
t2 = t8 * Ifges(1,4) + t7 * Ifges(1,2) + t39 * Ifges(1,6);
t141 = t2 / 0.2e1;
t140 = Ifges(1,1) * t138 + Ifges(1,4) * t139 + Ifges(1,5) * t133;
t137 = -t22 / 0.2e1;
t136 = t22 / 0.2e1;
t135 = -t23 / 0.2e1;
t134 = t23 / 0.2e1;
t132 = -t48 / 0.2e1;
t131 = t48 / 0.2e1;
t130 = -t62 / 0.2e1;
t129 = -t64 / 0.2e1;
t126 = Ifges(1,4) * t62;
t125 = Ifges(1,4) * t64;
t116 = t63 * t67;
t112 = t65 * t67;
t24 = -t104 + (-t64 * xD_base(4) + t74 * xD_base(6)) * t67;
t111 = t12 - t24;
t110 = t13 - t22;
t103 = t62 * xD_base(5);
t102 = t64 * xD_base(5);
t99 = t62 * xD_base(6);
t98 = t64 * xD_base(6);
t87 = mrSges(1,1) * t62 + mrSges(1,2) * t64;
t86 = Ifges(1,1) * t64 - t126;
t85 = -Ifges(1,2) * t62 + t125;
t84 = Ifges(1,5) * t64 - Ifges(1,6) * t62;
t77 = -t63 * xD_base(3) - t65 * xD_base(2);
t72 = t76 * t66;
t69 = -t101 * t65 - t63 * t95;
t68 = t101 * t63 - t65 * t95;
t56 = t77 * t67;
t50 = t72 + t105;
t47 = t145 * t64;
t46 = t145 * t62;
t43 = t53 * xD_base(3) + t55 * xD_base(2);
t42 = t52 * xD_base(3) - t54 * xD_base(2);
t35 = xD_base(5) * t72 + (t77 * xD_base(4) + xD_base(5) * xD_base(1)) * t67;
t21 = Ifges(1,4) * t22;
t20 = t32 * xD_base(3) + t34 * xD_base(2) + t71 * xD_base(1);
t19 = t31 * xD_base(3) + t33 * xD_base(2) + t70 * xD_base(1);
t18 = mrSges(1,1) * t48 - mrSges(1,3) * t23;
t17 = -mrSges(1,2) * t48 + t22 * mrSges(1,3);
t14 = -mrSges(1,1) * t22 + mrSges(1,2) * t23;
t11 = t23 * Ifges(1,1) + t48 * Ifges(1,5) + t21;
t10 = t23 * Ifges(1,4) + t22 * Ifges(1,2) + t48 * Ifges(1,6);
t6 = mrSges(1,1) * t39 - mrSges(1,3) * t8;
t5 = -mrSges(1,2) * t39 + mrSges(1,3) * t7;
t4 = -mrSges(1,1) * t7 + mrSges(1,2) * t8;
t1 = [(m(1) * (-t102 * t37 + t103 * t36 + t35) - t17 * t102 + t18 * t103 + t4) * t66 + (m(1) * (-t145 * xD_base(5) - t19 * t62 + t20 * t64 - t36 * t98 - t37 * t99) - t17 * t99 + t64 * t5 - t18 * t98 - t62 * t6 + xD_base(5) * t14) * t67; m(1) * (-t116 * t35 - t145 * t68 + t19 * t52 + t20 * t53 + t36 * t33 + t37 * t34) + t34 * t17 + t53 * t5 + t33 * t18 + t52 * t6 - t4 * t116 + t68 * t14; m(1) * (t112 * t35 - t145 * t69 + t19 * t54 - t20 * t55 + t36 * t31 + t37 * t32) + t32 * t17 - t55 * t5 + t31 * t18 + t54 * t6 + t4 * t112 + t69 * t14; -t56 * t14 - t42 * t18 - t43 * t17 - m(1) * (-t145 * t56 + t36 * t42 + t37 * t43) + (-t20 * mrSges(1,2) + t19 * mrSges(1,1) + 0.2e1 * t142 + (t84 * t132 + t145 * t87 + t86 * t135 + t85 * t137 + t62 * t10 / 0.2e1 + t11 * t129 + (t36 * t64 + t37 * t62) * mrSges(1,3)) * xD_base(5)) * t66 + (t86 * t138 + t85 * t139 + t35 * t87 + t84 * t133 + t2 * t130 + t64 * t140 + (-t19 * t64 - t20 * t62) * mrSges(1,3) + (t119 / 0.2e1 + t123 / 0.2e1 + t124 / 0.2e1 + t146) * xD_base(5) + ((-Ifges(1,5) * t62 - Ifges(1,6) * t64) * t131 - t145 * (mrSges(1,1) * t64 - mrSges(1,2) * t62) + (-Ifges(1,1) * t62 - t125) * t134 + (-Ifges(1,2) * t64 - t126) * t136 + t11 * t130 + t10 * t129 + (t36 * t62 - t37 * t64) * mrSges(1,3)) * xD_base(6)) * t67; (Ifges(1,5) * t28 + Ifges(1,6) * t27) * t132 + (Ifges(1,5) * t16 + Ifges(1,6) * t15) * t131 - t50 * t14 + t40 * t141 + t35 * (-mrSges(1,1) * t40 + mrSges(1,2) * t41) + (Ifges(1,5) * t41 + Ifges(1,6) * t40) * t133 + (Ifges(1,1) * t41 + Ifges(1,4) * t40) * t138 + (Ifges(1,4) * t41 + Ifges(1,2) * t40) * t139 + t41 * t140 + (Ifges(1,1) * t28 + Ifges(1,4) * t27) * t135 + (Ifges(1,4) * t28 + Ifges(1,2) * t27) * t137 + t46 * t18 - t47 * t17 + (Ifges(1,1) * t16 + Ifges(1,4) * t15) * t134 + (Ifges(1,4) * t16 + Ifges(1,2) * t15) * t136 - m(1) * (-t36 * t46 + t37 * t47) + (-t28 / 0.2e1 + t16 / 0.2e1) * t11 + (-t27 / 0.2e1 + t15 / 0.2e1) * t10 + (-t148 * t36 + t149 * t37 - t19 * t41 + t20 * t40) * mrSges(1,3) - (-m(1) * t50 - mrSges(1,1) * t149 + mrSges(1,2) * t148) * t145 + (Ifges(1,5) * t135 + Ifges(1,6) * t137 + Ifges(1,3) * t132 - t146) * (t107 * t93 + xD_base(4)) * t67; (Ifges(1,5) * t13 + Ifges(1,6) * t12 + Ifges(1,3) * t143) * t131 + (Ifges(1,5) * t22 + Ifges(1,6) * t24) * t132 + (-Ifges(1,5) * t30 + Ifges(1,6) * t29 + Ifges(1,3) * t51) * t133 + t20 * (-mrSges(1,2) * t51 + mrSges(1,3) * t29) + (-Ifges(1,1) * t30 + Ifges(1,4) * t29 + Ifges(1,5) * t51) * t138 + (-Ifges(1,4) * t30 + Ifges(1,2) * t29 + Ifges(1,6) * t51) * t139 + t19 * (mrSges(1,1) * t51 + mrSges(1,3) * t30) + t51 * t142 + (Ifges(1,1) * t13 + Ifges(1,4) * t12 + Ifges(1,5) * t143) * t134 + (Ifges(1,4) * t13 + Ifges(1,2) * t12 + Ifges(1,6) * t143) * t136 + t35 * (-mrSges(1,1) * t29 - mrSges(1,2) * t30) + t29 * t141 - t30 * t140 + (Ifges(1,1) * t22 + Ifges(1,4) * t24) * t135 + (Ifges(1,2) * t24 + t21) * t137 + t45 * t9 - (-mrSges(1,1) * t111 + mrSges(1,2) * t110) * t145 + (t137 + t13 / 0.2e1) * t11 + (-t24 / 0.2e1 + t12 / 0.2e1) * t10 + (mrSges(1,1) * t143 - t110 * mrSges(1,3) - t17) * t36 + (-0.2e1 * mrSges(1,2) * t45 + mrSges(1,3) * t111 + t18) * t37;];
Fc  = t1(:);
