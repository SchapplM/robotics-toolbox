% Calculate floating base base inertia matrix for
% rigidbody
% Use Code from Maple symbolic Code Generation
% 
% Input:
% phi_base [3x1]
%   Base orientation in world frame. Expressed with XYZ-Euler angles
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
% Mqb [6x6]
%   base inertia matrix (gives inertial forces on the base from base acceleration)

% Quelle: HybrDyn-Toolbox
% Datum: 2019-03-19 12:00
% Revision: 3380762d7a67e58abcb72423f3b6cbd7db453188 (2019-03-19)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Mqb = rigidbody_inertiaB_floatb_eulxyz_slag_vp2(phi_base, ...
  m, mrSges, Ifges)
%% Coder Information
%#codegen
%$cgargs {zeros(3,1),zeros(1,1),zeros(1,3),zeros(1,6)}
assert(isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  'rigidbody_inertiaB_floatb_eulxyz_slag_vp2: phi_base has to be [3x1] (double)');
assert(isreal(m) && all(size(m) == [1 1]), ...
  'rigidbody_inertiaB_floatb_eulxyz_slag_vp2: m has to be [1x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [1,3]), ...
  'rigidbody_inertiaB_floatb_eulxyz_slag_vp2: mrSges has to be [1x3] (double)');
assert(isreal(Ifges) && all(size(Ifges) == [1 6]), ...
  'rigidbody_inertiaB_floatb_eulxyz_slag_vp2: Ifges has to be [1x6] (double)'); 

%% Symbolic Calculation
% From inertia_base_base_floatb_eulxyz_par2_matlab.m
% OptimizationMode: 2
% StartTime: 2019-03-19 12:00:20
% EndTime: 2019-03-19 12:00:20
% DurationCPUTime: 0.19s
% Computational Cost: add. (405->81), mult. (1160->162), div. (0->0), fcn. (1329->6), ass. (0->49)
t78 = sin(phi_base(3));
t80 = cos(phi_base(3));
t82 = sin(phi_base(2));
t83 = cos(phi_base(2));
t64 = Ifges(1,6) * t82 + (Ifges(1,4) * t80 - Ifges(1,2) * t78) * t83;
t96 = t64 / 0.2e1;
t65 = Ifges(1,5) * t82 + (Ifges(1,1) * t80 - Ifges(1,4) * t78) * t83;
t95 = t65 / 0.2e1;
t77 = t83 ^ 2;
t79 = sin(phi_base(1));
t74 = t79 ^ 2 * t77;
t81 = cos(phi_base(1));
t75 = t81 ^ 2 * t77;
t76 = t82 ^ 2;
t66 = t76 + t74 + t75;
t94 = t66 / 0.2e1;
t93 = -t78 / 0.2e1;
t92 = t80 / 0.2e1;
t91 = t82 / 0.2e1;
t90 = m(1) * t83;
t89 = mrSges(1,3) * t83;
t88 = t79 * t82;
t87 = t79 * t83;
t86 = t80 * t82;
t85 = t81 * t82;
t84 = t81 * t83;
t73 = t82 * mrSges(1,1) - t80 * t89;
t72 = -t82 * mrSges(1,2) - t78 * t89;
t71 = (mrSges(1,1) * t78 + mrSges(1,2) * t80) * t83;
t70 = t79 * t78 - t80 * t85;
t69 = t78 * t85 + t79 * t80;
t68 = t81 * t78 + t79 * t86;
t67 = -t78 * t88 + t81 * t80;
t63 = Ifges(1,3) * t82 + (Ifges(1,5) * t80 - Ifges(1,6) * t78) * t83;
t62 = t68 * t81 + t70 * t79;
t61 = t67 * t81 + t69 * t79;
t60 = (-t68 * t79 + t70 * t81 + t86) * t83;
t59 = (-t67 * t79 + t69 * t81 - t78 * t82) * t83;
t58 = t66 * mrSges(1,1) - t60 * mrSges(1,3);
t57 = -t66 * mrSges(1,2) + t59 * mrSges(1,3);
t56 = Ifges(1,1) * t62 + Ifges(1,4) * t61;
t55 = Ifges(1,4) * t62 + Ifges(1,2) * t61;
t54 = Ifges(1,5) * t62 + Ifges(1,6) * t61;
t53 = -t61 * mrSges(1,1) + t62 * mrSges(1,2);
t52 = -t59 * mrSges(1,1) + t60 * mrSges(1,2);
t51 = Ifges(1,1) * t60 + Ifges(1,4) * t59 + Ifges(1,5) * t66;
t50 = Ifges(1,4) * t60 + Ifges(1,2) * t59 + Ifges(1,6) * t66;
t49 = Ifges(1,5) * t60 + Ifges(1,6) * t59 + Ifges(1,3) * t66;
t1 = [m(1) * (t76 + (t78 ^ 2 + t80 ^ 2) * t77); (-t67 * t78 + t68 * t80 - t88) * t90; m(1) * (t67 ^ 2 + t68 ^ 2 + t74); (-t69 * t78 + t70 * t80 + t85) * t90; m(1) * (-t81 * t77 * t79 + t69 * t67 + t70 * t68); m(1) * (t69 ^ 2 + t70 ^ 2 + t75); t82 * t71 + (t72 * t80 - t73 * t78) * t83; t67 * t73 + t68 * t72 - t71 * t87; t69 * t73 + t70 * t72 + t71 * t84; t82 * t63 + (-t64 * t78 + t65 * t80) * t83; t82 * t53 + (t61 * t80 + t62 * t78) * t89; -t53 * t87 + (t61 * t68 - t62 * t67) * mrSges(1,3); t53 * t84 + (t61 * t70 - t62 * t69) * mrSges(1,3); t62 * t95 + t61 * t96 + t54 * t91 + (t55 * t93 + t56 * t92) * t83; t61 * t55 + t62 * t56; t82 * t52 + (t57 * t80 - t58 * t78) * t83; -t52 * t87 + t68 * t57 + t67 * t58; t52 * t84 + t70 * t57 + t69 * t58; t60 * t95 + t59 * t96 + t49 * t91 + t63 * t94 + (t50 * t93 + t51 * t92) * t83; t62 * t51 / 0.2e1 + t60 * t56 / 0.2e1 + t61 * t50 / 0.2e1 + t59 * t55 / 0.2e1 + t54 * t94; t66 * t49 + t59 * t50 + t60 * t51;];
%% Postprocessing: Reshape Output
% From vec2symmat_6_matlab.m
res = [t1(1) t1(2) t1(4) t1(7) t1(11) t1(16); t1(2) t1(3) t1(5) t1(8) t1(12) t1(17); t1(4) t1(5) t1(6) t1(9) t1(13) t1(18); t1(7) t1(8) t1(9) t1(10) t1(14) t1(19); t1(11) t1(12) t1(13) t1(14) t1(15) t1(20); t1(16) t1(17) t1(18) t1(19) t1(20) t1(21);];
Mqb  = res;
