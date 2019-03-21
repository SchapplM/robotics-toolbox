% Calculate floating base base inertia matrix for
% rigidbody
% Use Code from Maple symbolic Code Generation
% 
% Input:
% phi_base [3x1]
%   Base orientation in world frame. Expressed with XYZ-Euler angles
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
% Mb [6x6]
%   base inertia matrix (gives inertial forces on the base from base acceleration)

% Quelle: HybrDyn-Toolbox
% Datum: 2019-03-19 12:00
% Revision: 3380762d7a67e58abcb72423f3b6cbd7db453188 (2019-03-19)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Mb = rigidbody_inertiaB_floatb_eulxyz_slag_vp1(phi_base, ...
  m, rSges, Icges)
%% Coder Information
%#codegen
%$cgargs {zeros(3,1),zeros(1,1),zeros(1,3),zeros(1,6)}
assert(isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  'rigidbody_inertiaB_floatb_eulxyz_slag_vp1: phi_base has to be [3x1] (double)');
assert(isreal(m) && all(size(m) == [1 1]), ...
  'rigidbody_inertiaB_floatb_eulxyz_slag_vp1: m has to be [1x1] (double)'); 
assert(isreal(rSges) && all(size(rSges) == [1,3]), ...
  'rigidbody_inertiaB_floatb_eulxyz_slag_vp1: rSges has to be [1x3] (double)');
assert(isreal(Icges) && all(size(Icges) == [1 6]), ...
  'rigidbody_inertiaB_floatb_eulxyz_slag_vp1: Icges has to be [1x6] (double)'); 

%% Symbolic Calculation
% From inertia_base_base_floatb_eulxyz_par1_matlab.m
% OptimizationMode: 2
% StartTime: 2019-03-19 12:00:20
% EndTime: 2019-03-19 12:00:20
% DurationCPUTime: 0.26s
% Computational Cost: add. (675->87), mult. (1737->168), div. (0->0), fcn. (1900->6), ass. (0->53)
t97 = cos(phi_base(1));
t112 = t97 / 0.2e1;
t94 = sin(phi_base(3));
t96 = cos(phi_base(3));
t98 = sin(phi_base(2));
t99 = cos(phi_base(2));
t89 = t98 * rSges(1,3) + (rSges(1,1) * t96 - rSges(1,2) * t94) * t99;
t111 = m(1) * t89;
t95 = sin(phi_base(1));
t110 = t95 * t98;
t109 = t95 * t99;
t108 = t97 * t98;
t107 = t97 * t99;
t106 = Icges(1,5) * t99;
t105 = Icges(1,6) * t99;
t104 = Icges(1,3) * t99;
t90 = -t110 * t94 + t96 * t97;
t91 = t110 * t96 + t94 * t97;
t74 = Icges(1,5) * t91 + Icges(1,6) * t90 - t104 * t95;
t76 = Icges(1,4) * t91 + Icges(1,2) * t90 - t105 * t95;
t78 = Icges(1,1) * t91 + Icges(1,4) * t90 - t106 * t95;
t65 = t98 * t74 + (-t76 * t94 + t78 * t96) * t99;
t86 = Icges(1,3) * t98 + (Icges(1,5) * t96 - Icges(1,6) * t94) * t99;
t87 = Icges(1,6) * t98 + (Icges(1,4) * t96 - Icges(1,2) * t94) * t99;
t88 = Icges(1,5) * t98 + (Icges(1,1) * t96 - Icges(1,4) * t94) * t99;
t68 = -t109 * t86 + t87 * t90 + t88 * t91;
t103 = t65 / 0.2e1 + t68 / 0.2e1;
t92 = t108 * t94 + t95 * t96;
t93 = -t108 * t96 + t94 * t95;
t75 = Icges(1,5) * t93 + Icges(1,6) * t92 + t104 * t97;
t77 = Icges(1,4) * t93 + Icges(1,2) * t92 + t105 * t97;
t79 = Icges(1,1) * t93 + Icges(1,4) * t92 + t106 * t97;
t66 = t98 * t75 + (-t77 * t94 + t79 * t96) * t99;
t69 = t107 * t86 + t87 * t92 + t88 * t93;
t102 = t66 / 0.2e1 + t69 / 0.2e1;
t80 = rSges(1,1) * t91 + rSges(1,2) * t90 - rSges(1,3) * t109;
t81 = rSges(1,1) * t93 + rSges(1,2) * t92 + rSges(1,3) * t107;
t101 = -t80 * t97 - t81 * t95;
t100 = t98 * t86 + (-t87 * t94 + t88 * t96) * t99;
t73 = t107 * t89 - t81 * t98;
t72 = t109 * t89 + t80 * t98;
t71 = t100 * t98;
t70 = -t80 * t95 + t81 * t97;
t67 = t101 * t99;
t64 = t107 * t75 + t77 * t92 + t79 * t93;
t63 = t107 * t74 + t76 * t92 + t78 * t93;
t62 = -t109 * t75 + t77 * t90 + t79 * t91;
t61 = -t109 * t74 + t76 * t90 + t78 * t91;
t60 = t63 * t97 + t64 * t95;
t59 = t61 * t97 + t62 * t95;
t58 = t69 * t98 + (-t63 * t95 + t64 * t97) * t99;
t57 = t68 * t98 + (-t61 * t95 + t62 * t97) * t99;
t1 = [m(1); 0; m(1); 0; 0; m(1); 0; -m(1) * t81; m(1) * t80; m(1) * (t80 ^ 2 + t81 ^ 2) + t100; m(1) * t70; t95 * t111; -t97 * t111; t101 * t111 + t102 * t95 + t103 * t97; m(1) * (t70 ^ 2 + (t95 ^ 2 + t97 ^ 2) * t89 ^ 2) + t97 * t59 + t95 * t60; m(1) * t67; m(1) * t73; m(1) * t72; m(1) * (t72 * t80 - t73 * t81) + t71 + (t102 * t97 - t103 * t95) * t99; m(1) * (t67 * t70 + (-t72 * t97 + t73 * t95) * t89) + t98 * (t65 * t97 + t66 * t95) / 0.2e1 + t57 * t112 + t95 * t58 / 0.2e1 + (-t95 * t59 / 0.2e1 + t60 * t112) * t99; m(1) * (t67 ^ 2 + t72 ^ 2 + t73 ^ 2) + t98 * t71 + (t98 * (-t65 * t95 + t66 * t97) - t95 * t57 + t97 * t58) * t99;];
%% Postprocessing: Reshape Output
% From vec2symmat_6_matlab.m
res = [t1(1) t1(2) t1(4) t1(7) t1(11) t1(16); t1(2) t1(3) t1(5) t1(8) t1(12) t1(17); t1(4) t1(5) t1(6) t1(9) t1(13) t1(18); t1(7) t1(8) t1(9) t1(10) t1(14) t1(19); t1(11) t1(12) t1(13) t1(14) t1(15) t1(20); t1(16) t1(17) t1(18) t1(19) t1(20) t1(21);];
Mb  = res;
