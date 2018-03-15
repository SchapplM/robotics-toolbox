% Calculate joint-base inertia matrix for
% rigidbody
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [0x1]
%   Joint Angles [rad]
% phi_base [3x1]
%   Base orientation in world frame. Expressed with RPY Euler angles (xyz)
% a_mdh, d_mdh, q_offset_mdh [0x1]
%   kinematic parameters
% m_num_mdh, rSges_num_mdh, Icges_num_mdh [1x1]
%   dynamic parameters (parameter set 1: center of mass and inertia about center of mass)
% 
% Output:
% M [(0+6)x(0+6)]
%   full inertia matrix (for base and joint dynamics)

function Mqb = rigidbody_inertia_floatb_eulangrpy_slag_vp1(q, phi_base, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh, m_num, rSges_num_mdh, Icges_num_mdh)
%%Coder Information
%#codegen
assert(isa(q,'double') && isreal(q) && all(size(q) == [0 1]), ...
  'rigidbody_inertia_floatb_eulangrpy_slag_vp1: q has to be [0x1] double');
assert(isa(phi_base,'double') && isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  'rigidbody_inertia_floatb_eulangrpy_slag_vp1: phi_base has to be [3x1] double');
assert(isa(alpha_mdh,'double') && isreal(alpha_mdh) && all(size(alpha_mdh) == [0 1]), ...
  'rigidbody_inertia_floatb_eulangrpy_slag_vp1: alpha_mdh has to be [0x1] double');
assert(isa(a_mdh,'double') && isreal(a_mdh) && all(size(a_mdh) == [0 1]), ...
  'rigidbody_inertia_floatb_eulangrpy_slag_vp1: a_mdh has to be [0x1] double');
assert(isa(d_mdh,'double') && isreal(d_mdh) && all(size(d_mdh) == [0 1]), ...
  'rigidbody_inertia_floatb_eulangrpy_slag_vp1: d_mdh has to be [0x1] double');
assert(isa(q_offset_mdh,'double') && isreal(q_offset_mdh) && all(size(q_offset_mdh) == [0 1]), ...
  'rigidbody_inertia_floatb_eulangrpy_slag_vp1: q_offset_mdh has to be [0x1] double');
assert(isa(b_mdh,'double') && isreal(b_mdh) && all(size(b_mdh) == [0 1]), ...
  'rigidbody_inertia_floatb_eulangrpy_slag_vp1: b_mdh has to be [0x1] double');
assert(isa(beta_mdh,'double') && isreal(beta_mdh) && all(size(beta_mdh) == [0 1]), ...
  'rigidbody_inertia_floatb_eulangrpy_slag_vp1: beta_mdh has to be [0x1] double');
assert(isa(m_num,'double') && isreal(m_num) && all(size(m_num) == [1 1]), ...
  'rigidbody_inertia_floatb_eulangrpy_slag_vp1: m_num has to be [1x1] double'); 
assert(isa(rSges_num_mdh,'double') && isreal(rSges_num_mdh) && all(size(rSges_num_mdh) == [1,3]), ...
  'rigidbody_inertia_floatb_eulangrpy_slag_vp1: rSges_num_mdh has to be [1x3] double');
assert(isa(Icges_num_mdh,'double') && isreal(Icges_num_mdh) && all(size(Icges_num_mdh) == [1 6]), ...
  'rigidbody_inertia_floatb_eulangrpy_slag_vp1: Icges_num_mdh has to be [1x6] double'); 

%% Variable Initialization

alphaxs_base = phi_base(1);
betays_base = phi_base(2);
gammazs_base = phi_base(3);


M1 = m_num(1);

SX1 = rSges_num_mdh(1,1);
SY1 = rSges_num_mdh(1,2);
SZ1 = rSges_num_mdh(1,3);

XXC1 = Icges_num_mdh(1,1);
XYC1 = Icges_num_mdh(1,4);
XZC1 = Icges_num_mdh(1,5);
YYC1 = Icges_num_mdh(1,2);
YZC1 = Icges_num_mdh(1,6);
ZZC1 = Icges_num_mdh(1,3);

%%Symbolic Calculation
%From rigidbody_inertia_floatb_eulangrpy_par1_matlab.m
t83 = cos(alphaxs_base);
t99 = t83 / 0.2e1;
t80 = sin(gammazs_base);
t82 = cos(gammazs_base);
t84 = sin(betays_base);
t85 = cos(betays_base);
t75 = t84 * SZ1 + (SX1 * t82 - SY1 * t80) * t85;
t98 = M1 * t75;
t81 = sin(alphaxs_base);
t97 = t81 * t84;
t96 = t81 * t85;
t95 = t83 * t84;
t94 = t83 * t85;
t93 = XZC1 * t85;
t92 = YZC1 * t85;
t91 = ZZC1 * t85;
t90 = t83 * t98;
t76 = -t80 * t97 + t83 * t82;
t77 = t83 * t80 + t82 * t97;
t59 = XZC1 * t77 + YZC1 * t76 - t81 * t91;
t61 = XYC1 * t77 + YYC1 * t76 - t81 * t92;
t63 = XXC1 * t77 + XYC1 * t76 - t81 * t93;
t44 = t84 * t59 + (-t61 * t80 + t63 * t82) * t85;
t72 = ZZC1 * t84 + (XZC1 * t82 - YZC1 * t80) * t85;
t73 = YZC1 * t84 + (XYC1 * t82 - YYC1 * t80) * t85;
t74 = XZC1 * t84 + (XXC1 * t82 - XYC1 * t80) * t85;
t48 = -t72 * t96 + t76 * t73 + t77 * t74;
t89 = t44 / 0.2e1 + t48 / 0.2e1;
t78 = t80 * t95 + t81 * t82;
t79 = t81 * t80 - t82 * t95;
t60 = XZC1 * t79 + YZC1 * t78 + t83 * t91;
t62 = XYC1 * t79 + YYC1 * t78 + t83 * t92;
t64 = XXC1 * t79 + XYC1 * t78 + t83 * t93;
t45 = t84 * t60 + (-t62 * t80 + t64 * t82) * t85;
t49 = t72 * t94 + t78 * t73 + t79 * t74;
t88 = t45 / 0.2e1 + t49 / 0.2e1;
t65 = t77 * SX1 + t76 * SY1 - SZ1 * t96;
t66 = t79 * SX1 + t78 * SY1 + SZ1 * t94;
t87 = -t65 * t83 - t66 * t81;
t86 = t84 * t72 + (-t73 * t80 + t74 * t82) * t85;
t69 = t81 * t98;
t58 = M1 * t66;
t57 = M1 * t65;
t56 = -t84 * t66 + t75 * t94;
t55 = t84 * t65 + t75 * t96;
t54 = M1 * t56;
t53 = M1 * t55;
t52 = t86 * t84;
t51 = -t81 * t65 + t83 * t66;
t50 = M1 * t51;
t47 = t87 * t85;
t46 = M1 * t47;
t43 = t60 * t94 + t78 * t62 + t79 * t64;
t42 = t59 * t94 + t78 * t61 + t79 * t63;
t41 = -t60 * t96 + t76 * t62 + t77 * t64;
t40 = -t59 * t96 + t76 * t61 + t77 * t63;
t39 = t42 * t83 + t43 * t81;
t38 = t40 * t83 + t41 * t81;
t37 = t49 * t84 + (-t42 * t81 + t43 * t83) * t85;
t36 = t48 * t84 + (-t40 * t81 + t41 * t83) * t85;
t35 = t88 * t81 + t89 * t83 + t87 * t98;
t34 = M1 * (t55 * t65 - t56 * t66) + t52 + (-t89 * t81 + t88 * t83) * t85;
t33 = M1 * (t47 * t51 + (-t55 * t83 + t56 * t81) * t75) + t84 * (t44 * t83 + t45 * t81) / 0.2e1 + t36 * t99 + t81 * t37 / 0.2e1 + (-t81 * t38 / 0.2e1 + t39 * t99) * t85;
t1 = [M1 0 0 0 t50 t46; 0 M1 0 -t58 t69 t54; 0 0 M1 t57 -t90 t53; 0 -t58 t57 M1 * (t65 ^ 2 + t66 ^ 2) + t86 t35 t34; t50 t69 -t90 t35 M1 * (t51 ^ 2 + (t81 ^ 2 + t83 ^ 2) * t75 ^ 2) + t83 * t38 + t81 * t39 t33; t46 t54 t53 t34 t33 M1 * (t47 ^ 2 + t55 ^ 2 + t56 ^ 2) + t84 * t52 + (t84 * (-t44 * t81 + t45 * t83) - t81 * t36 + t83 * t37) * t85;];
Mqb  = t1 ;
