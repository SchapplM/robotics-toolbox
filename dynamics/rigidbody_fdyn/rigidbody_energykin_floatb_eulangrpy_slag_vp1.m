% Calculate kinetic energy for
% rigidbody
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [0x1]
%   Joint Angles [rad]
% qD [0x1]
%   Joint Velocities [rad/s]
% phi_base [3x1]
%   Base orientation in world frame. Expressed with RPY Euler angles (xyz)
% xD_base [6x1]
%   time derivative of 
%   r_base (3x1 Base position in world frame) and 
%   phi_base (3x1)
% a_mdh, d_mdh, q_offset_mdh [0x1]
%   kinematic parameters
% m_num_mdh, rSges_num_mdh, Icges_num_mdh [1x1]
%   dynamic parameters (parameter set 1: center of mass and inertia about center of mass)
% 
% Output:
% T [1x1]
%   kinetic energy

function T = rigidbody_energykin_floatb_eulangrpy_slag_vp1(q, qD, phi_base, xD_base, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh, m_num, rSges_num_mdh, Icges_num_mdh)
%%Coder Information
%#codegen
assert(isa(q,'double') && isreal(q) && all(size(q) == [0 1]), ...
  'rigidbody_energykin_floatb_eulangrpy_slag_vp1: q has to be [0x1] double');
assert(isa(qD,'double') && isreal(qD) && all(size(qD) == [0 1]), ...
  'rigidbody_energykin_floatb_eulangrpy_slag_vp1: qD has to be [0x1] double');
assert(isa(phi_base,'double') && isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  'rigidbody_energykin_floatb_eulangrpy_slag_vp1: phi_base has to be [3x1] double');
assert(isa(xD_base,'double') && isreal(xD_base) && all(size(xD_base) == [6 1]), ...
  'rigidbody_energykin_floatb_eulangrpy_slag_vp1: xD_base has to be [6x1] double');
assert(isa(alpha_mdh,'double') && isreal(alpha_mdh) && all(size(alpha_mdh) == [0 1]), ...
  'rigidbody_energykin_floatb_eulangrpy_slag_vp1: alpha_mdh has to be [0x1] double');
assert(isa(a_mdh,'double') && isreal(a_mdh) && all(size(a_mdh) == [0 1]), ...
  'rigidbody_energykin_floatb_eulangrpy_slag_vp1: a_mdh has to be [0x1] double');
assert(isa(d_mdh,'double') && isreal(d_mdh) && all(size(d_mdh) == [0 1]), ...
  'rigidbody_energykin_floatb_eulangrpy_slag_vp1: d_mdh has to be [0x1] double');
assert(isa(q_offset_mdh,'double') && isreal(q_offset_mdh) && all(size(q_offset_mdh) == [0 1]), ...
  'rigidbody_energykin_floatb_eulangrpy_slag_vp1: q_offset_mdh has to be [0x1] double');
assert(isa(b_mdh,'double') && isreal(b_mdh) && all(size(b_mdh) == [0 1]), ...
  'rigidbody_energykin_floatb_eulangrpy_slag_vp1: b_mdh has to be [0x1] double');
assert(isa(beta_mdh,'double') && isreal(beta_mdh) && all(size(beta_mdh) == [0 1]), ...
  'rigidbody_energykin_floatb_eulangrpy_slag_vp1: beta_mdh has to be [0x1] double');
assert(isa(m_num,'double') && isreal(m_num) && all(size(m_num) == [1 1]), ...
  'rigidbody_energykin_floatb_eulangrpy_slag_vp1: m_num has to be [1x1] double'); 
assert(isa(rSges_num_mdh,'double') && isreal(rSges_num_mdh) && all(size(rSges_num_mdh) == [1,3]), ...
  'rigidbody_energykin_floatb_eulangrpy_slag_vp1: rSges_num_mdh has to be [1x3] double');
assert(isa(Icges_num_mdh,'double') && isreal(Icges_num_mdh) && all(size(Icges_num_mdh) == [1 6]), ...
  'rigidbody_energykin_floatb_eulangrpy_slag_vp1: Icges_num_mdh has to be [1x6] double'); 

%% Variable Initialization


alphaxs_base = phi_base(1);
betays_base = phi_base(2);
gammazs_base = phi_base(3);

vxs_base = xD_base(1);
vys_base = xD_base(2);
vzs_base = xD_base(3);
alphaDx_base = xD_base(4);
betaDy_base = xD_base(5);
gammaDz_base = xD_base(6);


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
%From rigidbody_energy_kinetic_floatb_eulangrpy_worldframe_par1_matlab.m
t33 = sin(alphaxs_base);
t36 = sin(betays_base);
t45 = t33 * t36;
t37 = cos(betays_base);
t44 = t33 * t37;
t35 = cos(alphaxs_base);
t43 = t35 * t36;
t42 = t35 * t37;
t41 = XZC1 * t37;
t40 = YZC1 * t37;
t39 = ZZC1 * t37;
t38 = t37 * gammaDz_base;
t34 = cos(gammazs_base);
t32 = sin(gammazs_base);
t31 = t36 * gammaDz_base + alphaDx_base;
t30 = -t33 * t38 + t35 * betaDy_base;
t29 = t33 * betaDy_base + t35 * t38;
t28 = t33 * t32 - t34 * t43;
t27 = t32 * t43 + t33 * t34;
t26 = t35 * t32 + t34 * t45;
t25 = -t32 * t45 + t35 * t34;
t24 = t36 * SZ1 + (SX1 * t34 - SY1 * t32) * t37;
t23 = XZC1 * t36 + (XXC1 * t34 - XYC1 * t32) * t37;
t22 = YZC1 * t36 + (XYC1 * t34 - YYC1 * t32) * t37;
t21 = ZZC1 * t36 + (XZC1 * t34 - YZC1 * t32) * t37;
t20 = t28 * SX1 + t27 * SY1 + SZ1 * t42;
t19 = t26 * SX1 + t25 * SY1 - SZ1 * t44;
t18 = XXC1 * t28 + XYC1 * t27 + t35 * t41;
t17 = XXC1 * t26 + XYC1 * t25 - t33 * t41;
t16 = XYC1 * t28 + YYC1 * t27 + t35 * t40;
t15 = XYC1 * t26 + YYC1 * t25 - t33 * t40;
t14 = XZC1 * t28 + YZC1 * t27 + t35 * t39;
t13 = XZC1 * t26 + YZC1 * t25 - t33 * t39;
t12 = -t31 * t20 + t29 * t24 + vys_base;
t11 = t31 * t19 - t30 * t24 + vzs_base;
t10 = -t29 * t19 + t30 * t20 + vxs_base;
t1 = M1 * (t10 ^ 2 + t11 ^ 2 + t12 ^ 2) / 0.2e1 + t31 * ((t13 * t30 + t14 * t29 + t21 * t31) * t36 + ((-t22 * t32 + t23 * t34) * t31 + (-t15 * t32 + t17 * t34) * t30 + (-t16 * t32 + t18 * t34) * t29) * t37) / 0.2e1 + t30 * ((-t21 * t44 + t25 * t22 + t26 * t23) * t31 + (-t13 * t44 + t25 * t15 + t26 * t17) * t30 + (-t14 * t44 + t25 * t16 + t26 * t18) * t29) / 0.2e1 + t29 * ((t21 * t42 + t27 * t22 + t28 * t23) * t31 + (t13 * t42 + t27 * t15 + t28 * t17) * t30 + (t14 * t42 + t27 * t16 + t28 * t18) * t29) / 0.2e1;
T  = t1 ;
