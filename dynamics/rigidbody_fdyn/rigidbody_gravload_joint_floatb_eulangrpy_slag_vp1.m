% Calculate Gravitation load on the joints for
% rigidbody
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [0x1]
%   Joint Angles [rad]
% phi_base [3x1]
%   Base orientation in world frame. Expressed with RPY Euler angles (xyz)
% g [3x1]
%   gravitation vector in world frame [m/s^2]
% a_mdh, d_mdh, q_offset_mdh [0x1]
%   kinematic parameters
% m_num_mdh, rSges_num_mdh [1x1]
%   dynamic parameters
% 
% Output:
% taug [0x1]
%   joint torques required to compensate gravitation load

function taug = rigidbody_gravload_joint_floatb_eulangrpy_slag_vp1(q, phi_base, g, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh, m_num, rSges_num_mdh)
%%Coder Information
%#codegen
assert(isa(q,'double') && isreal(q) && all(size(q) == [0 1]), ...
  'rigidbody_gravload_joint_floatb_eulangrpy_slag_vp1: q has to be [0x1] double');
assert(isa(phi_base,'double') && isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  'rigidbody_gravload_joint_floatb_eulangrpy_slag_vp1: phi_base has to be [3x1] double');
assert(isa(g,'double') && isreal(g) && all(size(g) == [3 1]), ...
  'rigidbody_gravload_joint_floatb_eulangrpy_slag_vp1: g has to be [3x1] double');
assert(isa(alpha_mdh,'double') && isreal(alpha_mdh) && all(size(alpha_mdh) == [0 1]), ...
  'rigidbody_gravload_joint_floatb_eulangrpy_slag_vp1: alpha_mdh has to be [0x1] double');
assert(isa(a_mdh,'double') && isreal(a_mdh) && all(size(a_mdh) == [0 1]), ...
  'rigidbody_gravload_joint_floatb_eulangrpy_slag_vp1: a_mdh has to be [0x1] double');
assert(isa(d_mdh,'double') && isreal(d_mdh) && all(size(d_mdh) == [0 1]), ...
  'rigidbody_gravload_joint_floatb_eulangrpy_slag_vp1: d_mdh has to be [0x1] double');
assert(isa(q_offset_mdh,'double') && isreal(q_offset_mdh) && all(size(q_offset_mdh) == [0 1]), ...
  'rigidbody_gravload_joint_floatb_eulangrpy_slag_vp1: q_offset_mdh has to be [0x1] double');
assert(isa(b_mdh,'double') && isreal(b_mdh) && all(size(b_mdh) == [0 1]), ...
  'rigidbody_gravload_joint_floatb_eulangrpy_slag_vp1: b_mdh has to be [0x1] double');
assert(isa(beta_mdh,'double') && isreal(beta_mdh) && all(size(beta_mdh) == [0 1]), ...
  'rigidbody_gravload_joint_floatb_eulangrpy_slag_vp1: beta_mdh has to be [0x1] double');
assert(isa(m_num,'double') && isreal(m_num) && all(size(m_num) == [1 1]), ...
  'rigidbody_gravload_joint_floatb_eulangrpy_slag_vp1: m_num has to be [1x1] double'); 
assert(isa(rSges_num_mdh,'double') && isreal(rSges_num_mdh) && all(size(rSges_num_mdh) == [1,3]), ...
  'rigidbody_gravload_joint_floatb_eulangrpy_slag_vp1: rSges_num_mdh has to be [1x3] double');

%% Variable Initialization

alphaxs_base = phi_base(1);
betays_base = phi_base(2);
gammazs_base = phi_base(3);

g1 = g(1);
g2 = g(2);
g3 = g(3);


M1 = m_num(1);

SX1 = rSges_num_mdh(1,1);
SY1 = rSges_num_mdh(1,2);
SZ1 = rSges_num_mdh(1,3);

%%Symbolic Calculation
%From rigidbody_joint_gravload_floatb_eulangrpy_par1_matlab.m
t1 = [];
taug  = t1 (:);
