% Calculate inertial parameters regressor of Gravitation load on the base for
% rigidbody
% Use Code from Maple symbolic Code Generation
% 
% Input:
% phi_base [3x1]
%   Base orientation in world frame. Expressed with XYZ-Euler angles
% g [3x1]
%   gravitation vector in world frame [m/s^2]
% 
% Output:
% taug_reg [6x(1*10)]
%   inertial parameter regressor of base forces required to compensate gravitation load
%   base moment as Euler angle moment

% Quelle: HybrDyn-Toolbox
% Datum: 2019-08-21 13:58
% Revision: f86a32e2b5f1abc78572314ec3e2c1664fd21c45 (2019-08-20)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function taug_reg = rigidbody_gravloadB_floatb_eulxyz_reg2_slag_vp(phi_base, g)
%% Coder Information
%#codegen
%$cgargs {zeros(3,1),zeros(3,1)}
assert(isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  'rigidbody_gravloadB_floatb_eulxyz_reg2_slag_vp: phi_base has to be [3x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'rigidbody_gravloadB_floatb_eulxyz_reg2_slag_vp: g has to be [3x1] (double)');

%% Symbolic Calculation
% From gravload_base_floatb_eulxyz_regressor_matlab.m
% OptimizationMode: 2
% StartTime: 2019-08-21 13:58:24
% EndTime: 2019-08-21 13:58:24
% DurationCPUTime: 0.11s
% Computational Cost: add. (24->18), mult. (61->31), div. (0->0), fcn. (69->6), ass. (0->16)
t10 = cos(phi_base(2));
t6 = sin(phi_base(1));
t8 = cos(phi_base(1));
t11 = g(2) * t6 - g(3) * t8;
t9 = sin(phi_base(2));
t16 = -g(1) * t9 + t11 * t10;
t14 = t6 * t9;
t13 = t8 * t9;
t12 = g(1) * t10;
t7 = cos(phi_base(3));
t5 = sin(phi_base(3));
t4 = t7 * t13 - t6 * t5;
t3 = t5 * t13 + t6 * t7;
t2 = t7 * t14 + t8 * t5;
t1 = -t5 * t14 + t8 * t7;
t15 = [0, 0, 0, 0, 0, 0, 0, 0, 0, -g(1); 0, 0, 0, 0, 0, 0, 0, 0, 0, -g(2); 0, 0, 0, 0, 0, 0, 0, 0, 0, -g(3); 0, 0, 0, 0, 0, 0, -g(2) * t4 - g(3) * t2, g(2) * t3 - g(3) * t1, (g(2) * t8 + g(3) * t6) * t10, 0; 0, 0, 0, 0, 0, 0, -t16 * t7, t16 * t5, -t11 * t9 - t12, 0; 0, 0, 0, 0, 0, 0, -g(2) * t1 - g(3) * t3 + t5 * t12, g(2) * t2 - g(3) * t4 + t7 * t12, 0, 0;];
taug_reg  = t15;
