% Calculate kinetic energy for
% rigidbody
% Use Code from Maple symbolic Code Generation
% 
% Input:
% phi_base [3x1]
%   Base orientation in world frame. Expressed with XYZ-Euler angles
% xD_base [6x1]
%   time derivative of r_base and phi_base
% pkin [1x1]
%   kinematic parameters (e.g. lengths of the links)
%   pkin=[dummy]';
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
% T [1x1]
%   kinetic energy

% Quelle: HybrDyn-Toolbox
% Datum: 2019-03-19 12:00
% Revision: 3380762d7a67e58abcb72423f3b6cbd7db453188 (2019-03-19)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function T = rigidbody_energykin_floatb_eulxyz_slag_vp1(phi_base, xD_base, ...
  m, rSges, Icges)
%% Coder Information
%#codegen
%$cgargs {zeros(3,1),zeros(6,1),zeros(1,1),zeros(1,3),zeros(1,6)}
assert(isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  'rigidbody_energykin_floatb_eulxyz_slag_vp1: phi_base has to be [3x1] (double)');
assert(isreal(xD_base) && all(size(xD_base) == [6 1]), ...
  'rigidbody_energykin_floatb_eulxyz_slag_vp1: xD_base has to be [6x1] (double)');
assert(isreal(m) && all(size(m) == [1 1]), ...
  'rigidbody_energykin_floatb_eulxyz_slag_vp1: m has to be [1x1] (double)'); 
assert(isreal(rSges) && all(size(rSges) == [1,3]), ...
  'rigidbody_energykin_floatb_eulxyz_slag_vp1: rSges has to be [1x3] (double)');
assert(isreal(Icges) && all(size(Icges) == [1 6]), ...
  'rigidbody_energykin_floatb_eulxyz_slag_vp1: Icges has to be [1x6] (double)'); 

%% Symbolic Calculation
% From energy_kinetic_floatb_eulxyz_worldframe_par1_matlab.m
% OptimizationMode: 2
% StartTime: 2019-03-19 12:00:14
% EndTime: 2019-03-19 12:00:14
% DurationCPUTime: 0.14s
% Computational Cost: add. (175->66), mult. (418->116), div. (0->0), fcn. (448->6), ass. (0->37)
t33 = sin(phi_base(1));
t36 = sin(phi_base(2));
t45 = t33 * t36;
t37 = cos(phi_base(2));
t44 = t33 * t37;
t35 = cos(phi_base(1));
t43 = t35 * t36;
t42 = t35 * t37;
t41 = Icges(1,5) * t37;
t40 = Icges(1,6) * t37;
t39 = Icges(1,3) * t37;
t38 = t37 * xD_base(6);
t34 = cos(phi_base(3));
t32 = sin(phi_base(3));
t31 = t36 * xD_base(6) + xD_base(4);
t30 = -t33 * t38 + t35 * xD_base(5);
t29 = t33 * xD_base(5) + t35 * t38;
t28 = t33 * t32 - t34 * t43;
t27 = t32 * t43 + t33 * t34;
t26 = t35 * t32 + t34 * t45;
t25 = -t32 * t45 + t35 * t34;
t24 = t36 * rSges(1,3) + (rSges(1,1) * t34 - rSges(1,2) * t32) * t37;
t23 = Icges(1,5) * t36 + (Icges(1,1) * t34 - Icges(1,4) * t32) * t37;
t22 = Icges(1,6) * t36 + (Icges(1,4) * t34 - Icges(1,2) * t32) * t37;
t21 = Icges(1,3) * t36 + (Icges(1,5) * t34 - Icges(1,6) * t32) * t37;
t20 = t28 * rSges(1,1) + t27 * rSges(1,2) + rSges(1,3) * t42;
t19 = t26 * rSges(1,1) + t25 * rSges(1,2) - rSges(1,3) * t44;
t18 = Icges(1,1) * t28 + Icges(1,4) * t27 + t35 * t41;
t17 = Icges(1,1) * t26 + Icges(1,4) * t25 - t33 * t41;
t16 = Icges(1,4) * t28 + Icges(1,2) * t27 + t35 * t40;
t15 = Icges(1,4) * t26 + Icges(1,2) * t25 - t33 * t40;
t14 = Icges(1,5) * t28 + Icges(1,6) * t27 + t35 * t39;
t13 = Icges(1,5) * t26 + Icges(1,6) * t25 - t33 * t39;
t12 = -t31 * t20 + t29 * t24 + xD_base(2);
t11 = t31 * t19 - t30 * t24 + xD_base(3);
t10 = -t29 * t19 + t30 * t20 + xD_base(1);
t1 = m(1) * (t10 ^ 2 + t11 ^ 2 + t12 ^ 2) / 0.2e1 + t31 * ((t13 * t30 + t14 * t29 + t21 * t31) * t36 + ((-t22 * t32 + t23 * t34) * t31 + (-t15 * t32 + t17 * t34) * t30 + (-t16 * t32 + t18 * t34) * t29) * t37) / 0.2e1 + t30 * ((-t21 * t44 + t25 * t22 + t26 * t23) * t31 + (-t13 * t44 + t25 * t15 + t26 * t17) * t30 + (-t14 * t44 + t25 * t16 + t26 * t18) * t29) / 0.2e1 + t29 * ((t21 * t42 + t27 * t22 + t28 * t23) * t31 + (t13 * t42 + t27 * t15 + t28 * t17) * t30 + (t14 * t42 + t27 * t16 + t28 * t18) * t29) / 0.2e1;
T  = t1;
