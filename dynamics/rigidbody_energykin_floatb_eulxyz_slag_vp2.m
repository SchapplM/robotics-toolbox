% Calculate kinetic energy for
% rigidbody
% Use Code from Maple symbolic Code Generation
% 
% Input:
% phi_base [3x1]
%   Base orientation in world frame. Expressed with XYZ-Euler angles
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
% T [1x1]
%   kinetic energy

% Quelle: HybrDyn-Toolbox
% Datum: 2019-03-19 12:00
% Revision: 3380762d7a67e58abcb72423f3b6cbd7db453188 (2019-03-19)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function T = rigidbody_energykin_floatb_eulxyz_slag_vp2(phi_base, xD_base, ...
  m, mrSges, Ifges)
%% Coder Information
%#codegen
%$cgargs {zeros(3,1),zeros(6,1),zeros(1,1),zeros(1,3),zeros(1,6)}
assert(isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  'rigidbody_energykin_floatb_eulxyz_slag_vp2: phi_base has to be [3x1] (double)');
assert(isreal(xD_base) && all(size(xD_base) == [6 1]), ...
  'rigidbody_energykin_floatb_eulxyz_slag_vp2: xD_base has to be [6x1] (double)');
assert(isreal(m) && all(size(m) == [1 1]), ...
  'rigidbody_energykin_floatb_eulxyz_slag_vp2: m has to be [1x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [1,3]), ...
  'rigidbody_energykin_floatb_eulxyz_slag_vp2: mrSges has to be [1x3] (double)');
assert(isreal(Ifges) && all(size(Ifges) == [1 6]), ...
  'rigidbody_energykin_floatb_eulxyz_slag_vp2: Ifges has to be [1x6] (double)'); 

%% Symbolic Calculation
% From energy_kinetic_floatb_eulxyz_linkframe_par2_matlab.m
% OptimizationMode: 2
% StartTime: 2019-03-19 12:00:19
% EndTime: 2019-03-19 12:00:19
% DurationCPUTime: 0.09s
% Computational Cost: add. (163->37), mult. (421->68), div. (0->0), fcn. (466->6), ass. (0->21)
t12 = sin(phi_base(3));
t13 = sin(phi_base(1));
t21 = t13 * t12;
t14 = cos(phi_base(3));
t16 = sin(phi_base(2));
t20 = t14 * t16;
t15 = cos(phi_base(1));
t19 = t15 * t16;
t17 = cos(phi_base(2));
t18 = t17 * xD_base(1);
t10 = -t14 * t19 + t21;
t9 = t12 * t19 + t13 * t14;
t8 = t15 * t12 + t13 * t20;
t7 = t15 * t14 - t16 * t21;
t6 = t16 * xD_base(1) + (-t13 * xD_base(2) + t15 * xD_base(3)) * t17;
t5 = t16 * xD_base(4) + (t16 ^ 2 + (t13 ^ 2 + t15 ^ 2) * t17 ^ 2) * xD_base(6);
t4 = t10 * xD_base(3) + t14 * t18 + t8 * xD_base(2);
t3 = -t12 * t18 + t7 * xD_base(2) + t9 * xD_base(3);
t2 = (t10 * t13 + t8 * t15) * xD_base(5) + (t14 * xD_base(4) + (t10 * t15 - t13 * t8 + t20) * xD_base(6)) * t17;
t1 = (t9 * t13 + t7 * t15) * xD_base(5) + (-t12 * xD_base(4) + (-t12 * t16 - t13 * t7 + t15 * t9) * xD_base(6)) * t17;
t11 = m(1) * (t3 ^ 2 + t4 ^ 2 + t6 ^ 2) / 0.2e1 + (t3 * mrSges(1,1) - t4 * mrSges(1,2) + Ifges(1,3) * t5 / 0.2e1) * t5 + (t6 * mrSges(1,2) - t3 * mrSges(1,3) + Ifges(1,5) * t5 + Ifges(1,1) * t2 / 0.2e1) * t2 + (-t6 * mrSges(1,1) + t4 * mrSges(1,3) + Ifges(1,4) * t2 + Ifges(1,6) * t5 + Ifges(1,2) * t1 / 0.2e1) * t1;
T  = t11;
