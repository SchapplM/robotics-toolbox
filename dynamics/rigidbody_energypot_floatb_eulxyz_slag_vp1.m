% Calculate potential energy for
% rigidbody
% Use Code from Maple symbolic Code Generation
% 
% Input:
% r_base [3x1]
%   Base position in world frame
% phi_base [3x1]
%   Base orientation in world frame. Expressed with XYZ-Euler angles
% g [3x1]
%   gravitation vector in world frame [m/s^2]
% m_mdh [1x1]
%   mass of all robot links (including the base)
% rSges [1x3]
%   center of mass of all robot links (in body frames)
%   rows: links of the robot (starting with base)
%   columns: x-, y-, z-coordinates
% 
% Output:
% U [1x1]
%   Potential energy

% Quelle: HybrDyn-Toolbox
% Datum: 2019-03-19 12:00
% Revision: 3380762d7a67e58abcb72423f3b6cbd7db453188 (2019-03-19)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function U = rigidbody_energypot_floatb_eulxyz_slag_vp1(r_base, phi_base, g, ...
  m, rSges)
%% Coder Information
%#codegen
%$cgargs {zeros(3,1),zeros(3,1),zeros(3,1),zeros(1,1),zeros(1,3)}
assert(isreal(r_base) && all(size(r_base) == [3 1]), ...
  'rigidbody_energypot_floatb_eulxyz_slag_vp1: r_base has to be [3x1] (double)');
assert(isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  'rigidbody_energypot_floatb_eulxyz_slag_vp1: phi_base has to be [3x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'rigidbody_energypot_floatb_eulxyz_slag_vp1: g has to be [3x1] (double)');
assert(isreal(m) && all(size(m) == [1 1]), ...
  'rigidbody_energypot_floatb_eulxyz_slag_vp1: m has to be [1x1] (double)'); 
assert(isreal(rSges) && all(size(rSges) == [1,3]), ...
  'rigidbody_energypot_floatb_eulxyz_slag_vp1: rSges has to be [1x3] (double)');

%% Symbolic Calculation
% From energy_potential_floatb_eulxyz_worldframe_par1_matlab.m
% OptimizationMode: 2
% StartTime: 2019-03-19 12:00:14
% EndTime: 2019-03-19 12:00:14
% DurationCPUTime: 0.07s
% Computational Cost: add. (16->16), mult. (29->25), div. (0->0), fcn. (29->6), ass. (0->10)
t2 = sin(phi_base(1));
t5 = sin(phi_base(2));
t9 = t2 * t5;
t4 = cos(phi_base(1));
t8 = t4 * t5;
t6 = cos(phi_base(2));
t7 = t6 * rSges(1,3);
t3 = cos(phi_base(3));
t1 = sin(phi_base(3));
t10 = -m(1) * (g(1) * (t5 * rSges(1,3) + r_base(1) + (rSges(1,1) * t3 - rSges(1,2) * t1) * t6) + g(2) * (r_base(2) + (t4 * t1 + t3 * t9) * rSges(1,1) + (-t1 * t9 + t4 * t3) * rSges(1,2) - t2 * t7) + g(3) * (r_base(3) + (t2 * t1 - t3 * t8) * rSges(1,1) + (t1 * t8 + t2 * t3) * rSges(1,2) + t4 * t7));
U  = t10;
