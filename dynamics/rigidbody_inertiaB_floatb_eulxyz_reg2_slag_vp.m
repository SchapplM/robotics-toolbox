% Calculate inertial parameters regressor of base floating base inertia matrix for
% rigidbody
% Use Code from Maple symbolic Code Generation
% 
% Input:
% phi_base [3x1]
%   Base orientation in world frame. Expressed with XYZ-Euler angles
% 
% Output:
% MM_reg [(6*7/2)x(1*10)]
%   inertial parameter regressor of base floating base inertia matrix
%   (only lower left triangular matrix (including diagonal) due to symmetry

% Quelle: HybrDyn-Toolbox
% Datum: 2019-08-21 13:58
% Revision: f86a32e2b5f1abc78572314ec3e2c1664fd21c45 (2019-08-20)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [MM_reg, MM_reg_nonsym] = rigidbody_inertiaB_floatb_eulxyz_reg2_slag_vp(phi_base)
%% Coder Information
%#codegen
%$cgargs {zeros(3,1)}
assert(isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  'rigidbody_inertiaB_floatb_eulxyz_reg2_slag_vp: phi_base has to be [3x1] (double)');

%% Symbolic Calculation
% From inertia_base_base_floatb_eulxyz_regressor_matlab.m
% OptimizationMode: 2
% StartTime: 2019-08-21 13:58:25
% EndTime: 2019-08-21 13:58:26
% DurationCPUTime: 0.23s
% Computational Cost: add. (295->52), mult. (917->123), div. (0->0), fcn. (1163->6), ass. (0->46)
t71 = sin(phi_base(3));
t73 = cos(phi_base(3));
t74 = cos(phi_base(1));
t85 = t74 * t73;
t72 = sin(phi_base(1));
t75 = sin(phi_base(2));
t90 = t72 * t75;
t61 = -t71 * t90 + t85;
t84 = t74 * t75;
t91 = t72 * t73;
t63 = t71 * t84 + t91;
t76 = cos(phi_base(2));
t83 = t74 * t76;
t51 = -t63 * t83 - (-t61 * t72 - t71 * t75) * t76;
t96 = -0.2e1 * t51;
t95 = -0.2e1 * t71;
t56 = t61 * t74 + t63 * t72;
t94 = t56 * t75;
t93 = t71 * t76;
t92 = t72 * t71;
t89 = t72 * t76;
t88 = t73 * t75;
t87 = t73 * t76;
t86 = t74 * t71;
t82 = t75 * t51;
t81 = t75 * t76;
t62 = t72 * t88 + t86;
t64 = -t73 * t84 + t92;
t53 = -t62 * t89 + (t64 * t74 + t88) * t76;
t80 = t53 * t93;
t57 = t62 * t74 + t64 * t72;
t79 = t57 * t93;
t67 = t72 ^ 2;
t68 = t74 ^ 2;
t69 = t75 ^ 2;
t70 = t76 ^ 2;
t60 = t69 + (t67 + t68) * t70;
t78 = t60 * t93;
t77 = t60 * t87;
t66 = t70 * t73 ^ 2;
t65 = t70 * t71 ^ 2;
t55 = t75 * t57;
t54 = t56 * t87;
t50 = t75 * t53;
t49 = t51 * t87;
t1 = [0, 0, 0, 0, 0, 0, 0, 0, 0, t66 + t65 + t69; 0, 0, 0, 0, 0, 0, 0, 0, 0, (-t61 * t71 + t62 * t73 - t90) * t76; 0, 0, 0, 0, 0, 0, 0, 0, 0, t61 ^ 2 + t62 ^ 2 + t67 * t70; 0, 0, 0, 0, 0, 0, 0, 0, 0, (-t63 * t71 + t64 * t73 + t84) * t76; 0, 0, 0, 0, 0, 0, 0, 0, 0, -t72 * t70 * t74 + t61 * t63 + t62 * t64; 0, 0, 0, 0, 0, 0, 0, 0, 0, t63 ^ 2 + t64 ^ 2 + t68 * t70; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, t61 * t75 - t70 * t92, -t62 * t75 - t70 * t91, (-t61 * t73 - t62 * t71) * t76, 0; 0, 0, 0, 0, 0, 0, t63 * t75 + t70 * t86, -t64 * t75 + t70 * t85, (-t63 * t73 - t64 * t71) * t76, 0; t66, t70 * t73 * t95, 0.2e1 * t73 * t81, t65, t81 * t95, t69, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, -t94, t55, t54 + t79, 0; 0, 0, 0, 0, 0, 0, t56 * t89, -t57 * t89, t62 * t56 - t61 * t57, 0; 0, 0, 0, 0, 0, 0, -t56 * t83, t57 * t83, t64 * t56 - t63 * t57, 0; t57 * t87, t54 - t79, t55, -t56 * t93, t94, 0, 0, 0, 0, 0; t57 ^ 2, 0.2e1 * t57 * t56, 0, t56 ^ 2, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, -t78 + t82, t50 - t77, -t49 + t80, 0; 0, 0, 0, 0, 0, 0, -t51 * t89 + t61 * t60, -t53 * t89 - t62 * t60, -t62 * t51 - t61 * t53, 0; 0, 0, 0, 0, 0, 0, t51 * t83 + t63 * t60, t53 * t83 - t64 * t60, -t64 * t51 - t63 * t53, 0; t53 * t87, -t49 - t80, t50 + t77, t51 * t93, -t78 - t82, t75 * t60, 0, 0, 0, 0; t57 * t53, -t57 * t51 + t53 * t56, t57 * t60, -t56 * t51, t56 * t60, 0, 0, 0, 0, 0; t53 ^ 2, t53 * t96, 0.2e1 * t53 * t60, t51 ^ 2, t60 * t96, t60 ^ 2, 0, 0, 0, 0;];
MM_reg  = t1;
%% Additional Output of Matrix stored non-symmetric
% The matrix with duplicate entries can be helpful if transformations into
% Cartesian moments lead to a non-symmetric inertia matrix.
if nargout == 2
  % Indices: A=vec2symmat(1:21); A(:);
  MM_reg_nonsym = MM_reg([1 2 4 7 11 16 2 3 5 8 12 17 4 5 6 9 13 18 7 8 9 10 14 19 11 12 13 14 15 20 16 17 18 19 20 21], :);
end