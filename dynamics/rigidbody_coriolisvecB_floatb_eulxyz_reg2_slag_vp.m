% Calculate inertial parameter regressor of vector of centrifugal and coriolis load on the base for
% rigidbody
% Use Code from Maple symbolic Code Generation
% 
% Input:
% phi_base [3x1]
%   Base orientation in world frame. Expressed with XYZ-Euler angles
% xD_base [6x1]
%   time derivative of r_base and phi_base
% 
% Output:
% tauc_reg [6x(1*10)]
%   inertial parameter regressor for generalized base forces and torques 
%   required to compensate coriolis and centrifugal load

% Quelle: HybrDyn-Toolbox
% Datum: 2019-08-21 13:58
% Revision: f86a32e2b5f1abc78572314ec3e2c1664fd21c45 (2019-08-20)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function tauc_reg = rigidbody_coriolisvecB_floatb_eulxyz_reg2_slag_vp(phi_base, xD_base)
%% Coder Information
%#codegen
%$cgargs {zeros(3,1),zeros(6,1)}
assert(isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  'rigidbody_coriolisvecB_floatb_eulxyz_reg2_slag_vp: phi_base has to be [3x1] (double)');
assert(isreal(xD_base) && all(size(xD_base) == [6 1]), ...
  'rigidbody_coriolisvecB_floatb_eulxyz_reg2_slag_vp: xD_base has to be [6x1] (double)');

%% Symbolic Calculation
% From coriolisvec_base_floatb_eulxyz_regressor_matlab.m
% OptimizationMode: 2
% StartTime: 2019-08-21 13:58:36
% EndTime: 2019-08-21 13:58:39
% DurationCPUTime: 2.35s
% Computational Cost: add. (3054->209), mult. (8902->323), div. (0->0), fcn. (9180->6), ass. (0->119)
t366 = sin(phi_base(1));
t367 = cos(phi_base(1));
t470 = t366 ^ 2 + t367 ^ 2;
t464 = -0.1e1 + t470;
t368 = cos(phi_base(2));
t479 = t464 * t368;
t447 = sin(phi_base(3));
t350 = t470 * t447;
t448 = cos(phi_base(3));
t425 = t448 * xD_base(4);
t413 = t368 * t425;
t449 = sin(phi_base(2));
t417 = t448 * t449;
t459 = t479 * t417;
t331 = t350 * xD_base(5) - t459 * xD_base(6) + t413;
t414 = t447 * t449;
t378 = t479 * t414;
t477 = t464 * xD_base(6);
t433 = t368 * t447;
t384 = -xD_base(6) * t417 - xD_base(5) * t433;
t379 = t425 - t384;
t404 = xD_base(4) * t414;
t422 = t447 * xD_base(6);
t389 = -t404 - t422;
t341 = t389 * t366 + t379 * t367;
t342 = -t379 * t366 + t389 * t367;
t443 = t449 ^ 2;
t415 = t443 * t447;
t410 = xD_base(5) * t415;
t388 = -t410 - t413;
t323 = t410 + t384 * t368 + (t341 * t368 + t388 * t367) * t367 + (-t342 * t368 + t388 * t366) * t366;
t474 = t323 + t331;
t325 = (t367 * t404 + t342) * t367 + (t366 * t404 + t341) * t366;
t458 = t368 ^ 2;
t473 = t325 - t404 - (t447 * t458 - t415) * t477;
t405 = xD_base(4) * t417;
t416 = t443 * t448;
t336 = t405 - (-t448 * t458 + t416) * t477;
t424 = t448 * xD_base(6);
t393 = t470 * t424;
t472 = t336 + t393;
t402 = t470 * t448;
t471 = xD_base(6) * t402 + t336;
t423 = t447 * xD_base(4);
t329 = t368 * t423 - t378 * xD_base(6) - t402 * xD_base(5);
t403 = xD_base(6) * t414;
t427 = xD_base(5) * t448;
t387 = t368 * t427 - t403;
t380 = -t423 + t387;
t390 = t405 + t424;
t370 = t380 * t366 + t390 * t367;
t371 = t390 * t366 - t380 * t367;
t437 = t366 * t447;
t395 = t367 * t417 - t437;
t435 = t367 * t447;
t397 = t366 * t417 + t435;
t411 = xD_base(5) * t416;
t454 = t367 * t368;
t455 = t366 * t368;
t428 = xD_base(5) * t449;
t442 = t368 * xD_base(4);
t466 = t366 * t428 - t367 * t442;
t467 = -t366 * t442 - t367 * t428;
t369 = t368 * t403 + t370 * t455 - t371 * t454 + t467 * t395 - t466 * t397 - t458 * t427 + t411;
t469 = t369 - t329;
t465 = -t366 * xD_base(2) + t367 * xD_base(3);
t463 = t329 + t464 * (-t387 * t368 + t411);
t412 = t368 * t424;
t320 = t323 * xD_base(6) - xD_base(4) * t412 + (t404 + t325) * xD_base(5);
t409 = xD_base(5) * t417;
t385 = t368 * t422 + t409;
t321 = t369 * xD_base(6) + t385 * xD_base(4) - t393 * xD_base(5);
t462 = -t320 * t417 + t321 * t414 + t380 * t329;
t398 = t464 * t449;
t444 = t368 * xD_base(5);
t355 = t398 * t444;
t452 = 0.2e1 * xD_base(6);
t349 = -t355 * t452 + xD_base(5) * t442;
t361 = t470 * t458 + t443;
t358 = t361 * xD_base(6) + t449 * xD_base(4);
t461 = -t368 * t321 - t331 * t428 + t349 * t417 + t380 * t358;
t434 = t368 * t448;
t460 = t349 * t434 - t385 * t358;
t346 = t395 * xD_base(3) - t397 * xD_base(2) - xD_base(1) * t434;
t457 = 0.2e1 * t355;
t456 = -0.2e1 * t355;
t453 = -t321 * t449 + t331 * t444;
t446 = t329 * xD_base(5);
t441 = t320 * t449;
t438 = t366 * t448;
t436 = t367 * t448;
t432 = t448 * t321;
t431 = t447 * t320;
t430 = t447 * t321;
t429 = t447 * t349;
t418 = -t366 * xD_base(3) - t367 * xD_base(2);
t408 = xD_base(5) * t414;
t401 = -t358 * t424 - t429;
t399 = t465 * t449;
t396 = -t366 * t414 + t436;
t394 = t367 * t414 + t438;
t386 = t412 - t408;
t377 = -t331 * t442 - t390 * t358 - t429;
t376 = -t390 * t329 + t431 + t432;
t362 = t418 * t368;
t360 = t368 * xD_base(1) - t399;
t359 = t465 * t368 + t449 * xD_base(1);
t357 = -xD_base(1) * t417 + (-xD_base(3) * t436 + xD_base(2) * t438) * t368;
t356 = xD_base(1) * t414 + (xD_base(3) * t435 - xD_base(2) * t437) * t368;
t354 = t358 * t408;
t353 = (-t398 * t452 + xD_base(4)) * t368;
t352 = t395 * xD_base(2) + t397 * xD_base(3);
t351 = -t394 * xD_base(2) + t396 * xD_base(3);
t344 = t394 * xD_base(3) + t396 * xD_base(2) - xD_base(1) * t433;
t343 = -t399 * xD_base(5) + (t418 * xD_base(4) + xD_base(5) * xD_base(1)) * t368;
t327 = -t385 * xD_base(1) + (-t380 * xD_base(3) + t390 * xD_base(2)) * t367 + (t380 * xD_base(2) + t390 * xD_base(3)) * t366;
t326 = t341 * xD_base(3) + t342 * xD_base(2) - t386 * xD_base(1);
t318 = t320 * t434;
t1 = [0, 0, 0, 0, 0, 0, -t441 + t354 + (t401 + t446) * t368, t453 - t460, t318 + (t329 * t417 - t331 * t414) * xD_base(5) + (-t430 + (t447 * t329 + t448 * t331) * xD_base(6)) * t368, t343 * t449 + (t344 * t414 + t346 * t417) * xD_base(5) + (t327 * t448 - t447 * t326 + t359 * xD_base(5) + (-t448 * t344 + t346 * t447) * xD_base(6)) * t368; 0, 0, 0, 0, 0, 0, t320 * t455 + t466 * t329 + t342 * t358 + t396 * t349, -t461 * t366 + t377 * t367, -t342 * t331 - t462 * t366 + t376 * t367, t326 * t396 + t327 * t397 + t344 * t342 - t343 * t455 - t346 * t370 + t466 * t359; 0, 0, 0, 0, 0, 0, -t320 * t454 + t467 * t329 + t341 * t358 + t394 * t349, t377 * t366 + t461 * t367, -t341 * t331 + t376 * t366 + t462 * t367, t326 * t394 - t327 * t395 + t344 * t341 + t343 * t454 - t346 * t371 + t467 * t359; -t385 * t331 - t368 * t432, t385 * t329 - t386 * t331 + t368 * t430 + t318, t453 + t460, t386 * t329 - t368 * t431, t441 + t354 + (t401 - t446) * t368, t349 * t449 + t358 * t444, t326 * t449 - t362 * t329 - t351 * t358 + (t447 * t343 + t344 * xD_base(5)) * t368 + t386 * t359, -t327 * t449 - t362 * t331 + t352 * t358 + (t343 * t448 + t346 * xD_base(5)) * t368 - t385 * t359, t352 * t329 + t351 * t331 - t346 * t408 + t344 * t409 + (-t326 * t448 - t327 * t447 + (t447 * t344 + t346 * t448) * xD_base(6)) * t368, -t344 * t351 + t346 * t352 - t359 * t362; -t321 * t350 + t471 * t331, t350 * t320 - t321 * t402 - t472 * t329 + t473 * t331, -t331 * t353 + t350 * t349 + t471 * t358, t320 * t402 - t473 * t329, t329 * t353 + t402 * t349 + t473 * t358, -t358 * t353, -t360 * t329 - t343 * t402 - t344 * t353 - t356 * t358 - t473 * t359, -t360 * t331 + t343 * t350 - t346 * t353 + t357 * t358 + t472 * t359, -t326 * t350 + t327 * t402 + t357 * t329 + t356 * t331 - t472 * t344 - t346 * t473, -t344 * t356 + t346 * t357 - t359 * t360; t321 * t459 + t463 * t331, -t320 * t459 - t321 * t378 + t469 * t329 + t474 * t331, -t321 * t361 + t331 * t456 - t349 * t459 + t463 * t358, t378 * t320 - t474 * t329, t320 * t361 + t329 * t457 + t349 * t378 + t474 * t358, t349 * t361 + t358 * t456, t326 * t361 - t343 * t378 + t344 * t456 - t346 * t358 - t474 * t359, -t327 * t361 - t343 * t459 + t344 * t358 - t346 * t457 - t469 * t359, t326 * t459 + t327 * t378 + t344 * t369 + (t331 - t474) * t346, 0;];
tauc_reg  = t1;
