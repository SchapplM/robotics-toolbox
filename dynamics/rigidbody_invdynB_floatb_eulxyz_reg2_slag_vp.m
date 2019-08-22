% Calculate inertial parameter regressor for vector of inverse dynamics generalized base forces for
% rigidbody
% Use Code from Maple symbolic Code Generation
% 
% Input:
% phi_base [3x1]
%   Base orientation in world frame. Expressed with XYZ-Euler angles
% xD_base [6x1]
%   time derivative of r_base and phi_base
% xDD_base [6x1]
%   second time derivative of r_base and phi_base
% g [3x1]
%   gravitation vector in world frame [m/s^2]
% 
% Output:
% tauB_reg [6x(10*1)]
%   inertial parameter regressor for generalized base forces of inverse dynamics
%   (contains inertial, gravitational coriolis and centrifugal forces)
%   base moment as Euler angle moment

% Quelle: HybrDyn-Toolbox
% Datum: 2019-08-21 13:58
% Revision: f86a32e2b5f1abc78572314ec3e2c1664fd21c45 (2019-08-20)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function tauB_reg = rigidbody_invdynB_floatb_eulxyz_reg2_slag_vp(phi_base, xD_base, xDD_base, g)
%% Coder Information
%#codegen
%$cgargs {zeros(3,1),zeros(6,1),zeros(6,1),zeros(3,1)}
assert(isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  'rigidbody_invdynB_floatb_eulxyz_reg2_slag_vp: phi_base has to be [3x1] (double)');
assert(isreal(xD_base) && all(size(xD_base) == [6 1]), ...
  'rigidbody_invdynB_floatb_eulxyz_reg2_slag_vp: xD_base has to be [6x1] (double)');
assert(isreal(xDD_base) && all(size(xDD_base) == [6 1]), ...
  'rigidbody_invdynB_floatb_eulxyz_reg2_slag_vp: xDD_base has to be [6x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'rigidbody_invdynB_floatb_eulxyz_reg2_slag_vp: g has to be [3x1] (double)');

%% Symbolic Calculation
% From invdyn_base_floatb_eulxyz_regressor_matlab.m
% OptimizationMode: 2
% StartTime: 2019-08-21 13:58:37
% EndTime: 2019-08-21 13:58:40
% DurationCPUTime: 2.42s
% Computational Cost: add. (3529->269), mult. (9966->387), div. (0->0), fcn. (10438->6), ass. (0->128)
t388 = sin(phi_base(1));
t386 = t388 ^ 2;
t389 = cos(phi_base(1));
t482 = cos(phi_base(3));
t483 = sin(phi_base(2));
t447 = t482 * t483;
t383 = t389 * t447;
t478 = t483 ^ 2;
t446 = t478 * t482;
t472 = t389 * t483;
t417 = t383 * t472 - t446;
t429 = xD_base(4) * t447;
t390 = cos(phi_base(2));
t492 = t390 ^ 2;
t387 = t389 ^ 2;
t502 = t386 + t387;
t497 = -0.1e1 + t502;
t355 = t429 - (-t497 * t492 * t482 + t386 * t446 + t417) * xD_base(6);
t454 = t482 * xD_base(6);
t411 = t429 + t454;
t403 = t411 * t389;
t394 = t386 * t454 - (t383 * xD_base(4) - t403) * t389;
t503 = t355 + t394;
t481 = sin(phi_base(3));
t444 = t481 * t483;
t506 = t387 - 0.1e1;
t397 = (t386 + t506) * t390 * t444;
t465 = t389 * t481;
t416 = t388 * t447 + t465;
t467 = t388 * t481;
t450 = t383 - t467;
t398 = -t450 * t388 + t416 * t389;
t476 = t390 * xD_base(4);
t430 = xD_base(6) * t447;
t457 = t481 * xD_base(5);
t405 = -t390 * t457 - t430;
t455 = t482 * xD_base(4);
t399 = t455 - t405;
t428 = xD_base(4) * t444;
t452 = t481 * xD_base(6);
t410 = -t428 - t452;
t362 = t388 * t410 + t389 * t399;
t363 = -t388 * t399 + t389 * t410;
t445 = t478 * t481;
t435 = xD_base(5) * t445;
t442 = t390 * t455;
t409 = -t435 - t442;
t342 = t435 + t405 * t390 + (t362 * t390 + t389 * t409) * t389 + (-t363 * t390 + t388 * t409) * t388;
t424 = -t383 * t389 + t447;
t438 = t390 * t447;
t401 = -t386 * t438 + t390 * t424;
t505 = t398 * xD_base(5) + t401 * xD_base(6) + t342 + t442;
t346 = (t389 * t428 + t363) * t389 + (t388 * t428 + t362) * t388;
t504 = t346 - t428 - (t481 * t492 - t445) * xD_base(6) * t497;
t423 = t502 * t482;
t453 = t481 * xD_base(4);
t348 = t390 * t453 - t397 * xD_base(6) - t423 * xD_base(5);
t427 = xD_base(6) * t444;
t458 = t482 * xD_base(5);
t408 = t390 * t458 - t427;
t400 = -t453 + t408;
t392 = t388 * t400 + t403;
t393 = t388 * t411 - t389 * t400;
t436 = xD_base(5) * t446;
t459 = xD_base(5) * t483;
t448 = t389 * t459;
t449 = t388 * t459;
t486 = t389 * t390;
t487 = t388 * t390;
t391 = t390 * t427 + t392 * t487 - t393 * t486 + t398 * t476 - t416 * t449 - t450 * t448 - t492 * t458 + t436;
t501 = t391 - t348;
t434 = xD_base(5) * t447;
t406 = t390 * t452 + t434;
t500 = -t388 * t476 - t448;
t499 = -t389 * t476 + t449;
t498 = -g(2) * t388 + g(3) * t389;
t496 = xD_base(6) * t423 + t355;
t464 = t390 * t482;
t340 = t391 * xD_base(6) - t394 * xD_base(5) - t398 * xDD_base(5) - t401 * xDD_base(6) + t406 * xD_base(4) - xDD_base(4) * t464;
t495 = t340 * t444 + t348 * t400;
t350 = t387 * t457 + (t424 * xD_base(6) + t455) * t390 + (-t390 * t430 + t457) * t386;
t380 = t502 * t492 + t478;
t377 = t380 * xD_base(6) + t483 * xD_base(4);
t494 = -t390 * t340 - t350 * t459 + t377 * t400;
t419 = t497 * t483;
t479 = t390 * xD_base(5);
t374 = t419 * t479;
t484 = 0.2e1 * xD_base(6);
t357 = -t374 * t484 + t380 * xDD_base(6) + xD_base(5) * t476 + t483 * xDD_base(4);
t493 = t357 * t464 - t406 * t377;
t365 = t416 * xD_base(2) - t450 * xD_base(3) + xD_base(1) * t464;
t491 = 0.2e1 * t374;
t490 = -0.2e1 * t374;
t485 = -t340 * t483 + t350 * t479;
t480 = t348 * xD_base(5);
t441 = t390 * t454;
t463 = t390 * t481;
t339 = t342 * xD_base(6) + t397 * xDD_base(6) + t423 * xDD_base(5) - xD_base(4) * t441 - xDD_base(4) * t463 + (t346 + t428) * xD_base(5);
t474 = t339 * t483;
t473 = t388 * t483;
t468 = t388 * t482;
t466 = t389 * t482;
t462 = t482 * t340;
t461 = t481 * t339;
t460 = t481 * t357;
t433 = xD_base(5) * t444;
t422 = -t377 * t454 - t460;
t420 = -xD_base(3) * t472 + xD_base(2) * t473;
t415 = t388 * t444 - t466;
t414 = t389 * t444 + t468;
t407 = t441 - t433;
t396 = -t350 * t476 - t411 * t377 - t460;
t395 = -t411 * t348 + t461 + t462;
t381 = (-t388 * xD_base(3) - t389 * xD_base(2)) * t390;
t379 = t390 * xD_base(1) + t420;
t378 = t483 * xD_base(1) + (-t388 * xD_base(2) + t389 * xD_base(3)) * t390;
t376 = -xD_base(1) * t447 + (-xD_base(3) * t466 + xD_base(2) * t468) * t390;
t375 = xD_base(1) * t444 + (xD_base(3) * t465 - xD_base(2) * t467) * t390;
t373 = t377 * t433;
t372 = (-t419 * t484 + xD_base(4)) * t390;
t371 = t416 * xD_base(3) + t450 * xD_base(2);
t370 = -t414 * xD_base(2) - t415 * xD_base(3);
t364 = t414 * xD_base(3) - t415 * xD_base(2) - xD_base(1) * t463;
t358 = t483 * xDD_base(1) + t420 * xD_base(5) + (xD_base(5) * xD_base(1) + (-xD_base(4) * xD_base(2) + xDD_base(3)) * t389 + (-xD_base(4) * xD_base(3) - xDD_base(2)) * t388) * t390;
t344 = t392 * xD_base(2) + t393 * xD_base(3) - t406 * xD_base(1) + t416 * xDD_base(2) - t450 * xDD_base(3) + xDD_base(1) * t464;
t343 = xD_base(1) * t433 + t363 * xD_base(2) - t415 * xDD_base(2) + t362 * xD_base(3) + t414 * xDD_base(3) + (-xD_base(1) * t454 - t481 * xDD_base(1)) * t390;
t337 = t339 * t464;
t1 = [0, 0, 0, 0, 0, 0, -t474 + t373 + (t422 + t480) * t390, t485 - t493, t337 + (t348 * t447 - t350 * t444) * xD_base(5) + (-t340 * t481 + (t481 * t348 + t482 * t350) * xD_base(6)) * t390, t358 * t483 - g(1) + (t364 * t444 - t365 * t447) * xD_base(5) + (t482 * t344 - t481 * t343 + t378 * xD_base(5) + (-t482 * t364 - t481 * t365) * xD_base(6)) * t390; 0, 0, 0, 0, 0, 0, t339 * t487 + t499 * t348 - t415 * t357 + t363 * t377, t396 * t389 + (-t357 * t447 - t494) * t388, -t363 * t350 + t395 * t389 + (t339 * t447 - t495) * t388, -t343 * t415 + t344 * t416 - t358 * t487 + t364 * t363 + t365 * t392 + t499 * t378 - g(2); 0, 0, 0, 0, 0, 0, -t339 * t486 + t500 * t348 + t414 * t357 + t362 * t377, t383 * t357 + t396 * t388 + t494 * t389, -t383 * t339 - t362 * t350 + t395 * t388 + t495 * t389, t343 * t414 - t344 * t450 + t358 * t486 + t364 * t362 + t365 * t393 + t500 * t378 - g(3); -t406 * t350 - t390 * t462, t340 * t463 + t406 * t348 - t407 * t350 + t337, t485 + t493, t407 * t348 - t390 * t461, t474 + t373 + (t422 - t480) * t390, t357 * t483 + t377 * t479, t343 * t483 - t370 * t377 - t381 * t348 - g(2) * t450 - g(3) * t416 + (t358 * t481 + t364 * xD_base(5)) * t390 + t407 * t378, -t344 * t483 + t371 * t377 - t381 * t350 + g(2) * t414 + g(3) * t415 + (t358 * t482 - t365 * xD_base(5)) * t390 - t406 * t378, t371 * t348 + t370 * t350 + t365 * t433 + t364 * t434 + (-t343 * t482 - t344 * t481 + g(2) * t389 + g(3) * t388 + (t481 * t364 - t482 * t365) * xD_base(6)) * t390, -t364 * t370 - t365 * t371 - t378 * t381; -t340 * t398 + t496 * t350, t339 * t398 - t340 * t423 - t503 * t348 + t504 * t350, -t350 * t372 + t357 * t398 + t496 * t377, t339 * t423 - t504 * t348, t348 * t372 + t357 * t423 + t504 * t377, -t377 * t372, g(1) * t447 - t379 * t348 - t358 * t423 - t364 * t372 - t375 * t377 - t504 * t378 + t498 * t464, -g(1) * t444 - t379 * t350 + t358 * t398 + t365 * t372 + t376 * t377 + t503 * t378 - t498 * t463, -g(1) * t390 - g(2) * t473 + g(3) * t472 - t343 * t398 + t344 * t423 + t376 * t348 + t375 * t350 - t364 * t503 + t504 * t365, -t364 * t375 - t365 * t376 - t378 * t379; (t417 * xD_base(5) + t348) * t350 + (-t506 * t350 * t408 - t340 * t424) * t390 + (t340 * t438 + t350 * (-t390 * t408 + t436)) * t386, t339 * t401 - t340 * t397 + t501 * t348 + t505 * t350, -t340 * t380 + t350 * t490 + t357 * t401 - t501 * t377, t397 * t339 - t505 * t348, t339 * t380 + t348 * t491 + t357 * t397 + t505 * t377, t357 * t380 + t377 * t490, g(1) * t463 + g(2) * t415 - g(3) * t414 + t343 * t380 - t358 * t397 + t364 * t490 + t365 * t377 - t505 * t378, g(1) * t464 + g(2) * t416 - g(3) * t450 - t344 * t380 + t358 * t401 + t364 * t377 + t365 * t491 - t501 * t378, -t343 * t401 + t344 * t397 + t364 * t391 + (-t350 + t505) * t365, 0;];
tauB_reg  = t1;
