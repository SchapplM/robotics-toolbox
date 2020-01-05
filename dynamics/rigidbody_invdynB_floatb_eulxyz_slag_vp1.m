% Calculate vector of inverse dynamics generalized base forces for
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
% tauB [6x1]
%   generalized base forces of inverse dynamics (contains inertial, gravitational coriolis and centrifugal forces)
%   base moment as Euler angle moment

% Quelle: HybrDyn-Toolbox
% Datum: 2019-08-21 13:58
% Revision: f86a32e2b5f1abc78572314ec3e2c1664fd21c45 (2019-08-20)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function tauB = rigidbody_invdynB_floatb_eulxyz_slag_vp1(phi_base, xD_base, xDD_base, g, ...
  m, rSges, Icges)
%% Coder Information
%#codegen
%$cgargs {zeros(3,1),zeros(6,1),zeros(6,1),zeros(3,1),zeros(1,1),zeros(1,3),zeros(1,6)}
assert(isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  'rigidbody_invdynB_floatb_eulxyz_slag_vp1: phi_base has to be [3x1] (double)');
assert(isreal(xD_base) && all(size(xD_base) == [6 1]), ...
  'rigidbody_invdynB_floatb_eulxyz_slag_vp1: xD_base has to be [6x1] (double)');
assert(isreal(xDD_base) && all(size(xDD_base) == [6 1]), ...
  'rigidbody_invdynB_floatb_eulxyz_slag_vp1: xDD_base has to be [6x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'rigidbody_invdynB_floatb_eulxyz_slag_vp1: g has to be [3x1] (double)');
assert(isreal(m) && all(size(m) == [1 1]), ...
  'rigidbody_invdynB_floatb_eulxyz_slag_vp1: m has to be [1x1] (double)'); 
assert(isreal(rSges) && all(size(rSges) == [1,3]), ...
  'rigidbody_invdynB_floatb_eulxyz_slag_vp1: rSges has to be [1x3] (double)');
assert(isreal(Icges) && all(size(Icges) == [1 6]), ...
  'rigidbody_invdynB_floatb_eulxyz_slag_vp1: Icges has to be [1x6] (double)'); 

%% Symbolic Calculation
% From invdyn_base_floatb_eulxyz_par1_matlab.m
% OptimizationMode: 2
% StartTime: 2019-08-21 13:58:18
% EndTime: 2019-08-21 13:58:23
% DurationCPUTime: 4.30s
% Computational Cost: add. (5259->384), mult. (13908->557), div. (0->0), fcn. (13838->6), ass. (0->172)
t346 = sin(phi_base(3));
t348 = cos(phi_base(3));
t349 = cos(phi_base(1));
t424 = t348 * t349;
t347 = sin(phi_base(1));
t350 = sin(phi_base(2));
t426 = t347 * t350;
t333 = -t346 * t426 + t424;
t334 = t346 * t349 + t348 * t426;
t351 = cos(phi_base(2));
t425 = t347 * t351;
t288 = Icges(1,5) * t334 + Icges(1,6) * t333 - Icges(1,3) * t425;
t423 = t349 * t350;
t427 = t347 * t348;
t335 = t346 * t423 + t427;
t336 = -t347 * t346 + t348 * t423;
t422 = t349 * t351;
t290 = -Icges(1,5) * t336 + Icges(1,6) * t335 + Icges(1,3) * t422;
t332 = Icges(1,4) * t336;
t293 = Icges(1,2) * t335 + Icges(1,6) * t422 - t332;
t331 = Icges(1,4) * t335;
t295 = Icges(1,1) * t336 - Icges(1,5) * t422 - t331;
t378 = -t335 * t293 - t336 * t295;
t440 = Icges(1,4) * t334;
t291 = Icges(1,2) * t333 - Icges(1,6) * t425 + t440;
t330 = Icges(1,4) * t333;
t294 = Icges(1,1) * t334 - Icges(1,5) * t425 + t330;
t419 = t333 * t291 + t334 * t294;
t473 = t378 + t419 + (-t288 * t347 - t290 * t349) * t351;
t257 = t288 * t422 + t335 * t291 - t336 * t294;
t438 = Icges(1,4) * t348;
t386 = -Icges(1,2) * t346 + t438;
t325 = Icges(1,6) * t350 + t351 * t386;
t439 = Icges(1,4) * t346;
t388 = Icges(1,1) * t348 - t439;
t327 = Icges(1,5) * t350 + t351 * t388;
t384 = Icges(1,5) * t348 - Icges(1,6) * t346;
t323 = Icges(1,3) * t350 + t351 * t384;
t421 = t351 * t323;
t272 = t335 * t325 - t336 * t327 + t349 * t421;
t406 = t351 * xD_base(6);
t343 = -t347 * t406 + t349 * xD_base(5);
t408 = t350 * xD_base(6);
t344 = xD_base(4) + t408;
t472 = -t257 * t343 - t272 * t344;
t302 = -t336 * rSges(1,1) + t335 * rSges(1,2) + rSges(1,3) * t422;
t390 = rSges(1,1) * t348 - rSges(1,2) * t346;
t329 = t350 * rSges(1,3) + t351 * t390;
t342 = t347 * xD_base(5) + t349 * t406;
t470 = t302 * t344 - t329 * t342;
t377 = t293 * t346 + t295 * t348;
t260 = t350 * t290 - t351 * t377;
t256 = -t290 * t425 + t333 * t293 - t334 * t295;
t258 = t290 * t422 - t378;
t359 = -t406 * xD_base(4) + xDD_base(5);
t456 = (-xD_base(4) + t408) * xD_base(5) - t351 * xDD_base(6);
t305 = t359 * t347 - t456 * t349;
t450 = t305 / 0.2e1;
t469 = t258 * t450;
t304 = t456 * t347 + t359 * t349;
t451 = t304 / 0.2e1;
t337 = t350 * xDD_base(6) + t406 * xD_base(5) + xDD_base(4);
t449 = t337 / 0.2e1;
t447 = t342 / 0.2e1;
t445 = t343 / 0.2e1;
t443 = t344 / 0.2e1;
t363 = t384 * t350;
t374 = t325 * t346 - t327 * t348;
t379 = t291 * t346 - t294 * t348;
t352 = t342 * (-t323 * t349 + t377) + t343 * (t323 * t347 + t379) + t344 * (Icges(1,3) * t351 - t363 + t374);
t463 = t352 * t351;
t362 = t344 * t349;
t372 = t350 * xD_base(4) + xD_base(6);
t411 = t351 * xD_base(5);
t457 = t347 * t372 - t349 * t411;
t283 = -t457 * t346 + t348 * t362;
t284 = t346 * t362 + t457 * t348;
t407 = t351 * xD_base(4);
t412 = t350 * xD_base(5);
t458 = t347 * t407 + t349 * t412;
t267 = t284 * rSges(1,1) + t283 * rSges(1,2) - rSges(1,3) * t458;
t300 = t334 * rSges(1,1) + t333 * rSges(1,2) - rSges(1,3) * t425;
t462 = -t300 * xD_base(4) + t267;
t285 = -t344 * t427 + (-t347 * t411 - t349 * t372) * t346;
t286 = t372 * t424 + (-t344 * t346 + t348 * t411) * t347;
t403 = t347 * t412;
t357 = -t349 * t407 + t403;
t268 = t286 * rSges(1,1) + t285 * rSges(1,2) + rSges(1,3) * t357;
t461 = -t302 * xD_base(4) - t268;
t271 = t333 * t325 + t334 * t327 - t347 * t421;
t460 = t256 * t342 + t271 * t344;
t262 = Icges(1,5) * t286 + Icges(1,6) * t285 + Icges(1,3) * t357;
t264 = Icges(1,4) * t286 + Icges(1,2) * t285 + Icges(1,6) * t357;
t266 = Icges(1,1) * t286 + Icges(1,4) * t285 + Icges(1,5) * t357;
t239 = t262 * t422 + t335 * t264 - t336 * t266 + t283 * t291 + t284 * t294 - t288 * t458;
t261 = Icges(1,5) * t284 + Icges(1,6) * t283 - Icges(1,3) * t458;
t263 = Icges(1,4) * t284 + Icges(1,2) * t283 - Icges(1,6) * t458;
t265 = Icges(1,1) * t284 + Icges(1,4) * t283 - Icges(1,5) * t458;
t240 = t261 * t422 + t335 * t263 - t336 * t265 + t283 * t293 - t284 * t295 - t290 * t458;
t383 = -Icges(1,5) * t346 - Icges(1,6) * t348;
t297 = -xD_base(5) * t363 + (Icges(1,3) * xD_base(5) + t383 * xD_base(6)) * t351;
t364 = t386 * t350;
t385 = -Icges(1,2) * t348 - t439;
t298 = -xD_base(5) * t364 + (Icges(1,6) * xD_base(5) + t385 * xD_base(6)) * t351;
t365 = t388 * t350;
t387 = -Icges(1,1) * t346 - t438;
t299 = -xD_base(5) * t365 + (Icges(1,5) * xD_base(5) + t387 * xD_base(6)) * t351;
t249 = t283 * t325 + t284 * t327 + t297 * t422 + t335 * t298 - t336 * t299 - t323 * t458;
t255 = -t288 * t425 + t419;
t245 = t343 * t255 + t460;
t453 = -t245 / 0.2e1;
t459 = t239 * t445 + t240 * t447 + t249 * t443 + t257 * t451 + t272 * t449 + xD_base(4) * t453 + t469;
t354 = t342 * (Icges(1,2) * t336 - t295 + t331) + t343 * (-Icges(1,2) * t334 + t294 + t330) + t344 * (t385 * t351 + t327);
t455 = t342 * (-Icges(1,1) * t335 + t293 - t332) + t343 * (-Icges(1,1) * t333 + t291 + t440) + t344 * (-t387 * t351 + t325);
t448 = -t342 / 0.2e1;
t446 = -t343 / 0.2e1;
t444 = -t344 / 0.2e1;
t243 = (t379 * xD_base(5) + t262) * t350 + (t288 * xD_base(5) + (-t291 * xD_base(6) + t266) * t348 + (-t294 * xD_base(6) - t264) * t346) * t351;
t434 = t243 * t343;
t244 = (t377 * xD_base(5) + t261) * t350 + (t290 * xD_base(5) + (-t293 * xD_base(6) + t265) * t348 + (t295 * xD_base(6) - t263) * t346) * t351;
t433 = t244 * t342;
t259 = t350 * t288 - t351 * t379;
t431 = t259 * t304;
t430 = t260 * t305;
t254 = (t374 * xD_base(5) + t297) * t350 + (t323 * xD_base(5) + (-t325 * xD_base(6) + t299) * t348 + (-t327 * xD_base(6) - t298) * t346) * t351;
t276 = t350 * t323 - t351 * t374;
t420 = t254 * t344 + t276 * t337;
t398 = t258 * xD_base(4) + t239;
t397 = -t257 * xD_base(4) + t240;
t241 = -t262 * t425 + t333 * t264 + t334 * t266 + t285 * t291 + t286 * t294 + t288 * t357;
t396 = t256 * xD_base(4) + t241;
t242 = -t261 * t425 + t333 * t263 + t334 * t265 + t285 * t293 - t286 * t295 + t290 * t357;
t395 = -t255 * xD_base(4) + t242;
t394 = t260 * xD_base(4) + t243;
t393 = -t259 * xD_base(4) + t244;
t366 = t390 * t350;
t389 = -rSges(1,1) * t346 - rSges(1,2) * t348;
t303 = -xD_base(5) * t366 + (rSges(1,3) * xD_base(5) + t389 * xD_base(6)) * t351;
t251 = t268 * t344 + t300 * t337 - t303 * t343 - t304 * t329 + xDD_base(3);
t275 = xD_base(2) - t470;
t392 = t275 * xD_base(4) - t251;
t252 = -t267 * t344 - t302 * t337 + t303 * t342 + t305 * t329 + xDD_base(2);
t375 = -t300 * t344 + t329 * t343;
t274 = -t375 + xD_base(3);
t391 = t274 * xD_base(4) + t252;
t382 = t255 * t347 - t256 * t349;
t381 = t257 * t347 - t258 * t349;
t380 = t259 * t347 - t260 * t349;
t376 = t300 * t349 + t302 * t347;
t361 = t288 * t343 + t290 * t342 + t323 * t344;
t360 = (Icges(1,5) * t333 - Icges(1,6) * t334) * t343 + (Icges(1,5) * t335 + Icges(1,6) * t336) * t342 + t383 * t351 * t344;
t356 = t251 * t300 - t252 * t302 - t275 * t267 + t274 * t268;
t269 = -t300 * t342 + t302 * t343 + xD_base(1);
t353 = t269 * t376 + (-t274 * t347 - t275 * t349) * t329;
t341 = t389 * t351;
t328 = rSges(1,3) * t351 - t366;
t326 = Icges(1,5) * t351 - t365;
t324 = Icges(1,6) * t351 - t364;
t321 = t329 * t349;
t320 = t329 * t347;
t319 = t327 * t349;
t318 = t327 * t347;
t317 = t325 * t349;
t316 = t325 * t347;
t313 = rSges(1,1) * t335 + rSges(1,2) * t336;
t312 = rSges(1,1) * t333 - rSges(1,2) * t334;
t250 = t285 * t325 + t286 * t327 - t297 * t425 + t333 * t298 + t334 * t299 + t323 * t357;
t248 = t259 * t343 + t260 * t342 + t344 * t276;
t247 = t267 * t343 - t268 * t342 - t300 * t305 + t302 * t304 + xDD_base(1);
t246 = t342 * t258 - t472;
t238 = t343 * t241 + t242 * t342 + t250 * t344 + t255 * t304 + t256 * t305 + t271 * t337;
t1 = [(t247 - g(1)) * m(1); (t252 - g(2)) * m(1); (t251 - g(3)) * m(1); t434 / 0.2e1 + t431 / 0.2e1 + t433 / 0.2e1 + t430 / 0.2e1 + t271 * t451 + t250 * t445 + t272 * t450 + ((t258 + t473) * t343 + t460) * t448 + t420 + (t249 + t245) * t447 + ((-t255 + t473) * t342 + t246 + t472) * t446 + (g(2) * t302 - g(3) * t300 + t470 * t274 - t275 * t375 + t356) * m(1); -t248 * t406 / 0.2e1 + (((-t324 * t346 + t326 * t348 + t323) * t344 + t276 * xD_base(6) + (-t316 * t346 + t318 * t348 + t288) * t343 + (t317 * t346 - t319 * t348 + t290) * t342) * t351 + (t380 * xD_base(6) + t352) * t350) * t444 + ((t333 * t324 + t334 * t326) * t344 + (t333 * t316 + t334 * t318) * t343 + (-t333 * t317 - t334 * t319) * t342 + (-t256 * t423 + t271 * t351) * xD_base(6)) * t446 + ((t335 * t324 - t336 * t326) * t344 + (t335 * t316 - t336 * t318) * t343 + (-t335 * t317 + t336 * t319) * t342 + (t257 * t426 + t272 * t351) * xD_base(6)) * t448 + (t259 * t449 + t394 * t443 + t255 * t451 + t396 * t445 + t257 * t450 + t398 * t447 + ((-t258 * xD_base(6) - t361) * t350 + t463) * t448) * t349 + (t260 * t449 + t393 * t443 + t256 * t451 + t395 * t445 + t469 + t397 * t447 + t408 * t453 + ((t255 * xD_base(6) + t361) * t350 - t463) * t446 + t459) * t347 + (t247 * (-t300 * t347 + t302 * t349) + (-t274 * t349 + t275 * t347) * t303 + (t347 * t391 + t349 * t392) * t329 - t275 * (t344 * t321 + t342 * t328) - t274 * (t344 * t320 - t343 * t328) - ((t274 * t300 - t275 * t302) * t351 + t353 * t350) * xD_base(6) - g(1) * t328 - g(2) * t320 + g(3) * t321 + (t320 * t342 + t321 * t343 + t461 * t347 + t462 * t349) * t269) * m(1) + (t344 * t246 + t238) * t349 / 0.2e1; t248 * t411 / 0.2e1 + t350 * (t420 + t430 + t431 + t433 + t434) / 0.2e1 + (t276 * t350 - t351 * t380) * t449 + ((t380 * xD_base(5) + t254) * t350 + (t276 * xD_base(5) - t347 * t394 + t349 * t393) * t351) * t443 + t245 * t403 / 0.2e1 - t238 * t425 / 0.2e1 + (t271 * t350 - t351 * t382) * t451 + ((t382 * xD_base(5) + t250) * t350 + (t271 * xD_base(5) - t347 * t396 + t349 * t395) * t351) * t445 + (t272 * t350 - t351 * t381) * t450 + ((t381 * xD_base(5) + t249) * t350 + (t272 * xD_base(5) - t347 * t398 + t349 * t397) * t351) * t447 + (t360 * t350 + (-t354 * t346 - t348 * t455) * t351) * t444 + (t333 * t354 - t334 * t455 - t360 * t425) * t446 + (t354 * t335 + t455 * t336) * t448 - t458 * t246 / 0.2e1 + (t360 * t448 + t459) * t422 + ((t353 * xD_base(5) + t356) * t350 + (-t247 * t376 + t269 * (-t462 * t347 + t461 * t349) + t275 * (-t302 * xD_base(5) + t303 * t349) + t274 * (t300 * xD_base(5) + t303 * t347) + (-t347 * t392 + t349 * t391) * t329) * t351 - t269 * (-t312 * t342 + t313 * t343) - t275 * (-t313 * t344 + t341 * t342) - t274 * (t312 * t344 - t341 * t343) - g(1) * t341 - g(2) * t312 - g(3) * t313) * m(1);];
tauB  = t1(:);

