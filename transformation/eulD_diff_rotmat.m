% Ableitung der zyz-Euler-Winkel nach der daraus berechneten Rotationsmatrix und der Zeit
%
% Eingabe:
% R [3x3]:
%   Rotationsmatrix
% RD [3x3]:
%   Zeitableitung der Rotationsmatrix
% conv [1x1]
%   Nummer der Euler-Winkel-Konvention
%   Siehe euler_angle_properties.m
%
% Ausgabe:
% GradMatD [3x9]:
%   Gradientenmatrix: Ableitung der Euler-Winkel nach der (spaltenweise gestapelten) Rotationsmatrix nochmals abgeleitet nach der Zeit

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function GradMatD = eulD_diff_rotmat(R, RD, conv)
%% Init
%#codegen
%$cgargs {zeros(3,3), zeros(3,3), uint8(2)}
assert(isreal(R) && all(size(R) == [3 3]), 'eulzyzD_diff_rotmat: R has to be [3x3] (double)');
assert(isreal(RD) && all(size(RD) == [3 3]), 'eulzyzD_diff_rotmat: RD has to be [3x3] (double)');
assert(isa(conv,'uint8') && isscalar(conv), 'euljacD: Number of Euler convention has to be [1x1] uint8');
r11s=R(1,1);r12s=R(1,2);r13s=R(1,3);
r21s=R(2,1);r22s=R(2,2);r23s=R(2,3);
r31s=R(3,1);r32s=R(3,2);r33s=R(3,3);
r11Ds=RD(1,1);r12Ds=RD(1,2);r13Ds=RD(1,3);
r21Ds=RD(2,1);r22Ds=RD(2,2);r23Ds=RD(2,3);
r31Ds=RD(3,1);r32Ds=RD(3,2);r33Ds=RD(3,3);
%% Berechnung
switch conv
% aus codeexport/eulxyxD_diff_rotmat_matlab.m (euler_angle_calculations.mw)
case 1 % yxD
t193 = r21s * r21Ds + r31s * r31Ds;
t201 = r21s ^ 2 + r31s ^ 2;
t195 = (t201 ^ (-0.1e1 / 0.2e1));
t209 = t195 * t193;
t197 = 1 / t201;
t208 = (t197 * t209);
t207 = (t193 / t201 ^ 2);
t202 = r12s ^ 2 + r13s ^ 2;
t206 = ((r12s * r12Ds + r13s * r13Ds) / t202 ^ 2);
t205 = (t195 * r11Ds);
t199 = 1 / t202;
t1 = [0 -r31Ds * t197 + 2 * r31s * t207 r21Ds * t197 - 2 * r21s * t207 0 0 0 0 0 0; -t209 r21s * t205 + (t195 * r21Ds - r21s * t208) * r11s r31s * t205 + (t195 * r31Ds - r31s * t208) * r11s 0 0 0 0 0 0; 0 0 0 r13Ds * t199 - 2 * r13s * t206 0 0 -r12Ds * t199 + 2 * r12s * t206 0 0;];
GradMatD = t1;
% aus codeexport/eulxyzD_diff_rotmat_matlab.m (euler_angle_calculations.mw)
case 2 % yzD
t210 = r23s * r23Ds + r33s * r33Ds;
t218 = r23s ^ 2 + r33s ^ 2;
t212 = (t218 ^ (-0.1e1 / 0.2e1));
t226 = t212 * t210;
t214 = 1 / t218;
t225 = (t214 * t226);
t224 = (t210 / t218 ^ 2);
t219 = r11s ^ 2 + r12s ^ 2;
t223 = ((r11s * r11Ds + r12s * r12Ds) / t219 ^ 2);
t222 = (t212 * r13Ds);
t216 = 1 / t219;
t1 = [0 0 0 0 0 0 0 -r33Ds * t214 + 2 * r33s * t224 r23Ds * t214 - 2 * r23s * t224; 0 0 0 0 0 0 t226 -r23s * t222 + (-t212 * r23Ds + r23s * t225) * r13s -r33s * t222 + (-t212 * r33Ds + r33s * t225) * r13s; r12Ds * t216 - 2 * r12s * t223 0 0 -r11Ds * t216 + 2 * r11s * t223 0 0 0 0 0;];
GradMatD = t1;
% aus codeexport/eulxzxD_diff_rotmat_matlab.m (euler_angle_calculations.mw)
case 3 % zxD
t227 = r21s * r21Ds + r31s * r31Ds;
t235 = r21s ^ 2 + r31s ^ 2;
t229 = (t235 ^ (-0.1e1 / 0.2e1));
t243 = t229 * t227;
t231 = 1 / t235;
t242 = (t231 * t243);
t241 = (t227 / t235 ^ 2);
t236 = r12s ^ 2 + r13s ^ 2;
t240 = ((r12s * r12Ds + r13s * r13Ds) / t236 ^ 2);
t239 = (t229 * r11Ds);
t233 = 1 / t236;
t1 = [0 -r31Ds * t231 + 2 * r31s * t241 r21Ds * t231 - 2 * r21s * t241 0 0 0 0 0 0; -t243 r21s * t239 + (t229 * r21Ds - r21s * t242) * r11s r31s * t239 + (t229 * r31Ds - r31s * t242) * r11s 0 0 0 0 0 0; 0 0 0 r13Ds * t233 - 2 * r13s * t240 0 0 -r12Ds * t233 + 2 * r12s * t240 0 0;];
GradMatD = t1;
% aus codeexport/eulxzyD_diff_rotmat_matlab.m (euler_angle_calculations.mw)
case 4 % zyD
t245 = r11s * r11Ds + r13s * r13Ds;
t253 = r11s ^ 2 + r13s ^ 2;
t246 = t253 ^ (-0.1e1 / 0.2e1);
t260 = t246 * t245;
t250 = 1 / t253;
t252 = r22s ^ 2 + r32s ^ 2;
t259 = ((r22s * r22Ds + r32s * r32Ds) / t252 ^ 2);
t258 = t250 * t260;
t257 = (t245 / t253 ^ 2);
t256 = t246 * r12Ds;
t248 = 1 / t252;
t1 = [0 0 0 0 -r32Ds * t248 + 2 * r32s * t259 r22Ds * t248 - 2 * r22s * t259 0 0 0; t246 * r11Ds * r12s + (-r12s * t258 + t256) * r11s 0 0 -t260 0 0 r13s * t256 + (t246 * r13Ds - r13s * t258) * r12s 0 0; -r13Ds * t250 + 2 * r13s * t257 0 0 0 0 0 r11Ds * t250 - 2 * r11s * t257 0 0;];
GradMatD = t1;
% aus codeexport/eulyxyD_diff_rotmat_matlab.m (euler_angle_calculations.mw)
case 5 % xyD
t261 = r21s * r21Ds + r23s * r23Ds;
t269 = r21s ^ 2 + r23s ^ 2;
t263 = t269 ^ (-0.1e1 / 0.2e1);
t277 = t263 * t261;
t265 = 1 / t269;
t276 = t265 * t277;
t275 = (t261 / t269 ^ 2);
t270 = r12s ^ 2 + r32s ^ 2;
t274 = ((r12s * r12Ds + r32s * r32Ds) / t270 ^ 2);
t273 = t263 * r22Ds;
t267 = 1 / t270;
t1 = [0 0 0 r32Ds * t267 - 2 * r32s * t274 0 -r12Ds * t267 + 2 * r12s * t274 0 0 0; 0 t263 * r21Ds * r22s + (-r22s * t276 + t273) * r21s 0 0 -t277 0 0 r23s * t273 + (t263 * r23Ds - r23s * t276) * r22s 0; 0 -r23Ds * t265 + 2 * r23s * t275 0 0 0 0 0 r21Ds * t265 - 2 * r21s * t275 0;];
GradMatD = t1;
% aus codeexport/eulyxzD_diff_rotmat_matlab.m (euler_angle_calculations.mw)
case 6 % xzD
t278 = r21s * r21Ds + r22s * r22Ds;
t286 = r21s ^ 2 + r22s ^ 2;
t280 = t286 ^ (-0.1e1 / 0.2e1);
t294 = t280 * t278;
t282 = 1 / t286;
t293 = r23s * t280;
t292 = (t278 / t286 ^ 2);
t287 = r13s ^ 2 + r33s ^ 2;
t291 = ((r13s * r13Ds + r33s * r33Ds) / t287 ^ 2);
t290 = -r23s * t282 * t294 + t280 * r23Ds;
t284 = 1 / t287;
t1 = [0 0 0 0 0 0 r33Ds * t284 - 2 * r33s * t291 0 -r13Ds * t284 + 2 * r13s * t291; 0 r21Ds * t293 + t290 * r21s 0 0 r22Ds * t293 + t290 * r22s 0 0 -t294 0; 0 r22Ds * t282 - 2 * r22s * t292 0 0 -r21Ds * t282 + 2 * r21s * t292 0 0 0 0;];
GradMatD = t1;
% aus codeexport/eulyzxD_diff_rotmat_matlab.m (euler_angle_calculations.mw)
case 7 % zxD
t295 = r22s * r22Ds + r23s * r23Ds;
t303 = r22s ^ 2 + r23s ^ 2;
t297 = t303 ^ (-0.1e1 / 0.2e1);
t311 = t297 * t295;
t299 = 1 / t303;
t310 = t299 * t311;
t309 = (t295 / t303 ^ 2);
t304 = r11s ^ 2 + r31s ^ 2;
t308 = ((r11s * r11Ds + r31s * r31Ds) / t304 ^ 2);
t307 = t297 * r21Ds;
t301 = 1 / t304;
t1 = [r31Ds * t301 - 2 * r31s * t308 0 -r11Ds * t301 + 2 * r11s * t308 0 0 0 0 0 0; 0 t311 0 0 -r22s * t307 + (-t297 * r22Ds + r22s * t310) * r21s 0 0 -r23s * t307 + (-t297 * r23Ds + r23s * t310) * r21s 0; 0 0 0 0 r23Ds * t299 - 2 * r23s * t309 0 0 -r22Ds * t299 + 2 * r22s * t309 0;];
GradMatD = t1;
% aus codeexport/eulyzyD_diff_rotmat_matlab.m (euler_angle_calculations.mw)
case 8 % zyD
t312 = r21s * r21Ds + r23s * r23Ds;
t320 = r21s ^ 2 + r23s ^ 2;
t314 = t320 ^ (-0.1e1 / 0.2e1);
t328 = t314 * t312;
t316 = 1 / t320;
t327 = t316 * t328;
t326 = (t312 / t320 ^ 2);
t321 = r12s ^ 2 + r32s ^ 2;
t325 = ((r12s * r12Ds + r32s * r32Ds) / t321 ^ 2);
t324 = t314 * r22Ds;
t318 = 1 / t321;
t1 = [0 0 0 r32Ds * t318 - 2 * r32s * t325 0 -r12Ds * t318 + 2 * r12s * t325 0 0 0; 0 t314 * r21Ds * r22s + (-r22s * t327 + t324) * r21s 0 0 -t328 0 0 r23s * t324 + (t314 * r23Ds - r23s * t327) * r22s 0; 0 -r23Ds * t316 + 2 * r23s * t326 0 0 0 0 0 r21Ds * t316 - 2 * r21s * t326 0;];
GradMatD = t1;
% aus codeexport/eulzxyD_diff_rotmat_matlab.m (euler_angle_calculations.mw)
case 9 % xyD
t329 = r31s * r31Ds + r33s * r33Ds;
t337 = r31s ^ 2 + r33s ^ 2;
t331 = t337 ^ (-0.1e1 / 0.2e1);
t345 = t331 * t329;
t333 = 1 / t337;
t344 = t333 * t345;
t343 = (t329 / t337 ^ 2);
t338 = r12s ^ 2 + r22s ^ 2;
t342 = ((r12s * r12Ds + r22s * r22Ds) / t338 ^ 2);
t341 = t331 * r32Ds;
t335 = 1 / t338;
t1 = [0 0 0 -r22Ds * t335 + 2 * r22s * t342 r12Ds * t335 - 2 * r12s * t342 0 0 0 0; 0 0 -t331 * r31Ds * r32s + (r32s * t344 - t341) * r31s 0 0 t345 0 0 -r33s * t341 + (-t331 * r33Ds + r33s * t344) * r32s; 0 0 -r33Ds * t333 + 2 * r33s * t343 0 0 0 0 0 r31Ds * t333 - 2 * r31s * t343;];
GradMatD = t1;
% aus codeexport/eulzxzD_diff_rotmat_matlab.m (euler_angle_calculations.mw)
case 10 % xzD
t346 = r31s * r31Ds + r32s * r32Ds;
t354 = r31s ^ 2 + r32s ^ 2;
t348 = t354 ^ (-0.1e1 / 0.2e1);
t362 = t348 * t346;
t350 = 1 / t354;
t361 = r33s * t348;
t360 = (t346 / t354 ^ 2);
t355 = r13s ^ 2 + r23s ^ 2;
t359 = ((r13s * r13Ds + r23s * r23Ds) / t355 ^ 2);
t358 = -r33s * t350 * t362 + t348 * r33Ds;
t352 = 1 / t355;
t1 = [0 0 0 0 0 0 -r23Ds * t352 + 2 * r23s * t359 r13Ds * t352 - 2 * r13s * t359 0; 0 0 r31Ds * t361 + t358 * r31s 0 0 r32Ds * t361 + t358 * r32s 0 0 -t362; 0 0 r32Ds * t350 - 2 * r32s * t360 0 0 -r31Ds * t350 + 2 * r31s * t360 0 0 0;];
GradMatD = t1;
% aus codeexport/eulzyxD_diff_rotmat_matlab.m (euler_angle_calculations.mw)
case 11 % yxD
t363 = r32s * r32Ds + r33s * r33Ds;
t371 = r32s ^ 2 + r33s ^ 2;
t365 = t371 ^ (-0.1e1 / 0.2e1);
t379 = t365 * t363;
t367 = 1 / t371;
t378 = t367 * t379;
t377 = (t363 / t371 ^ 2);
t372 = r11s ^ 2 + r21s ^ 2;
t376 = ((r11s * r11Ds + r21s * r21Ds) / t372 ^ 2);
t375 = t365 * r31Ds;
t369 = 1 / t372;
t1 = [-r21Ds * t369 + 2 * r21s * t376 r11Ds * t369 - 2 * r11s * t376 0 0 0 0 0 0 0; 0 0 -t379 0 0 r32s * t375 + (t365 * r32Ds - r32s * t378) * r31s 0 0 r33s * t375 + (t365 * r33Ds - r33s * t378) * r31s; 0 0 0 0 0 r33Ds * t367 - 2 * r33s * t377 0 0 -r32Ds * t367 + 2 * r32s * t377;];
GradMatD = t1;
% aus codeexport/eulzyzD_diff_rotmat_matlab.m (euler_angle_calculations.mw)
case 12 % yzD
t380 = r31s * r31Ds + r32s * r32Ds;
t388 = r31s ^ 2 + r32s ^ 2;
t382 = t388 ^ (-0.1e1 / 0.2e1);
t396 = t382 * t380;
t384 = 1 / t388;
t395 = r33s * t382;
t394 = (t380 / t388 ^ 2);
t389 = r13s ^ 2 + r23s ^ 2;
t393 = ((r13s * r13Ds + r23s * r23Ds) / t389 ^ 2);
t392 = -r33s * t384 * t396 + t382 * r33Ds;
t386 = 1 / t389;
t1 = [0 0 0 0 0 0 -r23Ds * t386 + 2 * r23s * t393 r13Ds * t386 - 2 * r13s * t393 0; 0 0 r31Ds * t395 + t392 * r31s 0 0 r32Ds * t395 + t392 * r32s 0 0 -t396; 0 0 r32Ds * t384 - 2 * r32s * t394 0 0 -r31Ds * t384 + 2 * r31s * t394 0 0 0;];
GradMatD = t1;
otherwise
error('eulD_diff_rotmat: conv has to be 1 to 12');
end
