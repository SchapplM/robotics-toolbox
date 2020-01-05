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
% tauB [6x1]
%   generalized base forces of inverse dynamics (contains inertial, gravitational coriolis and centrifugal forces)
%   base moment as Euler angle moment

% Quelle: HybrDyn-Toolbox
% Datum: 2019-08-21 13:58
% Revision: f86a32e2b5f1abc78572314ec3e2c1664fd21c45 (2019-08-20)
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function tauB = rigidbody_invdynB_floatb_eulxyz_slag_vp2(phi_base, xD_base, xDD_base, g, ...
  m, mrSges, Ifges)
%% Coder Information
%#codegen
%$cgargs {zeros(3,1),zeros(6,1),zeros(6,1),zeros(3,1),zeros(1,1),zeros(1,3),zeros(1,6)}
assert(isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  'rigidbody_invdynB_floatb_eulxyz_slag_vp2: phi_base has to be [3x1] (double)');
assert(isreal(xD_base) && all(size(xD_base) == [6 1]), ...
  'rigidbody_invdynB_floatb_eulxyz_slag_vp2: xD_base has to be [6x1] (double)');
assert(isreal(xDD_base) && all(size(xDD_base) == [6 1]), ...
  'rigidbody_invdynB_floatb_eulxyz_slag_vp2: xDD_base has to be [6x1] (double)');
assert(isreal(g) && all(size(g) == [3 1]), ...
  'rigidbody_invdynB_floatb_eulxyz_slag_vp2: g has to be [3x1] (double)');
assert(isreal(m) && all(size(m) == [1 1]), ...
  'rigidbody_invdynB_floatb_eulxyz_slag_vp2: m has to be [1x1] (double)'); 
assert(isreal(mrSges) && all(size(mrSges) == [1,3]), ...
  'rigidbody_invdynB_floatb_eulxyz_slag_vp2: mrSges has to be [1x3] (double)');
assert(isreal(Ifges) && all(size(Ifges) == [1 6]), ...
  'rigidbody_invdynB_floatb_eulxyz_slag_vp2: Ifges has to be [1x6] (double)'); 

%% Symbolic Calculation
% From invdyn_base_floatb_eulxyz_par2_matlab.m
% OptimizationMode: 2
% StartTime: 2019-08-21 13:58:13
% EndTime: 2019-08-21 13:58:18
% DurationCPUTime: 2.34s
% Computational Cost: add. (4032->301), mult. (11418->449), div. (0->0), fcn. (11719->6), ass. (0->138)
t220 = sin(phi_base(3));
t221 = sin(phi_base(1));
t222 = cos(phi_base(3));
t223 = cos(phi_base(1));
t225 = cos(phi_base(2));
t224 = sin(phi_base(2));
t276 = t222 * t223;
t278 = t221 * t220;
t209 = -t224 * t278 + t276;
t274 = t223 * t224;
t277 = t221 * t222;
t211 = t220 * t274 + t277;
t239 = t209 * t221 - t211 * t223;
t234 = t239 * t224;
t258 = t224 * xD_base(4);
t236 = xD_base(6) + t258;
t257 = t224 * xD_base(6);
t237 = xD_base(4) + t257;
t261 = t225 * xD_base(5);
t192 = -t237 * t277 + (-t221 * t261 - t236 * t223) * t220;
t246 = t211 * xD_base(4) + t192;
t231 = t237 * t223;
t309 = t236 * t221 - t223 * t261;
t190 = -t309 * t220 + t222 * t231;
t248 = -t209 * xD_base(4) + t190;
t218 = t224 ^ 2;
t219 = t225 ^ 2;
t269 = t218 - t219;
t169 = (t269 * t220 + t234) * xD_base(5) + (-t246 * t221 - t222 * t257 + t248 * t223) * t225;
t174 = t248 * t221 + t246 * t223;
t188 = (-t220 * t224 - t239) * t225;
t197 = t209 * t223 + t211 * t221;
t252 = xD_base(4) * xD_base(5);
t226 = t224 * t252 - t225 * xDD_base(4);
t256 = t225 * xD_base(4);
t250 = t222 * t256;
t164 = t174 * xD_base(5) + t188 * xDD_base(6) + t197 * xDD_base(5) + (t169 - t250) * xD_base(6) + t226 * t220;
t302 = t164 / 0.2e1;
t275 = t222 * t224;
t210 = t220 * t223 + t221 * t275;
t212 = t222 * t274 - t278;
t238 = t210 * t221 + t212 * t223;
t233 = t238 * t224;
t193 = t236 * t276 + (-t237 * t220 + t222 * t261) * t221;
t245 = t212 * xD_base(4) - t193;
t191 = t220 * t231 + t309 * t222;
t247 = -t210 * xD_base(4) + t191;
t170 = (-t269 * t222 + t233) * xD_base(5) + (-t220 * t257 + t245 * t221 + t247 * t223) * t225;
t175 = t247 * t221 - t245 * t223;
t230 = t238 - t275;
t189 = t230 * t225;
t198 = t210 * t223 - t212 * t221;
t251 = t220 * t256;
t165 = t175 * xD_base(5) - t189 * xDD_base(6) + t198 * xDD_base(5) + (t170 - t251) * xD_base(6) - t226 * t222;
t301 = t165 / 0.2e1;
t270 = t221 ^ 2 + t223 ^ 2;
t253 = t224 * (0.1e1 - t270);
t202 = t253 * t261;
t314 = t270 * t219;
t208 = t218 + t314;
t268 = 0.2e1 * xD_base(6);
t186 = t202 * t268 + t208 * xDD_base(6) + t224 * xDD_base(4) + t225 * t252;
t296 = t186 / 0.2e1;
t307 = (t269 + t314) * xD_base(6) + t258;
t184 = t220 * t307 + xD_base(6) * t234;
t313 = t174 - t184;
t185 = -t222 * t307 + xD_base(6) * t233;
t312 = t175 - t185;
t311 = -g(2) * t221 + g(3) * t223;
t205 = t208 * xD_base(6) + t258;
t280 = t205 * Ifges(1,3);
t265 = t198 * xD_base(5);
t180 = -t189 * xD_base(6) + t250 + t265;
t283 = t180 * Ifges(1,5);
t179 = t188 * xD_base(6) + t197 * xD_base(5) - t251;
t284 = t179 * Ifges(1,6);
t166 = t280 + t283 + t284;
t266 = t225 * xD_base(1);
t194 = t209 * xD_base(2) + t211 * xD_base(3) - t220 * t266;
t195 = t210 * xD_base(2) - t212 * xD_base(3) + t222 * t266;
t310 = t194 * mrSges(1,1) + t166 / 0.2e1 - t195 * mrSges(1,2);
t235 = t221 * xD_base(2) - t223 * xD_base(3);
t308 = -t224 * xD_base(1) + t235 * t225;
t305 = Ifges(1,5) * t301 + Ifges(1,6) * t302 + Ifges(1,3) * t296;
t306 = 0.2e1 * t202;
t159 = Ifges(1,4) * t165 + Ifges(1,2) * t164 + Ifges(1,6) * t186;
t304 = t159 / 0.2e1;
t303 = Ifges(1,1) * t301 + Ifges(1,4) * t302 + Ifges(1,5) * t296;
t300 = -t179 / 0.2e1;
t299 = t179 / 0.2e1;
t298 = -t180 / 0.2e1;
t297 = t180 / 0.2e1;
t295 = -t205 / 0.2e1;
t294 = t205 / 0.2e1;
t293 = -t220 / 0.2e1;
t292 = -t222 / 0.2e1;
t289 = Ifges(1,4) * t220;
t288 = Ifges(1,4) * t222;
t255 = xD_base(5) * xD_base(1);
t187 = (t235 * xD_base(5) + xDD_base(1)) * t224 + (t255 + (-xD_base(4) * xD_base(2) + xDD_base(3)) * t223 + (-xD_base(4) * xD_base(3) - xDD_base(2)) * t221) * t225;
t279 = t187 * t225;
t161 = -mrSges(1,1) * t164 + mrSges(1,2) * t165;
t273 = t225 * t161;
t181 = -t265 + (-t222 * xD_base(4) + t230 * xD_base(6)) * t225;
t272 = t169 - t181;
t271 = t170 - t179;
t264 = t220 * xD_base(5);
t263 = t222 * xD_base(5);
t262 = t224 * xD_base(5);
t260 = t220 * xD_base(6);
t259 = t222 * xD_base(6);
t249 = xD_base(6) * t266;
t244 = mrSges(1,1) * t222 - mrSges(1,2) * t220;
t243 = -mrSges(1,1) * t220 - mrSges(1,2) * t222;
t242 = Ifges(1,1) * t222 - t289;
t241 = -Ifges(1,2) * t220 + t288;
t240 = Ifges(1,5) * t222 - Ifges(1,6) * t220;
t229 = t224 * t255 - t225 * xDD_base(1);
t228 = -t221 * t256 - t223 * t262;
t227 = t221 * t262 - t223 * t256;
t214 = t243 * t225;
t213 = (-t221 * xD_base(3) - t223 * xD_base(2)) * t225;
t207 = t235 * t224 + t266;
t204 = t308 * t222;
t203 = t308 * t220;
t200 = t210 * xD_base(3) + t212 * xD_base(2);
t199 = t209 * xD_base(3) - t211 * xD_base(2);
t178 = Ifges(1,4) * t179;
t177 = mrSges(1,1) * t205 - mrSges(1,3) * t180;
t176 = -mrSges(1,2) * t205 + t179 * mrSges(1,3);
t173 = t191 * xD_base(3) + t193 * xD_base(2) + t210 * xDD_base(2) - t212 * xDD_base(3) - t220 * t249 - t229 * t222;
t172 = t190 * xD_base(3) + t192 * xD_base(2) + t209 * xDD_base(2) + t211 * xDD_base(3) + t229 * t220 - t222 * t249;
t171 = -mrSges(1,1) * t179 + mrSges(1,2) * t180;
t168 = Ifges(1,1) * t180 + Ifges(1,5) * t205 + t178;
t167 = Ifges(1,4) * t180 + Ifges(1,2) * t179 + Ifges(1,6) * t205;
t163 = mrSges(1,1) * t186 - mrSges(1,3) * t165;
t162 = -mrSges(1,2) * t186 + mrSges(1,3) * t164;
t1 = [-g(1) * m(1) + (m(1) * (t194 * t264 - t195 * t263 + t187) - t176 * t263 + t177 * t264 + t161) * t224 + (m(1) * (-t172 * t220 + t173 * t222 - t194 * t259 - t195 * t260 - t308 * xD_base(5)) - t176 * t260 + t222 * t162 - t177 * t259 - t220 * t163 + xD_base(5) * t171) * t225; -t221 * t273 + t210 * t162 + t209 * t163 + t193 * t176 + t192 * t177 + t227 * t171 + (t172 * t209 + t173 * t210 + t194 * t192 + t195 * t193 - t221 * t279 - t227 * t308 - g(2)) * m(1); t223 * t273 - t212 * t162 + t211 * t163 + t191 * t176 + t190 * t177 + t228 * t171 + (t172 * t211 - t173 * t212 + t194 * t190 + t195 * t191 + t223 * t279 - t228 * t308 - g(3)) * m(1); -g(2) * (t212 * mrSges(1,1) - t211 * mrSges(1,2)) - g(3) * (t210 * mrSges(1,1) + t209 * mrSges(1,2)) - t213 * t171 - t187 * t214 - t199 * t177 - t200 * t176 - m(1) * (t194 * t199 + t195 * t200 - t213 * t308) + (-t173 * mrSges(1,2) + t172 * mrSges(1,1) + 0.2e1 * t305 + (t240 * t295 - t308 * t243 + t241 * t300 + t242 * t298 + t168 * t292 + t220 * t167 / 0.2e1 + (t194 * t222 + t195 * t220) * mrSges(1,3)) * xD_base(5)) * t224 + (t222 * t303 + t159 * t293 + t240 * t296 + t242 * t301 + t241 * t302 + (t280 / 0.2e1 + t284 / 0.2e1 + t283 / 0.2e1 + t310) * xD_base(5) + (t167 * t292 + t168 * t293 + (-Ifges(1,5) * t220 - Ifges(1,6) * t222) * t294 - t308 * t244 + (-Ifges(1,2) * t222 - t289) * t299 + (-Ifges(1,1) * t220 - t288) * t297) * xD_base(6) + (g(2) * t223 + g(3) * t221 + (-t195 * xD_base(6) - t172) * t222 + (t194 * xD_base(6) - t173) * t220) * mrSges(1,3)) * t225; (-g(1) * t225 - t172 * t198 + t173 * t197 - t312 * t194 + t313 * t195 + t311 * t224) * mrSges(1,3) - t207 * t171 + t187 * (-mrSges(1,1) * t197 + mrSges(1,2) * t198) + t203 * t177 - t204 * t176 - m(1) * (-t194 * t203 + t195 * t204) + (-t184 / 0.2e1 + t174 / 0.2e1) * t167 + (Ifges(1,5) * t175 + Ifges(1,6) * t174) * t294 + (Ifges(1,5) * t185 + Ifges(1,6) * t184) * t295 + (Ifges(1,5) * t198 + Ifges(1,6) * t197) * t296 + (Ifges(1,1) * t175 + Ifges(1,4) * t174) * t297 + (Ifges(1,1) * t185 + Ifges(1,4) * t184) * t298 + (Ifges(1,4) * t175 + Ifges(1,2) * t174) * t299 + (Ifges(1,4) * t185 + Ifges(1,2) * t184) * t300 + (Ifges(1,1) * t198 + Ifges(1,4) * t197) * t301 + (Ifges(1,4) * t198 + Ifges(1,2) * t197) * t302 + t198 * t303 + t197 * t304 + (-t185 / 0.2e1 + t175 / 0.2e1) * t168 - (-m(1) * t207 - t313 * mrSges(1,1) + t312 * mrSges(1,2)) * t308 + (g(1) * t224 + t311 * t225) * t244 + (Ifges(1,5) * t298 + Ifges(1,6) * t300 + Ifges(1,3) * t295 - t310) * (t253 * t268 + xD_base(4)) * t225; (mrSges(1,1) * t306 - t271 * mrSges(1,3) - t176) * t194 + (t300 + t170 / 0.2e1) * t168 + (Ifges(1,5) * t170 + Ifges(1,6) * t169 + Ifges(1,3) * t306) * t294 + (Ifges(1,4) * t170 + Ifges(1,2) * t169 + Ifges(1,6) * t306) * t299 + (Ifges(1,1) * t170 + Ifges(1,4) * t169 + Ifges(1,5) * t306) * t297 - (-t272 * mrSges(1,1) + t271 * mrSges(1,2)) * t308 + (-t181 / 0.2e1 + t169 / 0.2e1) * t167 - g(1) * t214 - g(2) * (mrSges(1,1) * t209 - mrSges(1,2) * t210) - g(3) * (mrSges(1,1) * t211 + mrSges(1,2) * t212) + t173 * (-mrSges(1,2) * t208 + mrSges(1,3) * t188) + t172 * (mrSges(1,1) * t208 + mrSges(1,3) * t189) + t187 * (-mrSges(1,1) * t188 - mrSges(1,2) * t189) + (-Ifges(1,5) * t189 + Ifges(1,6) * t188 + Ifges(1,3) * t208) * t296 + (-Ifges(1,1) * t189 + Ifges(1,4) * t188 + Ifges(1,5) * t208) * t301 + (-Ifges(1,4) * t189 + Ifges(1,2) * t188 + Ifges(1,6) * t208) * t302 + t188 * t304 + t208 * t305 + (Ifges(1,5) * t179 + Ifges(1,6) * t181) * t295 + (Ifges(1,1) * t179 + Ifges(1,4) * t181) * t298 + (Ifges(1,2) * t181 + t178) * t300 - t189 * t303 + t202 * t166 + (-0.2e1 * mrSges(1,2) * t202 + t272 * mrSges(1,3) + t177) * t195;];
tauB  = t1(:);

