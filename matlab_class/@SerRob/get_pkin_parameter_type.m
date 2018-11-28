% Gebe die Art des Kinematikparameters in pkin zurück
% Das kann dazu dienen, die Parameter einer Optimierung zu unterziehen
% 
% Ausgabe:
% types (Dimension von pkin)
%   Art der Parameter in pkin. Möglichkeiten:
%   1 beta
%   2 b
%   3 alpha
%   4 a
%   5 theta
%   6 d
%   7 offset (TODO: Noch nicht implementiert)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Universität Hannover

function types = get_pkin_parameter_type(Rob)

mdh2pkin_hdl = eval(sprintf('@%s_mdhparam2pkin', Rob.mdlname));
types = NaN(size(Rob.pkin));

beta_test = ones(Rob.NJ);
b_test = ones(Rob.NJ);
alpha_test = ones(Rob.NJ);
a_test = ones(Rob.NJ);
theta_test = ones(Rob.NJ);
d_test = ones(Rob.NJ);

% Prüfe die Parameter nacheinander, indem NaN eingesetzt wird
pkin1=mdh2pkin_hdl(beta_test*NaN, b_test, alpha_test, a_test, theta_test, d_test, zeros(Rob.NJ,1)); 
types(isnan(pkin1)) = uint8(1);
pkin2=mdh2pkin_hdl(beta_test, b_test*NaN, alpha_test, a_test, theta_test, d_test, zeros(Rob.NJ,1)); 
types(isnan(pkin2)) = uint8(2);
pkin3=mdh2pkin_hdl(beta_test, b_test, alpha_test*NaN, a_test, theta_test, d_test, zeros(Rob.NJ,1)); 
types(isnan(pkin3)) = uint8(3);
pkin4=mdh2pkin_hdl(beta_test, b_test, alpha_test, a_test*NaN, theta_test, d_test, zeros(Rob.NJ,1)); 
types(isnan(pkin4)) = uint8(4);
pkin5=mdh2pkin_hdl(beta_test, b_test, alpha_test, a_test, theta_test*NaN, d_test, zeros(Rob.NJ,1)); 
types(isnan(pkin5)) = uint8(5);
pkin6=mdh2pkin_hdl(beta_test, b_test, alpha_test, a_test, theta_test, d_test*NaN, zeros(Rob.NJ,1)); 
types(isnan(pkin6)) = uint8(6);

if any(isnan(types))
  error('Parameterzuordnung nicht möglich');
end