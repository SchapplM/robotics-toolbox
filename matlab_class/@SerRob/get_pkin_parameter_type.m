% Gebe die Art des Kinematikparameters in pkin zurück
% Das kann dazu dienen, die Parameter einer Optimierung zu unterziehen
% 
% Ausgabe:
% types (Dimension von pkin)
%   Art der Parameter in pkin. Möglichkeiten:
%   0 Nicht zuzuordnen / Dummy-Parameter
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

% Funktion initialisieren
mdh2pkin_hdl = eval(sprintf('@%s_mdhparam2pkin', Rob.mdlname));

% Prüfe die Parameter nacheinander, indem NaN eingesetzt wird
pkin = NaN(6, length(Rob.pkin_gen));
pkin(1,:)=mdh2pkin_hdl(ones(Rob.NJ,1)*NaN, ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), zeros(Rob.NJ,1)); 
pkin(2,:)=mdh2pkin_hdl(ones(Rob.NJ,1), ones(Rob.NJ,1)*NaN, ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), zeros(Rob.NJ,1)); 
pkin(3,:)=mdh2pkin_hdl(ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1)*NaN, ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), zeros(Rob.NJ,1)); 
pkin(4,:)=mdh2pkin_hdl(ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1)*NaN, ones(Rob.NJ,1), ones(Rob.NJ,1), zeros(Rob.NJ,1)); 
pkin(5,:)=mdh2pkin_hdl(ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1)*NaN, ones(Rob.NJ,1), zeros(Rob.NJ,1)); 
pkin(6,:)=mdh2pkin_hdl(ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1)*NaN, zeros(Rob.NJ,1)); 

% Übersetze die Parameter zwischen Varianten und allgemeinem Modell
if ~strcmp(Rob.mdlname, Rob.mdlname_var)
  gen2var_hdl = eval(sprintf('@%s_pkin_gen2var', Rob.mdlname_var));
  pkin_var = NaN(6,length(Rob.pkin));
  for i = 1:6
    pkin_var(i,:) = gen2var_hdl(pkin(i,:));
  end
else
  pkin_var = pkin;
end

% Erkenne Pameter-Typ durch Ort der NaN in der Matrix
types = zeros(size(pkin_var,2),1);
for i = 1:6
  types(isnan(pkin_var(i,:))) = uint8(i);
end
