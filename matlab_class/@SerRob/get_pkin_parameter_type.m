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
%   7 offset
% jointnumber
%   Nummer des Gelenks in der MDH-Tabelle, das der Parameter beschreibt

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Universität Hannover

function [types, jointnumber] = get_pkin_parameter_type(Rob)

% Funktion initialisieren
mdh2pkin_hdl = eval(sprintf('@%s_mdhparam2pkin', Rob.mdlname));
pkin2mdh_hdl = eval(sprintf('@%s_pkin2mdhparam', Rob.mdlname));

% Prüfe die Parameter nacheinander, indem NaN eingesetzt wird
pkin = NaN(6, length(Rob.pkin_gen));
pkin(1,:)=mdh2pkin_hdl(ones(Rob.NJ,1)*NaN, ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), zeros(Rob.NJ,1)); 
pkin(2,:)=mdh2pkin_hdl(ones(Rob.NJ,1), ones(Rob.NJ,1)*NaN, ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), zeros(Rob.NJ,1)); 
pkin(3,:)=mdh2pkin_hdl(ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1)*NaN, ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), zeros(Rob.NJ,1)); 
pkin(4,:)=mdh2pkin_hdl(ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1)*NaN, ones(Rob.NJ,1), ones(Rob.NJ,1), zeros(Rob.NJ,1)); 
pkin(5,:)=mdh2pkin_hdl(ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1)*NaN, ones(Rob.NJ,1), zeros(Rob.NJ,1)); 
pkin(6,:)=mdh2pkin_hdl(ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1)*NaN, zeros(Rob.NJ,1)); 
pkin(7,:)=mdh2pkin_hdl(ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), ones(Rob.NJ,1), zeros(Rob.NJ,1), ones(Rob.NJ,1)*NaN); 

% Übersetze die Parameter zwischen Varianten und allgemeinem Modell
if ~strcmp(Rob.mdlname, Rob.mdlname_var)
  gen2var_hdl = eval(sprintf('@%s_pkin_gen2var', Rob.mdlname_var));
  pkin_var = NaN(6,length(Rob.pkin));
  for i = 1:7
    pkin_var(i,:) = gen2var_hdl(pkin(i,:));
  end
else
  pkin_var = pkin;
end

% Erkenne Pameter-Typ durch Ort der NaN in der Matrix
% Offset-Parameter hat Einfluss auf theta- und auf Offset-Spalte. Da Offset
% am Ende kommt, wird die Zuweisung 5 (theta) dann korrekt überschrieben mit 7
types = zeros(size(pkin_var,2),1);
for i = 1:7
  types(isnan(pkin_var(i,:))) = uint8(i);
end

% Erkenne Gelenknummer durch einsetzen von NaN in pkin und Prüfung der
% MDH-Variablen
jointnumber = zeros(length(Rob.pkin),1);
if ~strcmp(Rob.mdlname, Rob.mdlname_var)
  var2gen_hdl = eval(sprintf('@%s_pkin_var2gen', Rob.mdlname_var));
else
  var2gen_hdl = @(p)(p); % Kein Varianten-Modell: Keine Änderung der Par.
end
for i = 1:length(Rob.pkin)
  pkin_test = zeros(length(Rob.pkin),1);
  pkin_test(i) = NaN;
  m=zeros(Rob.NJ,7);
  [m(:,1),m(:,2),m(:,3),m(:,4),m(:,5),m(:,6),m(:,7)]=pkin2mdh_hdl(var2gen_hdl(pkin_test));
  I = find(isnan(sum(m,2)));
  if length(I) == 1
    jointnumber(i) = I;
  elseif length(I) > 1
    % Kinematik-Parameter beeinflusst mehrere DH-Parameter. Kann bei
    % hybriden Robotern auftreten, oder wenn mehrere Längen auf gleichen
    % symbolischen Wert gesetzt werden. Lasse Wert auf NaN.
  else
    % Der Kinematik-Parameter hat keinen Einfluss (evtl Dummy-Parameter bei
    % Ketten wie UPS, die keine Parameter haben). Lasse Wert auf NaN.
  end
end

