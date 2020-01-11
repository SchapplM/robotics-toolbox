% Steifigkeitsmatrix einer PKM bezogen auf die Endeffektor-Plattform
% 
% Eingabe:
% q 
%   Gelenkwinkel
%
% Ausgabe:
% SF_Par [6x6]
%   Kartesische Steifigkeitsmatrix der PKM
%   Komponenten: [N/m (3x3), Nm/m (3x3); N/rad (3x3), Nm/rad (3x3)]
%
% Quellen:
% [Zhao2020] Modellierung und Maßsynthese serieller und paralleler Roboter
% hinsichtlich der strukturellen Steifigkeit (Masterarbeit)
% [Klimchik2011] Enhanced stiffness modeling of serial and parallel
% manipulators for robotic-based processing of high performance materials

% Masterarbeit Yuqi ZHAO, zhaoyuqi.nicolas@gmail.com, 2019-12
% Betreuer: Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function SF_Par = stiffness(R, q)

SF_Par = zeros(6,6);

for i = 1:R.NLEG
    q_i = q(R.I1J_LEG(i):R.I2J_LEG(i));
    % Steifigkeit serieller Kette berechnen
    SF_LB_LE = R.Leg(i).stiffness(q_i);
    %Rotation von Bein Basis zum Plattform Basis [Klimchik2011 P70]
    T_B0_W = R.Leg(i).T_W_0(1:3,1:3);
    SF_PB_LE = [T_B0_W,zeros(3);zeros(3),T_B0_W]*SF_LB_LE*[T_B0_W,zeros(3);zeros(3),T_B0_W]';
    %Jacobian Matrix vom jedem Bein zum Endeffektor berechnen [Klimchik2011 P71]
    v = -R.r_P_B_all(:,i); % TODO: Hier müsste der EE genommen werden
    VX = skew(v); %Vector mit Kreuzprodukt vom Leg_EE zum Plat_EE
    Jv = [eye(3), zeros(3); VX, eye(3)];
    SF_PB_PE = Jv*SF_PB_LE*Jv';
    SF_PP_i = SF_PB_PE;
    SF_Par = SF_Par + SF_PP_i;
end
