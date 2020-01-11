% Steifigkeitsmatrix einer seriellen Kette
% Berechnung basierend auf Werten aus der Literatur für serielle Roboter
% und Beinketten paralleler Roboter
% 
% Eingabe:
% q
%   Gelenkwinkel
% 
% Ausgabe:
% SF_Par [6x6]
%   Kartesische Steifigkeitsmatrix der seriellen Kette
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
function SF_L = stiffness(R, q)

%% Initialisierung

m_L = sum(R.DynPar.mges(2:end));

if ~R.islegchain
  I_qa = R.MDH.mu; % 1=aktiv, 0=passiv
else
  I_qa = R.MDH.mu>=2; % für PKM ist mu=2 aktives Gelenk
end

NL = R.NL;
NJ = R.NJ;
a = R.MDH.a;
d = R.MDH.d;
v = R.MDH.v;
alpha = R.MDH.alpha;

L_max=R.reach();
%% Nachgiebigkeit Datenbank für Schub und Drehgelenke
M_G = m_L*9.8*L_max/2; %Moment aus Selbstgewicht
if R.islegchain
  S = 14.7/(1.5*M_G); %Skalierungsfaktor (Norm: M = 12*9.8/6*0.5+2*9.8*0.25) . TODO: Quelle
else
  S = 11600/(2*M_G); %Skalierungsfaktor (Nach KUKA 240-2 Datenblatt: M = 240*9.8*3+400*9.8*1.5)
end
if R.islegchain
  %Für Drehgelenk (Aktiv). Nachgiebigkeit Daten aus [Klimchik2011] P52.
  Nd_a = S*diag([0.0000167, 0.0000167, 0.000022, 0.00002, 0.00002, 0.0000333]);
  %Für Kugelgelenk (Passiv). TODO: Quelle
  NK_p = S*diag([3.33E-7, 1E-7, 1E-7, 1e9, 1e9, 1e9]); 

  %Für Uni-Gelenk (Passiv). TODO: Quelle
  NU_p = S*diag([1.45E-6, 3E-5, 3E-5, 0.0022, 1e9, 1e9]); 
else % Serieller Roboter
  %Für Drehgelenke
  Nd_a = S*diag([1.428E-8, 1.428E-8, 2.2E-8, 2E-8, 2E-8, 1.52E-7]); %Nachgiebigkeit Daten aus [HAL] P52
  NK_p = NaN(6,6); % In seriellen Robotern kann es keine Kugelgelenke geben
  NU_p = NaN(6,6);
end

E = 7E10;    % E-Modul Aluminium 70GPa
G = 2.55E10; % Schubmodul Aluminium 25.5GPa

%% Geometrische Jacobi-Matrix
% TODO: Benutze neue Klassenmethode dafür
Tc_ges = R.fkine(q);
T_stack = NaN(3*NL,4);
for ii = 1:NL
  T_stack(3*ii-2:3*ii,:) = Tc_ges(1:3,:,ii);
end
j = uint8(NJ);
r_i_i_C = zeros(3,1);
Jg_C = robot_tree_jacobig_cutforce_m(T_stack, v, j, r_i_i_C);

%% Steifigkeitsmatrix für alle Glieder berechen
NGL_c_glob_sum = zeros(6,6);
for i = 1:NJ
  D_r = R.DesPar.seg_par(i,2); % Außendurchmesser Hohlzylinder
  d_r = D_r-2*R.DesPar.seg_par(i,1); % Innendurchmesser
  
  ai = a(i);
  di = d(i);
  alphai = alpha(i);
  Li = (a(i)^2 + d(i)^2)^0.5;  %Für L-förmiges Glied
  Iy = pi*(D_r^4-d_r^4)/64; % axiale Flächenträgheitsmoment des Gliedes um y und z achsen
  Iz = Iy;
  Ip = pi*(D_r^4-d_r^4)/32; % polare Flächenträgheitsmoment des Gliedes
  R_L = roty(atan2(di,ai))*rotx(alphai)';

  kx_Fx = Li/(E*pi*(D_r^2-d_r^2)/4);
  ky_Fy = Li^3/(3*E*Iy);
  kz_Fz = Li^3/(3*E*Iz);
  ky_Mz = Li^2/(2*E*Iz);
  kz_My = -Li^2/(2*E*Iy);
  kdx_Mx = Li/(G*Ip);
  kdy_My = Li/(E*Iy);
  kdz_Mz = Li/(E*Iz);
  kdy_Fz = -Li^2/(2*E*Iy);
  kdz_Fy = Li^2/(2*E*Iz);

  % TODO: Warum Überlagerung und Rotation um 90°?
  ng1_1 = diag([kx_Fx, ky_Fy, kz_Fz, kdx_Mx, kdy_My, kdz_Mz]);
  ng1_2 = rot90(diag([kdz_Fy, kdy_Fz, 0, kz_My, ky_Mz],-1),1);
  ngL = ng1_1 + ng1_2; % Nachgiebigkeit des Gliedes im lokalen Koordinatensystem

  Jg = Jg_C(:,(6*i+1):(6*i+6));
  ngL_loc = [R_L, zeros(3); zeros(3), R_L] * ngL * [R_L', zeros(3); zeros(3), R_L'];
  ngL_c_glob = Jg*ngL_loc*transpose(Jg);
  NGL_c_glob_sum = NGL_c_glob_sum + ngL_c_glob;
end

%% Steifigkeitsmatrix für allen Gelenken berechnen

NGJ_c_glob_sum = zeros(6,6);
for j = 1:NJ
  Jg = Jg_C(:,(6*j+1):(6*j+6));
  if R.DesPar.joint_type(j) == 3 % Ist_Kugel(j) == 1
    ngJ_c = NK_p; %Steifigkeit-Datenbank für Kugelgelenk benutzen
    ngJ_c_glob = Jg*ngJ_c*Jg';
    NGJ_c_glob_sum = NGJ_c_glob_sum + ngJ_c_glob;
  elseif R.DesPar.joint_type(j) == 2  % Ist_Kardan(j) == 1 
    ngJ_c = NU_p; %Steifigkeit-Datenbank für Kardangelenk benutzen
    ngJ_c_glob = Jg*ngJ_c*Jg';
    NGJ_c_glob_sum = NGJ_c_glob_sum + ngJ_c_glob;
  elseif R.DesPar.joint_type(j) == 0 % sigma(j) == 0 %Ist einzel Drehgelenk
    ngJ_c = Nd_a; %Steifigkeit-Datenbank für einzeln Drehgelenk benutzen
    if I_qa(j) == 0 %Passiv Gelenk
      ngJ_c(6,6) = 1E9; % Um z-Achse frei zu drehen
    end
    ngJ_c_glob = Jg*ngJ_c*transpose(Jg);
    NGJ_c_glob_sum = NGJ_c_glob_sum + ngJ_c_glob;
  elseif any(R.DesPar.joint_type(j) == [1 4 5])  % Ist Schubgelenk
    q_min = R.qlim(j,1);
    q_max = R.qlim(j,2);
    R_s = roty(pi/2);

    L_0 = max(abs(q_min),abs(q_max))/4; %L - Länge der Mutter (Null Position)

    L_s = abs(q(j));  % Länge der Stange
    Iy = pi*D_r^4/64;
    Iz = Iy;
    Ip = 2*Iy;
    Iy_r = pi*d_r^4/64; 
    Iz_r = Iy;
    Ip_r = 2*Iy_r; 

    % TODO: Woher kommt die Zahl?
    kx_Fx = S*(1.613e-6); %Actuator Steifigkeit des Schubgelenkes
    ky_Fy = L_0^3/(3*E*Iy)+L_s^3/(3*E*Iy_r);
    kz_Fz = L_0^3/(3*E*Iz)+L_s^3/(3*E*Iz_r);
    ky_Mz = L_0^2/(2*E*Iz)+L_s^2/(2*E*Iz_r);
    kz_My = -L_0^2/(2*E*Iy)-L_s^2/(2*E*Iy_r);
    kdx_Mx = L_0/(G*Ip)+L_s/(G*Ip_r);
    kdy_My = L_0/(E*Iy)+L_s/(E*Iy_r);
    kdz_Mz = L_0/(E*Iz)+L_s/(E*Iz_r);
    kdy_Fz = -L_0^2/(2*E*Iy)-L_s^2/(2*E*Iy_r);
    kdz_Fy = L_0^2/(2*E*Iz)+L_s^2/(2*E*Iz_r);

    ng2_1 = diag([kx_Fx, ky_Fy, kz_Fz, kdx_Mx, kdy_My, kdz_Mz]);
    ng2_2 = rot90(diag([kdz_Fy, kdy_Fz, 0, kz_My, ky_Mz],-1),1);
    ng2 = ng2_1 + ng2_2;
    Ns = [R_s, zeros(3); zeros(3), R_s]*ng2*[R_s, zeros(3); zeros(3), R_s]';
    ngJ_c = Ns;       % Datenbank für Schubgelenke benutzen
    if I_qa(j) == 0 % Passiv Gelenk (meistens kein passives Schubgelenk)
      ngJ_c = Ns; % Steifigkeit-Datenbank für einzeln Schubgelenk benutzen
      ngJ_c(1,1) = 1E9; % Entlang der x-Achse frei zu schieben. TODO: Warum x und nicht z?
    end
    ngJ_c_glob = Jg*ngJ_c*Jg';
    NGJ_c_glob_sum = NGJ_c_glob_sum + ngJ_c_glob;
  elseif R.DesPar.joint_type(j) == 6
    % Gelenk ist Fortsetzung eines anderen Gelenks. Nichts machen.
  else
    error('Nicht behandelter Fall');
  end
end

%% Ergebnis zusammenfassen
NG = NGJ_c_glob_sum + NGL_c_glob_sum;
SF_L = inv(NG); 
