% Beispiel- und Testskript für die Berechnung der Schnittkräfte einer PKM
% Inhalt:
% * Definition Roboter (6UPS) und Beispieltrajektorie (singularitätsfrei)
% * Berechnung inverse Dynamik, Antriebs- und Schnittkräfte
% * Prüfe energetische Konsistenz für die Umrechnung Antriebe/Plattform
% * Zeichne Auswertungsbilder zur Prüfung der Plausibilität (optional)
% 
% Siehe auch: ParRob_class_example_6UPS.m
%
% Quellen: 
% [AbdellatifHei2009] Computational efficient inverse dynamics of 6-DOF fully
% parallel manipulators by using the Lagrangian formalism

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-05
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc

if isempty(which('parroblib_path_init.m'))
  warning('Repo mit parallelen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end
rob_path = fileparts(which('robotics_toolbox_path_init.m'));
respath = fullfile(rob_path, 'examples_tests', 'results');
usr_jointspring = true; % <-- hier false, um die Gelenkelastizität zu deaktivieren
usr_nofigures = true; % <-- hier false, falls Bilder gezeichnet werden sollen
%% Definiere Roboter
RP = parroblib_create_robot_class('P6RRPRRR14V3G1P1A1', 0.5, 0.2);
parroblib_update_template_functions({'P6RRPRRR14V3G1P1'});
% Beinketten und PKM mit kompilierten Funktionen
RP.fill_fcn_handles(true, true);
% Aktuierung nochmal festlegen
II_qai = [3 3 3 3 3 3];
I_qa = false(RP.NJ,1);
for k = 1:RP.NLEG
  I_qa(RP.I1J_LEG(k)-1+II_qai(k)) = true;
end
RP.update_actuation(I_qa);
%% Plattform-Konfiguration verändern
% Mit einer Kreisförmigen Plattformkoppelpunktanordnung ist die PKM
% singulär (Jacobi der direkten Kinematik). Daher paarweise Anordnung
RP.align_platform_coupling(4, [0.2;0.1]);

%% Beispieltrajektorie definieren
X0 = [ [0;0;0.5]; [0;0;0]*pi/180 ];
% Trajektorie mit beliebigen Bewegungen der Plattform
XL = [X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.3]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.3, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [0.3, 0.0, 0.0]]; ...
      X0'+1*[[ 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]; ...
      X0'+1*[[ 0.2,-0.1, 0.3], [0.3, 0.2, 0.1]]; ...
      X0'+1*[[-0.1, 0.1,-0.1], [-0.1,-0.2,-0.2]]; ...
      X0'+1*[[ 0.2, 0.1, 0.2], [0.2, 0.1, 0.3]]];
XL = [XL; XL(1,:)]; % Rückfahrt zurück zum Startpunkt.
[X_t,XD_t,XDD_t,T,X_sp] = traj_trapez2_multipoint(XL, 1, 0.1, 0.01, 1e-3, 1e-1);
nt = length(T);
%% Grenzen für die Gelenkpositionen setzen
% Dadurch wird die Schrittweite bei der inversen Kinematik begrenzt (auf 5%
% der Spannbreite der Gelenkgrenzen) und die Konfiguration klappt nicht um.
for i = 1:RP.NLEG
  % Begrenze die Winkel der Kugel- und Kardangelenke auf +/- 360°
  RP.Leg(i).qlim = repmat([-2*pi, 2*pi], RP.Leg(i).NQJ, 1);
  % Begrenze die Länge der Schubgelenke
  RP.Leg(i).qlim(3,:) = [0.4, 0.7];
end
%% Inverse Kinematik berechnen
t1=tic();
for i = 1:10 % Mehrere Versuche für IK. Manchmal funktioniert es nicht.
  q0_ik = 0.5-rand(RP.NJ,1);
  q0_ik(RP.I_qa) = 0.5; % Damit Konfiguration nicht umklappt
  [q0, Phi0] = RP.invkin_ser(X0, q0_ik);
  if any(abs(Phi0) > 1e-8) || any(isnan(q0))
    warning('Versuch %d: IK konvergiert nicht für Startpunkt', i);
  elseif any(q0(RP.I_qa)<0)
    warning('Versuch %d: IK führt zu negativer Schubgelenkkoordinate', i);
  else
    break;
  end
end
fprintf('Inverse Kinematik für Trajektorie berechnen: %d Bahnpunkte\n', nt);
% Einstellungen: Versuche Restfehler so klein wie möglich zu bekommen,
% damit System konsistent ist.
s = struct( ...
             'n_min', 2, ... % Minimale Anzahl Iterationen: Damit Fehler kleinstmöglich wird
             'n_max', 1000, ... % Maximale Anzahl Iterationen
             'Phit_tol', 1e-12, ... % Toleranz für translatorischen Fehler
             'Phir_tol', 1e-12); % Toleranz für rotatorischen Fehler
[Q_t, QD_t, QDD_t, Phi_t, Jinv_t] = RP.invkin2_traj(X_t, XD_t, XDD_t, T, q0, s);
if any(any(abs(Phi_t(:,RP.I_constr_t_red)) > 1e-10)) || ...
   any(any(abs(Phi_t(:,RP.I_constr_r_red)) > 1e-10))
   error('Fehler in Trajektorie zu groß. IK nicht berechenbar');
end
fprintf('IK berechnet. Dauer: %1.1fs\n', toc(t1));

%% Debug: Gelenksteifigkeit setzen
% Annahme einer Drefeder in den Gelenken. Dadurch Änderung der Schnittkräfte.
if usr_jointspring
  for k = 1:RP.NLEG
    I_k = RP.I1J_LEG(k):RP.I2J_LEG(k);
    % Ruhelage der Feder ist die Startkonfiguration
    RP.Leg(k).DesPar.joint_stiffness_qref = q0(I_k);
    % Federsteifigkeit auf moderaten Wert
    RP.Leg(k).DesPar.joint_stiffness = ones(length(I_k),1)*100;
  end
end

%% Roboter in Startpose plotten
if ~usr_nofigures
  fhdl=figure(1);clf;set(fhdl, 'Name', 'Startpose', 'NumberTitle', 'off');
  hold on;grid on;
  xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
  view(3);
  s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
  RP.plot( q0, X0, s_plot );
end
%% Dynamik-Parameter
mges = zeros(size(RP.DynPar.mges));
rSges = zeros(size(RP.DynPar.rSges));
Icges = zeros(size(RP.DynPar.Icges));
% Erstes Beinsegment (Stab vor Schubgelenk)
mges(2) = 0.5;
Icges(3,1:3) = 0.1*ones(3,1);
% Letztes Beinsegment (Stab nach Schubgelenk)
mges(3) = 0.5;
Icges(3,1:3) = 0.1*ones(3,1);
% Plattform
mges(end) = 1;
Icges(end,1:2) = 0.1;
Icges(end,3) = 0.5;
% Dynamik-Parameter in Roboter-Klasse aktualisieren
RP.update_dynpar1(mges, rSges, Icges);

%% Dynamik berechnen
t1=tic();
Fp_t = NaN(nt, 6); % Plattform-Kräfte
TAUa_t = NaN(nt, sum(RP.I_qa)); % Antriebskräfte
Det_t = NaN(nt,11); % Verschiedene Determinanten
P_ges_t = NaN(nt,14); % Leistung versch. Komponenten
FA_t = NaN(nt, 8*RP.NLEG); % Kräfte und Momente in Gestell-Koppelpunkten
FB_t = NaN(nt, 8*RP.NLEG); % Kräfte und Momente in Plattform-Koppelpunkten
FLeg0_t = NaN(7*RP.NLEG, RP.NLEG, nt);
FLegl_t = NaN(7*RP.NLEG, RP.NLEG, nt);
for i = 1:nt
  % Initialisierung von Zuständen für diesen Zeitschritt
  q = Q_t(i,:)';
  qD = QD_t(i,:)';
  qDD = QDD_t(i,:)';
  x = X_t(i,:)';
  xD = XD_t(i,:)';
  xDD = XDD_t(i,:)';
  qa = q(RP.I_qa);
  qaD = qD(RP.I_qa);
  
  % Berechne (inverse) Jacobi-Matrix aus Zwangsbedingungs-Gradienten
  Phi_q = RP.constr1grad_q(q, x);
  Phi_x = RP.constr1grad_x(q, x);
  Jinv_num_voll = -Phi_q \ Phi_x;
  Jinv_num = Jinv_num_voll(RP.I_qa,:);
  % Aufteilung in aktive, abhängige und Plattformkoordinaten
  Phi_a = Phi_q(:,RP.I_qa);
  Phi_d = Phi_q(:,RP.I_qd);
  Phi_dx = [Phi_d, Phi_x];
  % Berechne Jacobi-Matrix
  J_tilde = - Phi_dx\ Phi_a;
  J_num = J_tilde(sum(RP.I_qd)+1:end, :);
  % Beziehe Jacobi-Matrix auf Winkelgeschwindigkeiten, statt auf
  % Euler-Winkel-Ableitungen
  T_euljac = [eye(3,3), zeros(3,3); zeros(3,3), euljac(x(4:6), RP.phiconv_W_E)];
  Jinv_sym = Jinv_num / T_euljac;
  % Berechne Dynamik in Plattform-Koordinaten (unabhängig von Aktuierung)
  tauX = RP.invdyn_platform(q,x,xD,xDD);
  if usr_jointspring
    tauX = tauX + RP.jointtorque_platform(q, x, RP.springtorque(q));
  end
  
  % Projiziere die Dynamik in die Koordinaten der Antriebsgelenke
  % [AbdellatifHei2009], Text nach Gl. (37)
  tauA = (Jinv_sym') \ tauX;
  % Speichern der Daten
  Fp_t(i,:) = tauX;
  TAUa_t(i,:) = tauA;
  Det_t(i,1:5) = [det(Phi_dx), det(Phi_q), det(J_num), det(Jinv_num), det(Jinv_sym)];
  for kk = 1:6
    II = RP.I1J_LEG(kk):RP.I2J_LEG(kk);
    Det_t(i,5+kk) = det(Phi_q(II,II));
  end

  % Berechne Leistungsfluss zur Probe
  p_a = sum( tauA .* qaD );
  p_x = sum( tauX .* xD );
  P_ges_t(i,:) = [(tauA .* qaD)', (tauX .* xD)', p_a, p_x];
  test_p = p_a - p_x;
  test_p_rel = test_p/p_a;
  % TODO: Dieser Test ist erstmal deaktiviert, weil qDD/xDD noch nicht
  % richtig ist. Eventuell gibt es auch noch andere Fehler
%   if any( abs(test_p) > 1e-10 ) && test_p_rel > 1e-2
%     error('Leistungsfluss stimmt nicht zwischen Plattform und Antrieben');
%   end
  
  % Berechne die Schnittkräfte mit der Klassenmethode
  [cf_w_B, cf_w_all_linkframe, cf_w_all_baseframe] = RP.internforce(q, qD, qDD, tauA);
  if usr_jointspring
    % Schnittkräfte verursacht durch Federmomente zusätzlich berechnen.
    % Nicht in vorherigem Aufruf enthalten.
    [cf_w_B_spring, cf_w_all_linkframe_spring, cf_w_all_baseframe_spring] = ...
      RP.internforce(q, 0*qD, 0*qDD, 0*tauA, RP.springtorque(q));
    cf_w_B = cf_w_B + cf_w_B_spring;
    cf_w_all_linkframe = cf_w_all_linkframe + cf_w_all_linkframe_spring;
    cf_w_all_baseframe = cf_w_all_baseframe + cf_w_all_baseframe_spring;
  end
  
  % Berechne die Schnittkräfte. Siehe Aufzeichnungen Schappler, 9.5.19
  Fx_sumB = zeros(6,1); % Kraftsumme der Beine auf die Plattform
  Fx_sumA = zeros(6,1); % Kraftsumme der Reaktionskraft auf die Basis
  for j = 1:RP.NLEG % Für alle Beinketten
    q_j = q(RP.I1J_LEG(j):RP.I2J_LEG(j));
    qD_j = qD(RP.I1J_LEG(j):RP.I2J_LEG(j));
    qDD_j = qDD(RP.I1J_LEG(j):RP.I2J_LEG(j));
    tau_j = RP.Leg(j).invdyn(q_j, qD_j, qDD_j);
    if usr_jointspring
      tau_j_spring = RP.Leg(j).springtorque(q_j);
      tau_j = tau_j + tau_j_spring;
    end
    tau_m_j = zeros(RP.Leg(j).NQJ,1); tau_m_j(II_qai(j)) = tauA(j); % Antriebsmomente dieses Beins (passive sind Null)
    R_0_0j = RP.Leg(j).T_W_0(1:3,1:3); % Rotation PKM-Basis - Beinkette-Basis
    % Bein-Jacobi-Matrix für Koppelpunkt. Im PKM-Basis-KS
    J_j_0 = [R_0_0j, zeros(3,3); zeros(3,3), R_0_0j] * RP.Leg(j).jacobig(q_j);
    % Kraft, mit der die Plattform auf die Beinketten drückt (PKM-Basis-KS)
    FB_j_0 = (J_j_0') \ (tau_j - tau_m_j);
    % Numerische Fehler abrunden (damit die Plots übersichtlicher sind)
    FB_j_0(abs(FB_j_0) < 5e-14) = 0;
    FB_t(i, (j-1)*8+1:(j*8)) = [FB_j_0; norm(FB_j_0(1:3)); norm(FB_j_0(4:6))];
    
    % Probe der Dynamik-Gleichung in Minimalkoordinaten des Beins
    % (Grundlage für obige Rechnung).
    if any( abs(((J_j_0') * FB_j_0) - (tau_j - tau_m_j)) > 10*max(eps(1+abs(FB_j_0))) )
      error('Dynamik-Gleichung der Beinkette (intern/extern) stimmt nicht');
    end
    
    % Kraft und Moment umrechnen auf Plattform-KS: Kraft/Moment, mit der alle
    % Beinketten auf die Plattform wirken. Umrechnung der Kräfte auf einen
    % gemeinsamen Punkt (P)
    r_P_P_Bj = RP.r_P_B_all(:,j);
    R_0_P = eul2r(x(4:6), RP.phiconv_W_0);
    A_B_P = adjoint_jacobian(R_0_P*r_P_P_Bj); % Adjunkt-Matrix B-P
    Fx_sumB = Fx_sumB + A_B_P'*FB_j_0; % Umrechnung der Kräfte auf Momente
    
    % Berechne die Schnittkräfte im Bein
    % Kraft in Bein-Basis-KS umrechnen
    F_B_j_0j = rotate_wrench(FB_j_0, R_0_0j');
    % Schnittkräfte des Beins im Bein-KS berechnen ("l"=Linkframe)
    W_j_l_ext = RP.Leg(j).internforce_ext(q_j, F_B_j_0j, RP.Leg(j).NL-1, zeros(3,1));
    % Schnittkräfte aufgrund der internen Kräfte (M, c, g; keine Feder)
    W_j_l_int = RP.Leg(j).internforce(q_j, qD_j, qDD_j);
    % Teste Schnittkräfte in Koppelpunkten mit den Schnittkräften im
    % letzten Segment. Müssen übereinstimmen
    if max(abs( norm(FB_j_0(1:3)) - norm(W_j_l_ext(1:3,end)) )) > 10*max(eps(1+abs(FB_j_0(1:3))))
      error('Betrag der Koppelpunkt-Kraft stimmt nicht mit Kraft aus Projektion auf Beingelenke überein');
    end
    % Umrechnung der im Körper-KS des letzten Bein-Segmentes definierten
    % Schnittkraft ins Basis-KS der PKM zum Vergleich mit der Schnittkraft
    % aus PKM-Berechnung
    Tc_j_0j = RP.Leg(j).fkine(q_j);
    R_0j_Bj = Tc_j_0j(1:3,1:3,end);
    if max(abs(rotate_wrench(W_j_l_ext(:,end), (R_0_0j*R_0j_Bj)) - FB_j_0)) > 10*max(eps(1+abs(FB_j_0)))
      error('Die Schnittkraft aus PKM-Berechnung stimmt nicht mit Schnittkraft aus Beinkette überein');
    end
    
    % Gesamte Schnittkräfte: Differenz entspricht Schnittkraft im Gelenk.
    % Je nach Gelenktyp in der Struktur aufgefangen oder in Richtung des
    % Gelenk-FG
    W_j_l = - W_j_l_ext + W_j_l_int;
    % Schnittkräfte wieder in Basis-KS zurückrechnen (sind vorher im
    % Körper-KS)
    W_j_0 = W_j_l*NaN;
    W_j_0j = W_j_l*NaN;
    for k = 1:size(W_j_l,2)
      R_0j_l = Tc_j_0j(1:3,1:3,k);
      W_j_0(:,k) = rotate_wrench(W_j_l(:,k), R_0_0j*R_0j_l);
      W_j_0j(:,k) = rotate_wrench(W_j_l(:,k), R_0j_l);
    end
    % Numerische Fehler abrunden
    W_j_0(abs(W_j_0)<1e-14) = 0;
    FLeg0_t(:,j,i) = W_j_0(:);
    FLegl_t(:,j,i) = W_j_l(:);

    % Berechne Antriebskräfte der Beinkette wieder aus den vorher
    % berechneten Schnittkräften. Für passive Gelenke muss die Gelenkkraft
    % in FG-Richtung Null sein. Für aktive Gelenke das Antriebsmoment.
    tau_m_j_from_W = NaN*tau_m_j;
    for k = 2:RP.Leg(j).NJ+1
      if RP.Leg(j).MDH.sigma(k-1) == 0 % Drehgelenk: 6. Eintrag
        tau_m_j_from_W(k-1) = W_j_l(6,k);
      else % Schubgelenk: 3. Eintrag
        tau_m_j_from_W(k-1) = W_j_l(3,k);
      end
    end
    if usr_jointspring
      % Zähle die Federmomente als virtuelle Aktuierung. Auf die Schnittkräfte
      % wirken die Gelenkmomente entgegengesetzt zu tau_m. Daher "doppeltes Minus".
      tau_m_j_from_W = tau_m_j_from_W + tau_j_spring;
    end
    test_taum = tau_m_j - tau_m_j_from_W;
    if any( abs(test_taum) > 1e4*max(eps(1+abs(tau_m_j))) )
      error('Schnittkräfte in den Gelenken stimmen nicht mit Antriebskräften der Beinkette überein');
    end
    % Schnittkräfte an den Gestell-Koppelpunkten
    % Schnittkräfte in Gestell-Koppelpunkt können aus Schnittkräften der
    % Beinkette geholt werden
    F_A_j_0 = W_j_0(1:6,1); % Kraft an Koppelpunkten. Momente bezogen auf Koppelpunkt
    FA_t(i, (j-1)*8+1:(j*8)) = [F_A_j_0; norm(F_A_j_0(1:3)); norm(F_A_j_0(4:6))];
    % Umrechnung auf PKM-Basis
    r_0_0_Aj = RP.Leg(j).r_W_0(1:3);
    A_Aj_0 = adjoint_jacobian(r_0_0_Aj); % Adjunkt-Matrix A-0
    F_0_j_0 = A_Aj_0'*F_A_j_0; % Umrechnung der Kräfte auf Momente an der Basis
    Fx_sumA = Fx_sumA + F_0_j_0; % Summe darf nur über Momente mit gleichem Bezugspunkt (Basis) gebildet werden
    
    % Prüfe Ergebnisse der Klassen-Methode
    test_wB = cf_w_B(:,j) - FB_j_0;
    if max(abs(test_wB)) > 1e-10
      error('Schnittkräfte in B stimmen nicht mit Klassenmethode überein');
    end
    test_wj = W_j_l - cf_w_all_linkframe(:,:,j);
    if max(abs(test_wj(:))) > 1e-10
      error('Schnittkräfte in Gelenken (Gelenk-KS) stimmen nicht mit Klassenmethode überein');
    end
    test_wj = W_j_0 - cf_w_all_baseframe(:,:,j);
    if max(abs(test_wj(:))) > 1e-10 || any(isnan(test_wj(:)))
      error('Schnittkräfte in Gelenken (Basis-KS) stimmen nicht mit Klassenmethode überein');
    end
  end
  
  % Test-Bereich. Der folgende Test funktioniert nur, wenn die Beine
  % masselos sind. Dann müssen die Beine die volle Plattform-Masse tragen
  if all(mges(1:end-1) == 0) && all(all(rSges(1:end-1,:)==0)) && all(all(Icges(1:end-1,:)))
    test_FB = Fx_sumB + tauX;
    if max(abs(test_FB)) > 1e-8
      error('Kraftsumme Beine-Plattform stimmt nicht');
    end
  end
end
fprintf('Inverse Dynamik berechnet. Dauer: %1.1fs\n', toc(t1));

%% Ergebnisse plotten: Zeitverlauf der Plattform-Trajektorie
if ~usr_nofigures
  fhdl=figure(8);clf;set(fhdl, 'Name', 'Plattform', 'NumberTitle', 'off');
  for k = 1:6
    subplot(4,6,sprc2no(4,6,1,k));hold on;
    plot(T, X_t(:,k));
    plot(T(X_sp), X_t(X_sp,k), 'o');
    grid on;
    if k < 4, ylabel(sprintf('%s_E in m', char(119+k)));
    else,     ylabel(sprintf('\\phi_{E,%d} in rad',k-3)); end
    xlabel('t in s'); grid on;
    subplot(4,6,sprc2no(4,6,2,k));hold on;
    plot(T, XD_t(:,k));
    xlabel('t in s'); grid on;
    if k < 4, ylabel(sprintf('D%s_E in m/s', char(119+k)));
    else,     ylabel(sprintf('D\\phi_{E,%d} in rad/s',k-3)); end
    grid on;
    subplot(4,6,sprc2no(4,6,3,k));hold on;
    plot(T, XDD_t(:,k));
    xlabel('t in s');
    if k < 4, ylabel(sprintf('DD%s_E in m/s²', char(119+k)));
    else,     ylabel(sprintf('DD\\phi_{E,%d} in rad/s²',k-3)); end
    grid on;
    subplot(4,6,sprc2no(4,6,4,k));hold on;
    plot(T, Fp_t(:,k));
    xlabel('t in s');
    if k < 4, ylabel(sprintf('F_{P%s} in N', char(119+k)));
    else,     ylabel(sprintf('M_{P%s} in Nm', char(119+k-3))); end
    grid on;
  end
  linkxaxes
end
%% Ergebnisse plotten: Zeitverlauf der Antriebsgelenke
II_qa = find(RP.I_qa);
if ~usr_nofigures
  fhdl=figure(7);clf;set(fhdl, 'Name', 'Antriebsgelenke', 'NumberTitle', 'off');
  for k = 1:6
    subplot(4,6,sprc2no(4,6,1,k));hold on;
    plot(T, Q_t(:,II_qa(k)));
    plot(T(X_sp), Q_t(X_sp,II_qa(k)), 'o');
    xlabel('t in s');
    ylabel(sprintf('q_{a%d} in %s', k, RP.Leg(k).qunit_sci{II_qai(k)}));
    grid on;
    title(sprintf('Achse %d',k));
    subplot(4,6,sprc2no(4,6,2,k));hold on;
    plot(T, QD_t(:,II_qa(k)));
    xlabel('t in s');
    ylabel(sprintf('qD_{a%d} in %s/s', k, RP.Leg(k).qunit_sci{II_qai(k)}));
    grid on;
    subplot(4,6,sprc2no(4,6,3,k));hold on;
    plot(T, QDD_t(:,II_qa(k)));
    xlabel('t in s');
    ylabel(sprintf('qDD_{a%d} in %s/s^2', k, RP.Leg(k).qunit_sci{II_qai(k)}));
    grid on;
    subplot(4,6,sprc2no(4,6,4,k));hold on;
    plot(T, TAUa_t(:,k));
    xlabel('t in s');
    ylabel(sprintf('\\tau_{a%d} in %s', k, RP.Leg(k).tauunit_sci{II_qai(k)}));
    grid on;
  end
  linkxaxes
end
%% Ergebnisse plotten: Zeitverlauf weiterer Kenndaten der PKM
if ~usr_nofigures
  fhdl=figure(4);clf;set(fhdl, 'Name', 'Diagnose', 'NumberTitle', 'off');
  subplot(2,3,sprc2no(2,3,1,1)); hold on;
  plot(T, log10(abs(Det_t(:,1:4))));
  plot(T, log10(abs(Det_t(:,5))), '--');
  ylabel('Log. Determinante der Jacobi-Matrizen');
  xlabel('t in s');
  grid on;
  legend({'Phi_{dx}', 'Phi_{q}', 'J_{num}', 'Jinv_{num}', 'Jinv_{sym}'});
  title('Determinante (für Singularität)');
  subplot(2,3,sprc2no(2,3,2,1)); hold on;
  plot(T, log10(abs(Det_t(:,6:end))));
  ylabel('Log. Determinante der IK-Jacobis');
  xlabel('t in s');
  title('Determinante der Beinketten');
  grid on;
  legend({'Jinv_{1}', 'Jinv_{2}', 'Jinv_{3}', 'Jinv_{4}', 'Jinv_{5}', 'Jinv_{6}'});
  subplot(2,3,sprc2no(2,3,1,2)); hold on;
  plot(T, Phi_t(:,RP.I_constr_t));
  plot(T([1 end]), s.Phit_tol*[1;1], 'r--');
  plot(T([1 end]),-s.Phit_tol*[1;1], 'r--');
  grid on;
  ylabel('\Phi_{trans} in m');
  xlabel('t in s');
  title('Zwangsbedingungen (transl.)');
  subplot(2,3,sprc2no(2,3,2,2)); hold on;
  plot(T, Phi_t(:,RP.I_constr_r));
  plot(T([1 end]), s.Phir_tol*[1;1], 'r--');
  plot(T([1 end]),-s.Phir_tol*[1;1], 'r--');
  grid on;
  ylabel('\Phi_{rot} in rad');
  xlabel('t in s');
  title('Zwangsbedingungen (rot.)');
  subplot(2,3,sprc2no(2,3,1,3)); hold on;
  plot(T, P_ges_t(:,1:6));
  plot(T, P_ges_t(:,13), '--');
  ylabel('Leistung in Antriebskoord. in W');
  xlabel('t in s'); grid on;
  legend({'A1','A2','A3','A4','A5','A6','Summe'});
  title('Mech. Leistung (Plausibilität)');
  subplot(2,3,sprc2no(2,3,2,3)); hold on;
  plot(T, P_ges_t(:,7:12));
  plot(T, P_ges_t(:,14), '--');
  ylabel('Leistung in Plattformkoord. in W');
  xlabel('t in s'); grid on;
  legend({'Pfx','Pfy','Pfz','Pmx','Pmy','Pmz','Summe'});
  linkxaxes
end
%% Ergebnisse plotten: Zeitverlauf der Schnittkräfte
if ~usr_nofigures
  fhdl=figure(10);clf;set(fhdl, 'Name', 'Schnittkräfte_Koppelpunkte', 'NumberTitle', 'off');
  for j = 1:RP.NLEG
    subplot(4,RP.NLEG,sprc2no(4,RP.NLEG,1,j)); hold on;
    plot(T, FA_t(:, (j-1)*8+1:(j*8)-5))
    plot(T, FA_t(:, (j-1)*8+7))
    if j == RP.NLEG, legend({'fx', 'fy', 'fz', 'norm'}); end
    ylabel(sprintf('Schnittkraft A%d in N', j));
    grid on;
    subplot(4,RP.NLEG,sprc2no(4,RP.NLEG,2,j)); hold on;
    plot(T, FA_t(:, (j-1)*8+4:(j*8)-2));
    plot(T, FA_t(:, (j-1)*8+8));
    if j == RP.NLEG, legend({'mx', 'my', 'mz', 'norm'}); end
    ylabel(sprintf('Schnittmoment A%d in Nm', j));
    grid on;
    subplot(4,RP.NLEG,sprc2no(4,RP.NLEG,3,j)); hold on;
    plot(T, FB_t(:, (j-1)*8+1:(j*8)-5))
    plot(T, FB_t(:, (j-1)*8+7))
    if j == RP.NLEG, legend({'fx', 'fy', 'fz', 'norm'}); end
    ylabel(sprintf('Schnittkraft B%d in N', j));
    grid on;
    subplot(4,RP.NLEG,sprc2no(4,RP.NLEG,4,j)); hold on;
    plot(T, FB_t(:, (j-1)*8+4:(j*8)-2));
    plot(T, FB_t(:, (j-1)*8+8));
    if j == RP.NLEG, legend({'mx', 'my', 'mz', 'norm'}); end
    ylabel(sprintf('Schnittmoment B%d in Nm', j));
    xlabel('t in s'); grid on;
  end
  linkxaxes
end
%% Ergebnisse plotten: Zeitverlauf der Schnittkräfte und -momente aller Beingelenke aller Beine
if ~usr_nofigures
  for ks = [2 4]
    if ks == 2
      ksstr = '0';
    else
      ksstr = 'i';
    end
    for fm = [0 1]
      fhdl=figure(11+fm+ks);clf;
      if fm == 0
        set(fhdl, 'Name', sprintf('Schnittkräfte_Beine_KS%s',ksstr), 'NumberTitle', 'off');
      else
        set(fhdl, 'Name', sprintf('Schnittmoment_Beine_KS%s',ksstr), 'NumberTitle', 'off');
      end
      sphdl=NaN(RP.Leg(1).NL, RP.NLEG);
      for k = 1:RP.NLEG % Beine in den Spalten des Bildes
        % Vektor der Schnittkräfte für aktuelles Bein extrahieren
        if ks == 2
          W_k = squeeze(FLeg0_t(:,k,:))';
        else
          W_k = squeeze(FLegl_t(:,k,:))';
        end
        for j = 1:RP.Leg(1).NL % Beingelenke in den Zeilen des Bildes
          % Indizes zur Auswahl der Kräfte und Momente in den Gesamtdaten
          IIfm = ((j-1)*6+1:(j-1)*6+3);
          if fm ==1, IIfm = IIfm+3; end
          % Betrag bilden
          fmnorm_ges_jk = sum(W_k(:,IIfm).^2,2).^0.5;
          % Plotten
          sphdl(j,k)=subplot(size(sphdl,1),size(sphdl,2),sprc2no(size(sphdl,1),size(sphdl,2),j,k)); hold on;
          plot(T, [W_k(:,IIfm), fmnorm_ges_jk]);
          % Formatierung
          grid on;
          if j == 1
            title(sprintf('Beinkette %d', k));
          end
          if k == 1
            if fm == 0
              ylabel(sprintf('Kraft KS %d in N', j-1));
            else
              ylabel(sprintf('Moment KS %d in Nm', j-1));
            end
          end
          if k == RP.NLEG && j == RP.Leg(1).NL
            if fm == 0
              legend({'fx', 'fy', 'fz', 'norm'});
            else
              legend({'mx', 'my', 'mz', 'norm'});
            end
          end
        end
      end
      linkxaxes
      remove_inner_labels(sphdl,1); 
    end
  end
end
%% Animation des bewegten Roboters
if ~usr_nofigures
  s_anim = struct( 'gif_name', fullfile(respath, 'ParRob_class_example_6UPS.gif'));
  s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I1L_LEG+1; RP.I2L_LEG], 'straight', 0);
  fhdl=figure(5);clf;hold all;set(fhdl, 'Name', 'Animation', 'NumberTitle', 'off');
  set(fhdl, 'color','w', 'units','normalized', 'outerposition', [0 0 1 1]);
  view(3);
  axis auto
  hold on;grid on;
  xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
  RP.anim( Q_t(1:20:end,:), X_t(1:20:end,:), s_anim, s_plot);
  fprintf('Animation der Bewegung gespeichert: %s\n', fullfile(respath, 'ParRob_cutforce_6UPS_traj.gif'));
  fprintf('Test für 6UPS beendet\n');
end
save(fullfile(respath, 'ParRob_cutforce_example_final_workspace.mat'));
%% Debug: Vergleich verschiedener Implementierungen mit Parameterlinearer Form

t1=tic();
% load(fullfile(respath, 'ParRob_cutforce_example_final_workspace.mat'));
% Optional: Berechne Klassenmethode mit anderen Dynamik-Parametern
% mges(1:end-1) = 0;
% mges(5) = 1;
% mges(end) = 0;
% Icges(:,1:end-1) = 0;
% RP.update_dynpar1(mges, rSges, Icges);
% Setze Modus auf Inertialparameter-Regressor, damit es funktioniert
RP.DynPar.mode = 3;
for j = 1:RP.NLEG
  RP.Leg(j).DynPar.mode = 3;
end

% Daten aus Trajektorie nochmal durchgehen. Berechne nur die inverse
% Dynamik, keine Berücksichtigung der Gelenkfeder (egal ob gesetzt).
for i=1:size(Q_t,1)
  % Initialisierung
  q = Q_t(i,:)';
  qD = QD_t(i,:)';
  qDD = QDD_t(i,:)';
  x = X_t(i,:)';
  xD = XD_t(i,:)';
  xDD = XDD_t(i,:)';
  [~,Jinv_num_voll] = RP.jacobi_qa_x(q,x);
  tauA = RP.invdyn2_actjoint(q, qD, qDD, x,xD,xDD,Jinv_num_voll);

  % Schnittkräfte mit Klassen-Methoden (Parameterlinear und normal)
  [cf_w_B, cf_w_all_linkframe, cf_w_all_baseframe] = RP.internforce(q, qD, qDD, tauA);
  [cf_w_B_reg, cf_w_all_linkframe_reg, cf_w_all_baseframe_reg] = RP.internforce_regmat(q, qD, qDD, x,xD,xDD,Jinv_num_voll);
  % Prüfe Implementierung des Koppelpunkt-Schnittkraft-Regressors
  cf_w_B_fromreg = reshape(cf_w_B_reg*RP.DynPar.ipv_n1s, 6, RP.NLEG);
  test_cf_w_B = cf_w_B - cf_w_B_fromreg;
  if any(abs(test_cf_w_B(:)) > 1e-8)
    error('Schnittkraft in Plattform-Koppelgelenken stimmt nicht mit parameterlinearer Form');
  end
  % Teste Implementierung der Schnittkraft in Körper-KS
  cf_w_all_linkframe_fromreg = RP.internforce3(cf_w_all_linkframe_reg);
  test_cf_w_all_linkframe = cf_w_all_linkframe_fromreg - cf_w_all_linkframe;
  if any(abs(test_cf_w_all_linkframe(:)) > 1e-8)
    error('Gesamtheit der Schnittkräfte (in Körper-KS) stimmt nicht mit parameterlinearer Form');
  end
  % Teste Implementierung der Schnittkraft im PKM-Basis-KS
  cf_w_all_baseframe_fromreg = RP.internforce3(cf_w_all_baseframe_reg);
  test_cf_w_all_baseframe = cf_w_all_baseframe_fromreg - cf_w_all_baseframe;
  if any(abs(test_cf_w_all_baseframe(:)) > 1e-8)
    error('Gesamtheit der Schnittkräfte (in Basis-KS) stimmt nicht mit parameterlinearer Form');
  end
end
% Trajektorien-Funktion testen. Entferne den Einfluss der Gelenkfeder
% wieder
TAUa_spring = RP.jointtorque_actjoint_traj(Q_t, X_t, RP.springtorque_traj(Q_t), Jinv_t);
I = 1:size(Q_t,1);
[FLegl_t, FLeg0_t] = RP.internforce_traj(Q_t(I,:), QD_t(I,:), QDD_t(I,:), TAUa_t(I,:)-TAUa_spring(I,:));
[Regmatl_traj, Regmat0_traj] = RP.internforce_regmat_traj(Q_t(I,:), QD_t(I,:), QDD_t(I,:), X_t(I,:), XD_t(I,:), XDD_t(I,:));
% Schnittkräfte wieder aus Regressor-Trajektorie erzeugen und dann vergleichen
FLegl_t_fromreg = RP.internforce3_traj(Regmatl_traj);
FLeg0_t_fromreg = RP.internforce3_traj(Regmat0_traj);
test_FLeg0_t = FLeg0_t - FLeg0_t_fromreg;
test_FLegl_t = FLegl_t - FLegl_t_fromreg;
I_abserr_0 = abs(test_FLeg0_t(:)) > 1e-7;
I_rellerr_0 = abs(test_FLeg0_t(:)./FLeg0_t(:)) > 1e-3;
I_abserr_l = abs(test_FLegl_t(:)) > 1e-7;
I_rellerr_l = abs(test_FLegl_t(:)./FLegl_t(:)) > 1e-3;

if any(I_abserr_0 & I_rellerr_0) || any(I_abserr_l & I_rellerr_l)
  error('Trajektorie der Schnittkräfte stimmt nicht mit verschiedenen Berechnungen');
end
fprintf('Trajektorien-Funktionen für Schnittkräfte und deren Regressor-Matrizen getestet. Dauer: %1.1fs\n', toc(t1));
