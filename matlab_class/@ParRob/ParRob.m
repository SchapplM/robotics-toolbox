%   Klasse für parallele Roboter
%
%   Mit dieser Klasse können alle Kinematik- und Dynamikfunktionen eines
%   parallelen Roboters aufgerufen werden.
% 
%   Koordinatensystem-Definitionen:
%   * W: Welt-KS (Absolutes Inertial-KS, auf das sich auch mehrere Roboter
%        gleichzeitig beziehen können)
%   * 0: Basis-KS (Inertial-KS des Roboters)
%   * A: Basis-Koppel-KS der einzelnen Beinketten (auch als "0i" bezeichnet)
%   * B: Plattform-Koppel-KS der Beinketten. Orientierung abhängig von
%        gewählter Anordnung der Koppelgelenk-Orientierung.
%   * P: Plattform-KS des Roboters, an dem der Endeffektor befestigt ist.
%        Es werden nur parallele Roboter betrachtet, bei denen die
%        Plattform am Ende aller Beinketten sitzt (also ohne zusätzliches
%        serielles "Ende" an der Plattform). Bezugs-KS für Dynamik.
%        Entspricht KS "N" bei SerRob.
%   * E: Endeffektor-KS des Roboters (z.B. Bohrerspitze), auf dieses KS
%        bezieht sich die Bahnplanung und Kinematik
% 
%   Gelenkkoordinaten-Definitionen:
%   * Zähle die aktiven, passiven und Schnittgelenke aller Beine 
%   (abhängig = passiv und Schnitt-Gelenkkoordinaten)
% 
%   Definition der Dynamik-Parameter:
%   DynPar.mode: Umschalten zwischen verschiedenen Dynamik-Parametern als
%   Eingang der Dynamik-Funktionen. Entspricht Ziffer hinter Funktions-
%   Handles. Sollte konsistent mit SerRob-Dynamikparameter-Modus sein.
%   1: Dynamik-Parameter bezogen auf Schwerpunkt m,rS,Ic (baryzentrisch)
%      (noch nicht implementiert)
%   2: Dynamik-Parameter bezogen auf Körper-KS-Ursprung m,mrS,If (inertial)
%   3: Dynamik-Parametervektor (ohne Minimierung);
%   4: Dynamik-Minimalparametervektor
% 
% Siehe auch: SerRob.m (SerRob-Klasse)
%
% Quellen. 
% [AbdellatifHei2009] Computational efficient inverse dynamics of 6-DOF fully
% parallel manipulators by using the Lagrangian formalism

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

classdef ParRob < RobBase

  properties (Access = public)
      NLEG % Anzahl der Beinketten
      I1J_LEG % Start-Indizes der Gelenkwinkel der einzelnen Beine in allen Gelenkwinkeln
      I2J_LEG % End-Indizes er Gelenkwinkel ...
      I1L_LEG % Start-Indizes der Segmente der einzelnen Beinketten
      I2L_LEG % End-Indizes der Segmente ..
      r_P_B_all % [3xNLEG] Ortsvektoren der Plattform
      phi_P_B_all % [3xNLEG] Orientierung der Plattform-Koppel-KS gegen die Plattform.
      r_0_A_all % [3xNLEG]
      MDH % Struktur mit MDH-Parametern für Roboterkinematik (der PKM)
      I_qd % Zähl-Indizes der abhängigen Gelenke in allen Gelenken
      I_qa % Zähl-Indizes der aktiven Gelenke in allen Gelenken
      phiconv_P_E % Winkelkonvention der Euler-Winkel vom Plattform-KS zum EE
      T_P_E % Homogene Transformationsmatrix zwischen Endeffektor-KS und Plattform-KS
      r_P_E % Position des Endeffektors im Plattform-KS
      phi_P_E % Orientierung des EE-KS im Plattform-KS (ausgedrückt in Euler-Winkeln)
      DynPar % Struktur mit Dynamikparatern (Masse, Schwerpunkt, Trägheit)
      DesPar % Struktur mit Entwurfsparameter (Gestell,Plattform)
      mdlname % Name des PKM-Robotermodells, das in den Matlab-Funktionen benutzt wird.
      Leg % Matlab-Klasse SerRob für jede Beinkette
      issym % true für rein symbolische Berechnung
      NQJ_LEG_bc % Anzahl relevanter Beingelenke vor Schnittgelenk (von Basis an gezählt) ("bc"="before cut")
      I1constr   % Start-Indizes der Beinketten in allen Zwangsbedingungen
      I2constr   % End-Indizes der Beinketten in allen Zwangsbedingungen
      I1constr_red   % Start-Indizes der Beinketten in allen reduzierten Zwangsbedingungen
      I2constr_red   % End-Indizes der Beinketten in allen reduzierten Zwangsbedingungen
      I_constr_t % Indizes der translatorischen Zwangsbedingungen in allen ZB
      I_constr_r % Indizes der rotatorischen ZB in allen ZB
      I_constr_red % Indizes der reduzierten ZB in allen ZB
      I_constr_t_red % Indizes der translatorischen ZB in den reduzierten ZB
      I_constr_r_red % ... Indizes der rotatorischen ZB in den reduzierten ZB
      I_platform_dynpar % Auswahlvektor für Dynamikparameter der Plattform
      collbodies_nonleg % Struktur mit Ersatzkörpern zur Kollisionserkennung (nur Plattform und Gestell)
      collbodies_instspc_nonleg % das gleiche für Bauraumprüfungs-Ersatzkörper
      collbodies % Enthält auch die Kollisionskörper der Beinketten
      collbodies_instspc % das gleiche für Bauraumprüfung
      collchecks % Liste von zu prüfenden Kollisionen in `collbodies`
      collchecks_instspc % Liste für Bauraumprüfungen aus `collbodies_instspc`
  end
  properties (Access = private)
      jacobi_qa_x_fcnhdl % Funktions-Handle für Jacobi-Matrix zwischen Antrieben und Plattform-KS
      invdyn_x_fcnhdl2       % Funktions-Handle für Inverse Dynamik in Plattform-Koordinaten
      invdyn_x_fcnhdl4       % ... mit anderen Parametern
      inertia_x_fcnhdl2      % Funktions-Handle für Massenmatrix in Plattform-Koordinaten
      inertia_x_fcnhdl4      % ... mit anderen Parametern
      gravload_x_fcnhdl2     % Funktions-Handle für Gravitationslast in Plattform-Koordinaten
      gravload_x_fcnhdl4     % ... mit anderen Parametern
      coriolisvec_x_fcnhdl2  % Funktions-Handle für Corioliskraft in Plattform-Koordinaten
      coriolisvec_x_fcnhdl4  % ... mit anderen Parametern
      dynparconvfcnhdl       % Funktions-Handle zur Umwandlung von DynPar 2 zu MPV
      fkintrajfcnhdl % Funktions-Handle zum Aufruf der direkten Kinematik aller Beinketten als Trajektorie
      fkincollfcnhdl % ... für direkte Kinematik mit Ausgabe der Kollisions-KS und Gelenkpositionen
      constr4Dtrajfcnhdl % F.H. zum Aufruf der Zwangsbedingungs-Zeitableitung als Trajektorie
      invkinfcnhdl % Funktions-Handle für IK (führt zu pkm_invkin.m.template)
      invkintrajfcnhdl % Funktions-Handle für Trajektorien-IK (zu pkm_invkin_traj.m.template)
      invkin3fcnhdl % Funktions-Handle für IK (zu pkm_invkin3_tpl.m.template)
      all_fcn_hdl % Cell-Array mit allen Funktions-Handles des Roboters sowie den Dateinamen der Matlab-Funktionen und deren Verfügbarkeit
      extfcn_available % Array mit Markern, welche Funktion aus all_fcn_hdl verfügbar ist.
  end
  methods
    % Konstruktor
    function R=ParRob(mdlname)
      % Standardwerte vorbelegen
      R.mdlname = mdlname;
      R.Type = 2; % Parallel
      R.Leg = [];
      R.MDH = struct('sigma', []);
      R.phiconv_P_E = uint8(2); % Euler-XYZ
      R.issym = false;
      R.r_P_E = zeros(3,1);
      R.phi_P_E = zeros(3,1);
      R.T_P_E = eye(4);
      R.DesPar = struct(...
        'base_method', uint8(0), ... % Modellierungsart Gestell siehe align_base_coupling.m
        'base_par', 0, ... % Parameter dafür (Radius,...)
        'platform_method', uint8(0), ... % Modellierungsart Plattform siehe align_platform_coupling.m
        'platform_par', zeros(1,2)); % Parameter dafür (Radius, ..., Stärke)
    
      % Liste der Funktionshandle-Variablen mit zugehörigen
      % Funktionsdateien (aus Maple-Toolbox)
      R.all_fcn_hdl = { ...
        {'jacobi_qa_x_fcnhdl', 'Jinv'}, ...
        {'invdyn_x_fcnhdl2', 'invdyn_para_pf_slag_vp2'}, ...
        {'inertia_x_fcnhdl2', 'inertia_para_pf_slag_vp2'}, ...
        {'gravload_x_fcnhdl2', 'gravload_para_pf_slag_vp2'}, ...
        {'coriolisvec_x_fcnhdl2', 'coriolisvec_para_pf_slag_vp2'}, ...
        {'invdyn_x_fcnhdl4', 'invdyn_para_pf_mdp'}, ...
        {'inertia_x_fcnhdl4', 'inertia_para_pf_mdp'}, ...
        {'gravload_x_fcnhdl4', 'gravload_para_pf_mdp'}, ...
        {'coriolisvec_x_fcnhdl4', 'coriolisvec_para_pf_mdp'}, ...
        {'dynparconvfcnhdl', 'minimal_parameter_para'},...
        {'fkintrajfcnhdl', 'fkineEE_traj'},...
        {'fkincollfcnhdl', 'fkine_coll'},...
        {'constr4Dtrajfcnhdl', 'constr4D_traj'}, ...
        {'invkinfcnhdl', 'invkin'},...
        {'invkintrajfcnhdl', 'invkin_traj'},...
        {'invkin3fcnhdl', 'invkin3'}};
      R.extfcn_available = false(length(R.all_fcn_hdl),1);
      R.I_platform_dynpar = true(1,10);
      % Struktur der Kollisions-Ersatzkörper. Siehe SerRob-Klasse.
      % Hier werden nur Kollisionskörper der PKM gelistet.
      R.collbodies_nonleg = struct( ...
        'link', uint8(zeros(0,2)), ... % nx2 uint8, Nummer der zugehörigen Segmente (0=Basis).
        'type', uint8(zeros(0,1)), ... % nx1 uint8, Art des Ersatzkörpers
        'params', zeros(0,10)); % Parameter des jeweiligen Ersatzkörpers
      % Platzhalter-Variable aller Kollisionskörper (auch der Beinketten)
      R.collbodies = R.collbodies_nonleg; % wird in update_collbodies eingestellt
      % Liste der Kollisionsprüfungen. Enthält zwei Spalten mit Index der
      % Kollisionsobjekte aus R.collbodies und danach R.Leg(1).collbodies
      % und allen weiteren Beinketten.
      R.collchecks = uint8(zeros(0,2));
      % Platzhalter-Variablen auch für Bauraumprüfung
      R.collbodies_instspc_nonleg = R.collbodies_nonleg;
      R.collbodies_instspc = R.collbodies_nonleg;
      R.collchecks_instspc = R.collchecks;
    end
    function [X,XD,XDD] = fkineEE_traj(R, Q, QD, QDD, idx_leg, platform_frame)
      % Direkte Kinematik für komplette Trajektorie berechnen. Basierend
      % auf der einer Beinkette.
      % Eingabe:
      % Q: Gelenkkoordinaten (Trajektorie)
      % QD: Gelenkgeschwindigkeiten (Trajektorie)
      % QDD: Gelenkbeschleunigung (Trajektorie)
      % idx_leg: Index der Beinkette, für die die direkte Kinematik
      % bestimmt wird.
      % platform_frame: Schalter zum Ignorieren der Transformation P-E
      %
      % Ausgabe:
      % X,XD,XDD: Lage, -Geschw., -Beschl. des EE-KS (als Zeitreihe;
      % bezogen auf PKM-Basis); des Plattform-KS, falls platform_frame=true
      if nargin < 5, idx_leg = 1; end
      if nargin < 6, platform_frame = false; end
      if ~platform_frame
        T_P_E = R.T_P_E; %#ok<PROPLC>
      else
        T_P_E = eye(4); %#ok<PROPLC>
      end
      X = NaN(size(Q,1),6);
      XD = X; XDD = X;
      % Transformation zum KS: Siehe fkine_legs
      T_0_A1 = R.Leg(idx_leg).T_W_0; % von PKM-Basis zu Beinketten-Basis
      r_P_P_B1 = R.r_P_B_all(:,idx_leg);
      R_P_B1 = eulxyz2r(R.phi_P_B_all(:,idx_leg));
      T_P_B1 = rt2tr(R_P_B1, r_P_P_B1); % Plattform-KS zu Plattform-Koppel-KS
      T_B1_E = invtr(T_P_B1) * T_P_E; %#ok<PROPLC> % Plf-Koppel-KS zu Plattform-EE-KS
      for i = 1:size(Q,1)
        q1_i   = Q  (i,R.I1J_LEG(idx_leg):R.I2J_LEG(idx_leg))';
        % Direkte Kinematik der Beinkette
        T_A1_E1 = R.Leg(idx_leg).fkineEE(q1_i);
        T_0_E1 = T_0_A1*T_A1_E1;
        % Annahme: E1 (virt. EE der Beinkette) = B1 (Koppelgelenk-KS);
        % (setzt erfüllte kinematische Zwangsbedingungen voraus)
        X(i,:) = R.t2x(T_0_E1*T_B1_E);
        if nargout < 2, continue; end % Keine Geschwindigkeit gefragt
        qD1_i  = QD (i,R.I1J_LEG(idx_leg):R.I2J_LEG(idx_leg))';
        % Geschwindigkeit der ersten Beinkette umrechnen auf PKM-Plattform
        Jg = R.Leg(idx_leg).jacobig(q1_i); % geom. Jacobi der Beinkette (bez. auf Beinketten-Basis)
        % Geschw. des virt. Beinketten-EE bezogen auf PKM-Basis
        V_0_E1 = rotate_wrench(Jg*qD1_i, t2r(T_0_A1));
        % Umrechnen auf PKM-EE mit Adjunkt-Jacobi-Matrix
        r_0_E1_E = t2r(T_0_E1) * T_B1_E(1:3,4);
        V_0_E = adjoint_jacobian(r_0_E1_E) * V_0_E1;
        % Umrechnen auf Euler-Winkel-Zeitableitung bezogen auf PKM-Koord.
        Tw = euljac(X(i,4:6)', R.phiconv_W_E);
        XD(i,:) = [V_0_E(1:3); Tw\V_0_E(4:6)];
        if nargout < 3, continue; end % Keine Beschleunigung gefragt
        qDD1_i = QDD(i,R.I1J_LEG(idx_leg):R.I2J_LEG(idx_leg))';
        % Beschleunigung der ersten Beinkette
        JgD = R.Leg(idx_leg).jacobigD(q1_i, qD1_i);
        VD_0_E1 = rotate_wrench(Jg*qDD1_i+JgD*qD1_i, t2r(T_0_A1));
        % Umrechnen auf PKM-EE
        VD_0_E = adjointD_jacobian(t2r(T_0_E1)'*r_0_E1_E, t2r(T_0_E1), V_0_E1(4:6)) * V_0_E1 + ...
                  adjoint_jacobian(r_0_E1_E) * VD_0_E1;
        TwD = euljacD(X(i,4:6)', XD(i,4:6)', R.phiconv_W_E);
        XDD(i,:) = [VD_0_E(1:3); Tw\(VD_0_E(4:6)-TwD*XD(i,4:6)')];
      end
    end
    function [X,XD,XDD] = fkineEE2_traj(R, Q, QD, QDD, idx_leg, platform_frame)
      % Direkte Kinematik für komplette Trajektorie berechnen. Basierend
      % auf der einer Beinkette. Genau wie Funktion fkineEE_traj, aber Auf-
      % ruf einer kompilierten Funktion für die ganze Trajektorie.
      % Eingabe:
      % Q, QD, QDD: Gelenk-Trajektorie
      % idx_leg: Nummer der Beinkette
      % platform_frame: Schalter zum Ignorieren der Transformation P-E
      % Ausgabe:
      % X,XD,XDD: Lage, -Geschw., -Beschl. des EE-KS (als Zeitreihe;
      % bezogen auf PKM-Basis); des Plattform-KS, falls platform_frame=true
      if nargin < 5, idx_leg = uint8(1); end
      if nargin < 6, platform_frame = false; end
      %% Eingabe-Struktur mit PKM-Parametern zusammenstellen
      Leg_pkin_gen = cat(2,R.Leg.pkin_gen)';
      Leg_T_N_E_vec = zeros(6,R.NLEG);% 1:3 Euler-Winkel, 4:6 Position
      Leg_T_0_W_vec = zeros(6,R.NLEG);% 1:3 Euler-Winkel, 4:6 Position
      Leg_phi_W_0 = zeros(3,R.NLEG);
      Leg_phiconv_W_0 = uint8(zeros(R.NLEG,1));
      for i = 1:R.NLEG
        T_N_E = R.Leg(i).T_N_E;
        Leg_T_N_E_vec(1:3,i) = r2eulxyz(T_N_E(1:3,1:3));
        Leg_T_N_E_vec(4:6,i) = T_N_E(1:3,4);
        T_0_W = R.Leg(i).T_0_W;
        Leg_T_0_W_vec(1:3,i) = r2eulxyz(T_0_W(1:3,1:3));
        Leg_T_0_W_vec(4:6,i) = T_0_W(1:3,4);
        Leg_phi_W_0(:,i) = R.Leg(i).phi_W_0;
        Leg_phiconv_W_0(i) = R.Leg(i).phiconv_W_0;
      end
      if ~platform_frame
        s_T_P_E = R.T_P_E;
      else
        s_T_P_E = eye(4);
      end
      s = struct( ...
        'T_P_E', s_T_P_E, ...
        'r_P_B_all', R.r_P_B_all,...
        'phi_P_B_all', R.phi_P_B_all,...
        'Leg_pkin_gen', Leg_pkin_gen,...
        'Leg_T_N_E_vec', Leg_T_N_E_vec,...
        'Leg_T_0_W_vec', Leg_T_0_W_vec, ...
        'Leg_phi_W_0', Leg_phi_W_0,...
        'Leg_phiconv_W_0', Leg_phiconv_W_0);
      if nargout <= 1
        QD = NaN(size(Q)); QDD = QD;
        X = R.fkintrajfcnhdl(Q, QD, QDD, uint8(idx_leg), s);
      elseif nargout == 2
        QDD = NaN(size(Q));
        [X, XD] = R.fkintrajfcnhdl(Q, QD, QDD, uint8(idx_leg), s);
      else
        [X, XD, XDD] = R.fkintrajfcnhdl(Q, QD, QDD, uint8(idx_leg), s);
      end
    end
    function [Tc_stack_PKM, JointPos_all] = fkine_coll2(R, q)
      % Direkte Kinematik für alle KS bzw. Gelenkpositionen der PKM, die für
      % Kollisionen notwendig sind
      % Eingabe:
      % q: Gelenkwinkel
      % Ausgabe:
      % Tc_stack_PKM: Gestapelte Transformationsmatrizen der PKM.
      % JointPos_all: Gestapelte Positionen aller Gelenke der PKM
      Leg_pkin_gen = cat(2,R.Leg.pkin_gen)';
      Leg_T_0_W_vec = zeros(6,R.NLEG);% 1:3 Euler-Winkel, 4:6 Position
      for i = 1:R.NLEG
        T_0_W = R.Leg(i).T_0_W;
        Leg_T_0_W_vec(1:3,i) = r2eulxyz(T_0_W(1:3,1:3));
        Leg_T_0_W_vec(4:6,i) = T_0_W(1:3,4);
      end
      s = struct( ...
        'Leg_pkin_gen', Leg_pkin_gen,...
        'Leg_T_0_W_vec', Leg_T_0_W_vec);
      [Tc_stack_PKM, JointPos_all] = R.fkincollfcnhdl(q, s);
    end
    function [q, Phi] = invkin(R, xE_soll, q0)
      % Inverse Kinematik berechnen
      % Eingabe:
      % xE_soll [6x1]
      %   Endeffektorpose des Roboters bezüglich des Basis-KS (Soll)
      % q0 [Nx1]
      %   Startkonfiguration: Alle Gelenkwinkel aller serieller Beinketten der PKM
      %
      % Ausgabe:
      % q: Gelenkposition
      % Phi: Kinematische Zwangsbedingungen für die Lösung.
      [q, Phi] = R.invkin_ser(xE_soll, q0);
    end
    function [q, Phi, Tc_stack_PKM, Stats] = invkin2(R, x, q0, s_in_ser, s_in_par)
      % Berechne die inverse Kinematik mit eigener Funktion für den Roboter
      % Die Berechnung erfolgt dadurch etwas schneller als durch die
      % Klassen-Methode `invkin_ser`, die nur teilweise kompilierbar ist.
      % Ruft Vorlagen_Funktion pkm_invkin.m.template auf.
      % Eingabe:
      % x: EE-Lage (Soll)
      % q0: Start-Pose
      % s_in_ser: Parameter s in Seriell-IK
      % s_in_par: Parameter für Parallel-IK
      % Ausgabe:
      % q: Gelenkposition
      % Phi: Residuum
      % 
      % Siehe auch: invkin_ser
      % Alle Einstellungen in Eingabestruktur für Funktion schreiben
      
      % Einstellungen für PKM-Parameter zusammenstellen
      Leg_I_EE_Task = cat(1,R.Leg.I_EE_Task);
      Leg_pkin_gen = cat(2,R.Leg.pkin_gen)';
      Leg_T_N_E_vec = zeros(6,R.NLEG);% 1:3 Euler-Winkel,4:6 Position
      Leg_T_0_W_vec = zeros(6,R.NLEG);% 1:3 Euler-Winkel,4:6 Position
      Leg_sigmaJ = zeros(R.Leg(1).NJ,R.NLEG);
      Leg_qlim = zeros(6,2*R.NLEG);
      Leg_phiconv_W_E = uint8(zeros(R.NLEG,1));
      for i = 1:R.NLEG
        T_N_E = R.Leg(i).T_N_E;
        Leg_T_N_E_vec(1:3,i) = r2eulxyz(T_N_E(1:3,1:3));
        Leg_T_N_E_vec(4:6,i) = T_N_E(1:3,4);
        T_0_W = R.Leg(i).T_0_W;
        Leg_T_0_W_vec(1:3,i) = r2eulxyz(T_0_W(1:3,1:3));
        Leg_T_0_W_vec(4:6,i) = T_0_W(1:3,4);
        Leg_sigmaJ(:,i) = R.Leg(i).MDH.sigma(R.Leg(i).MDH.mu>=1);
        Leg_qlim(1:R.Leg(i).NJ,(1+2*(i-1)):(2+2*(i-1))) = R.Leg(i).qlim;
        Leg_phiconv_W_E(i) = R.Leg(i).phiconv_W_E;
      end

      s_par = struct( ...
        'r_P_B_all', R.r_P_B_all, ...
        'phi_P_B_all', R.phi_P_B_all, ...
        'phiconv_W_E', R.phiconv_W_E, ...
        'I1constr_red', R.I1constr_red, ...
        'I2constr_red', R.I2constr_red, ...
        'T_P_E', R.T_P_E, ...
        'Leg_I_EE_Task', Leg_I_EE_Task, ...
        'Leg_pkin_gen', Leg_pkin_gen, ...
        'Leg_T_N_E_vec', Leg_T_N_E_vec, ...
        'Leg_T_0_W_vec', Leg_T_0_W_vec, ...             
        'I_EE_Task', R.I_EE_Task,...
        'Leg_sigmaJ',Leg_sigmaJ,...
        'Leg_qlim',Leg_qlim,...
        'Leg_phiconv_W_E',Leg_phiconv_W_E, ...
        'abort_firstlegerror', false);
      
      % Einstellungen für IK. Siehe auch: SerRob/invkin2
      s_ser = struct( ...
        'reci', false, ... % Keine reziproken Winkel für ZB-Def.
        'K', ones(R.Leg(1).NQJ,1), ... % Verstärkung Aufgabenbewegung
        'scale_lim', 0.0, ... % Herunterskalierung bei Grenzüberschreitung
        'maxrelstep', 0.05, ... % Maximale auf Grenzen bezogene Schrittweite
        'normalize', true, ... % Normalisieren auf +/- 180°
        'condlimDLS', 1, ... % Grenze der Konditionszahl, ab der die Pseudo-Inverse gedämpft wird (1=immer)
        'lambda_min', 2e-4, ... % Untergrenze für Dämpfungsfaktor der Pseudo-Inversen
        'n_min', 0, ... % Minimale Anzahl Iterationen
        'n_max', 1000, ... % Maximale Anzahl Iterationen
        'rng_seed', NaN, ... Initialwert für Zufallszahlengenerierung
        'Phit_tol', 1e-10, ... % Toleranz für translatorischen Fehler
        'Phir_tol', 1e-10, ... % Toleranz für rotatorischen Fehler
        'retry_on_limitviol', false, ... % Bei Grenzverletzung neu versuchen mit true
        'retry_limit', 100); % Anzahl der Neuversuche);
      % Alle Standard-Einstellungen in s_ser mit in s_in_ser übergebenen Einstellungen
      % überschreiben. Diese Reihenfolge ermöglicht für Kompilierung
      % geforderte gleichbleibende Feldreihenfolge in Eingabevariablen.
      if nargin >= 4
        for ff = fields(s_in_ser)'
          if ~isfield(s_ser, ff{1}) 
            if ~any(strcmp(ff{1},{'wn','maxstep_ns'})) % Aus Kompatibilitätsgründen akzeptieren
              error('Feld %s kann nicht übergeben werden', ff{1});
            end
          else
            s_ser.(ff{1}) = s_in_ser.(ff{1});
          end
        end
      end
      if nargin == 5 && ~isempty(s_in_par)
        for ff = fields(s_in_par)'
          if ~isfield(s_par, ff{1})
            error('Feld %s kann nicht übergeben werden');
          else
            s_par.(ff{1}) = s_in_par.(ff{1});
          end
        end
      end
      % Funktionsaufruf. Entspricht pkm_invkin.m.template
      if nargout == 3
        [q, Phi, Tc_stack_PKM] = R.invkinfcnhdl(x, q0, s_par, s_ser);
      elseif nargout <= 2
        [q, Phi] = R.invkinfcnhdl(x, q0, s_par, s_ser);
      else
        [q, Phi, Tc_stack_PKM, Stats] = R.invkinfcnhdl(x, q0, s_par, s_ser);
      end
    end
    function update_actuation(R, I_qa)
      R.I_qa = logical(I_qa); % Index der aktiven (und damit unabhängigen Gelenke)
      R.I_qd = ~R.I_qa; % Index der abhängigen Gelenke
      for i = 1:R.NLEG
        % Setze Marker mu auf 0 für nicht Teil der Minimalkoordinaten der
        % Beinkette, 1 für passiv für die PKM, 2 für aktiv bezogen auf PKM
        I_mincoord_Leg = (R.Leg(i).MDH.mu > 0);
        % Alle Gelenke in R.Leg(i) müssen als aktiv gekennzeichnet sein,
        % auch wenn sie in der PKM passiv sind (für Koordinatendefinition
        % hybrider Beinketten)
        if sum(I_mincoord_Leg) ~= R.Leg(i).NQJ
          error('Die Anzahl der als aktive Minimalkoordinaten gesetzten Gelenke passt nicht zur gespeicherten Anzahl der Gelenke');
        end
        R.Leg(i).MDH.mu(I_mincoord_Leg) = 1+double( I_qa(R.I1J_LEG(i):R.I2J_LEG(i)) );
      end
    end
    function update_EE(R, r_P_E, phi_P_E, phiconv_P_E)
      % Aktualisiere die Transformationsmatrix T_P_E für den Endeffektor
      % Eingabe:
      % r_P_E: Neuer Vektor vom EE-Körper-KS (Plattform) zum EE
      % phi_P_E: Neue Euler-Winkel für Drehung vom Plattform-KS zum EE-KS
      % phiconv_P_E: Nummer der Euler-Winkel-Konvention
      if nargin > 1 && ~isempty(r_P_E)
        R.r_P_E = r_P_E;
      end
      if nargin > 2 && ~isempty(phi_P_E)
        R.phi_P_E = phi_P_E;
      end
      if nargin > 3 && ~isempty(phiconv_P_E)
        R.phiconv_P_E = phiconv_P_E;
      end
      R.T_P_E = [[eul2r(R.phi_P_E, R.phiconv_P_E), R.r_P_E]; [0 0 0 1]];
    end
    function update_base(R, r_W_0, phi_W_0, phiconv_W_0)
      % Aktualisiere die Transformationsmatrix T_W_0 für die Basis
      % Eingabe:
      % r_W_0: Neuer Vektor vom Welt-KS-Ursprung zum Basis-KS-Ursprung
      % phi_W_0: Neue Euler-Winkel für Drehung vom Welt-KS zum Basis-KS
      % phiconv_W_0: Nummer der Euler-Winkel-Konvention
      if nargin > 1 && ~isempty(r_W_0)
        R.r_W_0 = r_W_0;
      end
      if nargin > 2 && ~isempty(phi_W_0)
        R.phi_W_0 = phi_W_0;
      end
      if nargin > 3 && ~isempty(phiconv_W_0)
        R.phiconv_W_0 = phiconv_W_0;
      end
      R.T_W_0 = [[eul2r(R.phi_W_0, R.phiconv_W_0), R.r_W_0]; [0 0 0 1]];
    end
    function [Jinv_qaD_xD, Jinv_num_voll] = jacobi_qa_x(R, q, xE, platform_frame)
      % Analytische Jacobi-Matrix zwischen Antriebs- und Endeffektorkoord.
      % Eingabe:
      % q: Gelenkkoordinaten
      % xE: EE-Koordinaten (6x1) (nicht: Plattform-Koordinaten) (im Basis-KS)
      % platform_frame: Beziehe Jacobi-Matrix auf Plattform-KS. Dann auch
      % Übergabe von x in Plattform-Koordinaten
      %
      % Ausgabe:
      % Jinv_qaD_xD: Inverse Jacobi-Matrix (Verhältnis Antriebs-Geschw. -
      % Endeffektor-Geschw. mit Euler-Zeitableitung)
      % Jinv_num_voll: Vollständige Jacobi-Matrix bezogen auf alle Gelenke
      
      if nargin == 3, platform_frame = false; end
      
      if R.extfcn_available(1) && nargout ~= 2
        % Berechnung der geometrischen Jacobi-Matrix aus Funktionsaufruf
        if platform_frame, xP = xE;
        else,              xP = R.xE2xP(xE); end
        [qJ, xPred, pkin, koppelP, legFrame] = convert_parameter_class2toolbox(R, q, xP);
        % Aufruf der symbolisch generierten Funktion. Diese enthält den
        % Bezug von Antriebsgeschwindigkeiten zur Plattform-Geschwindigkeit.
        Jinv_qD_sDred = R.jacobi_qa_x_fcnhdl(xPred, qJ, pkin, koppelP, legFrame);
        % Erweitere auf 6FG, falls es sich um eine 2T1R/3T0R PKM handelt
        Jinv_qD_sD = zeros(sum(R.I_qa),6);
        Jinv_qD_sD(:,R.I_EE) = Jinv_qD_sDred;
        % Korrekturmatrix (symbolische Jacobi bezieht sich auf
        % Winkelgeschwindigkeit, numerische auf Euler-Winkel-Zeitableitung)
        % Hier Bezug auf EE-Euler-Winkel-Geschwindigkeit (nicht: Plattform)
        % Notwendig, wenn 3T3R oder wenn T_P_E ungleich Null.
        T = [eye(3,3), zeros(3,3); zeros(3,3), euljac(xE(4:6), R.phiconv_W_E)];
        Jinv_qD_xDvoll = Jinv_qD_sD*T;
        % Reduziere die FG wiede rauf 2T1R o.ä.
        Jinv_qaD_xD = Jinv_qD_xDvoll(:,R.I_EE);
      else % Funktion ist nicht verfügbar. Nehme numerische Berechnung
        if R.I_EE(6) && ~ R.I_EE_Task(6)
          % Aufgabenredundanz: Reduzierte FG entsprechen dem Aufgaben-FG.
          % Nehme vollständige Zwangsbedingungen (6 pro Beinkette)
          [~, Phi_q_voll] = R.constr4grad_q(q);
          [~, Phi_x_voll] = R.constr4grad_x(xE, platform_frame);
          Jinv_num_voll = -Phi_q_voll \ Phi_x_voll(:,R.I_EE);
        else
          % Normalfall: Reduzierte ZB (z.B. bei 2T1R).
          G_q = R.constr4grad_q(q);
          G_x = R.constr4grad_x(xE, platform_frame);
          Jinv_num_voll = -G_q \ G_x;
        end
        % Reduziere auf aktive Gelenke
        Jinv_qaD_xD = Jinv_num_voll(R.I_qa,:);
      end
    end
    function [JinvD_qD_xD,JinvD_num_voll] = jacobiD_qa_x(R, q, qD, xE, xED)
      % Zeitableitung der analytischen Jacobi-Matrix zwischen Antriebs- und Plattformkoord.
      % Eingabe:
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeit
      % xE: EE-Koordinaten (6x1) (nicht: Plattform-Koordinaten) (im Basis-KS)
      % xED: EE-Geschwindigkeit (6x1)
      %
      % Ausgabe:
      % JinvD: Zeitableitung der inverse Jacobi-Matrix (Verhältnis Gelenk-Geschw. -
      % Plattform-Geschw. mit Euler-Zeitableitung)
      G_q = R.constr1grad_q(q, xE);
      G_x = R.constr1grad_x(q, xE);
      GD_q = R.constr1gradD_q(q, qD, xE, xED);
      GD_x = R.constr1gradD_x(q, qD, xE, xED);
      JinvD_num_voll = G_q\(GD_q*(G_q\G_x)) - G_q\GD_x;
      JinvD_qD_xD = JinvD_num_voll(R.I_qa,:);
    end
    function Fx = invdyn_platform(R, q, xP, xPD, xPDD)
      % Inverse Dynamik bezogen auf Plattformkoordinaten
      % q: Gelenkkoordinaten
      % xP: Plattform-Koordinaten (6x1) (nicht: End-Effektor-Koordinaten) (im Basis-KS)
      % xPD: Plattform-Geschwindigkeit (im Basis-KS)
      % xPDD: Plattform-Beschleunigung (im Basis-KS)
      %
      % Ausgabe:
      % Fx: Kraft auf Plattform aus Inverser Dynamik (nicht: Endeffektor)
      xPDred = xPD(R.I_EE);
      xPDDred = xPDD(R.I_EE);
      [qJ, xPred, pkin, koppelP, legFrame, Idp] = convert_parameter_class2toolbox(R, q, xP);
      if R.DynPar.mode == 2
        Fx = R.invdyn_x_fcnhdl2(xPred, xPDred, xPDDred, qJ, R.gravity, legFrame, koppelP, pkin, ...
          R.DynPar.mges(Idp), R.DynPar.mrSges(Idp,:), R.DynPar.Ifges(Idp,:));
      elseif R.DynPar.mode == 3 || R.DynPar.mode == 4
        Fx = R.invdyn_x_fcnhdl4(xPred, xPDred, xPDDred, qJ, R.gravity, legFrame, koppelP, pkin, ...
          R.DynPar.mpv_sym);
      else
        error('Modus %d noch nicht implementiert', R.DynPar.mode);
      end
    end
    function [Fa, Fa_reg] = invdyn2_actjoint(Rob, q, qD, qDD, xP, xDP, xDDP, JinvP)
      % Berechne Antriebskraft aufgrund der Effekte der inversen Dynamik
      % Eingabe:
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeiten
      % qDD: Gelenkbeschleunigungen
      % xP: Plattform-Koordinaten (nicht: Endeffektor)
      % xDP: Plattform-Geschwindigkeit
      % xDDP: Plattform-Beschleunigung
      % JinvP: Inverse Jacobi-Matrix (bezogen auf Plattform-Koordinaten und
      % alle Gelenke). Siehe ParRob/jacobi_qa_x
      %
      % Ausgabe:
      % Fa: Kraft auf Antriebsgelenke (kartesische Momente)
      % Fa_reg: Regressor-Matrix der Kraft
      % Inversdynamik-Kräfte in Endeffektor-Koordinaten berechnen
      if nargout == 1
        Fx = Rob.invdyn2_platform(q, qD, qDD, xP, xDP, xDDP, JinvP);
      else
        [Fx, Fx_reg] = Rob.invdyn2_platform(q, qD, qDD, xP, xDP, xDDP, JinvP);
      end
      % Umrechnen der vollständigen inversen Jacobi
      Jinv_qaD_xD = JinvP(Rob.I_qa,:);
      % Jacobi-Matrix auf Winkelgeschwindigkeiten beziehen. Siehe ParRob/jacobi_qa_x
      if size(Jinv_qaD_xD,2) == 6
        T = [eye(3,3), zeros(3,3); zeros(3,3), euljac(xP(4:6), Rob.phiconv_W_E)];
        Jinv_qaD_sD = Jinv_qaD_xD / T;
      else
        % Nehme an, dass keine räumliche Drehung vorliegt. TODO: Fall 3T2R
        % genauer prüfen, wenn Roboter verfügbar sind.
        Jinv_qaD_sD = Jinv_qaD_xD;
      end
      % Umrechnen auf Antriebskoordinaten. [AbdellatifHei2009], Text nach Gl. (37)
      Fa = Jinv_qaD_sD' \ Fx;
      % Umrechnen der Regressor-Matrix
      if nargout == 2
        Fa_reg = Jinv_qaD_sD' \ Fx_reg;
      end
    end
    function Fx_traj = invdyn_platform_traj(R, Q, XP, XPD, XPDD)
      % Vektor der inversen Dynamik als Trajektorie (Zeit als Zeilen)
      % Eingabe:
      % Q: Gelenkkoordinaten (Trajektorie)
      % XP: Plattform-Koordinaten (Trajektorie)
      % XPD: Plattform-Geschwindigkeit (Trajektorie)
      % XPDD: Plattform-Beschleunigung (Trajektorie)
      %
      % Ausgabe:
      % Fx_traj: Kraft auf Plattform (Inverse Dynamik, als Zeitreihe)
      Fx_traj = NaN(size(Q,1),sum(R.I_EE));
      for i = 1:size(Q,1)
        Fx_traj(i,:) = R.invdyn_platform(Q(i,:)', XP(i,:)', XPD(i,:)', XPDD(i,:)');
      end
    end
    function [Fx_traj,Fx_traj_reg] = invdyn2_platform_traj(R, Q, QD, QDD, XP, XPD, XPDD, JinvP_ges)
      % Inverse Dynamik in Endeffektor-Koordinaten als Trajektorie (Zeit als Zeilen)
      % Eingabe:
      % Q: Gelenkkoordinaten (Trajektorie)
      % QD: Gelenkgeschwindigkeiten (Trajektorie)
      % QDD: Gelenkbeschleunigungen (Trajektorie)
      % xP: Plattform-Koordinaten (nicht: Endeffektor)
      % xDP: Plattform-Geschwindigkeit
      % xDDP: Plattform-Beschleunigung
      % JinvP_ges: Zeilenweise inverse Jacobi-Matrix für alle Gelenke (Traj.)
      % (bezogen auf Plattform-Koordinaten; siehe jacobi_qa_x)
      %
      % Ausgabe:
      % Fx_traj: Kraft auf Plattform (Inverse Dynamik, als Zeitreihe)
      % Fx_traj_reg: Regressormatrizen von Fx_traj (als Zeitreihe)
      Fx_traj = NaN(size(Q,1),sum(R.I_EE));
      if nargout == 2
        if R.DynPar.mode == 3
          Fx_traj_reg = NaN(size(Q,1),sum(R.I_EE)*length(R.DynPar.ipv_n1s));
        else
          Fx_traj_reg = NaN(size(Q,1),sum(R.I_EE)*length(R.DynPar.mpv_n1s));
        end
      end
      for i = 1:size(Q,1)
        Jinv_full = reshape(JinvP_ges(i,:), R.NJ, sum(R.I_EE));
        if nargout < 2
          Fx_traj(i,:) = R.invdyn2_platform(Q(i,:)', QD(i,:)', QDD(i,:)', ...
            XP(i,:)', XPD(i,:)', XPDD(i,:)', Jinv_full);
        else
          [Fx_traj(i,:), Fx_traj_reg_i] = R.invdyn2_platform(Q(i,:)', QD(i,:)', ...
            QDD(i,:)', XP(i,:)', XPD(i,:)', XPDD(i,:)', Jinv_full);
          Fx_traj_reg(i,:) = Fx_traj_reg_i(:);
        end
      end
    end
    function [Fa_traj,Fa_traj_reg] = invdyn2_actjoint_traj(R, Q, QD, QDD, XP, XPD, XPDD, JinvP_ges)
      % Inverse Dynamik in Antriebskoordinaten als Trajektorie (Zeit als Zeilen)
      % Eingabe:
      % Q: Gelenkkoordinaten (Trajektorie)
      % QD: Gelenkgeschwindigkeiten (Trajektorie)
      % QDD: Gelenkbeschleunigungen (Trajektorie)
      % xP: Plattform-Koordinaten (nicht: Endeffektor)
      % xDP: Plattform-Geschwindigkeit
      % xDDP: Plattform-Beschleunigung
      % JinvP_ges: Zeilenweise inverse Jacobi-Matrix für alle Gelenke (Traj.)
      % (bezogen auf Plattform-Koordinaten; siehe jacobi_qa_x)
      %
      % Ausgabe:
      % Fa_traj: Kraft auf Antriebe (Inverse Dynamik, als Zeitreihe)
      % Fa_traj_reg: Regressormatrizen von Fa_traj (als Zeitreihe)
      Fa_traj = NaN(size(Q,1),sum(R.I_qa));
      if nargout == 2
        if R.DynPar.mode == 3
          Fa_traj_reg = NaN(size(Q,1),sum(R.I_qa)*length(R.DynPar.ipv_n1s));
        else
          Fa_traj_reg = NaN(size(Q,1),sum(R.I_qa)*length(R.DynPar.mpv_n1s));
        end
      end
      for i = 1:size(Q,1)
        Jinv_full = reshape(JinvP_ges(i,:), R.NJ, sum(R.I_EE));
        if nargout < 2
          Fa_traj(i,:) = R.invdyn2_actjoint(Q(i,:)', QD(i,:)', QDD(i,:)', ...
            XP(i,:)', XPD(i,:)', XPDD(i,:)', Jinv_full);
        else
          [Fa_traj(i,:), Fa_traj_reg_i] = R.invdyn2_actjoint(Q(i,:)', QD(i,:)', ...
            QDD(i,:)', XP(i,:)', XPD(i,:)', XPDD(i,:)', Jinv_full);
          Fa_traj_reg(i,:) = Fa_traj_reg_i(:);
        end
      end
    end
    function Fx_traj = invdyn3_platform_traj(R, Fx_traj_reg)
      % Vektor der inversen Dynamik (EE-Koordinaten) als Trajektorie (Zeit als Zeilen)
      % Benutzt eine eigene Funktion für die Inverse Dynamik der Traj.
      % Eingabe:
      % RV: Regressormatrizen (als Zeitreihe); aus invdyn2_platform_traj
      %
      % Ausgabe:
      % Fx_traj: Inverse Dynamik (als Zeitreihe)
      Fx_traj = NaN(size(Fx_traj_reg,1),sum(R.I_EE));
      if R.DynPar.mode == 3
        ndimpar = length(R.DynPar.ipv_n1s);
        dpv = R.DynPar.ipv_n1s;
      else
        ndimpar = length(R.DynPar.mpv_n1s);
        dpv = R.DynPar.mpv_n1s;
      end
      for i = 1:size(Fx_traj_reg,1)
        Fx_traj_reg_i = reshape(Fx_traj_reg(i,:), sum(R.I_EE), ndimpar);
        Fx_traj(i,:) = Fx_traj_reg_i*dpv;
      end
    end
    function Fa_traj = invdyn3_actjoint_traj(R, Fa_traj_reg)
      % Vektor der inversen Dynamik (Antriebskoordinaten) als Trajektorie (Zeit als Zeilen)
      % Benutzt eine eigene Funktion für die Inverse Dynamik der Traj.
      % Eingabe:
      % RV: Regressormatrizen (als Zeitreihe); aus invdyn2_actjoint_traj
      %
      % Ausgabe:
      % Fa_traj: Inverse Dynamik (als Zeitreihe)
      Fa_traj = NaN(size(Fa_traj_reg,1),sum(R.I_qa));
      if R.DynPar.mode == 3
        ndimpar = length(R.DynPar.ipv_n1s);
        dpv = R.DynPar.ipv_n1s;
      else
        ndimpar = length(R.DynPar.mpv_n1s);
        dpv = R.DynPar.mpv_n1s;
      end
      for i = 1:size(Fa_traj_reg,1)
        Fa_traj_reg_i = reshape(Fa_traj_reg(i,:), sum(R.I_qa), ndimpar);
        Fa_traj(i,:) = Fa_traj_reg_i*dpv;
      end
    end
    function Mx = inertia_platform(R, q, xP)
      % Massenmatrix bezogen auf Plattformkoordinaten
      % q: Gelenkkoordinaten
      % xP: Plattform-Koordinaten (6x1) (nicht: End-Effektor-Koordinaten) (im Basis-KS)
      %
      % Ausgabe:
      % Mx: Massenmatrix
      [qJ, xPred, pkin, koppelP, legFrame, Idp] = convert_parameter_class2toolbox(R, q, xP);
      if R.DynPar.mode == 2
        Mx = R.inertia_x_fcnhdl2(xPred, qJ, legFrame, koppelP, pkin, ...
          R.DynPar.mges(Idp), R.DynPar.mrSges(Idp,:), R.DynPar.Ifges(Idp,:));
      elseif R.DynPar.mode == 3 || R.DynPar.mode == 4
        % Benutze immer Modus 4 (Minimalparameter), auch wenn Inertial-
        % parameter velangt ist
        Mx = R.inertia_x_fcnhdl4(xPred, qJ, legFrame, koppelP, pkin, ...
          R.DynPar.mpv_sym);
      else
        error('Modus %d noch nicht implementiert', R.DynPar.mode);
      end
    end
    function Fgx = gravload_platform(R, q, xP)
      % Gravitationskraft bezogen auf Plattformkoordinaten
      % q: Gelenkkoordinaten
      % xP: Plattform-Koordinaten (6x1) (nicht: End-Effektor-Koordinaten) (im Basis-KS)
      %
      % Ausgabe:
      % Fgx: Gravitationskräfte (bezogen auf EE-Koordinaten der PKM)
      [qJ, xPred, pkin, koppelP, legFrame, Idp] = convert_parameter_class2toolbox(R, q, xP);
      if R.DynPar.mode == 2
        Fgx = R.gravload_x_fcnhdl2(xPred, qJ, R.gravity, legFrame, koppelP, pkin, ...
          R.DynPar.mges(Idp), R.DynPar.mrSges(Idp,:));
      elseif R.DynPar.mode == 3 || R.DynPar.mode == 4
        Fgx = R.gravload_x_fcnhdl4(xPred, qJ, R.gravity, legFrame, koppelP, pkin, ...
          R.DynPar.mpv_sym);
      else
        error('Modus %d noch nicht implementiert', R.DynPar.mode);
      end
    end
    function Fcx = coriolisvec_platform(R, q, xP, xPD)
      % Corioliskraft bezogen auf Plattformkoordinaten
      % q: Gelenkkoordinaten
      % xP: Plattform-Koordinaten (6x1) (nicht: End-Effektor-Koordinaten) (im Basis-KS)
      % xPD: Plattform-Geschwindigkeit (im Basis-KS)
      %
      % Ausgabe:
      % Fcx: Coriolis-Kräfte (bezogen auf EE-Koordinaten der PKM)
      xPDred = xPD(R.I_EE);
      [qJ, xPred, pkin, koppelP, legFrame, Idp] = convert_parameter_class2toolbox(R, q, xP);
      if R.DynPar.mode == 2
        Fcx = R.coriolisvec_x_fcnhdl2(xPred, xPDred, qJ, legFrame, koppelP, pkin, ...
          R.DynPar.mges(Idp), R.DynPar.mrSges(Idp,:), R.DynPar.Ifges(Idp,:));
      elseif R.DynPar.mode == 3 || R.DynPar.mode == 4
        Fcx = R.coriolisvec_x_fcnhdl4(xPred, xPDred, qJ, legFrame, koppelP, pkin, ...
          R.DynPar.mpv_sym);
      else
        error('Modus %d noch nicht implementiert', R.DynPar.mode);
      end
    end
    function [T_ges, T_legs, T_plattform] = ekin(R, q, qD, xP, xPD)
      % Kinetische Energie der PKM
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeiten
      % xP: Plattform-Koordinaten (6x1) (nicht: End-Effektor-Koordinaten) (im Basis-KS)
      % xPD: Plattform-Geschwindigkeit (im Basis-KS)
      %
      % Ausgabe:
      % T_ges: Kinetische Energie der PKM (Beine und Plattform)
      % T_legs: Kinetische Energie der Beine
      % T_plattform: Kinetische Energie der Plattform
      if R.phiconv_W_E ~= 2
        error('Für Winkelkonvention %d nicht definiert', R.phiconv_W_E);
      end
      T_legs = NaN(R.NLEG,1);
      T_plattform = rigidbody_energykin_floatb_eulxyz_slag_vp2(xP(1:3), xPD, ...
        R.DynPar.mges(end), R.DynPar.mrSges(end,:), R.DynPar.Ifges(end,:));
      for i = 1:R.NLEG
        I = R.I1J_LEG(i):R.I2J_LEG(i);
        T_legs(i) = R.Leg(i).ekin(q(I), qD(I));
      end
      T_ges = T_plattform + sum(T_legs);
    end
    function [U_ges, U_legs, U_plattform] = epot(R, q, xP)
      % Potentielle Energie der PKM
      % q: Gelenkkoordinaten
      % xP: Plattform-Koordinaten (6x1) (nicht: End-Effektor-Koordinaten) (im Basis-KS)
      %
      % Ausgabe:
      % U_ges: Potentielle Energie der PKM (Beine und Plattform)
      % U_legs: Potentielle Energie der Beine
      % U_plattform: Potentielle Energie der Plattform
      if R.phiconv_W_E ~= 2
        error('Für Winkelkonvention %d nicht definiert', R.phiconv_W_E);
      end
      U_legs = NaN(R.NLEG,1);
      U_plattform = rigidbody_energypot_floatb_eulxyz_slag_vp2(xP(1:3), xP(4:6), R.gravity, ...
        R.DynPar.mges(end), R.DynPar.mrSges(end,:));
      for i = 1:R.NLEG
        I = R.I1J_LEG(i):R.I2J_LEG(i);
        U_legs(i) = R.Leg(i).epot(q(I));
      end
      U_ges = U_plattform + sum(U_legs);
    end   
    function w_all = internforce3(R, Fintreg, tau_add)
      % Berechne interne Schnittkräfte basierend auf gegebenem Regressor
      % Eingabe
      % Fintreg: Regressor der Schnittkräfte, aus internforce_regmat
      % (egal, ob w_all_linkframe_reg oder w_all_baseframe_reg)
      % tau_add: Vektor zusätzlicher Gelenkmomente. Dient als Umschalter
      % zur Berechnung der Schnittkräfte ohne Dynamik, mit Zusatz-Momenten
      % 
      % Ausgabe:
      % w_all: Alle Schnittkräfte und -momente
      % Indizes: Wie ParRob.internforce
      % 1: Kräfte, dann Momente (immer 6 Einträge)
      % 2: Segmente der Beine
      % 3: Einzelne Beinketten der PKM
      w_all = NaN(6, R.Leg(1).NL, R.NLEG);
      if nargin < 3
        w_all_stack = Fintreg * R.DynPar.ipv_n1s;
      else
        w_all_stack = Fintreg * tau_add;
      end
      for i = 1:R.NLEG
        w_all_stack_i = w_all_stack(6*R.Leg(1).NL*(i-1)+1 : R.Leg(1).NL*6*i);
        w_all(:,:,i) = [reshape(w_all_stack_i(1:R.Leg(1).NL*3), 3, R.Leg(1).NL); ...
                        reshape(w_all_stack_i(R.Leg(1).NL*3+1:end), 3, R.Leg(1).NL)];
      end
    end
    function [Regmatl_traj, Regmat0_traj] = internforce_regmat_traj(R, ...
        Q, QD, QDD, XP, XPD, XPDD, JinvP_ges, TAU_add)
      % Zeitreihe der Schnittkraft-Dynamik-Regressor-Matrix (Trajektorie)
      % Eingabe:
      % Q: Gelenkkoordinaten (Trajektorie)
      % QD: Gelenkgeschwindigkeiten (Trajektorie)
      % QDD: Gelenkbeschleunigungen (Trajektorie)
      % XP: Plattform-Koordinaten (nicht: Endeffektor) (Trajektorie)
      % XPD: Endeffektor-Geschwindigkeit (Trajektorie)
      % XPDD: Endeffektor-Beschleunigung (Trajektorie)
      % JinvP_ges: Zeilenweise inverse Jacobi-Matrix für alle Gelenke (Traj.)
      % (siehe jacobi_qa_x). Hier Bezug auf Plattform-KS und Euler-Winkel
      % TAU_add: Zusätzliche Gelenkmomente (als Umschalter); Trajektorie.
      %
      % Ausgabe:
      % Regmatl_traj: Regressormatrizen (als Zeitreihe; zeilenweise)
      % (bezogen auf Schnittkräfte im Körper-KS)
      % Regmat0_traj: Bezogen auf Schnittkräfte im PKM-Basis-KS
      if nargin < 9
        % Keine zusätzlichen Momente gegeben. Regressor der inversen Dynamik
        % bezüglich der Dynamik-Parameter (Inertialparameter)
        Regmatl_traj = NaN(size(Q,1), 6*R.Leg(1).NL*R.NLEG*length(R.DynPar.ipv_n1s));
      else
        % Zusätzliche Momente gegeben. Regressor bezüglich der Gelenkmomente.
        % Nicht bezüglich Dynamik-Parameter. Also auch keine Inversdynamik-Effekte.
        Regmatl_traj = NaN(size(Q,1), 6*R.Leg(1).NL*R.NLEG*size(TAU_add,2));
      end
      if nargout == 2
        Regmat0_traj = Regmatl_traj;
      end
      for i = 1:size(Q,1)
        if nargin < 8
          [~,Jinv_full] = R.jacobi_qa_x(Q(i,:)', XP(i,:)');
        else
          Jinv_full = reshape(JinvP_ges(i,:), R.NJ, sum(R.I_EE));
        end
        if nargout == 2 % auch Kräfte im Basis-KS fordern.
          % Übergebe den optionalen Parameter, je nachdem ob er da ist.
          if nargin < 9
            [~, w_all_linkframe_reg, w_all_baseframe_reg] = R.internforce_regmat(Q(i,:)', QD(i,:)', ...
              QDD(i,:)', XP(i,:)', XPD(i,:)', XPDD(i,:)', Jinv_full);
          else
            [~, w_all_linkframe_reg, w_all_baseframe_reg] = R.internforce_regmat(Q(i,:)', QD(i,:)', ...
              QDD(i,:)', XP(i,:)', XPD(i,:)', XPDD(i,:)', Jinv_full, TAU_add(i,:)');
          end
          Regmat0_traj(i,:) = w_all_baseframe_reg(:);
        else % nur eine Ausgabe. Keine Schnittkräfte im Basis-KS anfordern (einfachere Rechnung).
          if nargin < 9
            [~, w_all_linkframe_reg] = R.internforce_regmat(Q(i,:)', QD(i,:)', ...
              QDD(i,:)', XP(i,:)', XPD(i,:)', XPDD(i,:)', Jinv_full);
          else
            [~, w_all_linkframe_reg] = R.internforce_regmat(Q(i,:)', QD(i,:)', ...
              QDD(i,:)', XP(i,:)', XPD(i,:)', XPDD(i,:)', Jinv_full, TAU_add(i,:)');
          end
        end
        Regmatl_traj(i,:) = w_all_linkframe_reg(:);
      end
    end
    function [FLegl_t, FLeg0_t] = internforce_traj(R, Q, QD, QDD, TAU, TAU_add)
      % Schnittkräfte der inversen Dynamik als Trajektorie (Zeit als Zeilen)
      % Eingabe:
      % Q: Gelenkkoordinaten (Trajektorie)
      % QD: Gelenkgeschwindigkeiten (Trajektorie)
      % QDD: Gelenkbeschleunigungen (Trajektorie)
      % TAU: Antriebskräfte
      % TAU_add: Zusätzliche Gelenkmomente (als Umschalter); Trajektorie.
      %
      % Ausgabe:
      % FLegl_t: Schnittkräfte und -momente (Inverse Dynamik, als Zeitreihe)
      % Indizes: 1: Einträge für Kraft und Moment in allen Segmenten
      %          2: Beinketten
      %          3: Zeit
      % FLeg0_t: Genauso, aber Schnittkräfte nicht im Körper-KS, sondern im
      % PKM-Basis-KS angegeben
      
      % Initialisierung
      FLegl_t = NaN(R.NLEG, 6*R.Leg(1).NL, size(Q,1));
      if nargout == 2
        FLeg0_t = FLegl_t;
      end
      for i = 1:size(Q,1)
        % Aufruf der Funktion und Modus der Berechnung je nachdem, ob zusätzliche Gelenkmomente gegeben.
        if nargout == 2 % vollständiger Aufruf (mehr Berechnungen)
          if nargin < 6
            [~,cf_w_all_linkframe,cf_w_all_baseframe] = R.internforce(Q(i,:)', QD(i,:)', QDD(i,:)', TAU(i,:)');
          else
            [~,cf_w_all_linkframe,cf_w_all_baseframe] = R.internforce(Q(i,:)', QD(i,:)', QDD(i,:)', TAU(i,:)', TAU_add(i,:)');
          end
        else % einfacher Aufruf
          if nargin < 6
            [~,cf_w_all_linkframe] = R.internforce(Q(i,:)', QD(i,:)', QDD(i,:)', TAU(i,:)');
          else
            [~,cf_w_all_linkframe] = R.internforce(Q(i,:)', QD(i,:)', QDD(i,:)', TAU(i,:)', TAU_add(i,:)');
          end
        end
        for j = 1:R.NLEG
          W_j_l = cf_w_all_linkframe(:,:,j);
          W_j_l_stack = [reshape(W_j_l(1:3,:), 3*R.Leg(1).NL, 1); ... % erst alle Kräfte
                         reshape(W_j_l(4:6,:), 3*R.Leg(1).NL, 1)]; % dann alle Momente
          FLegl_t(j,:,i) = W_j_l_stack;
          if nargout == 2
            W_j_0 = cf_w_all_baseframe(:,:,j);
            W_j_0_stack = [reshape(W_j_0(1:3,:), 3*R.Leg(1).NL, 1); ... % erst alle Kräfte
                           reshape(W_j_0(4:6,:), 3*R.Leg(1).NL, 1)]; % dann alle Momente
            FLeg0_t(j,:,i) = W_j_0_stack;
          end
        end
      end
    end
    function FLeg0_t = internforce3_traj(R, Regmat_traj, TAU_add)
      % Schnittkräfte der inversen Dynamik als Trajektorie (Zeit als Zeilen)
      % Eingabe:
      % Regmat_traj: Regressor der Schnittkräfte, aus internforce_regmat
      %              (Zeitschritte als Zeilen, Spalten je nachdem, welcher Regressor-Modus)
      % TAU_add: Zusätzliche Gelenkmomente (als Umschalter für Regressor-Modus); Trajektorie.
      % 
      % Ausgabe:
      % FLeg0_t: Alle Schnittkräfte und -momente
      % Indizes: Wie in ParRob.internforce3
      FLeg0_t = NaN(R.NLEG, 6*R.Leg(1).NL, size(Regmat_traj,1)); % Schnittkräfte in allen Gelenken (Basis-KS)
      for i = 1:size(Regmat_traj,1)
        if nargin < 3
          % Normaler Modus: Regressor bezüglich Inertialparameter
          w_all_linkframe_reg_i = reshape(Regmat_traj(i,:), ...
            6*R.Leg(1).NL*R.NLEG, length(R.DynPar.ipv_n1s));
          cf_w_all_linkframe = R.internforce3(w_all_linkframe_reg_i);
        else
          % Regressor bezüglich Gelenkmomente (z.B. Reibung, Gelenkfeder)
          w_all_linkframe_reg_i = reshape(Regmat_traj(i,:), ...
            6*R.Leg(1).NL*R.NLEG, size(TAU_add,2));
          cf_w_all_linkframe = R.internforce3(w_all_linkframe_reg_i, TAU_add(i,:)');
        end
        
        for j = 1:R.NLEG
          % Umrechnung in anderes Format
          W_j_l = cf_w_all_linkframe(:,:,j);
          W_j_l_stack = [reshape(W_j_l(1:3,:), 3*R.Leg(1).NL, 1); ... % erst alle Kräfte
                         reshape(W_j_l(4:6,:), 3*R.Leg(1).NL, 1)]; % dann alle Momente
          FLeg0_t(j,:,i) = W_j_l_stack;
        end
      end
    end
    function [U_ges, U_legs] = epotspring(R, q)
      % Potentielle Energie von in Gelenken angebrachten Drehfedern
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % U_ges: Potentielle Energie aller Gelenk-Federn der gesamten PKM
      % U_legs: Potentielle Energie der der Federn in jedem Bein einzeln.
      U_legs = NaN(R.NLEG,1);
      for i = 1:R.NLEG
        I = R.I1J_LEG(i):R.I2J_LEG(i);
        U_legs(i) = R.Leg(i).epotspring(q(I));
      end
      U_ges = sum(U_legs);
    end
    function tau = springtorque(R, q)
      % Gestapeltes Gelenkmoment aller Drehfedern in den Gelenken
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % tau: Gestapeltes Gelenkmoment (nicht auf PKM bezogen).
      tau = NaN(R.NJ,1);
      for i = 1:R.NLEG
        q_i = q(R.I1J_LEG(i):R.I2J_LEG(i));
        tau_i = R.Leg(i).springtorque(q_i);
        tau(R.I1J_LEG(i):R.I2J_LEG(i)) = tau_i;
      end
    end
    function TAU = springtorque_traj(R, Q)
      % Gestapeltes Gelenkmoment aller Drehfedern in den Gelenken als
      % Zeitreihe
      % Eingabe:
      % Q: Gelenkkoordinaten als Zeitreihe (Zeile: Zeit)
      %
      % Ausgabe:
      % TAU: Zeitreihe der Gelenkmomente
      TAU = NaN(size(Q,1), R.NJ);
      for i = 1:size(Q,1)
        TAU(i,:) = R.springtorque(Q(i,:)');
      end
    end
    function [Fa, Fa_reg] = jointtorque_actjoint(R, q, xP, tau, JinvP)
      % Berechne Antriebskraft aufgrund von Gelenkmomenten (unabhängig
      % von inverser Dynamik). Z.B. Gelenkfeder oder Reibung.
      % Eingabe:
      % q: Gelenkkoordinaten
      % xP: Plattform-Koordinaten (nicht: Endeffektor)
      % tau: Gelenkmomente in den Gelenken der Beinketten
      % JinvP: Inverse Jacobi-Matrix (bezogen auf Plattform-Koordinaten und
      % alle Gelenke). Siehe ParRob/jacobi_qa_x
      %
      % Ausgabe:
      % Fa: Kraft auf Antriebsgelenke (kartesische Momente)
      % Fa_reg: Regressor-Matrix der Kraft (bezogen auf Gelenke)
      % 
      % Siehe auch: invdyn2_actjoint
      if nargout == 1 % keine Regressorform
        Fx = R.jointtorque_platform(q, xP, tau, JinvP);
      else % mit Regressorform
        [Fx, Fx_reg] = R.jointtorque_platform(q, xP, tau, JinvP);
      end
      % Umrechnen der vollständigen inversen Jacobi
      Jinv_qaD_xD = JinvP(R.I_qa,:);
      % Jacobi-Matrix auf Winkelgeschwindigkeiten beziehen. Siehe ParRob/jacobi_qa_x
      if size(Jinv_qaD_xD,2) == 6
        T = [eye(3,3), zeros(3,3); zeros(3,3), euljac(xP(4:6), R.phiconv_W_E)];
        Jinv_qaD_sD = Jinv_qaD_xD / T;
      else
        % Nehme an, dass keine räumliche Drehung vorliegt. TODO: Fall 3T2R
        % genauer prüfen, wenn Roboter verfügbar sind.
        Jinv_qaD_sD = Jinv_qaD_xD;
      end
      % Umrechnen auf Antriebskoordinaten. [AbdellatifHei2009], Text nach Gl. (37)
      Fa = Jinv_qaD_sD' \ Fx;
      % Umrechnen der Regressor-Matrix
      if nargout == 2
        Fa_reg = Jinv_qaD_sD' \ Fx_reg;
      end
    end
    function [Fx_traj,Fx_traj_reg] = jointtorque_platform_traj(R, Q, XP, TAU, JinvP_ges)
      % Plattform-Kraft aufgrund von Gelenkmomenten als Trajektorie (Zeit als Zeilen)
      % Eingabe:
      % Q: Gelenkkoordinaten (Trajektorie)
      % XP: Plattform-Koordinaten (nicht: Endeffektor), Trajektorie
      % TAU: Gelenkmomente (Trajektorie)
      % JinvP_ges: Zeilenweise inverse Jacobi-Matrix für alle Gelenke (Traj.)
      % (bezogen auf Plattform-Koordinaten; siehe jacobi_qa_x)
      %
      % Ausgabe:
      % Fx_traj: äquivalente Kraft auf Plattform (Inverse Dynamik, als Zeitreihe)
      % Fx_traj_reg: Regressormatrizen von Fx_traj (als Zeitreihe)
      Fx_traj = NaN(size(Q,1),sum(R.I_EE));
      if nargout == 2
        Fx_traj_reg = NaN(size(Q,1), size(JinvP_ges,2));
      end
      for i = 1:size(Q,1)
        Jinv_full = reshape(JinvP_ges(i,:), R.NJ, sum(R.I_EE));
        if nargout < 2 % ohne Regressorform
          Fx_traj(i,:) = R.jointtorque_platform(Q(i,:)', XP(i,:)', TAU(i,:)', Jinv_full);
        else % mit Regressorform
          [Fx_traj(i,:),Fx_traj_reg_i] = R.jointtorque_platform(Q(i,:)', XP(i,:)', TAU(i,:)', Jinv_full);
          Fx_traj_reg(i,:) = Fx_traj_reg_i(:);
        end
      end
    end
    function [Fa_traj, Fa_traj_reg] = jointtorque_actjoint_traj(R, Q, XP, TAU, JinvP_ges)
      % Antriebskräfte für gegebene Gelenkmomente als Trajektorie (Zeit als Zeilen)
      % Eingabe:
      % Q: Gelenkkoordinaten (Trajektorie)
      % XP: Plattform-Koordinaten (nicht: Endeffektor); als Trajektorie
      % TAU: Trajektorie der Gelenkmomente
      % JinvP_ges: Zeilenweise inverse Jacobi-Matrix für alle Gelenke (Traj.)
      % (bezogen auf Plattform-Koordinaten; siehe jacobi_qa_x)
      %
      % Ausgabe:
      % Fa_traj: Äquivalente Kraft auf Antriebe (Inverse Dynamik, als Zeitreihe)
      % Fa_traj_reg: Regressormatrizen von Fa_traj (als Zeitreihe)
      %
      % Siehe auch: invdyn2_actjoint_traj
      Fa_traj = NaN(size(Q,1),sum(R.I_qa));
      if nargout == 2
        Fa_traj_reg = NaN(size(Q,1), size(JinvP_ges,2));
      end
      for i = 1:size(Q,1)
        Jinv_full = reshape(JinvP_ges(i,:), R.NJ, sum(R.I_EE));
        if nargout < 2
          Fa_traj(i,:) = R.jointtorque_actjoint(Q(i,:)', XP(i,:)', TAU(i,:)', Jinv_full);
        else
          [Fa_traj(i,:), Fa_traj_reg_i] = R.jointtorque_actjoint(Q(i,:)', XP(i,:)', TAU(i,:)', Jinv_full);
          Fa_traj_reg(i,:) = Fa_traj_reg_i(:);
        end
      end
    end
    function Fx_traj = jointtorque2_platform_traj(R, Fx_traj_reg, TAU)
      % Vektor der Plattform-Kraft aus Gelenkkräften (EE-Koordinaten) als
      % Trajektorie (Zeit als Zeilen)
      % Eingabe:
      % Fx_traj_reg: Regressormatrizen (als Zeitreihe); aus jointtorque_platform_traj
      % TAU: Zeitreihe von Gelenkmomenten
      %
      % Ausgabe:
      % Fx_traj: Äquivalente Plattformkraft zu Eingabe TAU
      Fx_traj = NaN(size(Fx_traj_reg,1),sum(R.I_EE));
      for i = 1:size(Fx_traj_reg,1)
        tau_i = TAU(i,:)';
        Fx_traj_reg_i = reshape(Fx_traj_reg(i,:), sum(R.I_EE), R.NJ);
        Fx_traj(i,:) = Fx_traj_reg_i*tau_i;
      end
    end
    function Fa_traj = jointtorque2_actjoint_traj(R, Fa_traj_reg, TAU)
      % Vektor der Antriebskräfte aus Gelenkkräften als Trajektorie
      % Eingabe:
      % Fa_traj_reg: Regressormatrizen (als Zeitreihe); aus jointtorque_actjoint_traj
      % TAU: Zeitreihe von Gelenkmomenten
      %
      % Ausgabe:
      % Fa_traj: Antriebskräfte zur Kompensation von TAU (als Zeitreihe)
      % Identische Berechnung wie für Plattform. Nur eigene Funktion zur
      % besseren Nachvollziehbarkeit beim Aufruf der Funktion.
      Fa_traj = R.jointtorque2_platform_traj(Fa_traj_reg, TAU);
    end
    function update_mdh_legs(R, pkin)
      % Aktualisiere die Kinematik-Parameter aller Beinketten
      % Nehme eine symmetrische PKM an
      % Eingabe:
      % pkin (Kinematikparameter der Beine)
      for i = 1:R.NLEG
        R.Leg(i).update_mdh(pkin);
      end
    end
    function update_EE_FG(R, I_EE, I_EE_Task, I_EE_Legs)
      % Aktualisiere die Freiheitsgrade des Endeffektors
      % Eingabe:
      % I_EE [1x6] logical; Belegung der EE-FG (frei oder blockiert)
      % I_EE_Task [1x6] logical; Belegung der EE-FG der Aufgabe
      % I_EE_Legs [NLEGx6] EE-FG der einzelnen Beinketten
      R.I_EE = logical(I_EE);
      if nargin < 3
        I_EE_Task = I_EE;
      end
      if nargin < 4
        % Bestimme die Beinketten-Freiheitsgrade (und damit Zwangsbedingungen)
        %  anhand des Falls der Plattform-FG und der Gelenkzahl der Beinkette.
        I_EE_Legs = false(R.NLEG,6); % Initialisierung
        Leg_NQJ = cat(1,R.Leg.NQJ);
        if all(I_EE == [1 1 0 0 0 1]) && all(I_EE_Task == [1 1 0 0 0 0])
          % 2T1R (planar) mit Aufgabenredundanz (Rotation egal)
          % Annahme: Alle Beinketten sind auch 2T1R (planar) und haben
          % keine FG außerhalb der xy-Ebene der Basis
          I_EE_Legs(1,:) = I_EE_Task; % Führungskette 2T0R
          I_EE_Legs(2:end,:) = repmat(I_EE,size(I_EE_Legs,1)-1,1); % Folgeketten 2T1R
        elseif all(I_EE == [1 1 0 0 0 0]) && all(I_EE_Task == [1 1 0 0 0 0])
          % 2T0R (planar). Rotation fixiert. Ist kein relevanter Fall, wird
          % aber in einem Beispiel der Vollständigkeit halber benutzt.
          I_EE_Legs = repmat(I_EE,size(I_EE_Legs,1),1);
        elseif all(I_EE == [1 1 0 0 0 1]) % 2T1R (planar), Normaler Fall
          % Annahme: Alle Beinketten sind auch 2T1R (planar) und haben
          % keine FG außerhalb der xy-Ebene der Basis
          I_EE_Legs = logical(repmat(logical(I_EE),R.NLEG,1));
        elseif all(I_EE == [1 1 1 0 0 1]) && all(I_EE_Task == [1 1 1 0 0 0])
          % 3T1R (ebene Drehung, räumliche Bewegung) mit Aufgabenredundanz
          % (Rotation egal). Annahme: Alle Beinketten sind räumlich und
          % haben beliebige  Freiheitsgrade im Raum (auch Rotationen). 
          % Überbestimmte ZB (s.u.)
          I_EE_Legs(1,:) = [1 1 1 1 1 0]; % Führungskette 3T2R; bezogen auf EE-z-Achse
          I_EE_Legs(2:end,:) = repmat([1 1 1 1 1 1],size(I_EE_Legs,1)-1,1); % Folgeketten 3T3R
          % Entferne die Komponente für die X-Rotation. Annahme: Die
          % Beinketten müssen 5 Gelenke haben, sonst überbestimmtes LGS.
          % In der Implementierung wird nicht X weggelassen, sondern die
          % Betragssumme aus X und Y gebildet.
          I_EE_Legs(Leg_NQJ<6,4) = false;
          % Falls die Beinkette nur 4FG hat, werden nur 4 ZB gesetzt. Wird
          % einer der ZB für die erste Beinkette verletzt, wird das evtl
          % nicht direkt erkannt. Dann geht aber sowieso die IK nicht für
          % die folgenden Beinketten. Annahme ist, dass die PKM-FG stimmen
          I_EE_Legs(Leg_NQJ==4,5) = false; % Y-FG entfernen
        elseif all(I_EE == [1 1 1 0 0 1]) % 3T1R-PKM ohne Aufgabenredundanz
          % Bei 3T1R wird die Annahme getroffen, dass die Beinkette für die
          % Rotations-FG überbestimmt ist und eine ZB weggelassen werden
          % kann (bzw. die Betragssumme beider ZB gebildet werden kann.)
          I_EE_Legs(:) = true;
          % Überbestimmtheit nur, wenn weniger als 6 Gelenk-FG in Beinkette
          I_EE_Legs(Leg_NQJ<6,4) = false;
          % 3T1R-Beinkette für 3T1R-PKM. FG müssen übereinstimmen. Sonst
          % geht es sowieso nicht.
          I_EE_Legs(Leg_NQJ==4,5) = false; % Y-FG entfernen
        elseif all(I_EE == [1 1 1 0 0 0])
          % Bei 3T0R wird bei Beinketten immer volle 3T3R-Sollvorgabe
          % gegeben. Dadurch entstehen überbestimmte Zwangsbedingungen
          % (mehr Zwangsbedingungen als Gelenke für die Beinketten). Ist in
          % InvKin mit Pseudo-Inverse lösbar. Im Gegenzug zu 3T1R ist hier
          % nicht genau klar, welche der Rotations-ZB zusammengefasst werden sollten.
          % Außerdem gibt es keine Aufgabenredundanz, bei der ein unterbestimmtes Gleichungssystem benötigt wird.
          I_EE_Legs(:) = true;
        elseif all(I_EE == [1 1 1 1 1 0])
          % 3T2R-PKM (strukturell) benutze constr3-Methode.
          I_EE_Legs(1,:) = I_EE; % Führungsbeinkette hat 3T2R-FG
          % Folge-Beinketten: Setze vollständige FG. Bei Aufgabenredundanz
          % oder bei normalen 3T2R-PKM muss die Orientierung der Folge- 
          % ketten vollständig vorgegeben werden
          I_EE_Legs(2:end,:) = true;
        elseif all(I_EE == [1 1 1 1 1 1]) && all(I_EE_Task == [1 1 1 1 1 0])
          I_EE_Legs(1,:) = I_EE_Task; % Führungskette 3T2R
          I_EE_Legs(2:end,:) = true; % Folgeketten 3T3R
        elseif all(I_EE == [1 1 1 1 1 1])
          % 3T3R-PKM: Volle Vorgabe für alle Beinketten
          I_EE_Legs(:) = true;
        else
          error('Fall noch nicht vorgesehen');
        end
      end
      R.I_EE_Task = logical(I_EE_Task);
      
      % Setze die Aufgaben-FG der PKM-Beinketten
      for i = 1:R.NLEG
        R.Leg(i).I_EE_Task = I_EE_Legs(i,:);
      end
      
      % Anzahl der kinematischen Zwangsbedingungen der Beinketten
      % feststellen. Annahme: Beinketten haben nicht die selben FG wie
      % Plattform (bei überbestimmten PKM). Wurde oben festgelegt.
      % Indizes der relevanten Zwangsbedingungen daraus ableiten.
      % Werden benötigt für Ausgabe von constr1
      R.I_constr_t = zeros(1,0); % Initialisierung 1x0 für mex-Eingangsprüfung
      R.I_constr_r = zeros(1,0);
      R.I_constr_red = zeros(1,0);
      R.I_constr_t_red = zeros(1,0);
      R.I_constr_r_red = zeros(1,0);
      
      R.I1constr = zeros(1,0);
      R.I2constr = zeros(1,0);
      R.I1constr_red = zeros(1,0);
      R.I2constr_red = zeros(1,0);
      
      i_red = 1; % Lauf-Index für I_constr_red (jew. erster Eintrag pro Bein)
      i_tred = 1; i_rred = 1; % Lauf-Index für I_constr_t_red und I_constr_r_red
      ii_tred = 1; % Lauf-Index für ersten Eintrag in Gesamt-ZB für aktuelles Bein
      for i = 1:R.NLEG
        % Zähle Zwangsbedingungen für das Bein (werden in I_EE_Task der
        % Beinkette abgelegt)
        nPhit = sum(R.Leg(i).I_EE_Task(1:3));
        nPhir = sum(R.Leg(i).I_EE_Task(4:6));
        nPhi = nPhit + nPhir;
        
        % Start- und End-Indizes belegen ...
        R.I1constr(i) = (i-1)*6+1; % ... bezogen auf alle ZB (immer 6)
        R.I2constr(i) = (i)*6;
        R.I1constr_red(i) = ii_tred; % ... bezogen auf red. ZB (untersch. Zahl)
        R.I2constr_red(i) = ii_tred+nPhi-1;
       
        % Indizes bestimmen
        R.I_constr_t(3*(i-1)+1:3*i) = (i-1)*6+1:(i)*6-3;
        R.I_constr_r(3*(i-1)+1:3*i) = (i-1)*6+1+3:(i)*6;
        R.I_constr_t_red(i_tred:i_tred+nPhit-1) = ii_tred:ii_tred+nPhit-1;
        R.I_constr_r_red(i_rred:i_rred+nPhir-1) = ii_tred+nPhit:ii_tred+nPhi -1;

        % Indizes der reduzierten ZB bestimmen
        if i == 1 && (R.I_EE_Task(6) == false && R.I_EE(6) == true || ...
            all(R.I_EE == [1 1 1 1 1 0])) % Strukturell 3T2R-FG
          % Aufgabenredundanz mit freiem Rotations-Freiheitsgrad oder erste
          % Beinkette in 3T2R-PKM (strukturell).
          % Führungskette für 3T2R anders: Reziproke Euler-Winkel. Setzt
          % Wahl von constr3 oder constr2 voraus.
          if all(R.I_EE_Task == logical([1 1 1 1 1 0]))
            % Wähle bei Rotation nur die Y- und Z-Komponente
            R.I_constr_red(i_red:i_red+nPhi-1) = [1 2 3 5 6];
          elseif all(R.I_EE_Task == logical([1 1 0 0 0 0]))
            % Ignoriere Rotation der ersten Beinkettekomplett
            R.I_constr_red(i_red:i_red+nPhi-1) = [1 2];
          elseif all(R.I_EE_Task == logical([1 1 1 0 0 0]))
            if Leg_NQJ(i) ~= 4
              % Wähle bei Rotation nur die Y- und Z-Komponente;
              % (Vorgabe von 0 als Zwangsbedingung für 3T1R/3T0R)
              % In Implementierung wird dann die Betragssumme aus X und Y gebildet
              R.I_constr_red(i_red:i_red+nPhi-1) = [1 2 3 5];
            else % Leg_NQJ(i) == 4
              % Wähle keine Rotations-Komponenten. Annahme: Die Y- und X-
              % Rotation sind kinematisch fixiert und können sich sowieso
              % nicht bewegen. Ansonsten würde diese Beinkette mit 4
              % Gelenken sowieso nicht für 3T1R-PKM funktionieren
              R.I_constr_red(i_red:i_red+nPhi-1) = [1 2 3];
            end
          else
            error('Nicht behandelter Fall');
          end
        else
          if all(R.I_EE == logical([1 1 0 0 0 1])) && ...
              all(R.I_EE_Task == logical([1 1 0 0 0 0]))
            % Gehe für Aufgabenredundanz bei 2T1R davon aus, dass reziproke
            % Euler-Winkel und die constr3-Modellierung benutzt wird
            % (Ansatz mit Führungskette und Folgekette)
            Phi_r_ind = fliplr(R.Leg(i).I_EE_Task(4:6));
          elseif all(R.I_EE == logical([1 1 1 0 0 1])) && ...
              all(R.I_EE_Task == logical([1 1 1 0 0 0]))
            % Aufgabenredundanz bei 3T1R. Nehme nur die Y und Z Komponente
            if Leg_NQJ(i) == 4
              % Nur die Z-Komponente. Folge-Beinkette ist auch 3T1R
              Phi_r_ind = logical([1 0 0]);
            elseif Leg_NQJ(i) == 5
              Phi_r_ind = logical([1 1 0]);
            else % Bei 6FG-Beinketten müssen alle ZB-Komp. genommen werden
              Phi_r_ind = logical([1 1 1]);
            end
          else
            Phi_r_ind = R.Leg(i).I_EE_Task(4:6);
          end
          % Folgekette für 3T2R oder beliebige Beinketten
          R.I_constr_red(i_red:i_red+nPhi-1) = ...
              [R.I1constr(i)-1+  find(R.Leg(i).I_EE_Task(1:3)), ...
               R.I1constr(i)-1+3+find(Phi_r_ind)];
        end
        % Lauf-Indizes hochzählen mit der Anzahl ZB für diese Beinkette
        i_tred = i_tred + nPhit;
        i_rred = i_rred + nPhir;
        i_red = i_red + nPhi;
        ii_tred = ii_tred + nPhi;
      end
      % Prüfe die Index-Listen
      assert(isempty(intersect(R.I_constr_t, R.I_constr_r)), ...
          'Variablen I_constr_t und I_constr_r überschneiden sich');
      assert(isempty(intersect(R.I_constr_t_red, R.I_constr_r_red)), ...
          'Variablen I_constr_t_red und I_constr_r_red überschneiden sich');
      
      % Speichere einen Index-Vektor, mit dem die für die PKM-Dynamik
      % relevanten Dynamikparameter der Plattform gewählt werden.
      % Dies dient zur Reduzierung des Dynamik-Parametervektors
      % Reihenfolge der vollständigen Parameter:
      % (XX, XY, XZ, YY, YZ, ZZ, MX, MY, MZ, M)
      if R.phiconv_W_E == 2
        % Andere Konventionen als XYZ noch nicht implementiert
        if all(R.I_EE == [1 1 0 0 0 1]) || all(R.I_EE == [1 1 1 0 0 1])
          R.I_platform_dynpar = logical([0 0 0 0 0 1 1 1 0 1]);
        elseif all(R.I_EE == [1 1 1 0 0 0])
          R.I_platform_dynpar = logical([0 0 0 0 0 0 1 1 0 1]);
        elseif all(R.I_EE == [1 1 1 1 1 1])
          R.I_platform_dynpar = logical([1 1 1 1 1 1 1 1 1 1]);
        end
      end
    end
    function [qJ, xred, pkin, koppelP, legFrame, I_dynpar] = convert_parameter_class2toolbox(R, q, x)
      % Wandle die PKM-Parameter vom Format der Matlab-Klasse ins
      % Toolbox-Format (HybrDyn)
      % Eingabe:
      % q: Gestapelte Gelenkwinkel aller Beinketten
      % x: Plattform-Pose der PKM
      %
      % Ausgabe:
      % qJ: Gelenkwinkel der Beinketten Spaltenweise für jedes Bein ohne
      %     die letzten Gelenkkoordinaten, die die Dynamik nicht beeinflussen
      % xred: Reduzierte Plattform-Pose (nur die für Dynamik relevanten FG)
      % pkin: Kinematik-Parameter für die Funktionen aus der Toolbox
      % koppelP: Plattform-Koppelpunkt-Koordinaten im Toolbox-Format
      % legFrame: Ausrichtung der Beinketten-Basis-KS im Toolbox-Format
      assert(all(size(q) == [R.NJ 1]), 'convert_parameter_class2toolbox: q muss %dx1 sein', R.NJ);
      assert(all(size(x) == [6 1]), 'convert_parameter_class2toolbox: x muss 6x1 sein');
      pkin = R.Leg(1).pkin;
      NLEGJ_NC = R.NQJ_LEG_bc;
      if ~R.issym
        qJ = NaN(NLEGJ_NC, R.NLEG);
      else
        qJ = sym('xx', [NLEGJ_NC R.NLEG]);
        qJ(:)=0;
      end
      for i = 1:R.NLEG
        qJ(:,i) = q(R.I1J_LEG(i):(R.I1J_LEG(i)+NLEGJ_NC-1));
      end
      % Hier wird nicht zwischen EE- und Plattform-Koordinaten
      % unterschieden. Bzgl der EE-FG sollten sie gleich sein.
      xred = x(R.I_EE);
      koppelP = R.r_P_B_all';
      if ~R.issym
        legFrame = NaN(R.NLEG, 3);
      else
        legFrame = sym('xx', [R.NLEG, 3]);
        legFrame(:)=0;
      end
      for i = 1:R.NLEG
        legFrame(i,:) = R.Leg(i).phi_W_0;
      end
      I_dynpar = [1:R.NQJ_LEG_bc,length(R.DynPar.mges)];
    end
    function update_dynpar1(R, mges, rSges, Icges)
      % Aktualisiere die hinterlegten Dynamikparameter ausgehend von
      % gegebenen Parametern bezogen auf den Körper-KS-Ursprung
      % Eingabe:
      % mges: Massen aller Robotersegmente (inkl Plattform, exkl Basis)
      % rSges: Schwerpunktskoordinaten aller Robotersegmente (bezogen auf
      % jeweiliges Körper-KS)
      % Icges: Trägheitstensoren der Robotersegmente (bezogen auf Schwerpkt)
      
      % Prüfung der Eingabe: Dynamik-Parameter für alle Segmente der
      % Beinkette und für die Plattform. Annahme: Kinematisch symmetrischer
      % Roboter
      if length(mges) ~= (R.Leg(1).NL-1)+1
        error('Es müssen Dynamikparameter für %d Körper übergeben werden', (R.Leg(1).NL-1)+1);
      end
        
      [mrSges, Ifges] = inertial_parameters_convert_par1_par2(rSges, Icges, mges);
      % Umwandlung der Dynamik-Parameter (Masse, erstes Moment, zweites
      % Moment) in Minimalparameter-Vektor
      [mpv_num, mpv_sym] = R.dynpar_convert_par2_mpv(mges, mrSges, Ifges);
      
      % Aktualisiere die gespeicherten Dynamikparameter aller Beinketten
      for i = 1:R.NLEG
        m_Leg = [0; mges(1:end-1);];
        rSges_Leg = [zeros(1,3); rSges(1:end-1,:);];
        Icges_Leg = [zeros(1,6); Icges(1:end-1,:);];
        R.Leg(i).update_dynpar1(m_Leg, rSges_Leg, Icges_Leg);
      end
      % Inertialparameter der PKM bestimmen
      ipv_num = R.dynpar_get_inertial_parameter_vector(mges, mrSges, Ifges);
      
      % Parameter für PKM belegen
      R.DynPar.mges   = mges;
      R.DynPar.rSges  = rSges;
      R.DynPar.Icges  = Icges;
      R.DynPar.mrSges = mrSges;
      R.DynPar.Ifges  = Ifges;
      R.DynPar.mpv_sym= mpv_sym;
      R.DynPar.mpv_n1s= mpv_num;
      R.DynPar.ipv_n1s= ipv_num;
    end
    function update_dynpar2(R, mges, mrSges, Ifges)
      % Aktualisiere die hinterlegten Dynamikparameter ausgehend von
      % gegebenen Parametern bezogen auf den Körper-KS-Ursprung
      % Eingabe:
      % mges: Massen aller Robotersegmente (inkl Plattform, exkl Basis)
      % mrSges: Schwerpunktskoordinaten aller Robotersegmente multipliziert
      % mit Masse (bezogen auf jeweiliges Körper-KS)
      % Ifges: Trägheitstensoren der Robotersegmente (bezogen auf Ursprung)
      
      % Prüfung der Eingabe: Dynamik-Parameter für alle Segmente der
      % Beinkette und für die Plattform. Annahme: Kinematisch symmetrischer
      % Roboter
      if length(mges) ~= (R.Leg(1).NL-1)+1
        error('Es müssen Dynamikparameter für %d Körper übergeben werden', (R.Leg(1).NL-1)+1);
      end
        
      % Umwandlung der Dynamik-Parameter (Masse, erstes Moment, zweites
      % Moment) in Minimalparameter-Vektor
      [mpv_num, mpv_sym] = R.dynpar_convert_par2_mpv(mges, mrSges, Ifges);
      
      % Aktualisiere die gespeicherten Dynamikparameter aller Beinketten
      for i = 1:R.NLEG
        m_Leg = [0; mges(1:end-1)];
        mrSges_Leg = [zeros(1,3); mrSges(1:end-1,:)];
        Ifges_Leg = [zeros(1,6); Ifges(1:end-1,:)];
        R.Leg(i).update_dynpar2(m_Leg, mrSges_Leg, Ifges_Leg);
      end
      % Inertialparameter der PKM bestimmen
      ipv_num = R.dynpar_get_inertial_parameter_vector(mges, mrSges, Ifges);
      
      % Umwandlung in baryzentrische Parameter
      [rSges, Icges] = inertial_parameters_convert_par2_par1(mrSges, Ifges, mges);
      
      % Parameter für PKM belegen.
      R.DynPar.mges   = mges;
      R.DynPar.rSges  = rSges;
      R.DynPar.Icges  = Icges;
      R.DynPar.mrSges = mrSges;
      R.DynPar.Ifges  = Ifges;
      R.DynPar.mpv_sym= mpv_sym;
      R.DynPar.mpv_n1s= mpv_num;
      R.DynPar.ipv_n1s= ipv_num;
    end
    function ipv_num = dynpar_get_inertial_parameter_vector(R, mges, mrSges, Ifges)
      % Eingabe:
      % mges: Massen aller Robotersegmente (inkl Plattform, exkl Basis)
      % mrSges: Schwerpunktskoordinaten aller Robotersegmente multipliziert mit Massen
      % Ifges: Trägheitstensoren der Robotersegmente (bezogen auf KS-Ursprung)
      %
      % Ausgabe:
      % ipv_num: Inertial-Parameter (vollständig) der PKM
      %
      % Annahme: Die Dynamikparameter der Beinketten wurden in Leg-Klasse
      % eingetragen
      ipv_leg = R.Leg(1).DynPar.ipv;
      % Siehe dynpar_convert_par2_mpv
      ipv_platform = [Ifges(end,[1 4 5 2 6 3])'; mrSges(end,:)'; mges(end)'];
      ipv_num = [ipv_leg; ipv_platform(R.I_platform_dynpar)];
    end
    function [mpv_num, mpv_sym] = dynpar_convert_par2_mpv(R, mges, mrSges, Ifges)
      % Konvertiere die Dynamikparameter zum Minimalparametervektor
      % Eingabe:
      % mges: Massen aller Robotersegmente (inkl Plattform, exkl Basis)
      % mrSges: Schwerpunktskoordinaten aller Robotersegmente multipliziert mit Massen
      % Ifges: Trägheitstensoren der Robotersegmente (bezogen auf KS-Ursprung)
      % Ausgabe:
      % mpv: Dynamik-Minimalparametervektor
      
      % MPV für symbolisch generierten Code: Annahme ist, dass hinter den
      % Koppelgelenken keine Segmente mit Masseneigenschaften sind
      if ~R.extfcn_available(10)
        % Funktion zur Umwandlung nach MPV wurde nicht generiert. Leer lassen.
        mpv_sym = [];
      else
        I = [1:R.NQJ_LEG_bc,R.Leg(1).NJ+1];
        mges_sym = mges(I);
        mrSges_sym = mrSges(I,:);
        Ifges_sym = Ifges(I,:);
        if any(mges(R.NQJ_LEG_bc+1:end-1) ~= 0)
          % Parameter sind nicht für symbolisch generierten Code zulässig
          mpv_sym = [];
        else
          mpv_sym = R.dynparconvfcnhdl(R.Leg(1).pkin, mges_sym, mrSges_sym, Ifges_sym, R.r_P_B_all');
        end
      end
      
      % MPV für numerischen Ansatz der inversen Dynamik
      % Dynamik-Parameter der Plattform (vollständig). TODO: Anpassung an
      % reduzierte FG je nach EE
      % Khalil-Notation: Reihenfolge (XX, XY, XZ, YY, YZ, ZZ, MX, MY, MZ, M)
      pv2_platform = [Ifges(end,[1 4 5 2 6 3])'; mrSges(end,:)'; mges(end)'];
      % Dynamik-Parameter der (identischen) Beinketten. Basis-Segment Null.
      mges_leg = [0;mges(1:end-1)]; 
      mrSges_leg = [zeros(1,3); mrSges(1:end-1,:)]; 
      Ifges_leg = [zeros(1,6); Ifges(1:end-1,:)];
      mpv_leg = R.Leg(1).dynpar_convert_par2_mpv(mges_leg, mrSges_leg, Ifges_leg);
      mpv_num = [mpv_leg; pv2_platform(R.I_platform_dynpar)];
      % TODO: Behandlung des Falls eines nicht symmetrischen Roboters
    end
    function update_gravity(R, g_world)
      % Aktualisiere den Gravitationsvektor für den Roboter
      % Der Vektor wird für die PKM und die Beinketten gespeichert.
      % Eingabe:
      % g_world: Gravitations-Vektor im Welt-KS
      if nargin == 2
        R_W_0 = R.T_W_0(1:3,1:3);
        g_base = R_W_0' * g_world;
        % Aktualisiere Klassenvariable für PKM (hier Gravitation im Basis-KS)
        R.gravity = g_base;
      else % Nur Aktualisierung der Gravitation im Beinketten-Basis-KS
        g_base = R.gravity;
      end
      % Aktualisiere Gravitationsvektor für die Beinketten. Das "Welt"-KS
      % der Beinketten entspricht dem Basis-KS der PKM (da die Drehung
      % Welt-Basis der Beinketten die Anordnung der Gestell-Koppelgelenke
      % ausdrückt)
      % Der g-Vektor wird für die serielle Kette in deren Basis-KS
      % gespeichert
      for i = 1:R.NLEG
        R.Leg(i).gravity = R.Leg(i).T_W_0(1:3,1:3)'*g_base;
      end
    end

    function [xP, xPD, xPDD] = xE2xP(R, xE, xED, xEDD)
      % Umrechnung von EE-Lagevektor xE zum Plattform-Lagevektor xP
      % Einige Dynamikfunktionen brauchen nur xP, da xE auf einer
      % zusätzlichen Transformation basiert, die beliebig geändert werden
      % kann
      % Eingabe:
      % xE (6x1): Lagevektor des EE-KS im Roboter-Basis-KS
      % xED (6x1): Geschwindigkeitsvektor EE-KS (Euler-Winkel)
      % xEDD (6x1): Beschleunigungsvektor EE-KS (Euler-Winkel)
      % Ausgabe:
      % xP (6x1): Lagevektor des Plattform-KS im Roboter-Basis-KS
      % xPD (6x1): Geschwindigkeitsvektor Plattform-KS (Euler-Winkel)
      % xPDD (6x1): Beschleunigungsvektor Plattform-KS (Euler-Winkel)
      % Quelle: Aufzeichnungen Schappler, 23.08.2019
      
      if all(all(R.T_P_E==eye(4)))
        % Keine Änderung der Transformation. Keine Umrechnung notwendig.
        xP = xE;
        if nargout > 1, xPD = xED;   end
        if nargout > 2, xPDD = xEDD; end
        return
      end
      T_0_E = R.x2t(xE);
      T_0_P = T_0_E * invtr(R.T_P_E);
      xP = R.t2x(T_0_P);
      % Geschwindigkeit berechnen
      if nargin > 2
        % Umrechnung auf Winkelgeschwindigkeit (bezogen auf Endeffektor)
        T_phiE = euljac_mex(xE(4:6), R.phiconv_W_E);
        omega_0_E = T_phiE * xED(4:6);
        % Umrechnung von Punkt E auf Punkt P (mit Adjunkt-Matrix)
        V_0_E = [xED(1:3); omega_0_E];
        r_P_P_E = R.T_P_E(1:3,4);
        R_0_P = T_0_P(1:3,1:3);
        r_0_E_P = -R_0_P*r_P_P_E;
        A_P_E = adjoint_jacobian_mex(r_0_E_P);
        V_0_P = A_P_E * V_0_E;
        % Umrechnung auf Euler-Winkel-Geschwindigkeit bezogen auf Plattform
        T_phiP = euljac_mex(xP(4:6), R.phiconv_W_E);
        xPD = [V_0_P(1:3); T_phiP\V_0_P(4:6)];
      end
      % Beschleunigung berechnen
      if nargin > 3
        % Umrechnung auf Winkelbeschleunigung (bezogen auf Endeffektor)
        TD_phiE = euljacD_mex(xE(4:6), xED(4:6), R.phiconv_W_E);
        omegaD_0_E = TD_phiE*xED(4:6) + T_phiE * xEDD(4:6);
        % Umrechnung von Punkt E auf Punkt P (mit Adjunkt-Matrix)
        VD_0_E = [xEDD(1:3); omegaD_0_E];
        omega_0_P = omega_0_E; % gleicher Starrkörper
        AD_P_E = adjointD_jacobian_mex(-r_P_P_E, R_0_P, omega_0_P);
        VD_0_P = AD_P_E*V_0_E + A_P_E * VD_0_E;
        % Umrechnung auf Euler-Winkel-Beschleunigung bezogen auf Plattform
        TD_phiP = euljacD_mex(xP(4:6), xPD(4:6), R.phiconv_W_E);
        xPDD = [VD_0_P(1:3); T_phiP\(VD_0_P(4:6)-TD_phiP*xPD(4:6))];
      end
    end
    function [XP, XPD, XPDD] = xE2xP_traj(R, XE, XED, XEDD)
      % Umrechnung von EE-Lagevektor xE zum Plattform-Lagevektor xP
      % Einige Dynamikfunktionen brauchen nur xP, da xE auf einer
      % zusätzlichen Transformation basiert, die beliebig geändert werden
      % kann
      % Eingabe:
      % xE (Nx6): Lagevektor des EE-KS im Roboter-Basis-KS für N Bahnpunkte
      % xED (Nx6): Geschwindigkeitsvektor EE-KS (Euler-Winkel)
      % xEDD (Nx6): Beschleunigungsvektor EE-KS (Euler-Winkel)
      % Ausgabe:
      % xP (Nx6): Lagevektor des Plattform-KS im Roboter-Basis-KS
      % xPD (Nx6): Geschwindigkeitsvektor Plattform-KS (Euler-Winkel)
      % xPDD (Nx6): Beschleunigungsvektor Plattform-KS (Euler-Winkel)
      % Quelle: Aufzeichnungen Schappler, 23.08.2019
      
      % Initialisierung der Ausgabevariablen mit Dimension der Eingabe
      XP = XE;
      if nargout > 1, XPD = XED;   end
      if nargout > 2, XPDD = XEDD; end
      for i = 1:size(XE,1)
        if nargin == 2
          XP(i,:) = R.xE2xP(XE(i,:)');
        elseif nargin == 3 
          [XP(i,:),XPD(i,:)] = R.xE2xP(XE(i,:)',XED(i,:)');
        else
          [XP(i,:),XPD(i,:),XPDD(i,:)] = R.xE2xP(XE(i,:)',XED(i,:)',XEDD(i,:)');
        end
      end
    end
    function [xE, xED, xEDD] = xP2xE(R, xP, xPD, xPDD)
      % Umrechnung von Plattform-Lagevektor xP zum Endeffektor-Lagevektor xE
      % Einige Dynamikfunktionen brauchen nur xP, da xE auf einer
      % zusätzlichen Transformation basiert, die beliebig geändert werden
      % kann. Die Rückrechnung in dieser Funktion dient zur Probe.
      % Eingabe:
      % xP (6x1): Lagevektor des Plattform-KS im Roboter-Basis-KS (N mal)
      % xPD (6x1): Geschwindigkeitsvektor Plattform-KS (Euler-Winkel)
      % xPDD (6x1): Beschleunigungsvektor Plattform-KS (Euler-Winkel)
      % Ausgabe:
      % xE (6x1): Lagevektor des EE-KS im Roboter-Basis-KS
      % xED (6x1): Geschwindigkeitsvektor EE-KS (Euler-Winkel)
      % xEDD (6x1): Beschleunigungsvektor EE-KS (Euler-Winkel)
      % Quelle: Aufzeichnungen Schappler, 23.08.2019
      
      % Initialisierung der Ausgabevariablen mit Dimension der Eingabe
      if all(all(R.T_P_E==eye(4)))
        xE = xP;
        if nargout > 1, xED = xPD;   end
        if nargout > 2, xEDD = xPDD; end
        % Keine Änderung der Transformation. Keine Umrechnung notwendig.
        return
      end
      T_0_P = R.x2t(xP);
      T_0_E = T_0_P * R.T_P_E;
      xE = R.t2x(T_0_E);
      % Geschwindigkeit berechnen
      if nargin > 2
        % Umrechnung auf Winkelgeschwindigkeit (bezogen auf Plattform)
        T_phiP = euljac_mex(xP(4:6), R.phiconv_W_E);
        omega_0_P = T_phiP * xPD(4:6);
        % Umrechnung von Punkt P auf Punkt E (mit Adjunkt-Matrix)
        V_0_P = [xPD(1:3); omega_0_P];
        r_P_P_E = R.T_P_E(1:3,4);
        R_0_P = T_0_P(1:3,1:3);
        r_0_P_E = R_0_P*r_P_P_E;
        A_E_P = adjoint_jacobian_mex(r_0_P_E);
        V_0_E = A_E_P * V_0_P;
        % Umrechnung auf Euler-Winkel-Geschwindigkeit bez. auf Endeffektor
        T_phiE = euljac_mex(xE(4:6), R.phiconv_W_E);
        xED = [V_0_E(1:3); T_phiE\V_0_E(4:6)];
      end
      % Beschleunigung berechnen
      if nargin > 3
        % Umrechnung auf Winkelbeschleunigung (bezogen auf Plattform)
        TD_phiP = euljacD_mex(xP(4:6), xPD(4:6), R.phiconv_W_E);
        omegaD_0_P = TD_phiP*xPD(4:6) + T_phiP * xPDD(4:6);
        % Umrechnung von Punkt P auf Punkt E (mit Adjunkt-Matrix)
        VD_0_P = [xPDD(1:3); omegaD_0_P];
        omega_0_E = omega_0_P; % gleicher Starrkörper
        AD_E_P = adjointD_jacobian_mex(r_P_P_E, R_0_P, omega_0_E);
        VD_0_E = AD_E_P*V_0_P + A_E_P * VD_0_P;
        % Umrechnung auf Euler-Winkel-Beschleunigung bez. auf Endeffektor
        TD_phiE = euljacD_mex(xE(4:6), xED(4:6), R.phiconv_W_E);
        xEDD = [VD_0_E(1:3); T_phiE\(VD_0_E(4:6)-TD_phiE*xED(4:6))];
      end
    end
    function [XE, XED, XEDD] = xP2xE_traj(R, XP, XPD, XPDD)
      % Umrechnung von Plattform-Lagevektor xP zum Endeffektor-Lagevektor xE
      % Einige Dynamikfunktionen brauchen nur xP, da xE auf einer
      % zusätzlichen Transformation basiert, die beliebig geändert werden
      % kann. Die Rückrechnung in dieser Funktion dient zur Probe.
      % Eingabe:
      % xP (6xN): Lagevektor des Plattform-KS im Roboter-Basis-KS (N mal)
      % xPD (6xN): Geschwindigkeitsvektor Plattform-KS (Euler-Winkel)
      % xPDD (6xN): Beschleunigungsvektor Plattform-KS (Euler-Winkel)
      % Ausgabe:
      % xE (6xN): Lagevektor des EE-KS im Roboter-Basis-KS
      % xED (6xN): Geschwindigkeitsvektor EE-KS (Euler-Winkel)
      % xEDD (6xN): Beschleunigungsvektor EE-KS (Euler-Winkel)
      % Quelle: Aufzeichnungen Schappler, 23.08.2019
      
      % Initialisierung der Ausgabevariablen mit Dimension der Eingabe
      XE = XP;
      if nargout > 1, XED = XPD;   end
      if nargout > 2, XEDD = XPDD; end
      for i = 1:size(XE,1)
        if nargin == 2
          XE(i,:) = R.xP2xE(XE(i,:)');
        elseif nargin == 3 
          [XE(i,:),XED(i,:)] = R.xP2xE(XP(i,:)',XPD(i,:)');
        else
          [XE(i,:),XED(i,:),XEDD(i,:)] = R.xP2xE(XP(i,:)',XPD(i,:)',XPDD(i,:)');
        end
      end
    end
    function update_collbodies(R, cbtype_selection)
      % Aktualisiere die Kollisionskörper für die PKM. Notwendig, da Körper
      % für die Beinketten getrennt gespeichert sind. Ergebnis: In Klassen-
      % variable R.collbodies sind alle Kollisionskörper (Beinkette+Platt-
      % form) gespeichert.
      % 
      % Eingabe:
      % cbtype_selection
      % 1: Nur Kollisionskörper
      % 2: Nur Bauraum-Körper
      % [1 2]: Beides (Standard)
      if nargin < 2
        cbtype_selection = [1 2];
      end
      % Beide Typen von Kollisionskörpern durchgehen
      for cbtype = cbtype_selection
        if cbtype == 1
          R_collbodies = R.collbodies_nonleg;
        else
          R_collbodies = R.collbodies_instspc_nonleg;
        end
        % Aktualisiere die Klassenvariable collbodies aus den Daten der
        % Beinkette und den PKM-spezifischen Körpern.
        for i = 1:R.NLEG
          if cbtype == 1
            R_Leg_i_collbodies = R.Leg(i).collbodies;
          else
            R_Leg_i_collbodies = R.Leg(i).collbodies_instspc;
          end
          % Hänge die Kollisionskörper der Beinketten an.
          R_collbodies.type = [R_collbodies.type; ...
            R_Leg_i_collbodies.type];
          R_collbodies.params = [R_collbodies.params; ...
            R_Leg_i_collbodies.params];
          % Offset für die Nummern der Körper. 0 in Beinketten-Variable ist
          % Beinketten-Basis, 1 ist erster bewegter Körper der Kette, ...
          NLoffset = 1; % Für Basis der Beinkette
          if i > 1
            NLoffset = 1+R.I2L_LEG(i-1)-(i-1); % in I1L wird auch Basis und EE-Link noch mitgezählt. Hier nicht.
          end
          R_collbodies.link = [R_collbodies.link; ...
            R_Leg_i_collbodies.link + uint8(repmat(NLoffset,size(R_Leg_i_collbodies.link,1),2))];
          % Modifiziere die Kollisionskörper: Zuordnung von Körpern der Bein-
          % ketten-Basis zur PKM-Basis (betrifft Führungsschienen).
          % (ist für Implementierung der Kollisionserkennung besser)
          I_legbase = R_collbodies.link == 0 + NLoffset;
          for j = find(any(I_legbase,2)') % alle Körper, die geändert werden müssen
            if R_collbodies.type(j) == 3 && all(I_legbase(j,:))
              % Kapsel mit absoluten Positionsangaben
              T_0_0i = R.Leg(i).T_W_0;
              % Punkte bezüglich Beinketten-Basis-KS
              pts_0i = R_collbodies.params(j,1:6);
              pts_0 = [eye(3,4)*T_0_0i*[pts_0i(1:3)';1]; ...   % Punkt 1
                       eye(3,4)*T_0_0i*[pts_0i(4:6)';1]]'; ... % Punkt 2
              % Als bezüglich PKM-Basis eintragen
              R_collbodies.params(j,1:6) = pts_0;
              R_collbodies.type(j) = 13;
              R_collbodies.link(j,:) = 0;
            end
          end
        end
        % Trage wieder in Klassen-Variable ein
        if cbtype == 1
          R.collbodies = R_collbodies;
        else
          R.collbodies_instspc = R_collbodies;
        end
      end
    end
  end
end
