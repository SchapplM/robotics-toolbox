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
%   * B: Plattform-Koppel-KS der Beinketten. Orientierung identisch mit "P"
%   * P: Plattform-KS des Roboters, an dem der Endeffektor befestigt ist.
%        Es werden nur parallele Roboter betrachtet, bei denen die
%        Plattform am Ende aller Beinketten sitzt (also ohne zusätzliches
%        serielles "Ende" an der Plattform).
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
%   3: Dynamik-Parametervektor (ohne Minimierung); nicht implementiert
%   4: Dynamik-Minimalparametervektor
% 
% Siehe auch: SerRob.m (SerRob-Klasse)
%
% Quellen. 
% [AbdellatifHei2009] Computational efficient inverse dynamics of 6-DOF fully
% parallel manipulators by using the Lagrangian formalism

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
% (C) Institut für Mechatronische Systeme, Universität Hannover

classdef ParRob < matlab.mixin.Copyable

  properties (Access = public)
      NLEG % Anzahl der Beinketten
      NJ % Anzahl der Gelenkkoordinaten des Roboters (Gelenkkoordinaten aller Beinketten)
      NL % Anzahl der Starrkörper der PKM insgesamt (inkl. Basis und Plattform)
      I1J_LEG % Start-Indizes der Gelenkwinkel der einzelnen Beine in allen Gelenkwinkeln
      I2J_LEG % End-Indizes er Gelenkwinkel ...
      I1L_LEG % Start-Indizes der Segmente der einzelnen Beinketten
      I2L_LEG % End-Indizes der Segmente ..
      r_P_B_all % [3xNLEG] Ortsvektoren der Plattform
      r_0_A_all % [3xNLEG]
      MDH % Struktur mit MDH-Parametern für Roboterkinematik (der PKM)
      I_qd % Zähl-Indizes der abhängigen Gelenke in allen Gelenken
      I_qa % Zähl-Indizes der aktiven Gelenke in allen Gelenken
      I_EE % Indizes der verfügbaren EE-FG des Roboters (EE-Position, Euler-Winkel aus phiconv_W_E)
      I_EE_Task % Indizes der durch die Aufgabe genutzten EE-FG (EE-Position, Euler-Winkel aus phiconv_W_E)
      phiconv_W_0 % Nummer der Basis-Euler-Winkelkonvention
      phiconv_P_E % Winkelkonvention der Euler-Winkel vom Plattform-KS zum EE
      phiconv_W_E % Winkelkonvention zur Darstellung der EE-Orientierung im Welt-KS mit Euler-Winkeln
      T_W_0 % Homogene Transformationsmatrix zwischen Welt- und Basis-KS des Roboters
      r_W_0 % Position der Basis im Welt-KS
      phi_W_0 % Orientierung des Basis-KS im Welt-KS (ausgedrückt in Euler-Winkeln)
      T_P_E % Homogene Transformationsmatrix zwischen Endeffektor-KS und Plattform-KS
      r_P_E % Position des Endeffektors im Plattform-KS
      phi_P_E % Orientierung des EE-KS im Plattform-KS (ausgedrückt in Euler-Winkeln)
      Type % Typ des Roboters (2=parallel; zur Abgrenzung von SerRob)
      DynPar % Struktur mit Dynamikparatern (Masse, Schwerpunkt, Trägheit)
      DesPar % Struktur mit Entwurfsparameter (Gestell,Plattform)
      mdlname % Name des PKM-Robotermodells, das in den Matlab-Funktionen benutzt wird.
      Leg % Matlab-Klasse SerRob für jede Beinkette
      issym % true für rein symbolische Berechnung
      gravity % Gravitationsvektor ausgedrückt im Basis-KS des Roboters
      NQJ_LEG_bc % Anzahl relevanter Beingelenke vor Schnittgelenk (von Basis an gezählt) ("bc"="before cut")
      I1constr   % Start-Indizes der Beinketten in allen Zwangsbedingungen
      I2constr   % End-Indizes der Beinketten in allen Zwangsbedingungen
      I1constr_red   % Start-Indizes der Beinketten in allen reduzierten Zwangsbedingungen
      I2constr_red   % End-Indizes der Beinketten in allen reduzierten Zwangsbedingungen
      I_constr_t % Indizes der translatorischen Zwangsbedingungen in allen ZB
      I_constr_r % Indizes der rotatorischen ZB in allen ZB
      I_constr_red % Indizes der reduzierten ZB in allen ZB
      I_constr_t_red % Indizes für reduzierte ZB (translatorisch)
      I_constr_r_red % ...                       (rotatorisch)
      I_platform_dynpar % Auswahlvektor für Dynamikparameter der Plattform
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
      R.phiconv_W_E = uint8(2); % Euler-XYZ
      R.phiconv_W_0 = uint8(2); % Euler-XYZ
      R.phiconv_P_E = uint8(2); % Euler-XYZ
      R.issym = false;
      R.r_P_E = zeros(3,1);
      R.phi_P_E = zeros(3,1);
      R.T_P_E = eye(4);
      R.r_W_0 = zeros(3,1);
      R.phi_W_0 = zeros(3,1);
      R.T_W_0 = eye(4);
      R.gravity = [0;0;-9.81];
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
        {'dynparconvfcnhdl', 'minimal_parameter_para'}};
      R.extfcn_available = false(length(R.all_fcn_hdl),1);
      R.I_platform_dynpar = true(1,10);
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
    function [Jinv_qD_xD, Jinv_num_voll] = jacobi_qa_x(R, q, xE)
      % Analytische Jacobi-Matrix zwischen Antriebs- und Plattformkoord.
      % Eingabe:
      % q: Gelenkkoordinaten
      % xE: EE-Koordinaten (6x1) (nicht: Plattform-Koordinaten) (im Basis-KS)
      %
      % Ausgabe:
      % Jinv: Inverse Jacobi-Matrix (Verhältnis Gelenk-Geschw. -
      % Plattform-Geschw. mit Euler-Zeitableitung)
      
      if R.extfcn_available(1) && nargout ~= 2
        % Berechnung der geometrischen Jacobi-Matrix aus Funktionsaufruf
        xP = R.xE2xP(xE);
        [qJ, xPred, pkin, koppelP, legFrame] = convert_parameter_class2toolbox(R, q, xP);
        Jinv_qD_sD = R.jacobi_qa_x_fcnhdl(xPred, qJ, pkin, koppelP, legFrame);
        if ~all(R.I_EE == [1 1 1 1 1 1])
          % Keine vollständigen FG. Annahme: Keine Umwandlung zwischen
          % Rotations-FG erforderlich
          Jinv_qD_xD = Jinv_qD_sD;
        else
          % Korrekturmatrix (symbolische Jacobi bezieht sich auf
          % Winkelgeschwindigkeit, numerische auf Euler-Winkel-Zeitableitung)
          % Hier Bezug auf EE-Euler-Winkel-Geschwindigkeit (nicht: Plattform)
          T = [eye(3,3), zeros(3,3); zeros(3,3), euljac(xE(4:6), R.phiconv_W_E)];
          Jinv_qD_xD = Jinv_qD_sD*T;
        end
      else % Funktion ist nicht verfügbar. Nehme numerische Berechnung
        G_q  = R.constr1grad_q(q, xE);
        G_x = R.constr1grad_x(q, xE);
        Jinv_num_voll = -G_q \ G_x;
        Jinv_qD_xD = Jinv_num_voll(R.I_qa,:);
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
      GD_q = R.constr1gradD_q(q, qD, xE, xED); % differentiation of the kinematic constraints of joint coordinate can be found in [Do Thanh]
      GD_x = R.constr1gradD_x(q, qD, xE, xED); %% differentiation of the kinematic constraints of platform  coordinate can be found in [Do Thanh]
      JinvD_num_voll = G_q\GD_q/G_q*G_x - G_q\GD_x;
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
      elseif R.DynPar.mode == 4
        Fx = R.invdyn_x_fcnhdl4(xPred, xPDred, xPDDred, qJ, R.gravity, legFrame, koppelP, pkin, ...
          R.DynPar.mpv_sym);
      else
        error('Modus %d noch nicht implementiert', R.DynPar.mode);
      end
    end
    function [Fa, Fa_reg] = invdyn2_actjoint(Rob, q, qD, xE, xDE, xDDE, Jinv, JinvD)
      % Berechne Antriebskraft aufgrund der Effekte der inversen Dynamik
      % Eingabe:
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeiten
      % xE: Endeffektor-Koordinaten
      % xDE: Endeffektor-Geschwindigkeit
      % xDDE: Endeffektor-Beschleunigung
      % Jinv: Inverse Jacobi-Matrix (bezogen auf EE-Koordinaten und alle
      % Gelenke). Siehe ParRob/jacobi_qa_x
      % JinvD: Zeitableitung von Jinv. Siehe 
      %
      % Ausgabe:
      % Fa: Kraft auf Antriebsgelenke (kartesische Momente)
      % Fa_reg: Regressor-Matrix der Kraft
      % Inversdynamik-Kräfte in Endeffektor-Koordinaten berechnen
      if nargout == 1
        Fx = Rob.invdyn2_platform(q, qD, xE, xDE, xDDE, Jinv, JinvD);
      else
        [Fx, Fx_reg] = Rob.invdyn2_platform(q, qD, xE, xDE, xDDE, Jinv, JinvD);
      end
      % Umrechnen der vollständigen inversen Jacobi
      Jinv_qaD_xD = Jinv(Rob.I_qa,:);
      % Jacobi-Matrix auf Winkelgeschwindigkeiten beziehen. Siehe ParRob/jacobi_qa_x
      if size(Jinv_qaD_xD,2) == 6
        T = [eye(3,3), zeros(3,3); zeros(3,3), euljac(xE(4:6)', Rob.phiconv_W_E)];
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
    function [Fx_traj,Fx_traj_reg] = invdyn2_platform_traj(R, Q, QD, XE, XED, XEDD, Jinv_ges, JinvD_ges)
      % Inverse Dynamik in Endeffektor-Koordinaten als Trajektorie (Zeit als Zeilen)
      % Eingabe:
      % Q: Gelenkkoordinaten (Trajektorie)
      % QD: Gelenkgeschwindigkeiten (Trajektorie)
      % XE: Endeffektor-Koordinaten (Trajektorie)
      % XED: Endeffektor-Geschwindigkeit (Trajektorie)
      % XEDD: Endeffektor-Beschleunigung (Trajektorie)
      % Jinv_ges: Zeilenweise inverse Jacobi-Matrix für alle Gelenke (Traj.)
      % (siehe jacobi_qa_x)
      % JinvD_ges: Zeitableitung von Jinv_ges. Siehe jacobiD_qa_x.
      %
      % Ausgabe:
      % Fx_traj: Kraft auf Plattform (Inverse Dynamik, als Zeitreihe)
      % Fx_traj_reg: Regressormatrizen von Fx_traj (als Zeitreihe)
      Fx_traj = NaN(size(Q,1),sum(R.I_EE));
      if nargout == 2
        Fx_traj_reg = NaN(size(Q,1),sum(R.I_EE)*length(R.DynPar.mpv_n1s));
      end
      for i = 1:size(Q,1)
        Jinv_full = reshape(Jinv_ges(i,:), R.NJ, sum(R.I_EE));
        JinvD_full = reshape(JinvD_ges(i,:), R.NJ, sum(R.I_EE));
        if nargout < 2
          Fx_traj(i,:) = R.invdyn2_platform(Q(i,:)', QD(i,:)', XE(i,:)', XED(i,:)', XEDD(i,:)', Jinv_full, JinvD_full);
        else
          [Fx_traj(i,:), Fx_traj_reg_i] = R.invdyn2_platform(Q(i,:)', QD(i,:)', XE(i,:)', XED(i,:)', XEDD(i,:)', Jinv_full, JinvD_full);
          Fx_traj_reg(i,:) = Fx_traj_reg_i(:);
        end
      end
    end
    function [Fa_traj,Fa_traj_reg] = invdyn2_actjoint_traj(R, Q, QD, XE, XED, XEDD, Jinv_ges, JinvD_ges)
      % Inverse Dynamik in Antriebskoordinaten als Trajektorie (Zeit als Zeilen)
      % Eingabe:
      % Q: Gelenkkoordinaten (Trajektorie)
      % QD: Gelenkgeschwindigkeiten (Trajektorie)
      % XE: Endeffektor-Koordinaten (Trajektorie)
      % XED: Endeffektor-Geschwindigkeit (Trajektorie)
      % XEDD: Endeffektor-Beschleunigung (Trajektorie)
      % Jinv_ges: Zeilenweise inverse Jacobi-Matrix für alle Gelenke (Traj.)
      % (siehe jacobi_qa_x)
      % JinvD_ges: Zeitableitung von Jinv_ges. Siehe jacobiD_qa_x.
      %
      % Ausgabe:
      % Fa_traj: Kraft auf Antriebe (Inverse Dynamik, als Zeitreihe)
      % Fa_traj_reg: Regressormatrizen von Fa_traj (als Zeitreihe)
      Fa_traj = NaN(size(Q,1),sum(R.I_qa));
      if nargout == 2
        Fa_traj_reg = NaN(size(Q,1),sum(R.I_qa)*length(R.DynPar.mpv_n1s));
      end
      for i = 1:size(Q,1)
        Jinv_full = reshape(Jinv_ges(i,:), R.NJ, sum(R.I_EE));
        JinvD_full = reshape(JinvD_ges(i,:), R.NJ, sum(R.I_EE));
        if nargout < 2
          Fa_traj(i,:) = R.invdyn2_actjoint(Q(i,:)', QD(i,:)', XE(i,:)', XED(i,:)', XEDD(i,:)', Jinv_full, JinvD_full);
        else
          [Fa_traj(i,:), Fa_traj_reg_i] = R.invdyn2_actjoint(Q(i,:)', QD(i,:)', XE(i,:)', XED(i,:)', XEDD(i,:)', Jinv_full, JinvD_full);
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
      for i = 1:size(Fx_traj_reg,1)
        Fx_traj_reg_i = reshape(Fx_traj_reg(i,:), sum(R.I_EE), length(R.DynPar.mpv_n1s));
        Fx_traj(i,:) = Fx_traj_reg_i*R.DynPar.mpv_n1s;
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
      for i = 1:size(Fa_traj_reg,1)
        Fa_traj_reg_i = reshape(Fa_traj_reg(i,:), sum(R.I_qa), length(R.DynPar.mpv_n1s));
        Fa_traj(i,:) = Fa_traj_reg_i*R.DynPar.mpv_n1s;
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
      elseif R.DynPar.mode == 4
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
      elseif R.DynPar.mode == 4
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
      % Mx: Coriolis-Kräfte (bezogen auf EE-Koordinaten der PKM)
      xPDred = xPD(R.I_EE);
      [qJ, xPred, pkin, koppelP, legFrame, Idp] = convert_parameter_class2toolbox(R, q, xP);
      if R.DynPar.mode == 2
        Fcx = R.coriolisvec_x_fcnhdl2(xPred, xPDred, qJ, legFrame, koppelP, pkin, ...
          R.DynPar.mges(Idp), R.DynPar.mrSges(Idp,:), R.DynPar.Ifges(Idp,:));
      elseif R.DynPar.mode == 4
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
      % TODO: Abgrenzung EE-KS, Plattform-KS
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
      R.I_EE = I_EE;
      if nargin < 3
        I_EE_Task = I_EE;
      end
      if nargin < 4
        % TODO: Bei 3T2R stimmt das eventuell nicht, wenn Beinketten 5
        % Gelenke haben
        I_EE_Legs = repmat(I_EE_Task, R.NLEG,1);
      end
      R.I_EE_Task = I_EE_Task;
      
      if all(R.I_EE_Task == logical([1 1 1 1 1 0]))
        % Führungs-Beinkette muss auch die 3T2R-FG haben.
        R.Leg(1).I_EE_Task = R.I_EE_Task;
        for i = 2:R.NLEG
          % Andere
          R.Leg(i).I_EE_Task = logical([1 1 1 1 1 1]);
        end
      else%if all(R.I_EE_Task == logical([1 1 1 1 1 1]))
        for i = 1:R.NLEG
          % Anderer Fall als 3T2R: Setze die Aufgaben-FG auch für die
          % Beinketten auf die der PKM.
          % TODO: Muss für überbestimmte PKM angepasst werden
          R.Leg(i).I_EE_Task = I_EE_Legs(i,:);
        end
      end
      
      % Anzahl der kinematischen Zwangsbedingungen der Beinketten
      % feststellen. Annahme: Beinketten haben selbe FG wie Plattform
      % TODO: Das ändert sich evtl bei überbestimmten PKM
      % Indizes der relevanten Zwangsbedingungen daraus ableiten
      % Werden benötigt für Ausgabe von constr1
      R.I_constr_t = [];
      R.I_constr_r = [];
      R.I_constr_red = [];
      R.I_constr_t_red = [];
      R.I_constr_r_red = [];
      
      R.I1constr = [];
      R.I2constr = [];
      R.I1constr_red = [];
      R.I2constr_red = [];
      
      i_red = 1; % Lauf-Index für I_constr_red (jew. erster Eintrag pro Bein)
      i_tred = 1; i_rred = 1; % Lauf-Index für I_constr_t_red und I_constr_r_red
      ii_tred = 1; % Lauf-Index für ersten Eintrag in Gesamt-ZB für aktuelles Bein
      for i = 1:R.NLEG
        % Zähle Zwangsbedingungen für das Bein (werden in I_EE_Task der
        % Beinkette abgelegt)
        nPhit = sum(R.Leg(i).I_EE_Task(1:3));
        nPhir = sum(R.Leg(i).I_EE_Task(4:6));

        % Sonderfall: 3T2R
        if all(R.I_EE_Task == logical([1 1 1 1 1 0])) && i > 1
          % Folgekette bei 3T2R muss wieder 3T3R FG haben
          if R.Leg(i).NJ == 6 % TODO: Bei Sonderfällen Bedingung ändern
            % TODO: Alternative: I_EE/I_EE_Task; Nur mit Basis-Orientierung?
            nPhir = 3;
          end
        end
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
        if all(R.I_EE_Task == logical([1 1 1 1 1 0])) && i == 1
          % Führungskette für 3T2R anders
          R.I_constr_red(i_red:i_red+nPhi-1) = [1 2 3 5 6];
        else
          % Folgekette für 3T2R oder beliebige Beinketten
          R.I_constr_red(i_red:i_red+nPhi-1) = ...
              [R.I1constr(i)-1+find(R.Leg(i).I_EE_Task(1:3)), ...
               R.I1constr(i)-1+3+find(R.Leg(i).I_EE_Task(4:6))];
        end
        % Lauf-Indizes hochzählen mit der Anzahl ZB für diese Beinkette
        i_tred = i_tred + nPhit;
        i_rred = i_rred + nPhir;
        i_red = i_red + nPhi;
        ii_tred = ii_tred + nPhi;
      end
      % Speichere einen Index-Vektor aus, mit dem die für die PKM-Dynamik
      % relevanten Dynamikparameter der Plattform gewählt werden
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
      % Parameter für PKM belegen
      R.DynPar.mges   = mges;
      R.DynPar.rSges  = rSges;
      R.DynPar.Icges  = Icges;
      R.DynPar.mrSges = mrSges;
      R.DynPar.Ifges  = Ifges;
      R.DynPar.mpv_sym= mpv_sym;
      R.DynPar.mpv_n1s= mpv_num;
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
    end
    
    function [mpv_num, mpv_sym] = dynpar_convert_par2_mpv(R, mges, mrSges, Ifges)
      % Konvertiere die Dynamikparameter zum Minimalparametervektor
      % Eingabe:
      % mges: Massen aller Robotersegmente (inkl Basis)
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
      R_W_0 = R.T_W_0(1:3,1:3);
      g_base = R_W_0' * g_world;
      
      % Aktualisiere Klassenvariable für PKM (hier Gravitation im Basis-KS)
      R.gravity = g_base;
      
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
    function x_W_E = t2x(R, T_W_E)
      % Umwandlung der homogenen Transformationsmatrix der EE-Lage in Minimalkoordinaten
      % Eingabe:
      % T_W_E: Transformationsmatrix zwischen Welt- und EE-KS
      % 
      % Ausgabe: Vektor aus Position und Euler-Winkeln
      x_W_E = [T_W_E(1:3,4); r2eul(T_W_E(1:3,1:3), R.phiconv_W_E)];
    end
    function T_W_E = x2t(R, x_W_E)
      % Umwandlung der EE-Lage in eine homogene Transformationsmatrix
      % Eingabe:
      % T_W_E: Vektor aus Position und Euler-Winkeln
      % 
      % Ausgabe: Transformationsmatrix zwischen Welt- und EE-KS
      T_W_E = [eul2r(x_W_E(4:6), R.phiconv_W_E), x_W_E(1:3); [0 0 0 1]];
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
  end
end
