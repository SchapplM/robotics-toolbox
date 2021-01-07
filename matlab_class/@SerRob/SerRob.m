%   Klasse für serielle (und seriell-hybride) Roboter
%
%   Mit dieser Klasse können alle Kinematik- und Dynamikfunktionen eines
%   seriellen Roboters aufgerufen werden.
%   Der transparente Aufruf von Funktionen als eigene Matlab-Datei und als
%   mex-kompilierte Datei ermöglicht eine höhere Effizienz als
%   bei anderen Implementierungen.
% 
%   Koordinatensystem-Definitionen:
%   * W: Welt-KS (Absolutes Inertial-KS, auf das sich auch mehrere Roboter
%   gleichzeitig beziehen können)
%   * 0: Basis-KS (Inertial-KS des Roboters)
%   * E: Endeffektor-KS des Roboters (z.B. Bohrerspitze), auf dieses KS
%   bezieht sich die Bahnplanung und Kinematik
%   * N: Körper-KS des Roboters, an dem der Endeffektor befestigt ist. Bei
%   seriellen Robotern das letzte Körper-KS der seriellen Kette.
% 
%   Definition der Dynamik-Parameter:
%   DynPar.mode: Umschalten zwischen verschiedenen Dynamik-Parametern als
%   Eingang der Dynamik-Funktionen. Entspricht Ziffer hinter
%   Funktions-Handles. Die Parameter können unabhängig vom Modus der
%   Funktionen übergeben werden.
%   1: Dynamik-Parameter bezogen auf Schwerpunkt m,rS,Ic (baryzentrisch)
%      (noch nicht implementiert)
%   2: Dynamik-Parameter bezogen auf Körper-KS-Ursprung m,mrS,If (inertial)
%   3: Dynamik-Parametervektor (ohne Minimierung);
%   4: Dynamik-Minimalparametervektor
%   6: Dynamik-MPV mit Regressor-Matrix als Eingang

%   Quellen
%   [Rob1] Robotik 1 Skript
%   [SchapplerTapOrt2019] Exploiting Dynamics Parameter Linearity for Design Optimization in
%   Combined Structural and Dimensional Robot Synthesis

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-08
% (C) Institut für mechatronische Systeme, Universität Hannover

classdef SerRob < RobBase

  properties (Access = public)
    qlim % Minimale und maximale Gelenkkoordinaten q zeilenweise für die Gelenke
    qref % Referenz-Gelenkstellung des Roboters (entspricht "Justage"-Position)
    qDlim % Minimale und maximale Gelenkgeschwindigkeiten zeilenweise für die Gelenke
    taulim % Minimale und maximale Gelenkkräfte zeilenweise
    descr % Beschreibung des Roboters (Längerer, ausführlicher Name)
    phiconv_N_E % Winkelkonvention der Euler-Winkel vom EE-Körper-KS zum EE
    MDH % Struktur mit MDH-Parametern für Roboterkinematik
    NQJ % Anzahl der Gelenkkoordinaten (abweichend von NJ bei hybriden Strukturen)
    pkin % Vektor der Kinematikparameter
    pkin_gen % Vektor der Kinematikparameter des allgemeinen Modells
    pkin_names % Namen der Kinematikparameter
    pkin_types % Typ der Kinematikparameter; siehe get_pkin_parameter_type()
    pkin_jointnumber % Gelenknummer des Kinematikparameters; siehe get_pkin_parameter_type()
    DynPar % Struktur mit Dynamikparametern (Masse, Schwerpunkt, Trägheit)
    DesPar % Struktur mit Entwurfsparameter (Segmentgeometrie, Motorauswahl, ...)
    T_N_E % Homogene Transformationsmatrix zwischen 
    r_N_E % Position des Endeffektors im Körper-KS
    phi_N_E % Orientierung des EE-KS im Körper-KS (ausgedrückt in Euler-Winkeln)
    qunit_eng % Ingenieursmäßige Einheiten der Gelenkkoordinaten
    qunit_sci % wissenschaftliche Einheiten der Gelenkkoordinaten
    qunitmult_eng_sci % Umrechnungsfaktor zwischen beiden Einheitenarten
    tauunit_sci % Name der Einheiten der Gelenkkräfte
    I_EElink % Index des Segmentes, an dem der Endeffektor befestigt ist.
    mdlname % Name des Robotermodells, das in den Matlab-Funktionen benutzt wird.
    mdlname_var % Name des Robotermodells dieser Variante des allgemeinen Modells
    CADstruct % Struktur mit Daten zu CAD-Modellen
    islegchain % Marker, ob diese serielle Kette eine PKM-Beinkette ist
    collbodies % Struktur mit Ersatzkörpern zur Kollisionserkennung
  end
  properties (Access = private)
    jtraffcnhdl % Funktions-Handle für Gelenk-Transformationen
    fkinfcnhdl % Funktions-Handle für direkte Kinematik
    ekinfcnhdl2     % Funktions-Handle für kinetische Energie
    ekinfcnhdl4     % ... mit anderen Parametern
    epotfcnhdl2     % Funktions-Handle für potentielle Energie
    epotfcnhdl4     % ... mit anderen Parametern
    gravlfcnhdl2    % Funktions-Handle für Gelenkkräfte durch Gravitation
    gravlfcnhdl4    % ... mit anderen Parametern
    inertiafcnhdl2  % Funktions-Handle für Massenmatrix
    inertiafcnhdl4  % ... mit anderen Parametern
    corvecfcnhdl2   % Funktions-Handle für Gelenkkräfte durch Coriolis- und Fliehkräfte
    corvecfcnhdl4   % ... mit anderen Parametern
    cormatfcnhdl2   % Funktions-Handle für Coriolis-Matrix
    cormatfcnhdl4   % ... mit anderen Parametern
    invdynfcnhdl2   % Funktions-Handle für Gelenkkräfte durch inverse Dynamik
    invdynfcnhdl4   % ... mit anderen Parametern
    intforcefcnhdl2   % Funktions-Handle für Schnittkräfte durch inverse Dynamik
    intforcemcnhdl2   % Funktions-Handle für Schnittkmomente durch inverse Dynamik
    invdynregmattrajfcnhdl % Funktions-Handle für Trajektorie von Regressor-Matrizen
    invdyntrajfcnhdl4 % ... für Inverse Dynamik einer Trajektorie von Gelenkvariablen
    invdyntrajfcnhdl6 % ... für Inverse Dynamik aus einer Trajektorie von Regressormatrizen
    intforcefregfcnhdl % Funktions-Handle für Regressor-M. der Schnittkräfte
    intforcemregfcnhdl % Funktions-Handle für Regressor-M. der Schnittmomente
    ekinregfcnhdl3     % Funktions-Handle für Inertialparameter-Regressor-Matrix der kinetischen Energie
    ekinregfcnhdl4     % ... mit anderen Parametern (Minimalparameter)
    epotregfcnhdl3     % Funktions-Handle für Inertialparameter-Regressor-Matrix der potentielle Energie
    epotregfcnhdl4     % ... mit anderen Parametern (Minimalparameter)
    gravlregfcnhdl3    % Funktions-Handle für Inertialparameter-Regressor-Matrix der Gelenkkräfte durch Gravitation
    gravlregfcnhdl4    % ... mit anderen Parametern (Minimalparameter)
    inertiaregfcnhdl3  % Funktions-Handle für Inertialparameter-Regressor-Matrix der Massenmatrix
    inertiaregfcnhdl4  % ... mit anderen Parametern (Minimalparameter)
    corvecregfcnhdl3   % Funktions-Handle für Inertialparameter-Regressor-Matrix der Gelenkkräfte durch Coriolis- und Fliehkräfte
    corvecregfcnhdl4   % ... mit anderen Parametern (Minimalparameter)
    cormatregfcnhdl3   % Funktions-Handle für Inertialparameter-Regressor-Matrix der Coriolis-Matrix
    cormatregfcnhdl4   % ... mit anderen Parametern (Minimalparameter)
    invdynregfcnhdl3   % Funktions-Handle für Inertialparameter-Regressor-Matrix der Gelenkkräfte durch inverse Dynamik
    invdynregfcnhdl4   % ... mit anderen Parametern (Minimalparameter)
    jacobiRfcnhdl   % Funktions-Handle für Jacobi-Matrix bzgl der Rotationsmatrix des Endeffektors
    jacobiRDfcnhdl  % Funktions-Handle für Zeitableitung der RotMat-Jacobi
    jacobigfcnhdl   % Funktions-Handle für geometrische Jacobi-Matrix
    jacobigDfcnhdl  % Funktions-Handle für Zeitableitung der geometrischen Jacobi-Matrix
    jacobitfcnhdl   % Funktions-Handle für Translationskomponente der Jacobi-Matrix
    jacobitDfcnhdl  % Funktions-Handle für Zeitableitung der Translationskomponente
    jacobiwfcnhdl   % Funktions-Handle für Rotationskomponente der geometrischen Jacobi-Matrix
    jacobiwDfcnhdl  % Funktions-Handle für Zeitableitung der Rotationskomponente der geometrischen Jacobi-Matrix
    jacobigcutfcnhdl% Funktions-Handle für Schnittkraft-Jacobi-Matrix
    invkinfcnhdl    % Funktions-Handle für inverse Kinematik
    invkintrajfcnhdl% Funktions-Handle für inverse Kinematik einer Trajektorie
    jointvarfcnhdl  % Funktions-Handle für Werte der Gelenkvariablen (bei hybriden Robotern)
    dynparconvfcnhdl% Funktions-Handle zur Umwandlung von DynPar 2 zu MPV
    all_fcn_hdl     % Cell-Array mit allen Funktions-Handles des Roboters sowie den Dateinamen der Matlab-Funktionen
  end

  methods
    % Konstruktor
    function R=SerRob(Par_struct, mdlname, mdlname_var)
      % Eingabe:
      % Par_struct: 
      %   Struktur mit Kinematik- und Dynamik-Parametern
      % mdlname: 
      %   Name des Roboters, der bei der Code-Generierung verwendet
      %   wurde (bildet den Präfix aller Funktionsdateien)
      % mdlname_var:
      %   Name der Variante, die dieser Roboter bezüglich mdlname darstellt

      R.mdlname = mdlname;
      if nargin < 3
        mdlname_var = mdlname;
      end
      R.mdlname_var = mdlname_var;
      R.NJ = sum(Par_struct.sigma~=2);
      if ~isfield(Par_struct, 'v')
        Par_struct.v = 0:R.NJ-1; % Vorgänger-Indizes
      end
      R.NL = Par_struct.NL;
      R.NJ = Par_struct.NJ;
      R.NQJ = Par_struct.NQJ;

      if R.NJ == R.NQJ
        R.Type = 0; % serielle Struktur
      else
        R.Type = 1; % hybride Struktur
      end

      R.MDH = Par_struct;
      % Fehlende Felder mit Standard-Werten beschreiben
      Par_struct_default = struct('beta', zeros(R.NJ,1), 'b', zeros(R.NJ,1), ...
        'mu', ones(R.NJ,1), 'offset', zeros(R.NJ,1));
      for f = fields(Par_struct_default)'
        if ~isfield(Par_struct, f{1})
          R.MDH.(f{1}) = Par_struct_default.(f{1});
        end
      end
      R.pkin = Par_struct.pkin;
      if isempty(R.pkin)
        % Keine pkin-Werte gegeben. Initialisiere Variable aus
        % MDH-Parametern
        R.update_pkin();
      else
        % Werte für pkin gegeben. Aktualisiere Parameter pkin_gen und
        % MDH-Paramter, damit die verschiedenen Parameter konsistent sind.
        R.update_mdh(R.pkin)
      end
      [R.pkin_types, R.pkin_jointnumber] = R.get_pkin_parameter_type();
      R.DynPar = struct('mges',   NaN(R.NL,1), ...
                        'rSges',  NaN(R.NL,3), 'Icges', NaN(R.NL,6), ...
                        'mrSges', NaN(R.NL,3), 'Ifges', NaN(R.NL,6), ...
                        'mpv', [], 'ipv_floatb', [], ...
                        'mode', 2);
      R.update_dynpar1();
      R.DesPar = struct(...
        'material', [2.7E3, 72e9, 27e9, 300e6], ... % Dichte, E-Modul, Schubmodul, Dehngrenze für Alu-Legierung AlCu4PbMgMn
        'seg_type', ones(R.NL,1), ... % Modellierungsart Segmente (1=Hohlzylinder)
        'seg_par', zeros(R.NL,2), ... % Parameter dafür (Wandstärke, Durchmesser)
        'joint_offset', zeros(R.NJ,1), ... % Offset-Parameter für zusätzliches Segment nach Schubgelenk. Verschiebt die Linearachse.
        'joint_stiffness', zeros(R.NJ,1), ...% Steifigkeit einer im Gelenk angebrachten Drehfeder (Sonderfall für Festkörpergelenke; für normale Roboter oder PKM-Beinketten nicht belegt)
        'joint_stiffness_qref', zeros(R.NJ,1), ... % Nullstellung der Ersatzfedern in den Gelenken
        'gear_index', uint8(zeros(R.NJ,1)), ... % Nr des Getriebes (aus Datenbank)
        'motor_index', uint8(zeros(R.NJ,1)), ... % Nr des Motors (aus Datenbank)
        'joint_index', uint8(zeros(R.NJ,1)), ... % Nr des passiven Gelenks (aus Datenbank)
        'joint_type', uint8(R.MDH.sigma)); % Art des Gelenks: 0=Drehgelenk, 1=Schub- (allgemein), 2=Kardan-, 3=Kugel-, 4=Schub- mit Führung, 5=Schub- mit Zylinder

      R.qref = zeros(R.NQJ,1);

      R.r_N_E = zeros(3,1);
      R.phi_N_E = zeros(3,1);
      R.T_N_E = eye(4);
      R.phiconv_N_E = uint8(2); % Euler-XYZ

      R.I_EE = true(1,6); % Initialisierung mit allen Freiheitsgraden (räumliche PKM). Muss logical sein, damit Binär-Indizes.
      R.I_EE_Task = true(1,6); % zunächst alle Aufgaben-FG vorsehen
      R.I_EElink = R.NL-1; % bezogen auf nummerierte Körper (Basis=0)

      % Liste der Funktionshandle-Variablen mit zugehörigen
      % Funktionsdateien (aus Maple-Toolbox)
      R.all_fcn_hdl = { ...
      {'fkinfcnhdl', 'fkine_fixb_rotmat_mdh_sym_varpar'}, ...
      {'jtraffcnhdl', 'joint_trafo_rotmat_mdh_sym_varpar'}, ...
      {'jacobiRfcnhdl', 'jacobiR_rot_sym_varpar'}, ...
      {'jacobiRDfcnhdl', 'jacobiRD_rot_sym_varpar'}, ...
      {'jacobigfcnhdl', 'jacobig_sym_varpar', 'jacobig_mdh_num'}, ...
      {'jacobigDfcnhdl', 'jacobigD_sym_varpar', 'jacobigD_mdh_num'}, ...
      {'jacobitfcnhdl', 'jacobia_transl_sym_varpar'}, ...
      {'jacobitDfcnhdl', 'jacobiaD_transl_sym_varpar'}, ...
      {'jacobiwfcnhdl', 'jacobig_rot_sym_varpar'}, ...
      {'jacobiwDfcnhdl', 'jacobigD_rot_sym_varpar'}, ...
      {'jacobigcutfcnhdl', 'jacobig_cutforce_mdh_num'}, ...
      {'invkinfcnhdl', 'invkin_eulangresidual'}, ...
      {'invkintrajfcnhdl', 'invkin_traj'}, ...
      {'ekinfcnhdl2', 'energykin_fixb_slag_vp2'}, ...
      {'epotfcnhdl2', 'energypot_fixb_slag_vp2'}, ...
      {'gravlfcnhdl2', 'gravloadJ_floatb_twist_slag_vp2'}, ...
      {'inertiafcnhdl2', 'inertiaJ_slag_vp2'}, ...
      {'corvecfcnhdl2', 'coriolisvecJ_fixb_slag_vp2'}, ...
      {'cormatfcnhdl2', 'coriolismatJ_fixb_slag_vp2'}, ...
      {'invdynfcnhdl2', 'invdynJ_fixb_slag_vp2'}, ...
      {'intforcefcnhdl2', 'invdynf_fixb_snew_vp2'}, ...
      {'intforcemcnhdl2', 'invdynm_fixb_snew_vp2'}, ...
      {'ekinfcnhdl4', 'energykin_fixb_mdp_slag_vp'}, ...
      {'epotfcnhdl4', 'energypot_fixb_mdp_slag_vp'}, ...
      {'gravlfcnhdl4', 'gravloadJ_floatb_twist_mdp_slag_vp'}, ...
      {'inertiafcnhdl4', 'inertiaJ_mdp_slag_vp'}, ...
      {'corvecfcnhdl4', 'coriolisvecJ_fixb_mdp_slag_vp'}, ...
      {'cormatfcnhdl4', 'coriolismatJ_fixb_mdp_slag_vp'}, ...
      {'invdynfcnhdl4', 'invdynJ_fixb_mdp_slag_vp'}, ...
      {'intforcefregfcnhdl', 'invdynf_fixb_reg2_snew_vp'}, ...
      {'intforcemregfcnhdl', 'invdynm_fixb_reg2_snew_vp'}, ...
      {'ekinregfcnhdl3', 'energykin_fixb_reg2_slag_vp'}, ...
      {'ekinregfcnhdl4', 'energykin_fixb_regmin_slag_vp'}, ...
      {'epotregfcnhdl3', 'energypot_fixb_reg2_slag_vp'}, ...
      {'epotregfcnhdl4', 'energypot_fixb_regmin_slag_vp'}, ...
      {'gravlregfcnhdl3', 'gravloadJ_reg2_slag_vp'}, ...
      {'gravlregfcnhdl4', 'gravloadJ_regmin_slag_vp'}, ...
      {'inertiaregfcnhdl3', 'inertiaJ_reg2_slag_vp'}, ...
      {'inertiaregfcnhdl4', 'inertiaJ_regmin_slag_vp'}, ...
      {'corvecregfcnhdl3', 'coriolisvecJ_fixb_reg2_slag_vp'}, ...
      {'corvecregfcnhdl4', 'coriolisvecJ_fixb_regmin_slag_vp'}, ...
      {'cormatregfcnhdl3', 'coriolismatJ_fixb_reg2_slag_vp'}, ...
      {'cormatregfcnhdl4', 'coriolismatJ_fixb_regmin_slag_vp'}, ...
      {'invdynregfcnhdl3', 'invdynJ_fixb_reg2_slag_vp'}, ...
      {'invdynregfcnhdl4', 'invdynJ_fixb_regmin_slag_vp'}, ...
      {'invdynregmattrajfcnhdl', 'invdynJ_fixb_regmin_slag_vp_traj'}, ...
      {'invdyntrajfcnhdl4', 'invdynJ_fixb_mdp_slag_vp_traj'}, ...
      {'invdyntrajfcnhdl6', 'invdynJ_fixb_mdp_slag_vr_traj'}, ...
      {'dynparconvfcnhdl', 'convert_par2_MPV_fixb'}, ...
      {'jointvarfcnhdl', 'kinconstr_expl_mdh_sym_varpar', 'kinconstr_expl_mdh_num_varpar'}};
      qunit_eng = cell(R.NJ,1);
      qunit_sci = cell(R.NJ,1);
      tauunit_sci = cell(R.NJ,1);
      qunitmult_eng_sci = NaN(R.NJ,1);
      for i = 1:R.NJ
        if R.MDH.sigma(i) == 0
          qunit_eng{i} = 'deg';
          qunit_sci{i} = 'rad';
          tauunit_sci{i} = 'Nm';
          qunitmult_eng_sci(i) = pi/180;
        else
          qunit_eng{i} = 'mm';
          qunit_sci{i} = 'm';
          tauunit_sci{i} = 'N';
          qunitmult_eng_sci(i) = 1/1000;
        end
      end
      R.qunit_eng = qunit_eng;
      R.qunit_sci = qunit_sci;
      R.qunitmult_eng_sci = qunitmult_eng_sci;
      R.tauunit_sci = tauunit_sci;
      R.CADstruct = struct('filepath', {}, 'link', [], 'T_body_visual', NaN(4,4,0), 'color', {});
      structkinpar_hdl = eval(sprintf('@%s_structural_kinematic_parameters', R.mdlname));
      try
        [~,~,~,~,~,~,pkin_names] = structkinpar_hdl();
        if ~strcmp(R.mdlname, R.mdlname_var)
          % Parameternamen für Variante anpassen (weniger Parameter)
          gen2var_hdl = eval(sprintf('@%s_pkin_gen2var', R.mdlname_var));
          pkin_names = gen2var_hdl(pkin_names);
        end
        R.pkin_names = pkin_names;
      catch
        warning('Funktion %s ist nicht aktuell', char(structkinpar_hdl));
        R.pkin_names = cell(1,length(R.pkin));
      end
      R.islegchain = false;
      % Struktur der Kollisions-Ersatzkörper. Es können 0, 1 oder mehrere
      % Ersatzkörper für jeden Starrkörper definiert werden. Die
      % Ersatzkörper sind auf das Körper-KS bezogen. Möglichkeiten:
      % * 1 Quader (10 Parameter: Aufpunkt, 2 Kantenvektoren, Länge 3. Kante)
      % * 2 Zylinder (7 Parameter: Punkt 1, Punkt 2, Radius)
      % * 3 Kapsel (7 Parameter: Punkt 1, Punkt 2, Radius)
      % * 4 Kugel (4 Parameter: Mittelpunkt, Radius)
      % * 5 Zylinder als schräge DH-Verbindung (1 Parameter: Radius)
      % * 6 Kapsel als schräge DH-Verbindung (1 Parameter: Radius)
      % * 7 Zylinder als gewinkelte DH-Verbindung (entlang a- und d-Parameter)
      % * 8 Kapsel als gewinkelte DH-Verbindung (1 Parameter: Radius)
      % * 9 Punkt am DH-KS-Ursprung (0 Parameter)
      R.collbodies = struct( ...
        'link', [], ... % nx1 uint8, Nummer des zugehörigen Segments (0=Basis)
        'type', [], ... % nx1 uint8, Art des Ersatzkörpers
        'params', []); % Parameter des jeweiligen Ersatzkörpers
    end
    
    function mex_dep(R, force)
      % Kompiliere alle abhängigen Funktionen dieses Roboters
      % Eingabe:
      % force (optional):
      % Erzwinge die Prüfung der Kompilierung aller Funktionen.
      fcnhdl_str = cell(3*length(R.all_fcn_hdl),1); % Liste zu kompilierender Funktionen (zu großinitialisieren)
      k=0; % laufende Nummer der zu kompilierenden Funktionen
      for i = 1:length(R.all_fcn_hdl)
        ca = R.all_fcn_hdl{i};
        % Alle Optionen für dieses Funktions-Handle durchgehen
        for j = 2:length(ca)
          fcnname = ca{j};
          if isempty(which(sprintf('%s_%s', R.mdlname, fcnname)))
            % Funktionsdatei gibt es nicht. Daher auch keine Kompilierung möglich
            continue
          end
          if nargin == 1 || ~force
            % nur Funktionen hinzufügen, deren mex-Dateien fehlen. Nichts neu kompilieren
            if ~isempty(which(sprintf('%s_%s_mex', R.mdlname, fcnname)))
              continue % diese Funktion nicht zur mex-Liste hinzufügen
            end
          end
          k=k+1;
          fcnhdl_str{k} = sprintf('%s_%s', R.mdlname, fcnname);
        end
      end
      if k == 0
        % keine noch zu kompilierenden Funktionen fehlen
        return
      end
      matlabfcn2mex(fcnhdl_str(1:k));
    end
    
    function [T, T_stack] = jtraf(R, q)
      % Homogene Transformationsmatrizen der einzelnen Gelenk-Transformationen
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % T: Transformationsmatrizen
      % T_stack: Gestapelte Transformationsmatrizen (jew. ohne 0001-Zeile)
      if nargout <= 1
        T = R.jtraffcnhdl(q, R.pkin_gen);
      else
        [T, T_stack] = R.jtraffcnhdl(q, R.pkin_gen);
      end
    end

    
    function [Tc_0, Tc_W, Tc_stack] = fkine(R, q)
      % Direkte Kinematik vom Inertial-KS zu den Körper-KS
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % Tc_0: Kumulierte Transformationsmatrizen von der Basis zu den Körper-KS
      % Tc_W: Bezugssystem ist das Welt-KS
      % Tc_stack: Gestapelte homogene Transformationsmatrizen für q (jew. ohne 0001-Zeile)
      if nargout == 3
        [Tc_0, Tc_stack] = R.fkinfcnhdl(q, R.pkin_gen);
      else
        Tc_0 = R.fkinfcnhdl(q, R.pkin_gen);
      end
      if nargout >= 2
        Tc_W = Tc_0;
        for i = 1:size(Tc_0,3)
          Tc_W(:,:,i) = R.T_W_0 * Tc_0(:,:,i);
        end
      end
    end
    function [Tc_0, Tc_W] = fkine_vp(R, q, pkin2)
      % Direkte Kinematik vom Inertial-KS zu den Körper-KS
      % wie fkine, nur mit variablen Kinematikparametern
      % Eingabe:
      % q: Gelenkkoordinaten
      % pkin2: Vektor der Kinematikparameter
      if ~strcmp(R.mdlname, R.mdlname_var)
        % Umrechnung von Parametern des abgeleiteten Modells auf allgemeines
        var2gen_hdl = eval(sprintf('@%s_pkin_var2gen', R.mdlname_var));
        pkin3 = var2gen_hdl(pkin2);
      else
        pkin3 = pkin2;
      end
      Tc_0 = R.fkinfcnhdl(q, pkin3);
      if nargout == 2
        Tc_W = Tc_0;
        for i = 1:size(Tc_0,3)
          Tc_W(:,:,i) = R.T_W_0 * Tc_0(:,:,i);
        end
      end
    end
    function [T_0_E, T_W_E] = fkineEE(R, q)
      % Direkte Kinematik zum End-Effektor
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % Homogene Transformationsmatrix von Basis-KS bzw. Welt-KS zum EE-KS
      if nargout == 2
        [Tc_0, Tc_W] = R.fkine(q);
        T_W_E = Tc_W(:,:,R.I_EElink+1)*R.T_N_E;
      else
        Tc_0 = R.fkine(q);
      end
      T_0_E = Tc_0(:,:,R.I_EElink+1)*R.T_N_E;
    end
    function [T_0_E, T_W_E] = fkineEE_vp(R, q, pkin2)
      % Direkte Kinematik zum End-Effektor
      % wie fkineEE, nur mit variablen Kinematikparametern
      [Tc_0, Tc_W] = R.fkine_vp(q, pkin2);
      T_W_E = Tc_W(:,:,R.I_EElink+1)*R.T_N_E;
      T_0_E = Tc_0(:,:,R.I_EElink+1)*R.T_N_E;
    end
    function [X,XD,XDD] = fkineEE_traj(R, Q, QD, QDD)
      % Direkte Kinematik für komplette Trajektorie berechnen
      % Eingabe:
      % Q: Gelenkkoordinaten (Trajektorie)
      % QD: Gelenkgeschwindigkeiten (Trajektorie)
      % QDD: Gelenkbeschleunigung (Trajektorie)
      %
      % Ausgabe:
      % X: EE-Lage bzgl Basis-KS (als Zeitreihe)
      % XD, XDD: Zeitableitungen
      X = NaN(size(Q,1),6);
      XD = X; XDD = X;
      for i = 1:size(Q,1)
        T_0_E_i = fkineEE(R, Q(i,:)');
        X(i,:) = R.t2x(T_0_E_i);
        Ja = jacobia(R, Q(i,:)');
        XD(i,:) = Ja*QD(i,:)';
        JaD = jacobiaD(R, Q(i,:)', QD(i,:)');
        XDD(i,:) = Ja*QDD(i,:)' + JaD*QD(i,:)';
      end
    end
    function Ja = jacobia(R, q)
      % Analytische Jacobi-Matrix des Roboters (End-Effektor)
      % Bezogen auf die gewählte Euler-Winkel-Konvention
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % Ja: Jacobi-Matrix
      
      % analytische Jacobi-Matrix berechnen
      % [Rob1] Kap. 4.3, Gl. 4.25
      Jg = R.jacobig(q);
      T_E = R.fkineEE(q);
      phi = r2eul(T_E(1:3,1:3), R.phiconv_W_E);
      Tw = euljac(phi, R.phiconv_W_E);
      Ja = [Jg(1:3,:); Tw \ Jg(4:6,:)];
    end
    function JR = jacobiR(R, q)
      % Jacobi-Matrix bezüglich der Rotationsmatrix des Endeffektors
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % JR: Jacobi-Matrix
      JR = R.jacobiRfcnhdl(q, uint8(R.I_EElink), R.pkin_gen);
    end
    function Jg = jacobig(R, q)
      % Geometrische Jacobi-Matrix (bzgl Winkelgeschwindigkeit)
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % Jg: Jacobi-Matrix
      Jg = R.jacobigfcnhdl(q, uint8(R.I_EElink), R.r_N_E, R.pkin_gen);
    end
    function Jt = jacobit(R, q)
      % Translatorischer Teil der geometrischen Jacobi-Matrix (Zusammenhang
      % zwischen translatorischer Geschwindigkeit des EE und Gelenkgeschwindigkeit)
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % Jt: Jacobi-Matrix
      Jt = R.jacobitfcnhdl(q, uint8(R.I_EElink), R.r_N_E, R.pkin_gen);
    end
    function Jw = jacobiw(R, q)
      % Rotatorischer Teil der geometrischen Jacobi-Matrix (Zusammenhang
      % zwischen Winkelgeschwindigkeit des EE und Gelenkgeschwindigkeit)
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % Jw: Jacobi-Matrix
      Jw = R.jacobiwfcnhdl(q, uint8(R.I_EElink), R.pkin_gen);
    end
    function JRD = jacobiRD(R, q, qD)
      % Zeitableitung der Jacobi-Matrix bezüglich der Rotationsmatrix des Endeffektors
      % Eingabe:
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeiten
      %
      % Ausgabe:
      % JRD: Jacobi-Matrix-Zeitableitung
      JRD = R.jacobiRDfcnhdl(q, qD, uint8(R.I_EElink), R.pkin_gen);
    end
    function JtD = jacobitD(R, q, qD)
      % Zeitableitung des Translatorischen Teils der geometrischen Jacobi-Matrix (Zusammenhang
      % zwischen translatorischer Geschwindigkeit des EE und Gelenkgeschwindigkeit)
      % Eingabe:
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeiten
      %
      % Ausgabe:
      % JtD: Jacobi-Matrix-Zeitableitung
      JtD = R.jacobitDfcnhdl(q, qD, uint8(R.I_EElink), R.r_N_E, R.pkin_gen);
    end
    function JgD = jacobigD(R, q, qD)
      % Zeitableitung der Geometrischen Jacobi-Matrix (bzgl Winkelbeschleunigung)
      % Eingabe:
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeiten
      %
      % Ausgabe:
      % JgD: Jacobi-Matrix-Zeitableitung
      JgD = R.jacobigDfcnhdl(q, qD, uint8(R.I_EElink), R.r_N_E, R.pkin_gen);
    end
    function JwD = jacobiwD(R, q, qD)
      % Zeitableitung des rotatorischer Teils der geometrischen Jacobi-Matrix
      % (Zusammenhang zwischen Winkelgeschwindigkeit des EE und Gelenkgeschwindigkeit)
      % Eingabe:
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeiten
      %
      % Ausgabe:
      % JDW: Jacobi-Matrix-Zeitableitung
      JwD = R.jacobiwDfcnhdl(q, qD, uint8(R.I_EElink), R.pkin_gen);
    end
    function JaD = jacobiaD(R, q, qD)
      % Zeitableitung der analytischen Jacobi-Matrix des Roboters (End-Effektor)
      % Bezogen auf die gewählte Euler-Winkel-Konvention
      % Eingabe:
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeiten
      %
      % Ausgabe:
      % JDa: Zeitableitung der Jacobi-Matrix
      
      % Siehe auch: Aufzeichnungen vom 28.11.2018
      
      % Zeitableitung translatorische Teil-Matrix
      JtD = R.jacobitD(q, qD);
      % Rotatorische Teilmatrix (geometrisch)
      Jw = R.jacobiw(q);
      % Zeitableitund der rotatorischen Teilmatrix
      JwD = R.jacobiwD(q, qD);
      % Endeffektor-Orientierung mit Euler-Winkeln
      T_E = R.fkineEE(q);
      phi = r2eul(T_E(1:3,1:3), R.phiconv_W_E);
      % Euler-Transformationsmatrix
      Tw = euljac(phi, R.phiconv_W_E);
      % Rotatorischer Teil der analytischen Jacobi (e="Euler")
      Je = Tw \ Jw;
      % Zeitableitung der Euler-Winkel
      phiD = Je*qD;
      % Zeitableitung der Euler-Transformationsmatrix
      TDw = euljacD(phi, phiD, R.phiconv_W_E);
      % Zeitableitung der inversen Euler-Transformationsmatrix
      TwD_inv = -Tw\TDw/Tw;
      % Zeitableitung der analytischen Jacobi (Rotationsteil)
      JeD = Tw\JwD + TwD_inv *Jw;
      % Gesamtmatrix
      JaD = [JtD; JeD];
    end
    function [q, Phi, Tc_stack0, Stats] = invkin2(R, x, q0, s_in)
      % Berechne die inverse Kinematik mit eigener Funktion für den Roboter
      % Die Berechnung erfolgt dadurch wesentlich schneller als durch die
      % Klassen-Methode `invkin`, die nicht kompilierbar ist.
      % Eingabe:
      % x: EE-Lage (Soll)
      % q0: Start-Pose
      % s_in: Einstellparameter für die IK. Felder, siehe Implementierung.
      %
      % Ausgabe:
      % q: Gelenkposition
      % Phi: Residuum
      % Tc_stack0: Gestapelte Transformationsmatrizen; siehe SerRob/fkine
      % Stats
      %   Struktur mit Detail-Ergebnissen für den Verlauf der Berechnung
      % 
      % Siehe auch: invkin

      % Einstellungen zusammenstellen:
      sigmaJ = R.MDH.sigma(R.MDH.mu>=1);
      
      % Alle Einstellungen in Eingabestruktur für Funktion schreiben
      s = struct('pkin', R.pkin_gen, ...
                 'sigmaJ', sigmaJ, ...
                 'qlim', R.qlim, ...
                 'I_EE', R.I_EE_Task, ...
                 'phiconv_W_E', R.phiconv_W_E, ...
                 'I_EElink', uint8(R.I_EElink), ...
                 'reci', true, ...
                 'T_N_E', R.T_N_E, ...
                 'K', ones(R.NQJ,1), ... % Verstärkung 1 am besten
                 'Kn', ones(R.NQJ,1), ... % Verstärkung 1 am besten
                 'wn', zeros(2,1), ... % Gewichtung der Nebenbedingung
                 'scale_lim', 0.0, ... % Herunterskalierung bei Grenzüberschreitung
                 'maxrelstep', 0.05, ... % Maximale auf Grenzen bezogene Schrittweite
                 'normalize', true, ... % Normalisieren auf +/- 180°
                 'n_min', 0, ... % Minimale Anzahl Iterationen
                 'n_max', 1000, ... % Maximale Anzahl Iterationen
                 'rng_seed', NaN, ... Initialwert für Zufallszahlengenerierung
                 'Phit_tol', 1e-10, ... % Toleranz für translatorischen Fehler
                 'Phir_tol', 1e-10, ... % Toleranz für rotatorischen Fehler
                 'retry_limit', 100); % Anzahl der Neuversuche);
      % Alle Standard-Einstellungen mit in s_in übergebenen Einstellungen
      % überschreiben. Diese Reihenfolge ermöglicht für Kompilierung
      % geforderte gleichbleibende Feldreihenfolge in Eingabevariablen
      if nargin == 4
        for f = fields(s_in)'
          if isfield(s, f{1})
            s.(f{1}) = s_in.(f{1});
          else % Fall soll eigentlich nicht vorkommen. Daher Prüfung als zweites
            error('Feld %s aus s_in kann nicht übergeben werden', f{1});
          end
        end
      end
      % Funktionsaufruf. Entspricht robot_invkin_eulangresidual.m.template
      if nargout == 3
        [q, Phi, Tc_stack0] = R.invkinfcnhdl(x, q0, s);
      elseif nargout <= 2
        [q, Phi] = R.invkinfcnhdl(x, q0, s);
      else
        [q, Phi, Tc_stack0, Stats] = R.invkinfcnhdl(x, q0, s);
      end
    end
    function [Q,QD,QDD,PHI,JointPos_all] = invkin2_traj(R, X, XD, XDD, T, q0, s_in)
      % Berechne die inverse Kinematik mit eigener Funktion für den Roboter
      % Die Berechnung erfolgt dadurch wesentlich schneller als durch die
      % Klassen-Methode `invkin_traj`, die nicht kompilierbar ist.
      % Eingabe:
      % X: EE-Lagen (Zeilen: Zeit, Spalten: EE-Koordinaten)
      % XD: EE-Geschwindigkeiten (in Euler-Winkeln)
      % XDD: EE-Beschleunigungen (Euler-Winkel)
      % T: Zeitbasis
      % q0: Start-Pose
      % s_in: Einstellparameter für die IK. Felder, siehe Implementierung.
      %
      % Ausgabe:
      % Q: Gelenkpositionen (Zeilen: Zeit, Spalten: Gelenkkoordinaten)
      % QD: Gelenkgeschwindigkeiten
      % QDD: Gelenkbeschleunigungen
      % PHI: IK-Fehler (entspricht Zwangsbedingungen)
      % JointPos_all: Position aller Körper-KS für alle Zeitschritte
      %
      % Siehe auch: invkin2, invkin_traj
      
      % Einstellungen zusammenstellen
      sigmaJ = R.MDH.sigma(R.MDH.mu>=1);
      s = struct( ...
         'pkin', R.pkin_gen, ...
         'sigmaJ', sigmaJ, ...
         'qlim', R.qlim, ...
         'qDlim', R.qDlim, ...
         'I_EE', R.I_EE_Task, ...
         'phiconv_W_E', R.phiconv_W_E, ...
         'I_EElink', uint8(R.I_EElink), ...
         'reci', true, ... % Reziproke Euler-Winkel für Orientierungs-Residuum
         'simplify_acc', false, ... % Vereinfachte Berechnung der Beschleunigung
         'T_N_E', R.T_N_E, ...
         'K', ones(R.NQJ,1), ... % Verstärkung 1 am besten
         'wn', zeros(4,1), ... % Gewichtung der Nebenbedingung
         'scale_lim', 0.1, ... % Herunterskalierung bei Grenzüberschreitung
         'maxrelstep', 0.1, ... % Maximale auf Grenzen bezogene Schrittweite
         'normalize', true, ... % Normalisieren auf +/- 180°
         'n_min', 0, ... % Minimale Anzahl Iterationen
         'n_max', 1000, ... % Maximale Anzahl Iterationen
         'rng_seed', NaN, ... Initialwert für Zufallszahlengenerierung
         'Phit_tol', 1e-10, ... % Toleranz für translatorischen Fehler
         'Phir_tol', 1e-10, ... % Toleranz für rotatorischen Fehler
         'retry_limit', 100); % Anzahl der Neuversuche
      if nargin == 7
        for f = fields(s_in)'
          if ~isfield(s, f{1})
            warning('Feld %s kann nicht übergeben werden', f{1});
          else
            s.(f{1}) = s_in.(f{1});
          end
        end
      end
      % Funktionsaufruf. Entspricht robot_invkin_traj.m.template
      [Q,QD,QDD,PHI,JointPos_all] = R.invkintrajfcnhdl(X, XD, XDD, T, q0, s);
    end
    function [T, Treg] = ekin(R, q, qD)
      % Kinetische Energie
      % Eingabe:
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeiten
      %
      % Ausgabe:
      % T: Kinetische Energie
      % Treg: Regressor-Matrix
      if R.DynPar.mode == 2
        T = R.ekinfcnhdl2(q, qD, R.pkin_gen, R.DynPar.mges, R.DynPar.mrSges, R.DynPar.Ifges);
      elseif R.DynPar.mode == 3
        Treg = R.ekinregfcnhdl3(q, qD, R.pkin_gen);
        T = Treg * R.DynPar.ipv;
      elseif R.DynPar.mode == 4
        % TODO: Eigene Funktion mit eingesetztem MPV existiert noch nicht
        % T = R.ekinfcnhdl4(q, qD, R.pkin_gen, R.DynPar.mpv);
        Treg = R.ekinregfcnhdl4(q, qD, R.pkin_gen);        
        T = Treg * R.DynPar.mpv;
      else
        error('Modus %d noch nicht implementiert', R.DynPar.mode);
      end
    end
    function [U, Ureg] = epot(R, q)
      % Potentielle Energie
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % Ausgabe:
      % U: Potentielle Energie (bezogen auf Null-Konfiguration)
      % Ureg: Regressor-Matrix
      % 
      % TODO: Bei jeder Methode kommt eine andere Energie raus, da der
      % Bezug anders ist. Daher jedes mal erneute Berechnung von U0
      if R.DynPar.mode == 2
        Uq = R.epotfcnhdl2(q,   R.gravity, R.pkin_gen, R.DynPar.mges, R.DynPar.mrSges);
        U0 = R.epotfcnhdl2(q*0, R.gravity, R.pkin_gen, R.DynPar.mges, R.DynPar.mrSges);
        U = Uq - U0;
      elseif R.DynPar.mode == 3
        Uregq = R.epotregfcnhdl3(q,   R.gravity, R.pkin_gen);
        Ureg0 = R.epotregfcnhdl3(q*0, R.gravity, R.pkin_gen);
        Ureg = Uregq-Ureg0;
        U = Ureg * R.DynPar.ipv;
      elseif R.DynPar.mode == 4
        % TODO: Eigene Funktion mit eingesetztem MPV existiert noch nicht
        % U = R.epotfcnhdl4(q, R.gravity, R.pkin, R.mpv);
        Uregq = R.epotregfcnhdl4(q,   R.gravity, R.pkin_gen);
        Ureg0 = R.epotregfcnhdl4(q*0, R.gravity, R.pkin_gen);
        Ureg = Uregq-Ureg0;
        U = Ureg*R.DynPar.mpv;
      else
        error('Modus %d noch nicht implementiert', R.DynPar.mode);
      end
    end
    function [gq, gqreg] = gravload(R, q)
      % Potentielle Energie
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % gq: Gravitations-Gelenkmoment
      % gqreg: Regressor-Matrix
      if R.DynPar.mode == 2
        gq = R.gravlfcnhdl2(q, R.gravity, R.pkin_gen, R.DynPar.mges, R.DynPar.mrSges);
      elseif R.DynPar.mode == 3
        gqreg = R.gravlregfcnhdl3(q, R.gravity, R.pkin_gen);
        gq = gqreg * R.DynPar.ipv;
      elseif R.DynPar.mode == 4
        gq = R.gravlfcnhdl4(q, R.gravity, R.pkin_gen, R.DynPar.mpv);
        if nargout == 2
          gqreg = R.gravlregfcnhdl4(q, R.gravity, R.pkin_gen);
        end
      else
        error('Modus %d noch nicht implementiert', R.DynPar.mode);
      end
    end
    function [Mq, Mqreg] = inertia(R, q)
      % Massenmatrix (in Gelenkkoordinaten)
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % Mq: Massenmatrix
      % Mqreg: Regressor-Matrix
      if R.DynPar.mode == 2
        Mq = R.inertiafcnhdl2(q, R.pkin_gen, R.DynPar.mges, R.DynPar.mrSges, R.DynPar.Ifges);
      elseif R.DynPar.mode == 3
        Mqreg = R.inertiaregfcnhdl3(q, R.pkin_gen);
        Mq = vec2symmat(Mqreg * R.DynPar.ipv);
      elseif R.DynPar.mode == 4
        Mq = R.inertiafcnhdl4(q, R.pkin_gen, R.DynPar.mpv);
        if nargout == 2
          Mqreg = R.inertiaregfcnhdl4(q, R.pkin_gen);
        end
      else
        error('Modus %d noch nicht implementiert', R.DynPar.mode);
      end
    end
    function [cq, cqreg] = corvec(R, q, qD)
      % Gelenkvektor der Flieh- und Corioliskräfte
      % Eingabe:
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeiten
      %
      % Ausgabe:
      % cq: Gelenkkräfte für Flieh- und Corioliskräfte
      % cqreg: Regressor-Matrix
      if R.DynPar.mode == 2
        cq = R.corvecfcnhdl2(q, qD, R.pkin_gen, R.DynPar.mges, R.DynPar.mrSges, R.DynPar.Ifges);
      elseif R.DynPar.mode == 3
        cqreg = R.corvecregfcnhdl3(q, qD, R.pkin_gen);
        cq = cqreg * R.DynPar.ipv;
      elseif R.DynPar.mode == 4
        cq = R.corvecfcnhdl4(q, qD, R.pkin_gen, R.DynPar.mpv);
        if nargout == 2
          cqreg = R.corvecregfcnhdl4(q, qD, R.pkin_gen);
        end
      else
        error('Modus %d noch nicht implementiert', R.DynPar.mode);
      end
    end
    function [Cq, Cqreg] = cormat(R, q, qD)
      % Matrix der Flieh- und Corioliskräfte
      % Eingabe:
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeiten
      %
      % Ausgabe:
      % Cq: Coriolis-Matrix
      % Cqreg: Regressor-Matrix
      if R.DynPar.mode == 2
        Cq = R.cormatfcnhdl2(q, qD, R.pkin_gen, R.DynPar.mges, R.DynPar.mrSges, R.DynPar.Ifges);
      elseif R.DynPar.mode == 3
        Cqreg = R.cormatregfcnhdl3(q, qD, R.pkin_gen);
        Cq = reshape(Cqreg*R.DynPar.ipv, R.NQJ, R.NQJ)';
      elseif R.DynPar.mode == 4
        % TODO: Funktion existiert noch nicht
        % Cq = R.cormatfcnhdl4(q, qD, R.pkin_gen, R.DynPar.mpv);
        Cqreg = R.cormatregfcnhdl4(q, qD, R.pkin_gen);
        Cq = reshape(Cqreg*R.DynPar.mpv, R.NQJ, R.NQJ)';
      else
        error('Modus %d noch nicht implementiert', R.DynPar.mode);
      end
    end
    function [tauq, tauqreg] = invdyn(R, q, qD, qDD)
      % Gelenkvektor der inversen Dynamik
      % Eingabe:
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeiten
      % qDD: Gelenkbeschleunigung
      %
      % Ausgabe:
      % tauq: Inverse Dynamik
      % tauqreg: Regressor-Matrix
      if R.DynPar.mode == 2
        tauq = R.invdynfcnhdl2(q, qD, qDD, R.gravity, R.pkin_gen, R.DynPar.mges, R.DynPar.mrSges, R.DynPar.Ifges);
      elseif R.DynPar.mode == 3
        tauqreg = R.invdynregfcnhdl3(q, qD, qDD, R.gravity, R.pkin_gen);
        tauq = tauqreg*R.DynPar.ipv;
      elseif R.DynPar.mode == 4
        tauq = R.invdynfcnhdl4(q, qD, qDD, R.gravity, R.pkin_gen, R.DynPar.mpv);
        if nargout == 2
          tauqreg = R.invdynregfcnhdl4(q, qD, qDD, R.gravity, R.pkin_gen);
        end
      else
        error('Methode invdyn für Modus %d noch nicht implementiert', R.DynPar.mode);
      end
    end
    function TAUq = invdyn_traj(R, Q, QD, QDD)
      % Gelenkvektor der inversen Dynamik als Trajektorie (Zeit als Zeilen)
      % Eingabe:
      % Q: Gelenkkoordinaten (Trajektorie)
      % QD: Gelenkgeschwindigkeiten (Trajektorie)
      % QDD: Gelenkbeschleunigung (Trajektorie)
      %
      % Ausgabe:
      % TAUq: Inverse Dynamik (als Zeitreihe)
      TAUq = NaN(size(Q));
      for i = 1:size(Q,1)
        TAUq(i,:) = R.invdyn(Q(i,:)', QD(i,:)', QDD(i,:)');
      end
    end
    function TAUq = invdyn2_traj(R, Q, QD, QDD)
      % Gelenkvektor der inversen Dynamik als Trajektorie (Zeit als Zeilen)
      % Benutzt eine eigene Funktion für die Inverse Dynamik der Traj.
      % Eingabe:
      % Q: Gelenkkoordinaten (Trajektorie)
      % QD: Gelenkgeschwindigkeiten (Trajektorie)
      % QDD: Gelenkbeschleunigung (Trajektorie)
      %
      % Ausgabe:
      % TAUq: Inverse Dynamik (als Zeitreihe)
      if R.DynPar.mode == 4
        TAUq = R.invdyntrajfcnhdl4(Q, QD, QDD, R.gravity, R.pkin_gen, R.DynPar.mpv);
      else
        % Benutze Schleife über die Zeit aus Klassenmethode statt eigener
        % Funktion für InvDyn-Trajektorie
        TAUq = R.invdyn_traj(Q, QD, QDD);
      end
    end
    function TAUq = invdyn3_traj(R, RV)
      % Gelenkvektor der inversen Dynamik als Trajektorie (Zeit als Zeilen)
      % Benutzt eine eigene Funktion für die Inverse Dynamik der Traj.
      % Eingabe:
      % RV: Regressormatrizen (als Zeitreihe)
      %
      % Ausgabe:
      % TAUq: Inverse Dynamik (als Zeitreihe)
      % Siehe auch: [SchapplerTapOrt2019]
      if R.DynPar.mode == 4
        TAUq = R.invdyntrajfcnhdl6(RV, R.DynPar.mpv);
      else
        error('Methode invdyn3_traj für Modus %d noch nicht implementiert', R.DynPar.mode);
      end
    end
    function RV = invdynregmat_traj(R, Q, QD, QDD)
      % Zeitreihe der Dynamik-Regressor-Matrix für eine Trajektorie
      % Eingabe:
      % Q: Gelenkkoordinaten (Trajektorie)
      % QD: Gelenkgeschwindigkeiten (Trajektorie)
      % QDD: Gelenkbeschleunigung (Trajektorie)
      %
      % Ausgabe:
      % RV: Regressormatrizen (als Zeitreihe)
      RV = R.invdynregmattrajfcnhdl(Q, QD, QDD, R.gravity, R.pkin_gen);
    end
    function [W, W_reg] = internforce(R, q, qD, qDD)
      % Interne Schnittkräfte resultierend aus internen Kräften
      % (inverser Dynamik)
      % Eingabe:
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeiten
      % qDD: Gelenkbeschleunigung
      %
      % Ausgabe:
      % W: Kraft und Moment in allen Gelenken (Zeilen: fx,fy,fz,mx,my,mz;
      %    Spalten: Basis, Robotergelenke)
      % W_reg: Regressormatrix (bezogen auf Inertialparameter-Vektor mit Basis)
      f_i_i_ges = R.intforcefcnhdl2(q, qD, qDD, R.gravity, R.pkin_gen, ...
        R.DynPar.mges, R.DynPar.mrSges, R.DynPar.Ifges);
      n_i_i_ges = R.intforcemcnhdl2(q, qD, qDD, R.gravity, R.pkin_gen, ...
        R.DynPar.mges, R.DynPar.mrSges, R.DynPar.Ifges);
      if nargout == 2
        f_reg = R.intforcefregfcnhdl(q, qD, qDD, R.gravity, R.pkin_gen);
        m_reg = R.intforcemregfcnhdl(q, qD, qDD, R.gravity, R.pkin_gen);
        W_reg = [f_reg; m_reg];
      end
      W = [f_i_i_ges; n_i_i_ges];
    end
    function Jg_C = jacobig_cutforce(R, q, link_index, r_i_i_C)
      % Jacobi-Matrix zur Berechnung der Schnittkräfte in allen Gelenken
      % Eingabe:
      % q: Gelenkpositionen
      % link_index: Nummer des Robotersegments, an dem die Kraft angreift (0=Basis)
      % r_i_i_C: Punkt, an dem die Kraft angreift. Im jew. Körper-KS
      %
      % Ausgabe:
      % Jg_C: Jacobi-Matrix bezogen auf Translation und Rotation um alle
      % Achsen aller Körper-Koordinatensysteme. Zur Berechnung von
      % Schnittkräften aufgrund externer Kräfte
      if R.Type == 1
        error('Nicht für hybride Systeme definiert');
      end
      % Nur Aufruf der spezifischen Funktion. Prüfung auf Existenz kostet
      % zu viel Leistung. Alternative Berechnung mit
      % robot_tree_jacobig_cutforce_m ist auch möglich.
      Jg_C = R.jacobigcutfcnhdl(q, uint8(link_index), r_i_i_C, R.pkin_gen);
    end
    function W_ext = internforce_ext(R, q, F_ext, link_index, r_i_i_C)
      % Interne Schnittkräfte resultierend aus externen Kräften
      % Eingabe:
      % q: Gelenkpositionen
      % F_ext [6x1]: Externe Kraft und Moment (im Basis-KS)
      % link_index: Nummer des Robotersegments, an dem die Kraft angreift (0=Basis)
      % r_i_i_C: Punkt, an dem die Kraft angreift. Im jew. Körper-KS
      %
      % Ausgabe:
      % W_ext: Kraft und Moment in allen Gelenken. Siehe SerRob/internforce()
      %        (Zeilen: fx,fy,fz,mx,my,mz; Spalten: Basis, Robotergelenke)
      Jg_C = R.jacobig_cutforce(q, link_index, r_i_i_C);
      W_ext = reshape(Jg_C' * F_ext, 6, R.NL);
    end
    function [W_traj, W_traj_reg] = internforce_traj(R, Q, QD, QDD)
      % Interne Schnittkräfte
      % Eingabe:
      % Q: Gelenkkoordinaten (Trajektorie)
      % QD: Gelenkgeschwindigkeiten (Trajektorie)
      % QDD: Gelenkbeschleunigung (Trajektorie)
      %
      % Ausgabe:
      % W_traj: Kraft und Moment in allen Gelenken, als Zeitreihe
      %         Jede Zeile ein Zeitschritt; erst alle Kräfte, dann alle Momente
      % W_traj_reg: Regressormatrix zu W_traj
      W_traj = NaN(size(Q,1), R.NL*6);
      if nargout == 2
        W_traj_reg = NaN(size(Q,1), (R.NL*6)*R.NL*10);
      end
      for i = 1:size(Q,1)
        if nargout == 2
          [w_i, w_i_reg] = R.internforce(Q(i,:)', QD(i,:)', QDD(i,:)');
          W_traj_reg(i,:) = w_i_reg(:);
        else
          w_i = R.internforce(Q(i,:)', QD(i,:)', QDD(i,:)');
        end
        f_i_i_ges = w_i(1:3,:);
        n_i_i_ges = w_i(4:end,:);
        W_traj(i,:) = [f_i_i_ges(:); n_i_i_ges(:)];
      end
    end
    function W_traj = internforce3_traj(R, W_traj_reg)
      % Interne Schnittkräfte als Trajektorie (Zeit als Zeilen)
      % Unterscheidet sich gegenüber internforce_traj durch die Eingabe der
      % Regressormatrizen als gestapelte Vektoren
      % Eingabe:
      % W_traj_reg: Regressormatrizen (als Zeitreihe). Aus internforce_traj
      %
      % Ausgabe:
      % W_traj: Kraft und Moment in allen Gelenken, als Zeitreihe
      % Siehe auch: [SchapplerTapOrt2019]
      W_traj = NaN(size(W_traj_reg,1), R.NL*6);
      for i = 1:size(W_traj_reg,1)
        W_reg_i = reshape(W_traj_reg(i,:), R.NL*6, length(R.DynPar.ipv_floatb));
        W_traj(i,:) = W_reg_i*R.DynPar.ipv_floatb;
      end
    end
    function tau_s = springtorque(R, q)
      % Federmoment einer in den Gelenken angebrachten Drehfeder
      % Eingabe:
      % q: Gelenk-Koordinaten
      %
      % Ausgabe:
      % tau_s: Gelenkmoment aus Federkraft (M*qDD+c+g+tau_s=tau_m+tau_ext)
      % Das Federmoment wirkt rückstellend bei Auslenkung der Feder.
      % Die resultierende Gl. ist M*qDD+...+K*(q-q0) = tau_ext
      tau_s = R.DesPar.joint_stiffness(:).*(q-R.DesPar.joint_stiffness_qref(:));
    end
    function U_s = epotspring(R, q)
      % Energie der in der in Gelenken angebrachten Drehfeder
      % Eingabe:
      % q: Gelenk-Koordinaten
      %
      % Ausgabe:
      % U: Energie der Drehfeder
      % Die Funktion springtorque und das Vorzeichen dort ergibt sich durch
      % Ableitung dieser Energie nach q.
      U_s = 0.5*R.DesPar.joint_stiffness(:)'*(q-R.DesPar.joint_stiffness_qref(:)).^2;
    end
    function jv = jointvar(R, qJ)
      % Vektor der Gelenkkoordinaten aller (auch passiver) Gelenke
      % Eingabe:
      % qJ: Minimale Gelenkkoordinaten
      %
      % Ausgabe:
      % jv: Koordinaten aller Gelenke (nicht nur der minimalen
      % Gelenkkoordinaten, sondern auch der abhängigen Gelenke)
      if R.Type == 0
        % Serieller Roboter: Gelenkwinkel sind direkt die
        % Minimalkoordinaten der Gelenke
        jv = qJ;
      else
        % Hybrider Roboter: Berechne die Gelenkwinkel aus den
        % Minimalkoordinaten mit vorher generierter Funktion
        jv = R.jointvarfcnhdl(qJ, R.pkin_gen);
      end
    end
    function update_EE(R, r_N_E, phi_N_E, phiconv_N_E)
      % Aktualisiere die Transformationsmatrix T_N_E für den Endeffektor
      % Eingabe:
      % r_N_E: Neuer Vektor vom EE-Körper-KS zum EE
      % phi_N_E: Neue Euler-Winkel für Drehung vom EE-Körper-KS zum EE-KS
      % phiconv_N_E: Nummer der Euler-Winkel-Konvention
      if nargin > 1 && ~isempty(r_N_E)
        R.r_N_E = r_N_E;
      end
      if nargin > 2 && ~isempty(phi_N_E)
        R.phi_N_E = phi_N_E;
      end
      if nargin > 3 && ~isempty(phiconv_N_E)
        R.phiconv_N_E = phiconv_N_E;
      end
      R.T_N_E = [[eul2r(R.phi_N_E, R.phiconv_N_E), R.r_N_E]; [0 0 0 1]];
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
        R.phiconv_N_E = phiconv_W_0;
      end
      R.T_W_0 = [[eul2r(R.phi_W_0, R.phiconv_W_0), R.r_W_0]; [0 0 0 1]];
      % Speichere die Inverse. Wird für ParRob-IK häufig benötigt.
      R.T_0_W = invtr(R.T_W_0);
    end
    function pkin = update_pkin(R)
      % Aktualisiere die Variable pkin aus den gespeicherten MDH-Parametern
      % Ausgabe:
      % pkin: Vektor der Kinematikparameter; aus MDH-Parametern generiert
      
      % Prüfe, ob Kinematikparameter leer sind. Dann erstelle
      % Platzhalter-Vektor, damit die folgenden Funktionen funktionieren
      % Das ist insbesondere für hybride Systeme wichtig, bei denen die
      % MDH-Parameter nicht alle Kinematikparameter enthalten
      if isempty(R.pkin)
        structkinpar_hdl = eval(sprintf('@%s_structural_kinematic_parameters', R.mdlname));
        [~,~,~,~,NKP] = structkinpar_hdl();
        R.pkin_gen = NaN(NKP,1);
        if ~strcmp(R.mdlname, R.mdlname_var)
          gen2var_hdl = eval(sprintf('@%s_pkin_gen2var', R.mdlname_var));
          R.pkin = gen2var_hdl(R.pkin_gen);
        else
          R.pkin = R.pkin_gen;
        end
      end
      
      if R.Type == 1
        % Die Umwandlung funktioniert nicht für hybride Roboter, bei denen
        % zusätzliche Kinematikparameter in den kinematischen
        % Zwangsbedingungen definiert sind. Deshalb hier Abbruch.
        pkin = R.pkin;
        return
      end
      
      % Die Umwandlung MDH->pkin wird nur für echt serielle Roboter gemacht
      mdh2pkin_hdl = eval(sprintf('@%s_mdhparam2pkin', R.mdlname));
      pkin2mdh_hdl = eval(sprintf('@%s_pkin2mdhparam', R.mdlname));
      pkin_gen2 = mdh2pkin_hdl(R.MDH.beta, R.MDH.b, R.MDH.alpha, R.MDH.a, ...
        R.MDH.theta, R.MDH.d, zeros(R.NJ,1));
      if ~strcmp(R.mdlname, R.mdlname_var)
        gen2var_hdl = eval(sprintf('@%s_pkin_gen2var', R.mdlname_var));
        pkin = gen2var_hdl(pkin_gen2);
      else
        pkin = pkin_gen2;
      end
      
      R.pkin = pkin;
      R.pkin_gen = pkin_gen2;
      [beta,b,alpha,a,theta,d] = pkin2mdh_hdl(pkin_gen2);
      % Teste Rück-Transformation der Kinematik-Parameter
      pkin_gen_test = mdh2pkin_hdl(beta, b, alpha, a, theta, d, zeros(R.NJ,1));
      if ~strcmp(R.mdlname, R.mdlname_var)
        pkin_test = gen2var_hdl(pkin_gen_test);
      else
        pkin_test = pkin_gen_test;
      end
      if any(abs(pkin-pkin_test) > 1e-10)
        error('Parameteraktualisierung von pkin hat nicht funktioniert');
      end
    end
    function update_mdh(R, pkin_neu)
      % Aktualisiere die gespeicherten Kinematikparameter
      % Eingabe:
      % pkin_neu:
      %   Neuer Wert für den Vektor der Kinematikparameter
      %   Der Vektor ist für jeden Roboter spezifisch. Die Reihenfolge der
      %   Parameter wird bei der Code-Generierung festgelegt.
      assert(all(size(pkin_neu) == size( R.pkin )), 'Eingabe muss Dimension von pkin haben (%dx1)', length(R.pkin))
      
      if ~strcmp(R.mdlname, R.mdlname_var)
        % Dieses Modell stellt eine abgeleitete Variante eines allgemeinen
        % Modells dar
        var2gen_hdl = eval(sprintf('@%s_pkin_var2gen', R.mdlname_var));
        pkin_gen_neu = var2gen_hdl(pkin_neu);
      else
        pkin_gen_neu = pkin_neu;
      end
      
      mdh2pkin_hdl = eval(sprintf('@%s_mdhparam2pkin', R.mdlname));
      pkin2mdh_hdl = eval(sprintf('@%s_pkin2mdhparam', R.mdlname));
      
      % Berechne MDH-Parameter aus pkin-Vektor
      [beta,b,alpha,a,theta,d,qoffset] = pkin2mdh_hdl(pkin_gen_neu);
      % Teste die Rück-Transformation zu pkin (funktioniert nur für
      % serielle Roboter; bei hybriden stehen nicht alle Par. in MDH
      if R.Type == 0
        pkin_gen_test = mdh2pkin_hdl(beta, b, alpha, a, theta, d, qoffset);
        if ~strcmp(R.mdlname, R.mdlname_var)
          gen2var_hdl = eval(sprintf('@%s_pkin_gen2var', R.mdlname_var));
          pkin_test = gen2var_hdl(pkin_gen_test);
        else
          pkin_test = pkin_gen_test;
        end
        if any(abs(pkin_neu-pkin_test) > 1e-10)
          error('Parameteraktualisierung pkin->mdh hat nicht funktioniert');
        end
      end
      % Parameter in Roboterklasse belegen
      R.MDH.beta=beta; R.MDH.b=b;
      R.MDH.alpha=alpha; R.MDH.a=a;
      R.MDH.theta=theta; R.MDH.d=d;
      R.MDH.offset = qoffset;
      R.pkin = pkin_neu;
      R.pkin_gen = pkin_gen_neu;
    end
    function update_dynpar1(R, mges, rSges, Icges)
      % Aktualisiere die hinterlegten Dynamikparameter ausgehend von
      % gegebenen Parametern bezogen auf den Schwerpunkt
      % Eingabe:
      % mges: Massen aller Robotersegmente (inkl Basis)
      % rSges: Schwerpunktskoordinaten aller Robotersegmente (bezogen auf
      % jeweiliges Körper-KS)
      % Icges: Trägheitstensoren der Robotersegmente (bezogen auf Schwerpkt)
      if nargin < 2 || isempty(mges)
        mges = R.DynPar.mges;
      end
      if nargin < 3 || isempty(rSges)
        rSges = R.DynPar.rSges;
      end
      if nargin < 4 || isempty(Icges)
        Icges = R.DynPar.Icges;
      end
      [mrSges, Ifges] = inertial_parameters_convert_par1_par2(rSges, Icges, mges);
      
      % Umwandlung in gestapelten Parametervektor. Nehme floatb, da Masse
      % der Basis zur Berechnung der Schnittkräfte benutzt wird.
      PV2floatb = NaN(10*R.NL,1);
      for i = 1:R.NL
        % different order: Ifges_num_mdh: [XX,YY,ZZ,XY,XZ,YZ], PV2: [XX,XY,XZ,YY,YZ,ZZ]
        PV2floatb((1:6) +10*(i-1)) = Ifges(i,[1,4,5,2,6,3]);
        PV2floatb((1:3) +10*(i-1)+6) = mrSges(i,:);
        PV2floatb(10*i) = mges(i);
      end
      
      % Umwandlung der Dynamik-Parameter (Masse, erstes Moment, zweites
      % Moment) in Minimalparameter-Vektor
      mpv = R.dynpar_convert_par2_mpv(mges, mrSges, Ifges);

      % Parameter in Roboterklasse belegen
      R.DynPar.mges   = mges;
      R.DynPar.rSges  = rSges;
      R.DynPar.Icges  = Icges;
      R.DynPar.mrSges = mrSges;
      R.DynPar.Ifges  = Ifges;
      R.DynPar.ipv_floatb    = PV2floatb;
      R.DynPar.ipv    = PV2floatb(11:end);
      R.DynPar.mpv    = mpv;
    end

    function update_dynpar2(R, mges, mrSges, Ifges)
      % Aktualisiere die hinterlegten Dynamikparameter ausgehend von
      % gegebenen Parametern bezogen auf den Körper-KS-Ursprung
      % Eingabe:
      % mges: Massen aller Robotersegmente (inkl Basis)
      % mrSges: Schwerpunktskoordinaten aller Robotersegmente multipliziert
      % mit Masse (bezogen auf jeweiliges Körper-KS)
      % Ifges: Trägheitstensoren der Robotersegmente (bezogen auf Ursprung)
      if nargin < 2 || isempty(mges)
        mges = R.DynPar.mges;
      end
      if nargin < 3 || isempty(mrSges)
        mrSges = R.DynPar.mrSges;
      end
      if nargin < 4 || isempty(Ifges)
        Ifges = R.DynPar.Ifges;
      end
      
      % Umwandlung in gestapelten Parametervektor. Nehme floatb, da Masse
      % der Basis zur Berechnung der Schnittkräfte benutzt wird.
      PV2floatb = NaN(10*R.NL,1);
      for i = 1:R.NL
        % different order: Ifges_num_mdh: [XX,YY,ZZ,XY,XZ,YZ], PV2: [XX,XY,XZ,YY,YZ,ZZ]
        PV2floatb((1:6) +10*(i-1)) = Ifges(i,[1,4,5,2,6,3]);
        PV2floatb((1:3) +10*(i-1)+6) = mrSges(i,:);
        PV2floatb(10*i) = mges(i);
      end
      
      % Umwandlung der Dynamik-Parameter (Masse, erstes Moment, zweites
      % Moment) in Minimalparameter-Vektor
      mpv = R.dynpar_convert_par2_mpv(mges, mrSges, Ifges);
      
      % Umwandlung in baryzentrische Parameter
      [rSges, Icges] = inertial_parameters_convert_par2_par1(mrSges, Ifges, mges);
      for i = 1:size(Icges,1)
        if any(eig(inertiavector2matrix(Icges(i,:)))<0)
          error('Berechneter Trägheitstensor zu Körper %d wird negativ', i);
        end
      end

      % Parameter in Roboterklasse belegen.
      R.DynPar.mges   = mges;
      R.DynPar.rSges  = rSges;
      R.DynPar.Icges  = Icges;
      R.DynPar.mrSges = mrSges;
      R.DynPar.Ifges  = Ifges;
      R.DynPar.ipv_floatb = PV2floatb;
      R.DynPar.ipv    = PV2floatb(11:end);
      R.DynPar.mpv    = mpv;
    end

    function update_dynpar_mpv(R, mpv)
      % Aktualisiere die hinterlegten Dynamikparameter ausgehend von
      % gegebenem Minimalparameter-Vektor
      % Eingabe:
      % mpv: Minimalparameter-Vektor

      % Parameter in Roboterklasse belegen
      % Die vollständigen Dynamikparameter werden zu NaN gesetzt, da bei
      % Vorgabe des MPV diese nicht mehr bestimmt werden können (durch die
      % Minimalform gehen Informationen verloren)
      R.DynPar.mges   = NaN*R.DynPar.mges;
      R.DynPar.rSges  = NaN*R.DynPar.rSges;
      R.DynPar.Icges  = NaN*R.DynPar.Icges;
      R.DynPar.mrSges = NaN*R.DynPar.mrSges;
      R.DynPar.Ifges  = NaN*R.DynPar.Ifges;
      R.DynPar.ipv    = NaN*R.DynPar.Ifges;
      R.DynPar.mpv    = mpv;
    end
    function mpv = dynpar_convert_par2_mpv(R, mges, mrSges, Ifges)
      % Eingabe:
      % mges: Massen aller Robotersegmente (inkl Basis)
      % mrSges: Schwerpunktskoordinaten aller Robotersegmente multipliziert mit Massen
      % Ifges: Trägheitstensoren der Robotersegmente (bezogen auf KS-Ursprung)
      % Ausgabe:
      % mpv: Dynamik-Minimalparametervektor
      if isempty(R.dynparconvfcnhdl)
        % Funktion zur Umwandlung nach MPV wurde nicht generiert. Leer lassen.
        mpv = [];
      else
        mpv = R.dynparconvfcnhdl(R.pkin_gen, mges, mrSges, Ifges);
      end
    end
    function update_gravity(R, g_world)
      % Aktualisiere den Gravitationsvektor für den Roboter
      % Eingabe:
      % g_world: Gravitations-Vektor im Welt-KS
      R_W_0 = R.T_W_0(1:3,1:3);
      g_base = R_W_0' * g_world;
      % Aktualisiere Klassenvariable (Gravitation im Basis-KS)
      R.gravity = g_base;
    end
    function CAD_add(R, filepath, link, T_body_CAD, unit, color)
      % Füge die CAD-Datei für einen Körper des Roboters hinzu
      % filepath: Absoluter Pfad zur STL-Datei
      % link: Nummer des Robotersegments (0=Basis)
      % T_body_CAD: Koordinatentransformation zum Ursprung der CAD-Datei
      % unit: Einheit des STL-Modells (1e-3 für STL in Millimeter)
      % color (optional): Farbe für hinzuzufügendes CAD-Modell
      colors_default = {'k', 'r', 'g', 'b', 'c', 'm', 'y'};
      if nargin < 5
        unit = 1;
      end
      if isempty(R.CADstruct)
        R.CADstruct = struct( ...
          'filepath', cell(1,1), ...
          'link', link, ...
          'T_body_visual', T_body_CAD, ...
          'color', cell(1,1));
        R.CADstruct.filepath{1} = filepath;

        if nargin < 6
          color = colors_default{1};
        end
        R.CADstruct.color{1} = color;
        R.CADstruct.unit = unit;
      else
        i = length(R.CADstruct.link)+1;
        R.CADstruct.filepath = {R.CADstruct.filepath{:}, filepath}; %#ok<CCAT>
        R.CADstruct.link = [R.CADstruct.link; link];
        R.CADstruct.T_body_visual(:,:,i) = T_body_CAD;
        if nargin < 6
          ci = mod(i-1, 7)+1; % Wähle der Reihe nach die Standardfarben (mit Wiederholung)
          color = colors_default{ci};
        end
        R.CADstruct.color = {R.CADstruct.color{:}, color}; %#ok<CCAT>
        R.CADstruct.unit = [R.CADstruct.unit; unit];
      end
    end
  end
end
