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

% TODO: Zeitableitung der Jacobi-Matrix-Funktionen mit "JDx" statt "JxD"

%   Quellen
%   [Rob1] Robotik 1 Skript

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-08
% (C) Institut für mechatronische Systeme, Universität Hannover

classdef SerRob < matlab.mixin.Copyable

  properties (Access = public)
    qlim % Minimale und maximale Gelenkkoordinaten q zeilenweise für die Gelenke
    qref % Referenz-Gelenkstellung des Roboters (entspricht "Justage"-Position)

    qDlim % Minimale und maximale Gelenkgeschwindigkeiten zeilenweise für die Gelenke
    taulim % Minimale und maximale Gelenkkräfte zeilenweise
    descr % Beschreibung des Roboters (Längerer, ausführlicher Name)
    phiconv_W_0 % Nummer der Basis-Euler-Winkelkonvention
    phiconv_N_E % Winkelkonvention der Euler-Winkel vom EE-Körper-KS zum EE
    phiconv_W_E % Winkelkonvention zur Darstellung der EE-Orientierung im Welt-KS mit Euler-Winkeln
    MDH % Struktur mit MDH-Parametern für Roboterkinematik
    NJ % Anzahl der Gelenke des Roboters
    NL % Anzahl der Starrkörper des Roboters
    NQJ % Anzahl der Gelenkkoordinaten (abweichend von NJ bei hybriden Strukturen)
    pkin % Vektor der Kinematikparameter
    DynPar % Struktur mit Dynamikparatern (Masse, Schwerpunkt, Trägheit)
    Type % Typ des Roboters (0=seriell, 1=hybrid, 2=parallel)
    r_W_0 % Position der Basis im Welt-KS
    phi_W_0 % Orientierung des Basis-KS im Welt-KS (ausgedrückt in Euler-Winkeln)
    T_W_0 % Homogene Transformationsmatrix zwischen Welt- und Basis-KS des Roboters
    T_N_E % Homogene Transformationsmatrix zwischen 
    r_N_E % Position des Endeffektors im Körper-KS
    phi_N_E % Orientierung des EE-KS im Körper-KS (ausgedrückt in Euler-Winkeln)
    qunit_eng % Ingenieursmäßige Einheiten der Gelenkkoordinaten
    qunit_sci % wissenschaftliche Einheiten der Gelenkkoordinaten
    qunitmult_eng_sci % Umrechnungsfaktor zwischen beiden Einheitenarten
    tauunit_sci % Name der Einheiten der Gelenkkräfte
    gravity % Gravitationsvektor ausgedrückt im Welt-KS
    I_EE % Indizes der genutzten EE-FG. TODO: Funktionen darauf umstellen. Achtung bei Euler-Winkeln
    I_EElink % Index des Segmentes, an dem der Endeffektor befestigt ist.
    mdlname % Name des Robotermodells, das in den Matlab-Funktionen benutzt wird.
    CADstruct % Struktur mit Daten zu CAD-Modellen
  end
  properties (Access = private)
    jtraffcnhdl % Funktions-Handle für Gelenk-Transformationen
    fkinfcnhdl % Funktions-Handle für direkte Kinematik
    ekinfcnhdl % Funktions-Handle für kinetische Energie
    epotfcnhdl % Funktions-Handle für potentielle Energie
    gravlfcnhdl % Funktions-Handle für Gelenkkräfte durch Gravitation
    inertiafcnhdl % Funktions-Handle für Massenmatrix
    corvecfcnhdl % Funktions-Handle für Gelenkkräfte durch Coriolis- und Fliehkräfte
    cormatfcnhdl % Funktions-Handle für Coriolis-Matrix
    invdynfcnhdl % Funktions-Handle für Gelenkkräfte durch inverse Dynamik
    jacobiRfcnhdl % Funktions-Handle für Jacobi-Matrix bzgl der Rotationsmatrix des Endeffektors
    jacobigfcnhdl % Funktions-Handle für geometrische Jacobi-Matrix
    jacobigDfcnhdl % Funktions-Handle für Zeitableitung der geometrischen Jacobi-Matrix
    jointvarfcnhdl % Funktions-Handle für Werte Gelenkvariablen (bei hybriden Robotern)
    all_fcn_hdl % Cell-Array mit allen Funktions-Handles des Roboters sowie den Dateinamen der Matlab-Funktionen
  end

  methods
    % Konstruktor
    function R=SerRob(Par_struct, mdlname)
      R.mdlname = mdlname;
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

      R.MDH = Par_struct; % TODO: Nur gewünschte Felder
      R.pkin = Par_struct.pkin;
      if isempty(R.pkin)
        R.update_pkin();
      end
      R.DynPar = struct('mges', NaN(R.NL,1), 'rSges', NaN(R.NL,3), 'Icges', NaN(R.NL,6));
      R.update_dynpar1();

      R.qref = zeros(R.NQJ,1);

      R.r_N_E = zeros(3,1);
      R.phi_N_E = zeros(3,1);
      R.T_N_E = eye(4);
      R.T_W_0 = eye(4);
      R.phiconv_N_E = uint8(2); % Euler-XYZ
      R.phiconv_W_E = uint8(2); % Euler-XYZ
      R.phiconv_W_0 = uint8(2); % Euler-XYZ
      R.I_EE = true(1,6); % Initialisierung mit allen Freiheitsgraden (räumliche PKM). Muss logical sein, damit Binär-Indizes.
      R.I_EElink = R.NL-1; % bezogen auf nummerierte Körper (Basis=0)
      R.gravity = [0;0;-9.81];
      % Liste der Funktionshandle-Variablen mit zugehörigen
      % Funktionsdateien (aus Maple-Toolbox)
      R.all_fcn_hdl = { ...
      {'fkinfcnhdl', 'fkine_fixb_rotmat_mdh_sym_varpar'}, ...
      {'jtraffcnhdl', 'joint_trafo_rotmat_mdh_sym_varpar'}, ...
      {'jacobiRfcnhdl', 'jacobiR_rot_sym_varpar'}, ...
      {'jacobigfcnhdl', 'jacobig_floatb_twist_sym_varpar', 'jacobig_mdh_num'}, ...
      {'jacobigDfcnhdl', 'jacobigD_floatb_twist_sym_varpar', 'palh1m1TE_jacobigD_mdh_num'}, ...
      {'ekinfcnhdl', 'energykin_fixb_slag_vp2'}, ...
      {'epotfcnhdl', 'energypot_fixb_slag_vp2'}, ...
      {'gravlfcnhdl', 'gravloadJ_floatb_twist_slag_vp2'}, ...
      {'inertiafcnhdl', 'inertiaJ_slag_vp2'}, ...
      {'corvecfcnhdl', 'coriolisvecJ_fixb_slag_vp2'}, ...
      {'cormatfcnhdl', 'coriolismatJ_fixb_slag_vp2'}, ...
      {'invdynfcnhdl', 'invdynJ_fixb_slag_vp2'}, ...
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
          qunit_eng{i} = 'm';
          qunit_sci{i} = 'mm';
          tauunit_sci{i} = 'N';
          qunitmult_eng_sci(i) = 1/1000;
        end
      end
      R.qunit_eng = qunit_eng;
      R.qunit_sci = qunit_sci;
      R.qunitmult_eng_sci = qunitmult_eng_sci;
      R.tauunit_sci = tauunit_sci;
      R.CADstruct = struct('filepath', {}, 'link', [], 'T_body_visual', NaN(4,4,0), 'color', {});
    end
    function mex_dep(R)
      fcnhdl_str = cell(length(R.all_fcn_hdl),1);
      for i = 1:length(R.all_fcn_hdl)
        ca = R.all_fcn_hdl{i};
        fcnname = ca{2};
        fcnhdl_str{i} = sprintf('%s_%s', R.mdlname, fcnname);
      end
      matlabfcn2mex(fcnhdl_str);
    end
    function T = jtraf(R, q)
      % Homogene Transformationsmatrizen der einzelnen Gelenk-Transformationen
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % T: Transformationsmatrizen
      T = R.jtraffcnhdl(q, R.pkin);
    end
    function [Tc_0, Tc_W] = fkine(R, q)
      % Direkte Kinematik vom Inertial-KS zu den Körper-KS
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % Tc_0: Kumulierte Transformationsmatrizen von der Basis zu den Körper-KS
      % Tc_W: Bezugssystem ist das Welt-KS
      Tc_0 = R.fkinfcnhdl(q, R.pkin);
      Tc_W = Tc_0;
      for i = 1:size(Tc_0,3)
        Tc_W(:,:,i) = R.T_W_0 * Tc_0(:,:,i);
      end
    end
    function [Tc_0, Tc_W] = fkine_vp(R, q, pkin2)
      % Direkte Kinematik vom Inertial-KS zu den Körper-KS
      % wie fkine, nur mit variablen Kinematikparametern
      % Eingabe:
      % q: Gelenkkoordinaten
      % pkin2: Vektor der Kinematikparameter
      Tc_0 = R.fkinfcnhdl(q, pkin2);
      Tc_W = Tc_0;
      for i = 1:size(Tc_0,3)
        Tc_W(:,:,i) = R.T_W_0 * Tc_0(:,:,i);
      end
    end
    function [T_0_E, T_W_E] = fkineEE(R, q)
      % Direkte Kinematik zum End-Effektor
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % Homogene Transformationsmatrix von Basis-KS bzw. Welt-KS zum EE-KS
      [Tc_0, Tc_W] = R.fkine(q);
      T_W_E = Tc_W(:,:,R.I_EElink+1)*R.T_N_E;
      T_0_E = Tc_0(:,:,R.I_EElink+1)*R.T_N_E;
    end
    function [T_0_E, T_W_E] = fkineEE_vp(R, q, pkin)
      % Direkte Kinematik zum End-Effektor
      % wie fkineEE, nur mit variablen Kinematikparametern
      [Tc_0, Tc_W] = R.fkine_vp(q, pkin);
      T_W_E = Tc_W(:,:,R.I_EElink+1)*R.T_N_E;
      T_0_E = Tc_0(:,:,R.I_EElink+1)*R.T_N_E;
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
      JR = R.jacobiRfcnhdl(q, uint8(R.I_EElink), R.pkin);
    end
    function Jg = jacobig(R, q)
      % Geometrische Jacobi-Matrix (bzgl Winkelgeschwindigkeit)
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % Jg: Jacobi-Matrix
      Jg = R.jacobigfcnhdl(q, uint8(R.I_EElink), R.r_N_E, R.pkin);
    end
    function JT = jacobiT(R, q)
      % Translatorischer Teil der geometrischen Jacobi-Matrix (Zusammenhang
      % zwischen translatorischer Geschwindigkeit des EE und Gelenkgeschwindigkeit)
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % JT: Jacobi-Matrix
      Jg = jacobig(R, q);
      JT = Jg(1:3,:); % TODO: Eigene Funktion
    end
    function JW = jacobiW(R, q)
      % Rotatorischer Teil der geometrischen Jacobi-Matrix (Zusammenhang
      % zwischen Winkelgeschwindigkeit des EE und Gelenkgeschwindigkeit)
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % JW: Jacobi-Matrix
      Jg = jacobig(R, q);
      JW = Jg(4:6,:); % TODO: Eigene Funktion
    end
    function JTD = jacobiTD(R, q, qD)
      % Zeitableitung des Translatorischen Teils der geometrischen Jacobi-Matrix (Zusammenhang
      % zwischen translatorischer Geschwindigkeit des EE und Gelenkgeschwindigkeit)
      % Eingabe:
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeiten
      %
      % Ausgabe:
      % JTD: Jacobi-Matrix-Zeitableitung
      JgD = jacobigD(R, q, qD);
      JTD = JgD(1:3,:); % TODO: Eigene Funktion
    end
    function JgD = jacobigD(R, q, qD)
      % Zeitableitung der Geometrischen Jacobi-Matrix (bzgl Winkelbeschleunigung)
      % Eingabe:
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeiten
      %
      % Ausgabe:
      % JgD: Jacobi-Matrix-Zeitableitung
      JgD = R.jacobigDfcnhdl(q, qD, uint8(R.I_EElink), R.r_N_E, R.pkin);
    end
    function JWD = jacobiWD(R, q, qD)
      % Zeitableitung des rotatorischer Teils der geometrischen Jacobi-Matrix
      % (Zusammenhang zwischen Winkelgeschwindigkeit des EE und Gelenkgeschwindigkeit)
      % Eingabe:
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeiten
      %
      % Ausgabe:
      % JDW: Jacobi-Matrix-Zeitableitung
      JgD = jacobigD(R, q, qD);
      JWD = JgD(4:6,:); % TODO: Eigene Funktion
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
      JTD = R.jacobiTD(q, qD);
      % Rotatorische Teilmatrix (geometrisch)
      Jw = R.jacobiW(q);
      % Zeitableitund der rotatorischen Teilmatrix
      JwD = R.jacobiWD(q, qD);
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
      JaD = [JTD; JeD];
    end
    function T = ekin(R, q, qD)
      % Kinetische Energie
      % Eingabe:
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeiten
      %
      % Ausgabe:
      % T: Kinetische Energie
      T = R.ekinfcnhdl(q, qD, R.pkin, R.DynPar.mges, R.DynPar.mrSges, R.DynPar.Ifges);
    end
    function U = epot(R, q)
      % Potentielle Energie
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % T: Potentielle Energie
      U = R.epotfcnhdl(q, R.gravity, R.pkin, R.DynPar.mges, R.DynPar.mrSges);
    end
    function gq = gravload(R, q)
      % Potentielle Energie
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % T: Potentielle Energie
      gq = R.gravlfcnhdl(q, R.gravity, R.pkin, R.DynPar.mges, R.DynPar.mrSges);
    end
    function Mq = inertia(R, q)
      % Massenmatrix (in Gelenkkoordinaten)
      % Eingabe:
      % q: Gelenkkoordinaten
      %
      % Ausgabe:
      % Mq: Massenmatrix
      Mq = R.inertiafcnhdl(q, R.pkin, R.DynPar.mges, R.DynPar.mrSges, R.DynPar.Ifges);
    end
    function cq = corvec(R, q, qD)
      % Gelenkvektor der Flieh- und Corioliskräfte
      % Eingabe:
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeiten
      %
      % Ausgabe:
      % cq: Gelenkkräfte für Flieh- und Corioliskräfte
      cq = R.corvecfcnhdl(q, qD, R.pkin, R.DynPar.mges, R.DynPar.mrSges, R.DynPar.Ifges);
    end
    function Cq = cormat(R, q, qD)
      % Matrix der Flieh- und Corioliskräfte
      % Eingabe:
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeiten
      %
      % Ausgabe:
      % Cq: Coriolis-Matrix
      Cq = R.cormatfcnhdl(q, qD, R.pkin, R.DynPar.mges, R.DynPar.mrSges, R.DynPar.Ifges);
    end
    function tauq = invdyn(R, q, qD, qDD)
      % Gelenkvektor der inversen Dynamik
      % Eingabe:
      % q: Gelenkkoordinaten
      % qD: Gelenkgeschwindigkeiten
      % qDD: Gelenkbeschleunigung
      %
      % Ausgabe:
      % tauq: Inverse Dynamik
      tauq = R.invdynfcnhdl(q, qD, qDD, R.gravity, R.pkin, R.DynPar.mges, R.DynPar.mrSges, R.DynPar.Ifges);
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
        jv = R.jointvarfcnhdl(qJ, R.pkin);
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
        R.pkin = NaN(NKP,1);
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
      pkin = mdh2pkin_hdl(R.MDH.beta, R.MDH.b, R.MDH.alpha, R.MDH.a, ...
        R.MDH.theta, R.MDH.d, zeros(R.NJ,1));
      R.pkin = pkin;
      [beta,b,alpha,a,theta,d] = pkin2mdh_hdl(pkin);
      % Teste Rück-Transformation der Kinematik-Parameter
      pkin_test = mdh2pkin_hdl(beta, b, alpha, a, theta, d, zeros(R.NJ,1));
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
      assert(all(size(pkin_neu) == size( R.pkin )), 'Eingabe muss Dimension von pkin haben')
      
      mdh2pkin_hdl = eval(sprintf('@%s_mdhparam2pkin', R.mdlname));
      pkin2mdh_hdl = eval(sprintf('@%s_pkin2mdhparam', R.mdlname));
      % Berechne MDH-Parameter aus pkin-Vektor
      [beta,b,alpha,a,theta,d,qoffset] = pkin2mdh_hdl(pkin_neu);
      % Teste die Rück-Transformation zu pkin (funktioniert nur für
      % serielle Roboter; bei hybriden stehen nicht alle Par. in MDH
      if R.Type == 0
        pkin_test = mdh2pkin_hdl(beta, b, alpha, a, theta, d, qoffset);
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
      
      % Umwandlung der Dynamik-Parameter (Masse, erstes Moment, zweites
      % Moment) in Minimalparameter-Vektor
      if isempty(which(sprintf('%s_convert_par2_MPV_fixb', R.mdlname)))
        % Funktion zur Umwandlung nach MPV wurde nicht generiert. Leer lassen.
        mpv = [];
      else
        dynpar2mpv_hdl = eval(sprintf('@%s_convert_par2_MPV_fixb', R.mdlname));
        mpv = dynpar2mpv_hdl(R.pkin, mges, mrSges, Ifges);
      end

      % Parameter in Roboterklasse belegen
      R.DynPar.mges   = mges;
      R.DynPar.rSges  = rSges;
      R.DynPar.Icges  = Icges;
      R.DynPar.mrSges = mrSges;
      R.DynPar.Ifges  = Ifges;
      R.DynPar.mpv    = mpv;
    end
    
    function x_W_E = t2x(R, T_W_E)
      % Umwandlung der homogenen Transformationsmatrix der EE-Lage in Minimalkoordinaten
      % Eingabe:
      % T_W_E: Transformationsmatrix zwischen Welt- und EE-KS
      % 
      % Ausgabe: Vektor aus Position und Euler-Winkeln
      x_W_E = [T_W_E(1:3,4); r2eul(T_W_E(1:3,1:3), R.phiconv_W_E)];
    end
    
    function CAD_add(R, filepath, link, T_body_CAD, color)
      % Füge die CAD-Datei für einen Körper des Roboters hinzu
      % filepath: Absoluter Pfad zur STL-Datei
      % link: Nummer des Robotersegments (0=Basis)
      % T_body_CAD: Koordinatentransformation zum Ursprung der CAD-Datei
      % color (optional): Farbe für hinzuzufügendes CAD-Modell
      colors_default = {'k', 'r', 'g', 'b', 'c', 'm', 'y'};
      if isempty(R.CADstruct)
        R.CADstruct = struct( ...
          'filepath', cell(1,1), ...
          'link', link, ...
          'T_body_visual', T_body_CAD, ...
          'color', cell(1,1));
        R.CADstruct.filepath{1} = filepath;
        if nargin < 5
          color = colors_default{1};
        end
        R.CADstruct.color{1} = color;
      else
        i = length(R.CADstruct.link)+1;
        R.CADstruct.filepath = {R.CADstruct.filepath{:}, filepath}; %#ok<CCAT>
        R.CADstruct.link = [R.CADstruct.link; link];
        R.CADstruct.T_body_visual(:,:,i) = T_body_CAD;
        if nargin < 5
          ci = mod(i-1, 7)+1; % Wähle der Reihe nach die Standardfarben (mit Wiederholung)
          color = colors_default{ci};
        end
        R.CADstruct.color = {R.CADstruct.color{:}, color}; %#ok<CCAT>
      end
    end
  end
end