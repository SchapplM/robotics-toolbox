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
% Siehe auch: SerRob.m (SerRob-Klasse)

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
      I_EE % Indizes der genutzten EE-FG. TODO: Funktionen darauf umstellen. Achtung bei Euler-Winkeln
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
      mdlname % Name des PKM-Robotermodells, das in den Matlab-Funktionen benutzt wird.
      Leg % Matlab-Klasse SerRob für jede Beinkette
      issym % true für rein symbolische Berechnung
      gravity % Gravitationsvektor ausgedrückt im Basis-KS des Roboters
      NQJ_LEG_bc % Anzahl relevanter Beingelenke vor Schnittgelenk (von Basis an gezählt) ("bc"="before cut")
      I_constr_t % Indizes der translatorischen Zwangsbedingungen in allen ZB
      I_constr_r % Indizes der rotatorischen ZB in allen ZB
      I_constr_t_red % Indizes für reduzierte ZB (translatorisch)
      I_constr_r_red % ...                       (rotatorisch)
  end
  properties (Access = private)
      jacobi_qa_x_fcnhdl % Funktions-Handle für Jacobi-Matrix zwischen Antrieben und Plattform-KS
      inertia_x_fcnhdl  % Funktions-Handle für Massenmatrix in Plattform-Koordinaten
      gravload_x_fcnhdl  % Funktions-Handle für Gravitationslast in Plattform-Koordinaten
      coriolisvec_x_fcnhdl  % Funktions-Handle für Corioliskraft in Plattform-Koordinaten
      dynparconvfcnhdl % Funktions-Handle zur Umwandlung von DynPar 2 zu MPV
      all_fcn_hdl % Cell-Array mit allen Funktions-Handles des Roboters sowie den Dateinamen der Matlab-Funktionen
  end
  methods
    % Konstruktor
    function R=ParRob(mdlname)
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
      R.T_W_0 = eye(4);
      R.gravity = [0;0;-9.81];
      % Liste der Funktionshandle-Variablen mit zugehörigen
      % Funktionsdateien (aus Maple-Toolbox)
      R.all_fcn_hdl = { ...
        {'jacobi_qa_x_fcnhdl', 'Jinv'}, ...
        {'inertia_x_fcnhdl', 'inertia_para_pf_slag_vp2'}, ...
        {'gravload_x_fcnhdl', 'gravload_para_pf_slag_vp2'}, ...
        {'coriolisvec_x_fcnhdl', 'coriolisvec_para_pf_slag_vp2'}, ...
        {'dynparconvfcnhdl', 'minimal_parameter_para'}};
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
      % Phi
      %   Kinematische Zwangsbedingungen für die Lösung. 
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
        R.phiconv_P_E = phiconv_W_0;
      end
      R.T_W_0 = [[eul2r(R.phi_W_0, R.phiconv_W_0), R.r_W_0]; [0 0 0 1]];
    end
    function Jinv = jacobi_qa_x(R, q, xP)
      % Analytische Jacobi-Matrix zwischen Antriebs- und Plattformkoord.
      % Eingabe:
      % q: Gelenkkoordinaten
      % xP: Plattform-Koordinaten (6x1) (nicht: End-Effektor-Koordinaten) (im Basis-KS)
      %
      % Ausgabe:
      % Jinv: Inverse Jacobi-Matrix
      [qJ, xred, pkin, koppelP, legFrame] = convert_parameter_class2toolbox(R, q, xP);
      Jinv = R.jacobi_qa_x_fcnhdl(xred, qJ, pkin, koppelP, legFrame);
    end
    function Mx = inertia_platform(R, q, xP)
      % Massenmatrix bezogen auf Plattformkoordinaten
      % q: Gelenkkoordinaten
      % xP: Plattform-Koordinaten (6x1) (nicht: End-Effektor-Koordinaten) (im Basis-KS)
      %
      % Ausgabe:
      % Mx: Massenmatrix
      [qJ, xPred, pkin, koppelP, legFrame] = convert_parameter_class2toolbox(R, q, xP);
      Mx = R.inertia_x_fcnhdl(xPred, qJ, legFrame, koppelP, pkin, ...
        R.DynPar.mges, R.DynPar.mrSges, R.DynPar.Ifges);
    end
    function Fgx = gravload_platform(R, q, xP)
      % Gravitationskraft bezogen auf Plattformkoordinaten
      % q: Gelenkkoordinaten
      % xP: Plattform-Koordinaten (6x1) (nicht: End-Effektor-Koordinaten) (im Basis-KS)
      %
      % Ausgabe:
      % Mx: Massenmatrix
      [qJ, xPred, pkin, koppelP, legFrame] = convert_parameter_class2toolbox(R, q, xP);
      Fgx = R.gravload_x_fcnhdl(xPred, qJ, R.gravity, legFrame, koppelP, pkin, ...
        R.DynPar.mges, R.DynPar.mrSges);
    end
    function Fcx = coriolisvec_platform(R, q, xP, xPD)
      % Corioliskraft bezogen auf Plattformkoordinaten
      % q: Gelenkkoordinaten
      % xP: Plattform-Koordinaten (6x1) (nicht: End-Effektor-Koordinaten) (im Basis-KS)
      % xPD: Plattform-Geschwindigkeit (im Basis-KS)
      %
      % Ausgabe:
      % Mx: Massenmatrix
      xPDred = xPD(R.I_EE);
      [qJ, xPred, pkin, koppelP, legFrame] = convert_parameter_class2toolbox(R, q, xP);
      Fcx = R.coriolisvec_x_fcnhdl(xPred, xPDred, qJ, legFrame, koppelP, pkin, ...
        R.DynPar.mges, R.DynPar.mrSges, R.DynPar.Ifges);
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
    function update_EE_FG(R, I_EE)
      % Aktualisiere die Freiheitsgrade des Endeffektors
      % Eingabe:
      % I_EE [1x6] logical; Belegung der EE-FG (frei oder blockiert)
      R.I_EE = I_EE;
      
      % Anzahl der kinematischen Zwangsbedingungen der Beinketten
      % feststellen. Annahme: Beinketten haben selbe FG wie Plattform
      % TODO: Das ändert sich evtl bei überbestimmten PKM
      nPhit = sum(I_EE(1:3));
      nPhir = sum(I_EE(4:6));
      nPhi = nPhit + nPhir;
      % Indizes der relevanten Zwangsbedingungen daraus ableiten
      % Werden benötigt für Ausgabe von constr1
      R.I_constr_t = zeros(3*R.NLEG,1);
      R.I_constr_r = zeros(3*R.NLEG,1);
      R.I_constr_t_red = zeros(nPhit*R.NLEG,1);
      R.I_constr_r_red = zeros(nPhir*R.NLEG,1);
      for i = 1:R.NLEG
        % Indizes bestimmen
        R.I_constr_t(3*(i-1)+1:3*i) = (i-1)*6+1:(i)*6-3;
        R.I_constr_r(3*(i-1)+1:3*i) = (i-1)*6+1+3:(i)*6;
        R.I_constr_t_red(nPhit*(i-1)+1:nPhit*i) = (i-1)*nPhi+1:(i)*nPhi-nPhir;
        R.I_constr_r_red(nPhir*(i-1)+1:nPhir*i) = (i-1)*nPhi+1+nPhit:(i)*nPhi;
      end
    end
    function [qJ, xred, pkin, koppelP, legFrame] = convert_parameter_class2toolbox(R, q, x)
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
      xred = x(R.I_EE); % TODO: Es muss zwischen EE- und Plattform-Koordinaten unterschieden werden.
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
    end
    function update_dynpar1(R, mges, rSges, Icges)
      % Aktualisiere die hinterlegten Dynamikparameter ausgehend von
      % gegebenen Parametern bezogen auf den Körper-KS-Ursprung
      % Eingabe:
      % mges: Massen aller Robotersegmente (inkl Plattform, exkl Basis)
      % rSges: Schwerpunktskoordinaten aller Robotersegmente (bezogen auf
      % jeweiliges Körper-KS)
      % Icges: Trägheitstensoren der Robotersegmente (bezogen auf Schwerpkt)
      if length(mges) ~= R.NQJ_LEG_bc+1
        error('Es müssen Dynamikparameter für %d Körper übergeben werden', R.NQJ_LEG_bc+1);
      end
        
      [mrSges, Ifges] = inertial_parameters_convert_par1_par2(rSges, Icges, mges);
      % Umwandlung der Dynamik-Parameter (Masse, erstes Moment, zweites
      % Moment) in Minimalparameter-Vektor
      mpv = R.dynpar_convert_par2_mpv(mges, mrSges, Ifges);
      
      % Aktualisiere die gespeicherten Dynamikparameter aller Beinketten
      for i = 1:R.NLEG
        m_Leg = [0; mges(1:end-1); zeros(R.Leg(i).NL-1-R.NQJ_LEG_bc,1)];
        rSges_Leg = [zeros(1,3); rSges(1:end-1,:); zeros(R.Leg(i).NL-1-R.NQJ_LEG_bc,3)];
        Icges_Leg = [zeros(1,6); Icges(1:end-1,:); zeros(R.Leg(i).NL-1-R.NQJ_LEG_bc,6)];
        R.Leg(i).update_dynpar1(m_Leg, rSges_Leg, Icges_Leg);
      end
      % Parameter für PKM belegen
      R.DynPar.mges   = mges;
      R.DynPar.rSges  = rSges;
      R.DynPar.Icges  = Icges;
      R.DynPar.mrSges = mrSges;
      R.DynPar.Ifges  = Ifges;
      R.DynPar.mpv    = mpv;
    end
    function mpv = dynpar_convert_par2_mpv(R, mges, mrSges, Ifges)
      % Konvertiere die Dynamikparameter zum Minimalparametervektor
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
        mpv = R.dynparconvfcnhdl(R.Leg(1).pkin, mges, mrSges, Ifges, R.r_P_B_all');
      end
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
  end
end