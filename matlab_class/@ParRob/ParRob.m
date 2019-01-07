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
      mdlname % Name des PKM-Robotermodells, das in den Matlab-Funktionen benutzt wird.
      Leg % Matlab-Klasse SerRob für jede Beinkette
      issym % true für rein symbolische Berechnung
      NQJ_LEG_bc % Anzahl relevanter Beingelenke vor Schnittgelenk (von Basis an gezählt) ("bc"="before cut")
  end
  properties (Access = private)
      jacobi_qa_x_fcnhdl % Funktions-Handle für Jacobi-Matrix zwischen Antrieben und Plattform-KS
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
      % Liste der Funktionshandle-Variablen mit zugehörigen
      % Funktionsdateien (aus Maple-Toolbox)
      R.all_fcn_hdl = { ...
        {'jacobi_qa_x_fcnhdl', 'Jinv'}};
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
        R.Leg(i).MDH.mu = double( I_qa(R.I1J_LEG(i):R.I2J_LEG(i)) );
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
    function Jinv = jacobi_qa_x(R, q, x)
      % Analytische Jacobi-Matrix zwischen Antriebs- und Plattformkoord.
      % Eingabe:
      % q: Gelenkkoordinaten
      % x: EE-Koordinaten
      %
      % Ausgabe:
      % Jinv: Inverse Jacobi-Matrix
      assert(all(size(q) == [R.NJ 1]), 'jacobi_qa_x: q muss %dx1 sein', R.NJ);
      assert(all(size(x) == [6 1]), 'jacobi_qa_x: x muss 6x1 sein');
      xred = x(R.I_EE);
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
      koppelP = R.r_P_B_all';
      if ~R.issym
        legFrame = NaN(R.NLEG, 3);
      else
        legFrame = sym('xx', [R.NLEG, 3]);
        legFrame(:)=0;
      end
      for i = 1:3
        legFrame(i,:) = R.Leg(i).phi_W_0;
      end
      Jinv = R.jacobi_qa_x_fcnhdl(xred, qJ, pkin, koppelP, legFrame);
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
  end
end