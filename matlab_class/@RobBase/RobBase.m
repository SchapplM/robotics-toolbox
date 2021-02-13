%   Allgemeine Klasse für Roboter zur Ableitung für andere Klassen
%   Genauere Details sind den Klassen SerRob und ParRob zu entnehmen
%
%   Koordinatensystem-Definitionen:
%   * W: Welt-KS (Absolutes Inertial-KS, auf das sich auch mehrere Roboter
%        gleichzeitig beziehen können)
%   * 0: Basis-KS (Inertial-KS des Roboters)
%   * E: Endeffektor-KS des Roboters (z.B. Bohrerspitze), auf dieses KS
%        bezieht sich die Bahnplanung und Kinematik

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

classdef RobBase < matlab.mixin.Copyable

  properties (Access = public)
    phiconv_W_0 % Nummer der Basis-Euler-Winkelkonvention
    phiconv_W_E % Winkelkonvention zur Darstellung der EE-Orientierung im Welt-KS mit Euler-Winkeln
    NJ % Anzahl der Gelenke des Roboters
    NL % Anzahl der Starrkörper des Roboters
    Type % Typ des Roboters (0=seriell, 1=hybrid, 2=parallel)
    r_W_0 % Position der Basis im Welt-KS
    phi_W_0 % Orientierung des Basis-KS im Welt-KS (ausgedrückt in Euler-Winkeln)
    T_W_0 % Homogene Transformationsmatrix zwischen Welt- und Basis-KS des Roboters
    T_0_W % Inverse von T_W_0
    gravity % Gravitationsvektor ausgedrückt im Basis-KS
    I_EE % Indizes der verfügbaren EE-FG des Roboters (EE-Position, Euler-Winkel aus phiconv_W_E)
    I_EE_Task % Indizes der durch die Aufgabe genutzten EE-FG (EE-Position, Euler-Winkel aus phiconv_W_E)
  end

  methods
    function R=RobBase() % Konstruktor
      % Keine spezielle Initialisierung. Wird in SerRob und ParRob gemacht.
      % Keine Transformation des Basis-KS bezüglich Welt-KS am Anfang
      R.r_W_0 = zeros(3,1);
      R.phi_W_0 = zeros(3,1);
      R.T_W_0 = eye(4);
      % Nehme Standardmäßig Euler-XYZ-Winkel für Orientierungen
      R.phiconv_W_E = uint8(2);
      R.phiconv_W_0 = uint8(2);
      % Bodenmontage
      R.gravity = [0;0;-9.81];
    end
    function x_W_E = t2x(R, T_W_E)
      % Umwandlung der homogenen Transformationsmatrix der EE-Lage in Minimalkoordinaten
      % Eingabe: Transformationsmatrix zwischen Welt- und EE-KS
      % Ausgabe: Vektor aus Position und Euler-Winkeln
      x_W_E = [T_W_E(1:3,4); r2eul(T_W_E(1:3,1:3), R.phiconv_W_E)];
    end
    function T_W_E = x2t(R, x_W_E)
      % Umwandlung der EE-Lage in eine homogene Transformationsmatrix
      % Eingabe: Vektor aus Position und Euler-Winkeln
      % Ausgabe: Transformationsmatrix zwischen Welt- und EE-KS
      T_W_E = [eul2r(x_W_E(4:6), R.phiconv_W_E), x_W_E(1:3); [0 0 0 1]];
    end
    function Tr_W_E = x2tr(R, x_W_E)
      % Umwandlung der EE-Lage in eine reduzierte homogene Transformations-
      % matrix (ohne letzte Zeile). Dient zur kompakteren Übergabe an Fkt.
      % Eingabe: Vektor aus Position und Euler-Winkeln
      % Ausgabe: reduzierte Transformationsmatrix zwischen Welt- und EE-KS
      Tr_W_E = [eul2r(x_W_E(4:6), R.phiconv_W_E), x_W_E(1:3)];
    end
    function Tr_W_E = x2tr_traj(R, X_W_E)
      % Umwandlung der EE-Lage in eine reduzierte homogene Transformations-
      % matrix (ohne letzte Zeile). Dient zur kompakteren Übergabe an Fkt.
      % Eingabe: Zeilenweise Vektoren aus Position und Euler-Winkeln
      % Ausgabe: Zeilenweise red. Transformationsmatrix zwischen Welt- und EE-KS
      Tr_W_E = NaN(size(X_W_E,1), 12);
      for i = 1:size(X_W_E,1)
        Tr_W_E(i,:) = R.x2tr(X_W_E(i,:)');
      end
    end
  end
end