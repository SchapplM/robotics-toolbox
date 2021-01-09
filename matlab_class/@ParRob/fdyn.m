% Direkte Dynamik (freie Bewegung) einer PKM simulieren
% 
% Eingabe:
% fdynstruct. Struktur mit Feldern:
%  q0 (nj x 1): Startkonfiguration der Gelenkkoordinaten
%  qD0 (nj x 1): Startkonfiguration der Gelenkgeschwindigkeiten
%  x0red (nx x 1): Startkonfiguration der Plattformkoordinaten
%  xD0 (nx x 1): Startkonfiguration der Plattformgeschwindigkeiten
%  dtmax (1x1): Maximale Schrittweite (in s)
%  t_End (1x1): Dauer der Vorwärtsdynamik-Simulation
%  J_method (1x1): Nummer der Jacobi-Methode
% 
% Ausgabe:
% fdynoutput. Struktur mit Feldern:
%   Tges (nt x 1); Zeitschritte der Vorwärtsdynamik
%   Qges (nt x nj); Trajektorie der Gelenkkoordinaten
%   QDges (nt x nj); ... der Gelenkgeschwindigkeiten
%   XPredges (nt x nx); Trajektorie der Plattformkoordinaten
%   XPDredges (nt x nx); ... der Plattformgeschwindigkeiten

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020.12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function fdynoutput = fdyn(RP, fdynstruct)
  % Eingabestruktur auslesen und Einstellungen für ode45 belegen. Gelenk-
  % geschwindigkeit wird nicht benötigt und zu Null gesetzt
  y0 = [fdynstruct.q0; fdynstruct.x0red; zeros(size(fdynstruct.qD0)); fdynstruct.xD0];
  jm = fdynstruct.J_method;
  t_End = fdynstruct.t_End;
  % Zu integrierende Funktion definieren (anonym zur Übergabe von
  % konstanten Parametern)
  odefun = @(t, y) odefun2(t, y, RP, jm);
  
  % Numerische Integration konfigurieren und durchführen
  options = odeset( ...
    'MaxStep',fdynstruct.dtmax); % mit automatischer Schrittweite ist der Fehler zu groß
  SolverOutput = ode45(odefun, [0 t_End], y0, options);
  
  % Ausgabe zusammenstellen
  fdynoutput = struct(...
    'Tges', SolverOutput.x(:), ...
    'Qges', SolverOutput.y(1:RP.NJ,:)',...
    'XPredges', SolverOutput.y(RP.NJ+1:RP.NJ+sum(RP.I_EE),:)',...
    'QDges', SolverOutput.y(RP.NJ+sum(RP.I_EE)+1:RP.NJ+sum(RP.I_EE)+RP.NJ,:)',...
    'XPDredges', SolverOutput.y(RP.NJ+sum(RP.I_EE)+RP.NJ+1:end,:)');
end

function f = odefun2(t,y,RP,jm) %#ok<INUSL>
  % Eingabe:
  % t: Zeit, Vorgabe in ode45, hier kein Einfluss
  % y: Zustandsvektor: Position und Geschwindigkeit Gelenke und Plattform
  % RP: Instanz der Roboter-Klasse
  % jm: Nummer der Jacobi-Matrix-Methode
  % 
  % Ausgabe:
  % f: Ableitung von y. Wird in ode45 aufintegriert.
  f = NaN(size(y)); 
  if any(isnan(y))
    return; % Wenn PKM einmal den Arbeitsraum verlassen hat, hierdurch Abbruch.
  end
  % Zustandsvariable in physikalische Größen umwandeln. Gelenkgrößen des
  % Roboters werden nicht für die Berechnung benutzt, da die Darstellung in
  % Minimalkoordinaten x ausreichend ist.
  % Benutze die Plattform-Koordinaten (xP) und nicht die EE-Koordinaten
  % (xE). Dadurch gibt es keine Probleme bei Deckenmontage o.ä.
  q_state = y(1:RP.NJ);
  xP_red = y(RP.NJ+1:RP.NJ+sum(RP.I_EE));
  qD_state = y(RP.NJ+sum(RP.I_EE)+1:RP.NJ+sum(RP.I_EE)+RP.NJ); %#ok<NASGU>
  xDP_red = y(RP.NJ+sum(RP.I_EE)+RP.NJ+1:end);
  
  % Vollständiger Vektor der Plattform-Koordinaten.
  xP = zeros(6,1);
  xP(RP.I_EE) = xP_red; % bei 3T2R kann die sechste Komponente ungleich Null sein. Ist hier egal.
  xPD = zeros(6,1);
  xPD(RP.I_EE) = xDP_red; % bei 3T2R x6~=0 möglich. Egal.
  % Korrektur der Gelenkwinkel (wenn IK nicht erfüllbar, ergeben folgende
  % Rechnungen keinen Sinn)
  s = struct( ...
    'normalize', false, ... % Winkel nicht normalisieren, da sonst Federmoment falsch.
    'n_min', 25, ... % Minimale Anzahl Iterationen
    'n_max', 1000, ... % Maximale Anzahl Iterationen
    'Phit_tol', 1e-12, ... % Toleranz für translatorischen Fehler
    'Phir_tol', 1e-12); % Toleranz für rotatorischen Fehler
  s_par = struct( ...
    'platform_frame', true); % IK bezogen auf Plattform-KS
  [q, Phi] = RP.invkin_ser(xP, q_state, s, s_par);
  if any(isnan(q)) || any(abs(Phi) > 1e-6) || any(isnan(Phi))
    return;
  end
  % Berechne die Jacobi-Matrizen bezogen auf das Plattform-KS
  % Gelenkgeschwindigkeit ist komplett abhängig von Plattform- 
  % Geschwindigkeit und braucht nicht integriert werden.
  if jm == 1 % Nehme Euler-Winkel-Jacobi für Dynamik
    [G1_q, ~] = RP.constr1grad_q(q, xP, true);
    [G1_x, ~] = RP.constr1grad_x(q, xP, true);
    Jinv_voll = -G1_q\G1_x; % Jacobi-Matrix als Hilfe für Dynamik-Fkt speichern
  elseif jm == 2  % Nehme neue Modellierung der Jacobi für die Dynamik
    [G4_q, ~] = RP.constr4grad_q(q);
    [G4_x, ~] = RP.constr4grad_x(xP, true);
    Jinv_voll = -G4_q\G4_x;
  else % Modellierung für 3T2R-PKM
    G2_q = RP.constr2grad_q(q, xP, true);
    G2_x = RP.constr2grad_x(q, xP, true);
    Jinv_voll = -G2_q\G2_x;
  end
  qD = Jinv_voll*xDP_red;
  % Berechne Jacobi-Zeitableitung
  if jm == 1
    GD1_q = RP.constr1gradD_q(q, qD, xP, xPD, true);
    GD1_x = RP.constr1gradD_x(q, qD, xP, xPD, true);
    JinvD_voll = G1_q\(GD1_q*(G1_q\G1_x)) - G1_q\GD1_x; % effizienter hier zu berechnen als in Dynamik
  elseif jm == 2
    % Nehme Modellierung 4 der Jacobi für die Dynamik
    GD4_q = RP.constr4gradD_q(q, qD);
    GD4_x = RP.constr4gradD_x(xP, xPD, true);
    JinvD_voll = G4_q\(GD4_q*(G4_q\G4_x)) - G4_q\GD4_x;
  else
    GD2_q = RP.constr2gradD_q(q, qD, xP, xPD, true);
    GD2_x = RP.constr2gradD_x(q, qD, xP, xPD, true);
    JinvD_voll = G2_q\(GD2_q*(G2_q\G2_x)) - G2_q\GD2_x;
  end
  % Berechne Terme der Dynamik-Gleichung. Übergabe der Jacobi, damit nicht
  % vielfache Neuberechnung notwendig.
  Mx = RP.inertia2_platform(q, xP, Jinv_voll);
  Gx = RP.gravload2_platform(q, xP, Jinv_voll);
  Cx = RP.coriolisvec2_platform(q, qD, xP, xPD, Jinv_voll, JinvD_voll);
  Kx = RP.jointtorque_platform(q, xP, RP.springtorque(q), Jinv_voll);
  % Beschleunigung der Plattform berechnen
  xDDP_red = Mx \ (-Gx - Cx - Kx);
  % Beschleunigung der Beingelenke
  qDD = Jinv_voll*xDDP_red + JinvD_voll*xDP_red;
  % Zeitableitung des Zustandsvektors enthält Geschw. und Beschl.
  f = [qD; xDP_red; qDD; xDDP_red];
  if any(abs(f) > 1e6)
    % Abbruch bei sehr großen Zahlenwerten (deutet auf eine Singularität
    % hin). Ansonsten geht die Schrittweite gegen Null und die
    % ode-Simulation dauert Tage.
    f(:) = NaN;
  end
end