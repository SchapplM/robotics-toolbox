% Geometrische Jacobi-Matrix für vollständige Schnittkräfte für einen
% beliebigen Roboter in kinematischer Baumstruktur
% Suffix "_m": Modulare Funktion (keine Übergabe von Gelenkwinkeln, sondern
% von Transformationsmatrizen. Daher für beliebige Roboter verwendbar
% unabhängig von der Gelenk-Transformations-Notation
% 
% Die Matrix kann verwendet werden, um die aus einer externen Kraft
% resultierenden Schnittkräfte in allen Gelenken des Roboters zu bestimmen
% 
% Eingabe:
% Tc_stack [NL*3x4]
%   Transformationsmatrizen von der Basis zum jeweiligen Körper-KS
%   Gestapelte Einträge: Letzte Zeile ([0 0 0 1]) weggelassen
%   NL: Anzahl der Starrkörper (Mit Eintrag für die Basis selbst)
% v [NJx1] uint8
%   Vorgänger-Indizes der Gelenke
% link_index [1x1] uint8
%   Index des Körpers, auf dem der Punkt C liegt. 0=Basis
% r_i_i_C [3x1]
%   Punkt C, zu dem die Jacobi-Matrix berechnet wird (Angriffspunkt der
%   Kraft)
% 
% Ausgabe:
% Jg [6x(6*NL)]
%   Geometrische Jacobimatrix für vollständige Schnittkräfte in allen
%   Körper-KS basierend auf externer Kraft am gegebenen Punkt
%
% Quellen:
% [1] Ortmaier: Robotik I Skript

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-05
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Jg_C = robot_tree_jacobig_cutforce_m(Tc_stack, v, link_index, r_i_i_C)

%% Init
assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ...
  'robot_tree_jacobig_m: link_index has to be [1x1] uint8');

NL = size(Tc_stack,1)/3;
% Initialisierung. Alle Spalten die nicht gesetzt werden haben keinen
% Einfluss.
Jg_C = zeros(6,6*NL);

% Variablen initialisieren (zur Kompilierbarkeit in Simulink)
% Zuordnung mit (:) weiter unten deshalb auch notwendig.
% Dadurch werden Fehler bei der Variablenzuweisung (falsche Dimension)
% leichter erkannt.
ax_0 = NaN(3,1);
r_0_j_C = NaN(3,1);
r_0_0_j = NaN(3,1);
R_0_j = NaN(3,3);
R_0_i = NaN(3,3);
r_0_0_i = NaN(3,1);
r_0_i_C = NaN(3,1);

%% Jacobi berechnen
I = uint8(zeros(3,1)); % Zeilenindex in gestapelten Transformationsmatrizen. ...
I(:) = (3*(link_index+1)-2:3*(link_index+1)); % ... Muss als 3x1 initialisiert werden (für Simulink/codegen), damit die Dimension vorher bekannt ist.
R_0_i(1:3,1:3) = Tc_stack(I, 1:3);
r_0_0_i(:) = Tc_stack(I, 4);
r_0_i_C(:) = R_0_i * (r_i_i_C);


j = link_index; % Die Indizes j und k haben die Basis als 0.
for tmp = 1:NL
  % Ortsvektor des aktuellen Gelenkes "j"
  I = uint8(zeros(3,1)); % Zeilenindex in gestapelten Transformationsmatrizen.
  I(:) = 3*(j+1)-2:3*(j+1);
  r_0_0_j(:) = Tc_stack(I, 4);
  R_0_j(1:3,1:3) = Tc_stack(I, 1:3);
  
  % Berechne Vektor vom aktuellen Gelenk zum betrachteten Punkt C
  r_0_j_i = -r_0_0_j + r_0_0_i;
  r_0_j_C(:) = r_0_j_i + r_0_i_C;

  % Geometrische Jacobi-Matrix für alle Achsen des Körper-KS bestimmen.
  % Normalerweise wird eine Schnittmomentkomponente auf ein Drehgelenk
  % projiziert.
  for i_xyz = 1:3
    ax_0(:) = Tc_stack(I,i_xyz); % Koordinatenachse für Projektion
    
    % Spalte für Kräfte: Entspricht Schubgelenk, [1], Gl. (4.18)
    jt = ax_0;
    jr = zeros(3,1);
    Jg_C(:,(j)*6+i_xyz) = [jt; jr];
    
    % Spalte für Momente: Entspricht Drehgelenk, [1], Gl. (4.19)
    jt = cross(ax_0, r_0_j_C); % Hebelarm vom Gelenk zum Punkt
    jr = ax_0;
    Jg_C(:,(j)*6+i_xyz+3) = [jt; jr];
  end
  % Indizes tauschen: Kinematische Kette weiter Richtung Basis entlanggehen
  if j == 0%-1
    % Nächerst wäre Basis. Ist bereits berechnet.
    return;
  end
  j = v(j); % Index mit Basis als 1
end