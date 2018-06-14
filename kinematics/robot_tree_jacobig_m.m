% Geometrische Jacobi-Matrix für einen beliebigen Roboter in kinematischer
% Baumstruktur mit einfachen Drehgelenken mit fester Basis
% Suffix "_m": Modulare Funktion (keine Übergabe von Gelenkwinkeln, sondern
% von Transformationsmatrizen. Daher für beliebige Roboter verwendbar
% unabhängig von der Gelenk-Transformations-Notation
% 
% Eingabe:
% Tc_stack [NJ*3x4]
%   Transformationsmatrizen von der Basis zum jeweiligen Körper-KS
%   Gestapelte Einträge: Letzte Zeile ([0 0 0 1]) weggelassen
%   NJ: Anzahl der Gelenke (Annahme: 1DoF Drehgelenke)
%   (Ohne Eintrag für die Basis selbst)
% ax_i [NJx3]
%   Rotationsachsen der Gelenke im Körper-KS des durch das Gelenk
%   bewegten Körpers
% v [NJx1] uint8
%   Vorgänger-Indizes der Gelenke
% link_index [1x1] uint8
%   Index des Körpers, auf dem der Punkt C liegt. 0=Basis
% r_i_i_C [3x1]
%   Punkt C, zu dem die Jacobi-Matrix berechnet wird
% 
% Ausgabe:
% Jg [6xNJ]
%   Geometrische Jacobimatrix

% Moritz Schappler, schappler@irt.uni-hannover.de, 2017-11
% (C) Institut für Regelungstechnik, Leibniz Universität Hannover

function Jg_C = robot_tree_jacobig_m(Tc_stack, ax_i, v, link_index, r_i_i_C)

%% Init
assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ...
  'robot_tree_jacobig_m: link_index has to be [1x1] uint8');

NJ = size(ax_i,1);
% Initialisierung. Alle Spalten die nicht gesetzt werden haben keinen
% Einfluss.
Jg_C = zeros(6,NJ);

if link_index == 0
  % Die Gelenkwinkel haben keinen Einfluss auf die Basis
  return;
end

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
I(:) = (3*link_index-2:3*link_index); % ... Muss als 3x1 initialisiert werden (für Simulink/codegen), damit die Dimension vorher bekannt ist.
R_0_i(1:3,1:3) = Tc_stack(I, 1:3);
r_0_0_i(:) = Tc_stack(I, 4);
r_0_i_C(:) = R_0_i * (r_i_i_C);


j = link_index; % Die Indizes j und k haben die Basis als 0.
for tmp = 1:NJ
  % Ortsvektor des aktuellen Gelenkes "j"
  I = uint8(zeros(3,1)); % Zeilenindex in gestapelten Transformationsmatrizen.
  I(:) = 3*j-2:3*j;
  r_0_0_j(:) = Tc_stack(I, 4);
  R_0_j(1:3,1:3) = Tc_stack(I, 1:3);
  
  % Rotation axis
  ax_0(:) = R_0_j*ax_i(j,:)';
  jr = ax_0;
  
  % Berechne Vektor vom aktuellen Gelenk zum betrachteten Punkt C
  r_0_j_i = -r_0_0_j + r_0_0_i;
  
  r_0_j_C(:) = r_0_j_i + r_0_i_C;
  
  % Hebelarm vom Gelenk zum Punkt
  jt = cross(ax_0, r_0_j_C);

  % Spalte der Jacobi-Matrix eintragen
  Jg_C(:,j) = [jt; jr];
  
  % Indizes tauschen: Kinematische Kette weiter Richtung Basis entlanggehen
  j = v(j); % Index mit Basis als 1
  if j == 0
    % An Basis angekommen
    return;
  end
end