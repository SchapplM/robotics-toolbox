% Testen der Gelenksteifigkeit (Drehfeder) eines seriellen Roboters
% * Definition Industrieroboter mit plausiblen Dynamikparametern
% * Annahme von Drehfedern in den Gelenken
% * Simulation der Vorwärtsdynamik
% Ergebnis: Verhalten ist plausibel und energiekonsistent.
% 
% Siehe: SerRob_class_example_Industrieroboter.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc
rob_path = fileparts(which('robotics_toolbox_path_init.m'));
resdir = fullfile(rob_path, 'examples_tests', 'results');
usr_plot_animation = false;
%% Init
if isempty(which('serroblib_path_init.m'))
  warning('Repo mit Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
end
g_world = [0;0;-9.81];
SName='S6RRRRRR10V2';
RName='S6RRRRRR10V2_KUKA1';
RS = serroblib_create_robot_class(SName, RName);
RS.update_EE([], [0;pi;0]); % EE drehen, damit z-Achse nach außen zeigt
RS.fill_fcn_handles(true, true); % Funktionen kompilieren
q0 = RS.qref;
RS.DesPar.joint_stiffness_qref = q0;
RS.DesPar.joint_stiffness = ones(RS.NJ,1)*5000; % 100 Nm/rad -> 17 Nm/deg

% Dynamik-Parameter des Roboters setzen (plausible Werte, willkürlich).
% Summe der Massen laute Datenblatt 630kg
mges_Rob = [250;130;110;50;43;42;5];
% Debug: Massen einzeln deaktivieren, um Trägheitsellipsen zu sehen
% mges([1:5 7:end])=0;
% Längen der MDH-Parameter herausziehen zur vereinfachten Berechnung der
% Ersatzkörper
d1 = RS.MDH.d(1); d4 = RS.MDH.d(4);
a2 = RS.MDH.a(2); a3 = RS.MDH.a(3); a4 = RS.MDH.a(4);
% Schwerpunkt jeweils in der Mitte des Segments (siehe KS im Debug-Bild)
rSges_Rob = [ ...
  0,0,0;...
  0,0,-d1/2;...
  a3/2,0,0;...
  a4/2,-d4/2,0;...
  0,0,0;...
  0,0,0;...
  0,0,0];
% https://de.wikipedia.org/wiki/Liste_von_Trägheitstensoren
ISges_Rob = [...
  ... % Seg. 0: Basis als Zylinder r=0.100; h=0.050
  mges_Rob(1)/12*[3*(0.100^2+0.050),3*(0.100^2+0.050),6*0.100^2,0,0,0];...
  ... % Seg. 1: Karussel als Zylinder r=0.050; h=d1
  mges_Rob(2)/12*[3*(0.050^2+d1),3*(0.050^2+d1),6*d1^2,0,0,0];...
  ... % Seg. 2: Schwinge als Quader x-Dim.=a3; y/z-Dim.=0.200
  mges_Rob(3)/12*[(0.200^2+0.050^2),(0.200^2+a3^2),(0.200^2+a3^2),0,0,0];...
  ... % Seg. 3: Arm als Quader y-Dim.=d4; y/z-Dim.=0.080
  mges_Rob(4)/12*[(0.080^2+d4^2),(0.080^2+0.080^2),(0.80^2+d4^2),0,0,0];...
  ... % Seg. 4-6: Hand-Segmente als Kugel (Radius 0.050)
  mges_Rob(5)*2/5*[0.050^2,0.050^2,0.050^2,0,0,0];...
  mges_Rob(6)*2/5*[0.050^2,0.050^2,0.050^2,0,0,0];...
  mges_Rob(7)*2/5*[0.050^2,0.050^2,0.050^2,0,0,0]];
[mrSges_Rob, Ifges_Rob] = ... % Parameter in anderes Format konvertieren
  inertial_parameters_convert_par1_par2(rSges_Rob, ISges_Rob, mges_Rob);
% Zusätzliche Testmasse am Endeffektor anbringen (verbessert
% Konditionierung des Identifikationsproblems). Bzw. der hier untersuchte
% Ansatz basiert darauf, dass es diese Masse überhaupt gibt.
mges_Zus = zeros(RS.NJ+1,1);
rSges_Zus = zeros(RS.NJ+1,3);
ISges_Zus =  zeros(RS.NJ+1,6);
% Masse 20kg mit Trägheitstensor wie Kugel, 150mm vom EE-Flansch entfernt.
% Zusätzlich noch 100mm Hebel zur Seite (für Belastung der Handgelenke)
mZ = 20;
rZ = [0.050;0.100;-0.150]; % bezogen auf letztes Körper-KS
IZges = zeros(1,6);
IZges(1:3) = 2/5 * mZ * (100e-3)^2; % Kugel Radius 100mm
IZges(4:6) = 0;
mges_Zus(end) = mZ;
rSges_Zus(end,:) = rZ;
ISges_Zus(end,:) = IZges;
[mrSges_Zus, Ifges_Zus] = ...% Parameter in anderes Format konvertieren
  inertial_parameters_convert_par1_par2(rSges_Zus, ISges_Zus, mges_Zus);
% Erstelle den gesamten Parametervektor, eintragen in Klasse
mges = mges_Rob + mges_Zus;
mrSges = mrSges_Rob + mrSges_Zus;
Ifges = Ifges_Rob + Ifges_Zus;
RS.update_dynpar2(mges, mrSges, Ifges);

%% Vorwärtsdynamik
t_End = 5; % Simulationsdauer so, dass Ausschwingbewegung vollständig.
data_ges = struct('t', [], 'q', [], 'qD', [], 'qDD', [], 'E', [], 'tau', []);
data_ges_grav = struct('I', [], 'phi_base', []);
for kk = 1:4
  % Verschiedene Szenarien für die Vorwärtsdynamik. Basis-Orientierung wird
  % gedreht, um die Richtigkeit davon zu testen
  if kk == 1 % nach unten hängend
    RS.update_base([], [-pi;0;0]);
  elseif kk == 2 % seitlich
    RS.update_base([], [pi/2;0;0]);
  elseif kk == 3 % schräg nach unten
    RS.update_base([], [0;3*pi/4;0]);
  elseif kk == 4 % anders schräg nach unten
    RS.update_base([], [-pi/6;-pi/6;0]);
  else
    RS.update_base([], rand(3,1));
  end
  RS.update_gravity(g_world);
  NQJ = RS.NQJ;
  % Funktion zur Berechnung der Beschleunigung (Inverse Massenmatrix
  % multipliziert mit Beschleunigungsmoment)
  odeInvDyn = @(x) RS.invdyn(x(1:NQJ),x(NQJ+1:2*NQJ), zeros(NQJ,1))+RS.springtorque(x(1:NQJ));
  % Anfangwerte für die Integration
  qD0 = zeros(NQJ,1); x0 = [q0; qD0]; 
  % Massenmatrix wird in Matlab-ode berücksichtigt. Einfacherer Code.
  odemass = @(t, x) [eye(NQJ), zeros(NQJ,NQJ);zeros(NQJ,NQJ),RS.inertia(x(1:NQJ))];
  odemass(0, x0);
  odefun2 = @(t, x) ([x(NQJ+1:2*NQJ); -odeInvDyn(x)]);
  options2=odeset('MaxStep',1e-3);
  options2.Mass = odemass;
  options2.MStateDependence = 'strong'; % MM ist stark abhängig von Zustand
  options2.MassSingular = 'no'; % Massenmatrix ist nie singulär
  SolverOutput2 = ode45(odefun2, [0 t_End], x0, options2); % Vorwärtsdynamik berechnen

  % Extrahieren der Ergebnisse
  Tges_kk = SolverOutput2.x(:);
  Qges_kk = SolverOutput2.y(1:NQJ,:)';
  QDges_kk = SolverOutput2.y(NQJ+1:end,:)';
  % Bestimme Gelenkmoment, Beschleunigung und Energie
  QDDges_kk = NaN(size(Qges_kk));
  Eges_kk = NaN(length(Tges_kk),4);
  Fges_kk = NaN(size(Qges_kk));
  Tauges_kk = NaN(length(Tges_kk),4*RS.NJ);
  for i = 1:length(SolverOutput2.x)
    y_i = SolverOutput2.y(:,i);
    t_i = SolverOutput2.x(i);
    M_i_ode = odemass(t_i, y_i);
    M_i = M_i_ode(RS.NJ+1:end,RS.NJ+1:end);
    f_i_ode = odefun2(t_i, y_i);
    f_i = f_i_ode(RS.NJ+1:end);
    qdd_i = M_i \ f_i;
    QDDges_kk(i,:) = qdd_i;
    % Energie berechnen
    Eges_kk(i,1:3) = [ ...
      RS.ekin(Qges_kk(i,:)', QDges_kk(i,:)'), ...
      RS.epot(Qges_kk(i,:)'), ...
      RS.epotspring(Qges_kk(i,:)')];
    Eges_kk(i,4) = sum(Eges_kk(i,1:3));
    % Dynamik mit abspeichern
    Fges_kk(i,:) = (M_i*qdd_i-f_i)';
    % Dynamik neu berechnen für Auswertung
    Tauges_kk(i,:) = [ ...
      (RS.inertia(Qges_kk(i,:)')*QDDges_kk(i,:)')', ...
      RS.gravload(Qges_kk(i,:)')', ...
      RS.corvec(Qges_kk(i,:)',QDges_kk(i,:)')', ...
      RS.springtorque(Qges_kk(i,:)')'];
  end
  % Plausibilitätstest
  err_energy = diff(minmax2(Eges_kk(:,4)'));
  if abs(err_energy) > 1e-6
    error('Fehler der Energiekonsistenz der Vorwärtsdynamik ist %1.4e.', err_energy);
  end
  
  % Geschwindigkeit neu mit Trapezregel berechnen aus Beschleunigung (Integration)
  QDgesnum = repmat(QDges_kk(1,:),size(QDges_kk,1),1)+cumtrapz(Tges_kk, QDDges_kk);

  % Zeitverlauf der Gelenkgrößen
  change_current_figure(kk);clf;
  subplot(2,2,1);hold on;
  plot(Tges_kk, Qges_kk);
  ylabel('q'); grid on;
  subplot(2,2,2);hold on;
  plot(Tges_kk, QDges_kk, '-');
  ylabel('qD'); grid on;
  subplot(2,2,3);hold on;
  plot(Tges_kk, QDDges_kk, '-');
  ylabel('qDD'); grid on;
  subplot(2,2,4);hold on;
  plot(Tges_kk, Eges_kk, '-');legend({'kin', 'grav', 'spring', 'total'});
  ylabel('Energie'); grid on;
  sgtitle('Kinematik und Energie');
  linkxaxes
  change_current_figure(10+kk);clf;
  for j = 1:RS.NJ
    subplot(3,2,j);hold on;
    plot(Tges_kk, Tauges_kk(:,j:RS.NJ:end), '-');
    plot(Tges_kk, -sum(Tauges_kk(:,RS.NJ+j:RS.NJ:end),2), '--');
    ylabel(sprintf('\\tau_%d', j)); grid on;
  end
  sgtitle('Dynamik');
  legend({'inertia', 'grav', 'coriolis', 'spring', 'acc'});
  linkxaxes
  if ~usr_plot_animation, continue; end
  %% Animation der freien Bewegung
  maxduration_animation = 10; % Dauer der Animation als mp4 (in s)
  t_Vid = (0:1/30*(Tges_kk(end)/maxduration_animation):Tges_kk(end))';
  I_anim = knnsearch( Tges_kk , t_Vid ); % Berechne Indizes in Traj.-Zeitstempeln
  s_anim = struct('mp4_name', fullfile(resdir, sprintf('SerRob_IndRob_fdyn_springtorque_start%d.mp4',kk)));
  s_plot = struct( 'ks', [1:RS.NJ, RS.NJ+2], 'straight', 0, 'mode', 1);
  fhdl=change_current_figure(100+kk);clf;
  set(fhdl,'units','normalized','outerposition',[0 0 1 1],'color','w');
  hold on; grid on;
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
  view(3);
  title(sprintf('Freie Bewegung mit Gelenkfeder. Fall %d', kk));
  RS.anim( Qges_kk(I_anim,:), [], s_anim, s_plot);
end
