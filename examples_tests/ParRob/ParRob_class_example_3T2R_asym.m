% Beispielskript für asymmetrische 3T2R-PKM (nur aktive Beinketten)
% * Eine 3T2R-Führungskette: PUU-Leading, RUU, UPU
% * kombiniert mit 3T3R-Hauptstruktur (vier Ketten): PUS, RUS, UPS

% MA Bejaoui (Bejaoui2020_M963; Betreuer: Moritz Schappler), 2020-04
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

if isempty(which('serroblib_path_init.m'))
  warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end
if isempty(which('parroblib_path_init.m'))
  warning('Repo mit parallelen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end

for LeadingNr = 1:3 % Führungskette: PUU, RUU, UPU
  for BeinNr = 1:3 % Hauptstruktur: PUS, RUS, UPS
    close all % sonst sind zu viele Bilder offen
    fignum = 100*LeadingNr+10*BeinNr;
    if LeadingNr == 1 % PUU-Leading
      RS1 = serroblib_create_robot_class('S5PRRRR10');% Fuehrungsbeinkette
      RS1.fill_fcn_handles(false);
      if BeinNr == 1 % PUS
        RS2 = serroblib_create_robot_class('S6PRRRRR6');% Fuehrungsbeinkette
        RS2.fill_fcn_handles(false);
        RP = ParRob('PKM_1PUU_4UPS_3T2R');
        RP.NLEG = 5;
        RP.Leg = copy(RS1); % Fuehrungsbeinkette PUU
        pkin_RS1 = [0, 0, 1.0, 0, pi/2, pi/2, 0, 0, 0, 0, 0]';
        RP.Leg(1).update_mdh(pkin_RS1(1:size(RP.Leg(1).pkin),1));
        for ii=2:RP.NLEG % Folgebeinketten 4 UPS( vielleicht Verallgemeinerung fuer spaeter)
          RP.Leg(ii) = copy(RS2);
        end
        %   a2 a3  a4  a5 a6 2  3     4  d2 d3 d4 d5 d6 t1
        pkin_RS2 = [0, 0, 1.0, 0, 0, 0, pi/2, 0, 0, 0, 0, 0, 0 ,0 ]';
        for ii=2:RP.NLEG % Folgebeinketten 4 UPS( vielleicht Verallgemeinerung fuer spaeter)
          RP.Leg(ii).update_mdh(pkin_RS2(1:size(RP.Leg(ii).pkin),1));
        end
        RP.align_base_coupling(1, 2.0);  % Wahl ist wichtig fuer Loesbarkeit der IK
        RP.align_platform_coupling(1, 0.2); % Wahl ist wichtig fuer Loesbarkeit der IK
        RP.initialize();
        
        RP.Leg(1).update_base( 3*RP.r_P_B_all(:,1), r2eulxyz(rotx(-pi)*roty(-pi))); % Fuehrungsbeinkette PUU
        RP.Leg(2).update_base( 3*RP.r_P_B_all(:,2), r2eulxyz(rotx(0)*rotz((-7*pi/10))));
        RP.Leg(3).update_base( 3*RP.r_P_B_all(:,3), r2eulxyz(rotx(0)*rotz(-pi/5)));
        RP.Leg(4).update_base( 3*RP.r_P_B_all(:,4), r2eulxyz(rotx(0)*rotz(pi/5)));
        RP.Leg(5).update_base( 3*RP.r_P_B_all(:,5), r2eulxyz(rotx(0)*rotz(7*pi/10)));
        I_qa_Typ1 = zeros(1,5); % Typ1: S5PRRRR1
        I_qa_Typ1(1) = 1; % das 2te Gelenk P ist aktuiert
        I_qa_Typ2 = zeros(1,6); % Typ2: S6RRPRRR14V3
        I_qa_Typ2(1) = 1;
        I_qa = [I_qa_Typ1,repmat(I_qa_Typ2,1,4)];
        RP.update_actuation(I_qa);
        % TODO
        for ii = 1:RP.NLEG
          RP.Leg(ii).update_EE(zeros(3,1),[pi/2;0;0]);
        end
             
      elseif BeinNr == 2 % RUS
        RS2 = serroblib_create_robot_class('S6RRRRRR10');% Fuehrungsbeinkette
        RS2.fill_fcn_handles(false);
        RP = ParRob('PKM_1PUU_4RUS_3T2R');
        % Beinketten definieren
        RP.NLEG = 5;
        RP.Leg = copy(RS1); % Fuehrungsbeinkette PUU
        pkin_RS1 = [0, 0, 1.7, 0, pi/2, pi/2, 0, 0, 0, 0, 0]';
        RP.Leg(1).update_mdh(pkin_RS1(1:size(RP.Leg(1).pkin),1));
        for ii=2:RP.NLEG % Folgebeinketten 4 UPS( vielleicht Verallgemeinerung fuer spaeter)
          RP.Leg(ii) = copy(RS2);
        end
        pkin_RS2 = [0.8, 0, 1.2, 0, 0, 0, pi/2, 0, 0, 0, 0, 0, 0, 0]';
        for ii=2:RP.NLEG
          RP.Leg(ii).update_mdh(pkin_RS2(1:size(RP.Leg(ii).pkin),1));
        end
        RP.align_base_coupling(1, 5);  % Wahl ist wichtig fuer Loesbarkeit der IK
        RP.align_platform_coupling(1, 0.3); % Wahl ist wichtig fuer Loesbarkeit der IK
        RP.initialize();
        % Orientierung der Gestell-Koppelpunkt-KS
        RP.Leg(1).update_base( 3*RP.r_P_B_all(:,1), r2eulxyz(rotx(-pi)*roty(-pi))); % Fuehrungsbeinkette PUU
        RP.Leg(2).update_base( 3*RP.r_P_B_all(:,2), r2eulxyz(rotx(pi/2)*roty((-7*pi/10)))); % Folgebeinketten RPS
        RP.Leg(3).update_base( 3*RP.r_P_B_all(:,3), r2eulxyz(rotx(pi/2)*roty(-pi/5))); % Folgebeinkette RUS
        RP.Leg(4).update_base( 3*RP.r_P_B_all(:,4), r2eulxyz(rotx(pi/2)*roty(pi/5))); % Folgebeinkette RUS
        RP.Leg(5).update_base( 3*RP.r_P_B_all(:,5), r2eulxyz(rotx(pi/2)*roty(7*pi/10)));
        
        I_qa_Typ1 = zeros(1,5);
        I_qa_Typ1(1) = 1;
        I_qa_Typ2 = zeros(1,6); % Typ2: S6RRPRRR14V3
        I_qa_Typ2(1) = 1;
        I_qa = [I_qa_Typ1,repmat(I_qa_Typ2,1,4)];
        RP.update_actuation(I_qa);
        % TODO
        for ii = 1:RP.NLEG
          RP.Leg(ii).update_EE(zeros(3,1),zeros(3,1));
        end

      elseif BeinNr == 3 % UPS
        RS2 = serroblib_create_robot_class('S6RRPRRR14V3');% Fuehrungsbeinkette
        RS2.fill_fcn_handles(false);
        RP = ParRob('PKM_1PUU_4UPS_3T2R');
        % Beinketten definieren
        RP.NLEG = 5;
        RP.Leg = copy(RS1); % Fuehrungsbeinkette PUU
        pkin_all = [0, 0, 0.3, 0, pi/2, pi/2, 0, 0, 0, 0, 0]';
        RP.Leg(1).update_mdh(pkin_all(1:size(RP.Leg(1).pkin),1));
        for ii=2:RP.NLEG % Folgebeinketten 4 UPS( vielleicht Verallgemeinerung fuer spaeter)
          RP.Leg(ii) = copy(RS2);
        end
        RP.align_base_coupling(1, 0.5);  % Wahl ist wichtig fuer Loesbarkeit der IK
        RP.align_platform_coupling(1, 0.15); % Wahl ist wichtig fuer Loesbarkeit der IK
        RP.initialize();
        % Orientierung der Gestell-Koppelpunkt-KS
        RP.Leg(1).update_base( 3*RP.r_P_B_all(:,1), r2eulxyz(rotx(0)*roty(0))); % Fuehrungsbeinkette PUU
        RP.Leg(2).update_base( 3*RP.r_P_B_all(:,2), r2eulxyz(rotx((pi)/3)*roty((-pi/5)))); % Folgebeinketten UPS
        RP.Leg(3).update_base( 3*RP.r_P_B_all(:,3), r2eulxyz(rotx(pi/6)*roty(pi/10))); % Folgebeinkette UPS
        RP.Leg(4).update_base( 3*RP.r_P_B_all(:,4), r2eulxyz(rotx(-pi/6)*roty(pi/10))); % Folgebeinkette UPS
        RP.Leg(5).update_base( 3*RP.r_P_B_all(:,5), r2eulxyz(rotx(-pi/3)*roty(-pi/5)));
        
        I_qa_Typ1 = zeros(1,5);
        I_qa_Typ1(1) = 1;
        I_qa_Typ2 = zeros(1,6);
        I_qa_Typ2(3) = 1;
        I_qa = [I_qa_Typ1,repmat(I_qa_Typ2,1,4)];
        RP.update_actuation(I_qa);
        % TODO
        for ii = 1:RP.NLEG
          RP.Leg(ii).update_EE(zeros(3,1),[pi/2;0;0]);
        end
      end
      
    elseif LeadingNr == 2 % RUU
      RS1 = serroblib_create_robot_class('S5RRRRR2');% Fuehrungsbeinkette
      RS1.fill_fcn_handles(false);
      if BeinNr == 1 % PUS
        RS2 = serroblib_create_robot_class('S6PRRRRR6');% Fuehrungsbeinkette
        RS2.fill_fcn_handles(false);
        RP = ParRob('PKM_1RUU_4PUS_3T2R');
        RP.NLEG = 5;
        RP.Leg = copy(RS1); % Fuehrungsbeinkette PUU
        pkin_RS1 = [1.0, 1.0]'; 
        RP.Leg(1).update_mdh(pkin_RS1(1:size(RP.Leg(1).pkin),1));
        for ii=2:RP.NLEG % Folgebeinketten 4 UPS( vielleicht Verallgemeinerung fuer spaeter)
          RP.Leg(ii) = copy(RS2);
        end
        %   a2 a3  a4  a5 a6 2  3     4  d2 d3 d4 d5 d6 t1
        pkin_RS2 = [0, 0, 1.2, 0, 0, 0, pi/2, 0, 0, 0, 0, 0, 0 ,0 ]';
        for ii=2:RP.NLEG % Folgebeinketten 4 UPS( vielleicht Verallgemeinerung fuer spaeter)
          RP.Leg(ii).update_mdh(pkin_RS2(1:size(RP.Leg(ii).pkin),1));
        end
        RP.align_base_coupling(1, 0.5);  % Wahl ist wichtig fuer Loesbarkeit der IK
        RP.align_platform_coupling(1, 0.2); % Wahl ist wichtig fuer Loesbarkeit der IK
        RP.initialize();
        
        RP.Leg(1).update_base( 3*RP.r_P_B_all(:,1), r2eulxyz(rotx(-pi/2)*rotz(pi))); % Fuehrungsbeinkette RUU
        RP.Leg(2).update_base( 3*RP.r_P_B_all(:,2), r2eulxyz(rotz(-7*pi/10))); 
        RP.Leg(3).update_base( 3*RP.r_P_B_all(:,3), r2eulxyz(rotz(-pi/5))); 
        RP.Leg(4).update_base( 3*RP.r_P_B_all(:,4), r2eulxyz(rotz(pi/5))); 
        RP.Leg(5).update_base( 3*RP.r_P_B_all(:,5), r2eulxyz(rotz(7*pi/10)));
        
        I_qa_Typ1 = zeros(1,5); % Typ1: S5PRRRR1
        I_qa_Typ1(1) = 1; % das 2te Gelenk P ist aktuiert
        I_qa_Typ2 = zeros(1,6); % Typ2: S6RRPRRR14V3
        I_qa_Typ2(1) = 1;
        I_qa = [I_qa_Typ1,repmat(I_qa_Typ2,1,4)];
        RP.update_actuation(I_qa);
        % TODO
        for ii = 1:RP.NLEG
          RP.Leg(ii).update_EE(zeros(3,1),[pi/2;0;0]);
        end
             
      elseif BeinNr == 2 % RUS
        RS2 = serroblib_create_robot_class('S6RRRRRR10');% Fuehrungsbeinkette
        RS2.fill_fcn_handles(false);
        RP = ParRob('PKM_1RUU_4RUS_3T2R');
        % Beinketten definieren
        RP.NLEG = 5;
        RP.Leg = copy(RS1); % Fuehrungsbeinkette PUU
        pkin_RS1 = [1.0, 1.0]'; 
        RP.Leg(1).update_mdh(pkin_RS1(1:size(RP.Leg(1).pkin),1));
        for ii=2:RP.NLEG % Folgebeinketten 4 UPS( vielleicht Verallgemeinerung fuer spaeter)
          RP.Leg(ii) = copy(RS2);
        end
        pkin_RS2 = [0.8, 0, 1.0, 0, 0, 0, pi/2, 0, 0, 0, 0, 0, 0, 0]';
        for ii=2:RP.NLEG
          RP.Leg(ii).update_mdh(pkin_RS2(1:size(RP.Leg(ii).pkin),1));
        end
        RP.align_base_coupling(1, 5);  % Wahl ist wichtig fuer Loesbarkeit der IK
        RP.align_platform_coupling(1, 0.2); % Wahl ist wichtig fuer Loesbarkeit der IK
        RP.initialize();
        % Orientierung der Gestell-Koppelpunkt-KS
        RP.Leg(1).update_base( 3*RP.r_P_B_all(:,1), r2eulxyz(rotx(-pi/2)*roty(-pi))); % Fuehrungsbeinkette PUU
        RP.Leg(2).update_base( 3*RP.r_P_B_all(:,2), r2eulxyz(rotx(pi/2)*roty((-7*pi/10)))); % Folgebeinketten RPS
        RP.Leg(3).update_base( 3*RP.r_P_B_all(:,3), r2eulxyz(rotx(pi/2)*roty(-pi/5))); % Folgebeinkette RUS
        RP.Leg(4).update_base( 3*RP.r_P_B_all(:,4), r2eulxyz(rotx(pi/2)*roty(pi/5))); % Folgebeinkette RUS
        RP.Leg(5).update_base( 3*RP.r_P_B_all(:,5), r2eulxyz(rotx(pi/2)*roty(7*pi/10)));
        
        I_qa_Typ1 = zeros(1,5);
        I_qa_Typ1(1) = 1;
        I_qa_Typ2 = zeros(1,6); % Typ2: S6RRPRRR14V3
        I_qa_Typ2(1) = 1;
        I_qa = [I_qa_Typ1,repmat(I_qa_Typ2,1,4)];
        RP.update_actuation(I_qa);
        % TODO
        for ii = 1:RP.NLEG
          RP.Leg(ii).update_EE(zeros(3,1),[pi/2;0;0]);
        end

      elseif BeinNr == 3 % UPS
        RS2 = serroblib_create_robot_class('S6RRPRRR14V3');% Fuehrungsbeinkette
        RS2.fill_fcn_handles(false);
        RP = ParRob('PKM_1RUU_4UPS_3T2R');
        % Beinketten definieren
        RP.NLEG = 5;
        RP.Leg = copy(RS1); % Fuehrungsbeinkette PUU
        pkin_all = [1.0, 1.0]'; 
        RP.Leg(1).update_mdh(pkin_all(1:size(RP.Leg(1).pkin),1));
        for ii=2:RP.NLEG % Folgebeinketten 4 UPS( vielleicht Verallgemeinerung fuer spaeter)
          RP.Leg(ii) = copy(RS2);
        end
        RP.align_base_coupling(1, 0.5);  % Wahl ist wichtig fuer Loesbarkeit der IK
        RP.align_platform_coupling(1, 0.2); % Wahl ist wichtig fuer Loesbarkeit der IK
        RP.initialize();
        % Orientierung der Gestell-Koppelpunkt-KS
        RP.Leg(1).update_base( 3*RP.r_P_B_all(:,1), r2eulxyz(rotx(-pi/2)*roty(pi))); % Fuehrungsbeinkette PUU
        RP.Leg(2).update_base( 3*RP.r_P_B_all(:,2), r2eulxyz(rotx((pi)/3)*roty((-pi/5)))); % Folgebeinketten UPS
        RP.Leg(3).update_base( 3*RP.r_P_B_all(:,3), r2eulxyz(rotx(pi/6)*roty(pi/10))); % Folgebeinkette UPS
        RP.Leg(4).update_base( 3*RP.r_P_B_all(:,4), r2eulxyz(rotx(-pi/6)*roty(pi/10))); % Folgebeinkette UPS
        RP.Leg(5).update_base( 3*RP.r_P_B_all(:,5), r2eulxyz(rotx(-pi/3)*roty(-pi/5)));
        
        I_qa_Typ1 = zeros(1,5);
        I_qa_Typ1(1) = 1;
        I_qa_Typ2 = zeros(1,6);
        I_qa_Typ2(3) = 1;
        I_qa = [I_qa_Typ1,repmat(I_qa_Typ2,1,4)];
        RP.update_actuation(I_qa);
        % TODO
        for ii = 1:RP.NLEG
          RP.Leg(ii).update_EE(zeros(3,1),[pi/2;0;0]);
        end
      end
      
      
    elseif LeadingNr == 3 % UPU-Leading
      RS1 = serroblib_create_robot_class('S5RRPRR12V1');% Fuehrungsbeinkette
      RS1.fill_fcn_handles(false);
      if BeinNr == 1 % PUS
        RS2 = serroblib_create_robot_class('S6PRRRRR6');% Fuehrungsbeinkette
        RS2.fill_fcn_handles(false);
        RP = ParRob('PKM_1UPU_4PUS_3T2R');
        RP.NLEG = 5;
        RP.Leg = copy(RS1); % Fuehrungsbeinkette PUU
        pkin_RS1 = zeros(4,1);
        RP.Leg(1).update_mdh(pkin_RS1(1:size(RP.Leg(1).pkin),1));
        for ii=2:RP.NLEG % Folgebeinketten 4 UPS( vielleicht Verallgemeinerung fuer spaeter)
          RP.Leg(ii) = copy(RS2);
        end
        %   a2 a3  a4  a5 a6 2  3     4  d2 d3 d4 d5 d6 t1
        pkin_RS2 = [0, 0, 1.2, 0, 0, 0, pi/2, 0, 0, 0, 0, 0, 0 ,0 ]';
        for ii=2:RP.NLEG % Folgebeinketten 4 UPS( vielleicht Verallgemeinerung fuer spaeter)
          RP.Leg(ii).update_mdh(pkin_RS2(1:size(RP.Leg(ii).pkin),1));
        end
        RP.align_base_coupling(1, 0.5);  % Wahl ist wichtig fuer Loesbarkeit der IK
        RP.align_platform_coupling(1, 0.2); % Wahl ist wichtig fuer Loesbarkeit der IK
        RP.initialize();
        
        RP.Leg(1).update_base( 3*RP.r_P_B_all(:,1), r2eulxyz(rotx(-pi/2)*rotz(pi))); % Fuehrungsbeinkette RUU
        RP.Leg(2).update_base( 3*RP.r_P_B_all(:,2), r2eulxyz(rotz(-7*pi/10))); 
        RP.Leg(3).update_base( 3*RP.r_P_B_all(:,3), r2eulxyz(rotz(-pi/5))); 
        RP.Leg(4).update_base( 3*RP.r_P_B_all(:,4), r2eulxyz(rotz(pi/5))); 
        RP.Leg(5).update_base( 3*RP.r_P_B_all(:,5), r2eulxyz(rotz(7*pi/10)));
        
        I_qa_Typ1 = zeros(1,5); % Typ1: S5PRRRR1
        I_qa_Typ1(3) = 1; % das 2te Gelenk P ist aktuiert
        I_qa_Typ2 = zeros(1,6); % Typ2: S6RRPRRR14V3
        I_qa_Typ2(1) = 1;
        I_qa = [I_qa_Typ1,repmat(I_qa_Typ2,1,4)];
        RP.update_actuation(I_qa);
        % TODO
        for ii = 1:RP.NLEG
          RP.Leg(ii).update_EE(zeros(3,1),[pi/2;0;0]);
        end
             
      elseif BeinNr == 2 % RUS
        RS2 = serroblib_create_robot_class('S6RRRRRR10');% Fuehrungsbeinkette
        RS2.fill_fcn_handles(false);
        RP = ParRob('PKM_1UPU_4RUS_3T2R');
        % Beinketten definieren
        RP.NLEG = 5;
        RP.Leg = copy(RS1); % Fuehrungsbeinkette PUU
        pkin_RS1 = zeros(4,1);
        RP.Leg(1).update_mdh(pkin_RS1(1:size(RP.Leg(1).pkin),1));
        for ii=2:RP.NLEG % Folgebeinketten 4 UPS( vielleicht Verallgemeinerung fuer spaeter)
          RP.Leg(ii) = copy(RS2);
        end
        pkin_RS2 = [0.8, 0, 1.0, 0, 0, 0, pi/2, 0, 0, 0, 0, 0, 0, 0]';
        for ii=2:RP.NLEG
          RP.Leg(ii).update_mdh(pkin_RS2(1:size(RP.Leg(ii).pkin),1));
        end
        RP.align_base_coupling(1, 5);  % Wahl ist wichtig fuer Loesbarkeit der IK
        RP.align_platform_coupling(1, 0.2); % Wahl ist wichtig fuer Loesbarkeit der IK
        RP.initialize();
        % Orientierung der Gestell-Koppelpunkt-KS
        RP.Leg(1).update_base( 3*RP.r_P_B_all(:,1), r2eulxyz(rotx(-pi/2)*roty(-pi))); % Fuehrungsbeinkette PUU
        RP.Leg(2).update_base( 3*RP.r_P_B_all(:,2), r2eulxyz(rotx(pi/2)*roty((-7*pi/10)))); % Folgebeinketten RPS
        RP.Leg(3).update_base( 3*RP.r_P_B_all(:,3), r2eulxyz(rotx(pi/2)*roty(-pi/5))); % Folgebeinkette RUS
        RP.Leg(4).update_base( 3*RP.r_P_B_all(:,4), r2eulxyz(rotx(pi/2)*roty(pi/5))); % Folgebeinkette RUS
        RP.Leg(5).update_base( 3*RP.r_P_B_all(:,5), r2eulxyz(rotx(pi/2)*roty(7*pi/10)));
        
        I_qa_Typ1 = zeros(1,5);
        I_qa_Typ1(3) = 1;
        I_qa_Typ2 = zeros(1,6); % Typ2: S6RRPRRR14V3
        I_qa_Typ2(1) = 1;
        I_qa = [I_qa_Typ1,repmat(I_qa_Typ2,1,4)];
        RP.update_actuation(I_qa);
        % TODO
        for ii = 1:RP.NLEG
          RP.Leg(ii).update_EE(zeros(3,1),[pi/2;0;0]);
        end

      elseif BeinNr == 3 % UPS
        RS2 = serroblib_create_robot_class('S6RRPRRR14V3');% Fuehrungsbeinkette
        RS2.fill_fcn_handles(false);
        RP = ParRob('PKM_1UPU_4UPS_3T2R');
        % Beinketten definieren
        RP.NLEG = 5;
        RP.Leg = copy(RS1); % Fuehrungsbeinkette PUU
        pkin_RS1 = zeros(4,1);
        RP.Leg(1).update_mdh(pkin_RS1(1:size(RP.Leg(1).pkin),1));
        for ii=2:RP.NLEG % Folgebeinketten 4 UPS( vielleicht Verallgemeinerung fuer spaeter)
          RP.Leg(ii) = copy(RS2);
        end
        RP.align_base_coupling(1, 0.5);  % Wahl ist wichtig fuer Loesbarkeit der IK
        RP.align_platform_coupling(1, 0.2); % Wahl ist wichtig fuer Loesbarkeit der IK
        RP.initialize();
        % Orientierung der Gestell-Koppelpunkt-KS
        RP.Leg(1).update_base( 3*RP.r_P_B_all(:,1), r2eulxyz(rotx(-pi/2)*roty(pi))); % Fuehrungsbeinkette PUU
        RP.Leg(2).update_base( 3*RP.r_P_B_all(:,2), r2eulxyz(rotx((pi)/3)*roty((-pi/5)))); % Folgebeinketten UPS
        RP.Leg(3).update_base( 3*RP.r_P_B_all(:,3), r2eulxyz(rotx(pi/6)*roty(pi/10))); % Folgebeinkette UPS
        RP.Leg(4).update_base( 3*RP.r_P_B_all(:,4), r2eulxyz(rotx(-pi/6)*roty(pi/10))); % Folgebeinkette UPS
        RP.Leg(5).update_base( 3*RP.r_P_B_all(:,5), r2eulxyz(rotx(-pi/3)*roty(-pi/5)));
        
        I_qa_Typ1 = zeros(1,5);
        I_qa_Typ1(3) = 1;
        I_qa_Typ2 = zeros(1,6);
        I_qa_Typ2(3) = 1;
        I_qa = [I_qa_Typ1,repmat(I_qa_Typ2,1,4)];
        RP.update_actuation(I_qa);
        % TODO
        for ii = 1:RP.NLEG
          RP.Leg(ii).update_EE(zeros(3,1),[pi/2;0;0]);
        end
      end
    end
    % Gelenkgrenzen setzen, damit IK-Neuversuche möglich sind
    for ii = 1:RP.NLEG
      RP.Leg(ii).qlim(RP.Leg(ii).MDH.sigma==0,:) = ...
        repmat([-pi,pi],sum(RP.Leg(ii).MDH.sigma==0),1);
      RP.Leg(ii).qlim(RP.Leg(ii).MDH.sigma==1,:) = ...
        repmat([-1,3],sum(RP.Leg(ii).MDH.sigma==1),1);
    end
    % allgemeine Einstellungen
    RP.Leg(1).I_EE = logical([1 1 1 1 1 0]);
    RP.update_EE_FG(logical([1 1 1 1 1 0]));
    % Startpose
    X0 = [[0.2;0.2;1.2];[5;10;-10]*pi/180]; % Plattform nur verdrehbar, keine Kipp-bwg
    q0 = rand(RP.NJ,1);
    q0(RP.I_qa) = 0.5;
    q = q0; % qs in constr2 und q sind ungleich ( also aktive Gelenke)
    %% IK
    X0(6) = 0;
    [q,phi] = RP.invkin_ser(X0, q);
    assert(all(abs(phi)<1e-6), 'IK funktioniert nicht, obwohl vorher ausprobiert');
    [Phi3_red,Phi3_voll] = RP.constr3(q, X0); % mit Fuehrungsbeinkette
    assert(all(size(Phi3_red)==[29,1]), 'Ausgabe 1 von constr3 muss 29x1 sein');
    assert(all(size(Phi3_voll)==[30,1]), 'Ausgabe 2 von constr3 muss 30x1 sein');
    X0(6) = X0(6) + Phi3_voll(4);
    % Aufruf der ZB-Modellierung 2. Ist für diese Art von Roboter aber
    % nicht zielführend.
    [Phi2_red,Phi2_voll] = RP.constr2(q, X0);
    assert(all(size(Phi2_red)==[30,1]), 'Ausgabe 1 von constr2 muss 30x1 sein');
    assert(all(size(Phi2_voll)==[30,1]), 'Ausgabe 2 von constr2 muss 30x1 sein');
    %% Roboter zeichnen
    figure(fignum+1);clf;
    hold on; grid on;
    xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
    view(3);
    s_plot = struct( 'ks_legs', [RP.I1L_LEG; RP.I2L_LEG], 'straight', 0);
    RP.plot( q, X0, s_plot );
    hold off;
    %% Jacobi-Matrix auswerten
    % Jacobi q-Anteile
    [G_q, G_q_voll] = RP.constr3grad_q(q, X0);
    assert(all(size(G_q)==[29 29]), 'ZB-matrix G_q muss 29x29 sein');
    assert(all(size(G_q_voll)==[30 29]), 'ZB-matrix G_q_voll muss 30x29 sein');
    % Jacobi x-Anteile (dim bei G_x_red von constr3grad noch nicht richtig)
    [G_x_red, G_x_voll] = RP.constr3grad_x(q, X0);
    assert(all(size(G_x_red)==[29 5]), 'ZB-matrix G_x_red muss 29x5 sein');
    assert(all(size(G_x_voll)==[30 6]), 'ZB-matrix G_x_voll muss 30x6 sein');
    G_x = G_x_voll(RP.I_constr_red,:);
    G_eta = G_x_voll(RP.I_constr_red,RP.I_EE_Task);
    % Aufteilung der Ableitung nach den Gelenken in Gelenkklassen
    G_a = G_q(:,RP.I_qa); % aktiv, phi_dqa [STO19]
    G_d = G_q(:,RP.I_qd); % passiv, phi_dpa [STO19]
    % Jacobi-Matrix zur Berechnung der abhaengigen Gelenke und EE-Koordinaten,
    % hier noch nicht quadratisch 30x29
    G_dx = [G_d, G_x]; % Gl. 50, phi_dxp, hier p-Anteile ueber x-Anteilen [STO19]
    J_voll1 = G_dx \ G_a; % inv(G_dx) * G_a = inv(phi_dxp)*phi_dqa, Gl. 50 vollstaendige Jacobi-Matrix bezogen auf x-Koordinaten [STO19]
    Jinv_voll1 = G_q \ G_x; % vollstaendige inverse Jacobi-Matrix in x-Koord
    
    % Jacobi-Matrix zur Berechnung der abhaengigen Gelenke und EE-Koordinaten,
    % hier quadratisch 29x29,  eta und nicht x fuer EE
    G_deta = [G_d, G_eta]; % Gl. 50, phi_dxp, hier p-Anteile ueber x-Anteilen [STO19]
    J_voll2 = G_deta \ G_a; % inv(G_dx) * G_a = inv(phi_dxp)*phi_dqa, Gl. 50, Jacobi-Matrix bezogen auf eta-Koordinaten [STO19]
    Jinv_voll2 = G_q \ G_eta; % Reduzierte inverse Jacobi-Matrix in eta-Koord
    
    J_qa_eta = Jinv_voll2(RP.I_qa,:); % Inverse Jacobi-Matrix des Roboters (eta-Koord)
    J_eta_qa = J_voll2(sum(RP.I_qd)+1:end,:); % Jacobi-Matrix (wird mit qaD multi.)
    
    % Prüfe inverse Jacobi-Matrix gegen nicht-invertierte
    matrix_test = J_eta_qa*J_qa_eta - eye(5); % 5x5
    if any(abs(matrix_test(:)) > 1e-4)
      error('Jacobi-Matrix und ihre Inverse passen nicht zueinander');
    end
    % FG test
    qaD = 100*rand(sum(RP.I_qa),1);
    qdDxD = J_voll1 * qaD;
    xD_test = qdDxD(sum(RP.I_qd)+1:end);
    if any(abs(xD_test(~RP.I_EE)) > 1e-4) % Genauigkeit hier ist abhaengig von Zwangsbed.
      fprintf('Falsche Koordinaten werden laut Jacobi-Matrix  durch die Antriebe bewegt\n');
    end
    
    %% Trajektorie berechnen
    k=1; XE = X0';
    d1=0.05;
    h1=0.05;
    r1=10*pi/180;
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0,0  r1,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0, 0, 0,-r1,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0,0, 0,r1,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0,0, -r1,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [d1,0,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [-d1,0,0, -0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,d1,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,-d1,0, -0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0,h1, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,-0,-h1, -0,0,0];
    [X,XD,XDD,T] = traj_trapez2_multipoint(XE, 1, 2e-1, 1e-1, 5e-3, 0);
    % Inverse Kinematik berechnen
    t1 = tic();
    fprintf('Berechne Trajektorien-IK für %d Zeitschritte\n', length(T));
    iksettings = struct('n_max', 5000, 'Phit_tol', 1e-8, 'Phir_tol', 1e-8, 'debug', true, ...
      'retry_limit', 0, 'mode_IK', 1, 'normalize', false);
    warning off
    [Q, QD, QDD, Phi] = RP.invkin_traj(X,XD,XDD,T,q,iksettings); % hier muessen einige Zeilen auskommentiert werden
    warning on
    fprintf('Trajektorien-IK in %1.2fs berechnet. Prüfe die Ergebnisse.\n', toc(t1));
    %% Trajektorie prüfen
    t1 = tic();
    i_error = -1;
    % Tatsächliche EE-Koordinaten mit vollständiger direkter Kinematik bestimmen
    for i = 1:length(T)
      if max(abs( Phi(i,:) )) > 1e-8 || any(isnan( Phi(i,:) ))
        warning('IK stimmt nicht bei i=%d. Wahrscheinliche Ursache: Ist innerhalb von n_max nicht konvergiert', i);
        i_error = i;
        break
      end
      % Direkte Kinematik berechnen
      Tc_ges = RP.fkine(Q(i,:)', NaN(6,1));
      % Schnitt-KS aller Beinketten bestimmen
      II_BiKS = 1;
      for j = 1:RP.NLEG
        II_BiKS = II_BiKS + RP.Leg(j).NL+1;
        T_Bi = Tc_ges(:,:,II_BiKS);
        R_0_Bj = T_Bi(1:3,1:3);
        R_Bj_P = eulxyz2r(RP.phi_P_B_all(:,j))';
        R_0_P = R_0_Bj * R_Bj_P;
        r_0_0_Bj = T_Bi(1:3,4);
        r_P_P_Bj = RP.r_P_B_all(:,j);
        r_0_Bj_P = -R_0_P*r_P_P_Bj;
        r_0_Ej = r_0_0_Bj + r_0_Bj_P;
        R_0_E = R_0_P * RP.T_P_E(1:3,1:3);
        if j == 1
          % die EE-Position aus der ersten Kette muss auch für folgende
          % Ketten eingehalten werden. Daher Speicherung.
          r_0_E_Legs = r_0_Ej;
          R_0_E_Legs = R_0_E;
        else
          test_eepos = r_0_E_Legs-r_0_Ej;
          if any(abs(test_eepos)>2e-6) % muss größer als IK-Toleranz sein
            error('i=%d: EE-Position aus Beinkette %d stimmt nicht mit Beinkette 1 überein. Fehler %1.2e', i, j, max(abs(test_eepos)));
          end
          test_eerot = R_0_E_Legs\R_0_E-eye(3);
          if any(abs(test_eerot(:)) > 1e-6)
            error('i=%d: EE-Rotation aus Beinkette %d stimmt nicht mit Beinkette 1 überein. Fehler %1.2e', i, j, max(abs(test_eerot(:))));
          end
        end
        T_E_Leg_j = rt2tr(R_0_E, r_0_Ej);
        x_i = RP.t2x(T_E_Leg_j);
        % Berechnet letzten Euler-Winkel neu aus der Beinkette
        X(i,6) = x_i(6);
      end
      % Neues xD berechnen
      XD(i,6) = 0; % Auf Null setzen, damit Aufruf auch mehrfach funktioniert.
      [~,Phi2D]=RP.constr2D(Q(i,:)', QD(i,:)', X(i,:)', XD(i,:)');
      XD(i,6) = Phi2D(4);

      % Berechne die Zeitableitung aller Zwangsbedingungs-Modellierungen
      % Alle müssen Null sein. Ansonsten bewegt sich die PKM auseinander.
      [~,Phi1D]=RP.constr1D(Q(i,:)', QD(i,:)', X(i,:)', XD(i,:)');
      if any(abs(Phi1D)>1e-2)
        error('Geschwindigkeit der Koppelpunkte mit constr1 ist nicht konsistent. Fehler %1.2e.', max(abs(Phi1D)));
      end
      [~,Phi2D]=RP.constr2D(Q(i,:)', QD(i,:)', X(i,:)', XD(i,:)');
      if any(abs(Phi2D)>1e-2)
        error('Geschwindigkeit der Koppelpunkte mit constr2 ist nicht konsistent. Fehler %1.2e.', max(abs(Phi2D)));
      end
      [~,Phi4D]=RP.constr4D(Q(i,:)', QD(i,:)', X(i,:)', XD(i,:)');
      if any(abs(Phi4D)>1e-2)
        error('Geschwindigkeit der Koppelpunkte mit constr4 ist nicht konsistent. Fehler %1.2e.', max(abs(Phi4D)));
      end
    end
    % Die Trajektorie hat nicht komplett funktioniert. Werte den Rest
    % trotzdem aus.
    if i_error ~= -1
      fprintf('Rob %d-%d (%s): Kein Erfolg der Trajektorien-IK. Abbruch bei %d/%d\n', ...
        LeadingNr, BeinNr, RP.mdlname, i_error-1, length(T));
      Q = Q(1:i_error-1,:);
      QD = QD(1:i_error-1,:);
      QDD = QDD(1:i_error-1,:);
      T = T(1:i_error-1);
      X = X(1:i_error-1,:);
      XD = XD(1:i_error-1,:);
      XDD = XDD(1:i_error-1,:);
    end
      
    Q_int = repmat(Q(1,:),length(T),1)+cumtrapz(T, QD);
    figure(fignum+2);clf;
    for i = 1:RP.NLEG
      for j = 1:RP.Leg(1).NJ
        subplot(RP.NLEG,6, sprc2no(RP.NLEG, 6, i, j)); hold all;
        hdl1=plot(T, Q    (:,RP.I1J_LEG(i)+j-1));
        hdl2=plot(T, Q_int(:,RP.I1J_LEG(i)+j-1));
        grid on;
        ylabel(sprintf('Leg %d, Joint %d', i, j));
      end
    end
    sgtitle('Konsistenz q-qD');
    legend([hdl1;hdl2], {'q', 'int(qD)'});
    linkxaxes;
    test_q_qD_kons = Q_int - Q;
    if max(abs(test_q_qD_kons(:))) > 1e-2
      warning('Q und QD aus Traj.-IK sind nicht konsistent');
    end
    fprintf(['Rob %d-%d (%s): Prüfung der Trajektorie abgeschlossen (Dauer: ', ...
      '%1.2f). Erstelle Animation.\n'], LeadingNr, BeinNr, RP.mdlname, toc(t1));
    %% Animation
    rob_path = fileparts(which('robotics_toolbox_path_init.m'));
    resdir = fullfile(rob_path, 'examples_tests', 'results');
    mkdirs(resdir);
    s_anim = struct('gif_name', fullfile(resdir, sprintf('3T2R_PKM_asym_test_%s.gif',RP.mdlname)));
    figure(fignum+3);clf;
    set(fignum+3,'units','normalized','outerposition',[0 0 1 1],'color','w'); % Vollbild
    hold on;
    plot3(X(:,1), X(:,2), X(:,3));
    grid on;
    xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
    view(3);
    title(RP.mdlname, 'interpreter', 'none');
    RP.anim( Q(1:20:length(T),:), X(1:20:length(T),:), s_anim, s_plot);
  end
end
