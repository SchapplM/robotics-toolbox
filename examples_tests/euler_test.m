% Teste unterschiedliche Transformationsfunktionen für Euler-Winkel
% Nutze alle möglichen Kombinationen für Euler-Winkel
%
% Literatur
% [Rob1] Skript Robotik 1

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

%% Init
clc
clear

%% Alle Euler-Winkel-Kombinationen durchgehen
N = 12;
zlr = 0;
axes_comb = NaN(N,3);
xyzstrings = {'x', 'y', 'z'};
eulstrings = cell(1,12);
for i = 1:3
  for j = 1:3
    for k = 1:3
      if i == j || j == k
        continue
      end
      zlr = zlr + 1;
      axes_comb(zlr,:) = [i,j,k];
      eulstrings{zlr} = [xyzstrings{i}, xyzstrings{j}, xyzstrings{k}];
    end
  end
end

for i_conv = uint8(1:N)
  eulstr = eulstrings{i_conv};
  
  %% zufällige Rotationsmatrizen generieren (mit passender Konvention)
  n = 10000;
  phi_ges = 2*(0.5-rand(n,3))*pi; % Winkel zwischen -pi und pi
  R_ges = NaN(3,3,n);

  for i = 1:n
    phi_i = phi_ges(i,:)';
    R_ges(:,:,i) = eye(3);
    for j = 1:3
      ax = axes_comb(i_conv,j);
      if ax == 1
        R_ges(:,:,i) = R_ges(:,:,i) * rotx(phi_i(j));
      elseif ax == 2
        R_ges(:,:,i) = R_ges(:,:,i) * roty(phi_i(j));
      elseif ax == 3
        R_ges(:,:,i) = R_ges(:,:,i) * rotz(phi_i(j));
      end
    end
  end
  
  %% Teste eul2r
  for i = 1:n
    phi_i = phi_ges(i,:)';
    R_i = R_ges(:,:,i);
    R_sym = eul2r(phi_i, i_conv);
    if any(abs(R_sym(:) - R_i(:)) > 1e-10)
      error('symbolisch generierte Funktion stimmt nicht');
    end
  end

  %% Teste r2eul
  for i = 1:n
    R_i = R_ges(:,:,i);
    phi_i = r2eul(R_i, i_conv);
    R_i_test = eul2r(phi_i, i_conv);

    if any( abs( R_i(:)-R_i_test(:) ) > 1e-10 )
      error('Umrechnung r2eul%s stimmt nicht', eulstr);
    end
  end
  fprintf('%d Umrechnungen mit r2eul%s getestet\n', n, eulstr);
  
  %% Aufruf der Gradientenmatrizen zwischen Rotationsmatrizen und Euler-Winkeln
  n_sing = 0;
  for i = 1:n
    % erste Orientierung zufällig vorgeben
    R_1 = R_ges(:,:,i);
    phi_1 = r2eul(R_1, i_conv); % Euler-Winkel-Darstellung der 1. Orientierung
    
    % Zur Diagnose
    J = euljac(phi_1, i_conv);
    if cond(J) > 1e2
      n_sing = n_sing+1;
      continue
    end
    
    % Zufällige infinitesimale Änderung der Orientierung führt zur 2.
    % Orientierung
    delta_phi = rand(3,1)*1e-8;
    phi_2 = phi_1 + delta_phi;
    R_2 = eul2r(phi_2, i_conv); % Darstellung der 2. Orientierung als Rotationsmatrix
    % Differentielle Änderung der Rotationsmatrix (aus absoluten
    % Orientierungen berechnet)
    delta_R = R_2 - R_1;
    
    % Berechnung differentielle Änderung der Euler-Winkel aus der
    % Gradientenmatrix (Ableitung der Euler-Winkel nach der Rot.-matrix)
    dphi_dr = eul_diff_rotmat(R_1, i_conv);
    delta_phi_test = dphi_dr * delta_R(:);
    if any(abs(delta_phi_test - delta_phi)>1e-10)
      error('Delta phi aus eul%s_diff_rotmat stimmt nicht mit direkter Berechnung', eulstr);
    end
    
    % Berechne differentielle Änderung der Rotationsmatrix aus
    % Gradienten-Matrix (Ableitung der Rotationsmatrix nach Euler-Winkeln)
    dr_dphi = rotmat_diff_eul(phi_1, i_conv);
    delta_R_test = reshape(dr_dphi*delta_phi,3,3);
    if any(abs(delta_R_test(:) - delta_R(:)) > 1e-10)
      error('Delta Rotationsmatrix aus rotmat_diff_eul%s stimmt nicht mit direkter Berechnung', eulstr);
    end

    % Die Multiplikation der Gradienten muss Eins ergeben
    % (Differentiale kürzen sich weg)
    % Nahe von Singularitäten wird der Gradient dphi_dr sehr groß
    % (Ist hauptsächlich ein Problem bei anderer Implementierung)
    test = dphi_dr*dr_dphi - eye(3);
    if max(abs(test(:))) > 1e5*eps(1+max(abs(dphi_dr(:)))) % feinere Toleranz bei anderer Implementierung
      % Dieser Fehler tritt nahe einer Orientierungssingularität auf
      % Die Werte sind aber trotzdem nicht viel zu groß
      % Singuläre Stellungen werden durch Abfrage oben vermieden
      error('i=%d: Gradientenmatrizen eul%s_diff_rotmat und rotmat_diff_eul%s stimmen nicht überein\n', i, eulstr, eulstr);
    end
  end
  fprintf('%d/%d Gradientenmatrizen eul%s_diff_rotmat und rotmat_diff_eul%s getestet. Der Rest (fast) singulär.\n', ...
    n-n_sing, n, eulstr, eulstr);
  
  %% Testen der Transformationsmatrizen euljac und euljacD bzgl der Zeitableitungen
  % Test: euljac, euljacD
  for i = 1:n
    % erste Orientierung zufällig vorgeben
    R_1 = R_ges(:,:,i);
    phi_1 = r2eul(R_1, i_conv); % Euler-Winkel-Darstellung der 1. Orientierung
    
    % Zufällige infinitesimale Änderung der Orientierung führt zur 2.
    % Orientierung
    delta_phi = rand(3,1)*1e-8;
    phi_2 = phi_1 + delta_phi;
    R_2 = eul2r(phi_2, i_conv); % Darstellung der 2. Orientierung als Rotationsmatrix
    % Orientierungsänderung als Geschwindigkeit darstellen
    delta_t = 1e-8;
    phiD = delta_phi / delta_t;
    % Zeitableitung der Rotationsmatrix aus Differenzenquotienten
    delta_R = R_2 - R_1;
    RD = delta_R / delta_t;
    % Winkelgeschwindigkeit aus Euler-Winkel-Zeitableitung
    omega_tilde = RD * R_1'; % [Rob1], Gl. 7.4 (Eulersche Differentiationsregel)
    omega = vex(omega_tilde);
    
    % Berechnung der Winkelgeschwindigkeit auf zweitem Weg über die
    % Transformationsmatrix der Euler-Winkel
    J = euljac(phi_1, i_conv);
    omega_test = J * phiD;
    
    % Vergleiche, ob Winkelgeschwindigkeit auf beiden Wegen gleich ist
    abserr = omega_test - omega;
    relerr = omega_test./omega-1;
    if any(abs(abserr(:))>1e-7) && max(abs(relerr)) > 1e-3
      error('Transformationsmatrix eul%sjac stimmt nicht mit r2eul/eul2r überein', eulstr);
    end    
    
    % Bestimme die Euler-Transformationsmatrix für die beiden infinitesimal
    % voneinander entfernten Orientierungen von vorher
    J1 = euljac(phi_1, i_conv);
    J2 = euljac(phi_2, i_conv);
    % Bestimme die Zeitableitung der Transformationsmatrix aus der
    % Geschwindigkeit von oben
    JD = euljacD(phi_1, phiD, i_conv);
    % Alternative Berechnung der Zeitableitung aus Differenzenquotienten
    JD_test = (J2-J1) / delta_t;
    % Vergleich ob beide Lösungen übereinstimmen und damit  ob die
    % symbolische  Herleitung in euljacD stimmt
    test = JD_test -JD;
    if any(abs(test(:))>1e-7)
      error('Transformationsmatrix eul%sjacD stimmt nicht mit euljac überein', eulstr);
    end 
  end
  fprintf('%d Transformationsmatrizen eul%sjac und eul%sjacD getestet\n', n, eulstr, eulstr);
  
  %% Teste Zeitableitungen der Euler-Gradientenmatrizen
  % Test: eulD_diff_rotmat, rotmatD_diff_eul
  for i = 1:n
    % erste Orientierung zufällig vorgeben
    R_1 = R_ges(:,:,i);
    phi_1 = r2eul(R_1, i_conv); % Euler-Winkel-Darstellung der 1. Orientierung

    % Zufällige infinitesimale Änderung der Orientierung führt zur 2.
    % Orientierung
    delta_phi = rand(3,1)*1e-9;
    phi_2 = phi_1 + delta_phi;
    R_2 = eul2r(phi_2, i_conv); % Darstellung der 2. Orientierung als Rotationsmatrix
    
    % Orientierungsänderung als Geschwindigkeit darstellen
    delta_t = 1e-9;
    phiD = delta_phi / delta_t;
    % Zeitableitung der Rotationsmatrix aus Differenzenquotienten
    delta_R = R_2 - R_1;
    RD = delta_R / delta_t;
    
    % Gradientenmatrix berechnen
    B_1 = rotmat_diff_eul(phi_1, i_conv);
    B_2 = rotmat_diff_eul(phi_2, i_conv);
    BD_sym = rotmatD_diff_eul(phi_1, phiD, i_conv);
    BD_num = (B_2-B_1)/delta_t;
    test_BD = BD_sym - BD_num;
    if max(abs(test_BD(:))) > 1e10*eps(1+max(abs(BD_num(:)))) % Schwellwert ca. 4e-6 für normale Werte
      error('Transformationsmatrix rotmatD_diff_eul%s stimmt nicht gegen rotmatD_diff_eul%s', eulstr, eulstr);
    end

    % Zur Diagnose
    J_1 = euljac(phi_1, i_conv);
    J_2 = euljac(phi_1, i_conv);
    if cond(J_1) > 1e2 || cond(J_2) > 1e2
      % Für fast-singuläre Orientierungen funktioniert eul_diff_rotmat nicht gut.
      % Für rotmat_diff_eul geht es immer (keine Singularitäten in diese
      % Transformationsrichtung)
      n_sing = n_sing+1;
      continue
    end

    % Gradientenmatrix für beide Orientierungen berechnen
    A_1 = eul_diff_rotmat(R_1, i_conv);
    A_2 = eul_diff_rotmat(R_2, i_conv);
    % Zeitableitung der Gradientenmatrix analytisch und per
    % Differenzenquotient
    AD_sym = eulD_diff_rotmat(R_1, RD, i_conv);
    AD_num = (A_2-A_1)/delta_t;
    test_AD = AD_sym - AD_num;
    if max(abs(test_AD(:))) > 1e10*eps(1+max(abs(AD_num(:)))) % Schwellwert ca. 1e-6 für normale Werte
      error('Transformationsmatrix eul%sD_diff_rotmat stimmt nicht gegen eul%s_diff_rotmat', eulstr, eulstr);
    end
    % Zeitableitung des Tests auf Einheitsmatrix von oben: Produktregel
    test_ABD = AD_sym*B_1 + A_1*BD_sym;
    if max(abs(test_ABD(:))) > 1e11*eps(1+max(abs(AD_sym(:)))) % Schwellwert ca. 1e-5 für normale Werte
      error('Transformationsmatrix eul%sD_diff_rotmat stimmt nicht gegen eul%s_diff_rotmat', eulstr, eulstr);
    end
  end
  fprintf('%d Gradienten rotmatD_diff_eul%s getestet\n', n, eulstr);
  fprintf('%d/%d Gradienten eul%sD_diff_rotmat getestet. Der Rest (nahezu) singulär\n', ...
    n-n_sing,n, eulstr);
end
matlabfcn2mex({'r2eul', 'eul2r', 'rotmat_diff_eul', 'rotmatD_diff_eul', ...
  'eul_diff_rotmat', 'eulD_diff_rotmat', 'euljac', 'euljacD'});