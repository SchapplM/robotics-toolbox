% Teste unterschiedliche Transformationsfunktionen für Euler-Winkel
% Nutze alle möglichen Kombinationen für Euler-Winkel

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

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

for i_conv = 1:N
  eulstr = eulstrings{i_conv};
  
  %% zufällige Rotationsmatrizen generieren (mit passender Konvention)
  n = 10000;
  phi_ges = (0.5-rand(n,3))*pi;
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
  
  %% Aufruf der Gradientenmatrizen
  for i = 1:n
    R_i = R_ges(:,:,i);
    phi_i = r2eul(R_i, i_conv);
    % Beide Gradienten berechnen
    dphi_dr = eul_diff_rotmat(R_i, i_conv);
    dr_dphi = rotmat_diff_eul(phi_i, i_conv);
    % Die Multiplikation der Gradienten muss eins ergeben
    % (Differentiale kürzen sich weg)
    test = dphi_dr*dr_dphi - eye(3);
    if any(abs(test(:))>1e-10)
      error('Gradientenmatrizen eul%s_diff_rotmat und rotmat_diff_eul%s stimmen nicht überein', eulstr, eulstr);
    end    
  end
  fprintf('%d Gradientenmatrizen eul%s_diff_rotmat und rotmat_diff_eul%s getestet\n', n, eulstr, eulstr);
end
