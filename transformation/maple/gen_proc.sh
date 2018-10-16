#!/bin/bash -e
# Erzeuge Maple-Prozeduren
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
# (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

# eul2r: Maple-Prozedur generieren
for f in `find codeexport/eul*2r`; do
  eulstr=${f:14:3}
  echo "Erstelle Maple eul2r für $eulstr"
  # Prozedur erstellen
  zd="proc_eul${eulstr}2r"
  echo "eul${eulstr}2r := proc (phi)" > $zd
  echo "#Berechnung der Rotationsmatrix aus (intrinsischen) $eulstr-Euler-Winkeln" >> $zd
  echo "local RM, phi1, phi2, phi3:" >> $zd
  echo "phi1:=phi(1): phi2:=phi(2): phi3:=phi(3):" >> $zd
  echo "# RM := rot${eulstr:0:1}(phi1) . rot${eulstr:1:1}(phi2) . rot${eulstr:2:1}(phi3):" >> $zd
  cat $f >> $zd
  echo "return RM:" >> $zd
  echo "end proc:" >> $zd
done

# eul2r: Matlab-Funktionen generieren
for f in `find codeexport/eul*2r_matlab.m`; do
  eulstr=${f:14:3}
  echo "Erstelle Matlab eul2r für $eulstr"
  # Matlab-Funktion erstellen
  zd="../eul${eulstr}2r.m"
  echo "% ${eulstr}-Euler-Winkel in eine Rotationsmatrix konvertieren" > $zd
  echo "% Konvention: R = rot${eulstr:0:1}(phi1) * rot${eulstr:1:1}(phi2) * rot${eulstr:2:1}(phi3). (mitgedrehte Euler-Winkel; intrinsisch)" >> $zd
  echo ""  >> $zd
  echo "% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10" >> $zd
  echo "% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover" >> $zd
  echo ""  >> $zd
  echo "function R = eul${eulstr}2r(phi)" >> $zd
  echo "%% Init" >> $zd
  echo "%#codegen" >> $zd
  echo "%\$cgargs {zeros(3,1)}" >> $zd
  echo "assert(isreal(phi) && all(size(phi) == [3 1]), 'eul${eulstr}2r: phi has to be [3x1] (double)');" >> $zd
  echo "phi1=phi(1); phi2=phi(2); phi3=phi(3);" >> $zd
  echo "%% Berechnung" >> $zd
  echo "% R = rot${eulstr:0:1}(phi1) * rot${eulstr:1:1}(phi2) * rot${eulstr:2:1}(phi3):" >> $zd
  echo "% aus $f (euler_angle_calculations.mw)" >> $zd
  cat $f >> $zd
  varname_tmp=`grep "=" $f | tail -1 | sed 's/\([a-zA-Z0-9_]*\).*/\1/'`
  echo "R = $varname_tmp;" >> $zd
done

# r2eul: Matlab-Funktionen generieren
# Die Funktionen sollten aus dem Maple-Code generiert werden, damit dieser direkt geprüft werden kann
for f in `find codeexport/r2eul*_matlab.m`; do
  eulstr=${f:16:3}
  echo "Erstelle Matlab r2eul für $eulstr"
  # Matlab-Funktion erstellen
  zd="../r2eul${eulstr}.m"
  echo "% Rotationsmatrix nach ${eulstr}-Euler-Winkel konvertieren" > $zd
  echo "% Konvention: R = rot${eulstr:0:1}(phi1) * rot${eulstr:1:1}(phi2) * rot${eulstr:2:1}(phi3)." >> $zd
  echo "% (mitgedrehte Euler-Winkel; intrinsisch)" >> $zd
  echo "%"  >> $zd
  echo "% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10" >> $zd
  echo "% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover" >> $zd
  echo "%"  >> $zd
  echo "function phi = r2eul${eulstr}(R)" >> $zd
  echo "%% Init" >> $zd
  echo "%#codegen" >> $zd
  echo "%\$cgargs {zeros(3,3)}" >> $zd
  echo "assert(isreal(R) && all(size(R) == [3 3]), 'r2eul${eulstr}: R has to be [3x3] (double)');" >> $zd
  echo "r11=R(1,1);r12=R(1,2);r13=R(1,3);" >> $zd
  echo "r21=R(2,1);r22=R(2,2);r23=R(2,3); %#ok<NASGU>" >> $zd
  echo "r31=R(3,1);r32=R(3,2);r33=R(3,3); %#ok<NASGU>" >> $zd
  echo "%% Berechnung" >> $zd
  echo "% aus $f (euler_angle_calculations.mw)" >> $zd
  cat $f >> $zd
  varname_tmp=`grep "=" $f | tail -1 | sed 's/\([a-zA-Z0-9_]*\).*/\1/'`
  echo "phi = $varname_tmp;" >> $zd
done

# eul_diff_rotmat: Matlab-Funktionen generieren
for f in `find codeexport/eul*_diff_rotmat_matlab.m`; do
  eulstr=${f:14:3}
  echo "Erstelle Matlab eul_diff_rotmat für $eulstr"
  # Matlab-Funktion erstellen
  zd="../eul${eulstr}_diff_rotmat.m"
  echo "% Ableitung der ${eulstr}-Euler-Winkel nach der daraus berechneten Rotationsmatrix" > $zd
  echo "% Konvention: R = rot${eulstr:0:1}(phi1) * rot${eulstr:1:1}(phi2) * rot${eulstr:2:1}(phi3)." >> $zd
  echo "% (mitgedrehte Euler-Winkel; intrinsisch)" >> $zd
  echo "%"  >> $zd
  echo "% Eingabe:"  >> $zd
  echo "% R [3x3]:"  >> $zd
  echo "%   Rotationsmatrix"  >> $zd
  echo "%"  >> $zd
  echo "% Ausgabe:"  >> $zd
  echo "% GradMat [3x9]:"  >> $zd
  echo "%   Gradientenmatrix: Ableitung der Euler-Winkel nach der (spaltenweise gestapelten) Rotationsmatrix"  >> $zd
  echo ""  >> $zd
  echo "% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10" >> $zd
  echo "% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover" >> $zd
  echo ""  >> $zd
  echo "function GradMat = eul${eulstr}_diff_rotmat(R)" >> $zd
  echo "%% Init" >> $zd
  echo "%#codegen" >> $zd
  echo "%\$cgargs {zeros(3,3)}" >> $zd
  echo "assert(isreal(R) && all(size(R) == [3 3]), 'eul${eulstr}_diff_rotmat: R has to be [3x3] (double)');" >> $zd
  echo "r11=R(1,1);r12=R(1,2);r13=R(1,3);" >> $zd
  echo "r21=R(2,1);r22=R(2,2);r23=R(2,3); %#ok<NASGU>" >> $zd
  echo "r31=R(3,1);r32=R(3,2);r33=R(3,3); %#ok<NASGU>" >> $zd
  echo "%% Berechnung" >> $zd
  echo "% aus $f (euler_angle_calculations.mw)" >> $zd
  cat $f >> $zd
  varname_tmp=`grep "=" $f | tail -1 | sed 's/\([a-zA-Z0-9_]*\).*/\1/'`
  echo "GradMat = $varname_tmp;" >> $zd
done

# rotmat_diff_eul: Matlab-Funktionen generieren
for f in `find codeexport/rotmat_diff_eul*_matlab.m`; do
  eulstr=${f:26:3}
  echo "Erstelle Matlab rotmat_diff_eul für $eulstr"
  # Matlab-Funktion erstellen
  zd="../rotmat_diff_eul${eulstr}.m"
  echo "% Ableitung der Rotationsmatrix nach den sie erzeugenden ${eulstr}-Euler-Winkel" > $zd
  echo "% Konvention: R = rot${eulstr:0:1}(phi1) * rot${eulstr:1:1}(phi2) * rot${eulstr:2:1}(phi3)." >> $zd
  echo "% (mitgedrehte Euler-Winkel; intrinsisch)" >> $zd
  echo "%"  >> $zd
  echo "% Eingabe:"  >> $zd
  echo "% phi [3x1]:"  >> $zd
  echo "%   ${eulstr}-Euler-Winkel"  >> $zd
  echo "%"  >> $zd
  echo "% Ausgabe:"  >> $zd
  echo "% GradMat [9x3]:"  >> $zd
  echo "%   Gradientenmatrix: Ableitung der (spaltenweise gestapelten) Rotationsmatrix nach den Euler-Winkeln"  >> $zd
  echo "%"  >> $zd
  echo "% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10" >> $zd
  echo "% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover" >> $zd
  echo ""  >> $zd
  echo "function GradMat = rotmat_diff_eul${eulstr}(phi)" >> $zd
  echo "%% Init" >> $zd
  echo "%#codegen" >> $zd
  echo "%\$cgargs {zeros(3,1)}" >> $zd
  echo "assert(isreal(phi) && all(size(phi) == [3 1]), 'rotmat_diff_eul${eulstr}: phi has to be [3x1] (double)');" >> $zd
  echo "phi1=phi(1); phi2=phi(2); phi3=phi(3);" >> $zd
  echo "%% Berechnung" >> $zd
  echo "% aus $f (euler_angle_calculations.mw)" >> $zd
  cat $f >> $zd
  varname_tmp=`grep "=" $f | tail -1 | sed 's/\([a-zA-Z0-9_]*\).*/\1/'`
  echo "GradMat = $varname_tmp;" >> $zd
done

