#!/bin/bash -e
# Erzeuge Maple-Prozeduren
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
# (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

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


# r2eul: Inert-Maple-Prozedur generieren
# Aus Vorlage der normalen Prozeduren (von Hand erstellt)
for f in `find codeexport/eul*2r_matlab.m`; do
  eulstr=${f:14:3}
  echo "Erstelle Inert-Maple r2eul für $eulstr"
  # Prozedur erstellen
  qd="proc_r2eul${eulstr}"
  zd="proc_r2eul${eulstr}_inert"
  echo "# Berechnung der (intrinsischen) $eulstr-Euler-Winkel aus der Rotationsmatrix" > $zd
  echo "# Nutze Inert-Form, damit die Arctan-Ausdrücke nicht sofort ausgewertet werden." >> $zd
  echo "# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-12" >> $zd
  echo "# (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover" >> $zd
  cat $qd >> $zd
  sed -i "s/arctan/%arctan/g" $zd
  sed -i "s/r2eul${eulstr}/r2eul${eulstr}_inert/g" $zd
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


# eul_diff_rotmat: Maple-Prozedur generieren
for f in `find codeexport/eul*_diff_rotmat_maple`; do
  eulstr=${f:14:3}
  echo "Erstelle Maple eul_diff_rotmat für $eulstr"
  # Prozedur erstellen
  zd="proc_eul${eulstr}_diff_rotmat"
  echo "eul${eulstr}_diff_rotmat := proc (R)" > $zd
  echo "#Berechnung der Ableitung der (intrinsischen) $eulstr-Euler-Winkeln nach der Rotationsmatrix (gestapelt; 9x1)" >> $zd
  echo "#Eingabe: Rotationsmatrix 3x3" >> $zd
  echo "local GradMat2, r11, r12, r13, r21, r22, r23, r31, r32, r33:" >> $zd
  echo "r11:=R(1,1):r12:=R(1,2):r13:=R(1,3):r21:=R(2,1):r22:=R(2,2):r23:=R(2,3):r31:=R(3,1):r32:=R(3,2):r33:=R(3,3):" >> $zd
  cat $f >> $zd
  echo "return GradMat2:" >> $zd
  echo "end proc:" >> $zd
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


# rotmat_diff_eul: Maple-Prozedur generieren
for f in `find codeexport/rotmat_diff_eul*_maple`; do
  eulstr=${f:26:3}
  echo "Erstelle Maple rotmat_diff_eul für $eulstr"
  # Prozedur erstellen
  zd="proc_rotmat_diff_eul${eulstr}"
  echo "rotmat_diff_eul${eulstr} := proc (phi)" > $zd
  echo "#Berechnung der Ableitung der Rotationsmatrix (gestapelt; 9x1) nach den (intrinsischen) $eulstr-Euler-Winkeln" >> $zd
  echo "local r_dphi, phi1, phi2, phi3:" >> $zd
  echo "phi1:=phi(1): phi2:=phi(2): phi3:=phi(3):" >> $zd
  cat $f >> $zd
  echo "return r_dphi:" >> $zd
  echo "end proc:" >> $zd
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


# euljac: Maple-Prozedur generieren
for f in `find codeexport/eul*jac_maple`; do
  eulstr=${f:14:3}
  echo "Erstelle Maple euljac für $eulstr"
  # Prozedur erstellen
  zd="proc_eul${eulstr}jac"
  echo "eul${eulstr}jac := proc (phi)" > $zd
  echo "# Jacobi-Matrix für ${eulstr}-Euler-Winkel" >> $zd
  echo "local J, phi1_s, phi2_s, phi3_s:" >> $zd
  echo "phi1_s:=phi(1): phi2_s:=phi(2): phi3_s:=phi(3):" >> $zd
  cat $f >> $zd
  echo "return J:" >> $zd
  echo "end proc:" >> $zd
done

# euljac: Matlab-Funktionen generieren
# Die Funktionen sollten aus dem Maple-Code generiert werden, damit dieser direkt geprüft werden kann
for f in `find codeexport/eul*jac_matlab.m`; do
  eulstr=${f:14:3}
  echo "Erstelle Matlab euljac für $eulstr"
  # Matlab-Funktion erstellen
  zd="../eul${eulstr}jac.m"
  echo "% Jacobi-Matrix für ${eulstr}-Euler-Winkel" > $zd
  echo "% Zur Umrechnung zwischen Euler-Winkel-Ableitung und Winkelgeschwindigkeit" >> $zd
  echo "% Konvention: R = rot${eulstr:0:1}(phi1) * rot${eulstr:1:1}(phi2) * rot${eulstr:2:1}(phi3)." >> $zd
  echo "% (mitgedrehte Euler-Winkel; intrinsisch)" >> $zd
  echo "%"  >> $zd
  echo "% Eingabe:"  >> $zd
  echo "% phi [3x1]:"  >> $zd
  echo "%   ${eulstr}-Euler-Winkel"  >> $zd
  echo "%"  >> $zd
  echo "% Ausgabe:"  >> $zd
  echo "% J [3x3]:"  >> $zd
  echo "%   Euler-Jacobimatrix: Linearer Zusammenhang zwischen"  >> $zd
  echo "%   Winkelgeschwindigkeit und Euler-Winkel-Zeitableitung"  >> $zd
  echo "%   w = J * phiD"  >> $zd
  echo "%"  >> $zd
  echo "% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10" >> $zd
  echo "% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover" >> $zd
  echo "%"  >> $zd
  echo "function J = eul${eulstr}jac(phi)" >> $zd
  echo "%% Init" >> $zd
  echo "%#codegen" >> $zd
  echo "%\$cgargs {zeros(3,1)}" >> $zd
  echo "assert(isreal(phi) && all(size(phi) == [3 1]), 'eul${eulstr}jac: phi has to be [3x1] (double)');" >> $zd
  echo "phi1_s=phi(1); phi2_s=phi(2); phi3_s=phi(3);" >> $zd
  echo "%% Berechnung" >> $zd
  echo "% aus $f (euler_angle_calculations.mw)" >> $zd
  cat $f >> $zd
  varname_tmp=`grep "=" $f | tail -1 | sed 's/\([a-zA-Z0-9_]*\).*/\1/'`
  echo "J = $varname_tmp;" >> $zd
done


# euljacD: Maple-Prozedur generieren
for f in `find codeexport/eul*jacD_maple`; do
  eulstr=${f:14:3}
  echo "Erstelle Maple euljacD für $eulstr"
  # Prozedur erstellen
  zd="proc_eul${eulstr}jacD"
  echo "eul${eulstr}jacD := proc (phi, phiD)" > $zd
  echo "# Zeitableitung der Jacobi-Matrix für ${eulstr}-Euler-Winkel" >> $zd
  echo "local JD, phi1_s, phi2_s, phi3_s, phi1D_s, phi2D_s, phi3D_s:" >> $zd
  echo "phi1_s:=phi(1): phi2_s:=phi(2): phi3_s:=phi(3):" >> $zd
  echo "phi1D_s:=phiD(1): phi2D_s:=phiD(2): phi3D_s:=phiD(3):" >> $zd
  cat $f >> $zd
  echo "return JD:" >> $zd
  echo "end proc:" >> $zd
done

# euljacD: Matlab-Funktionen generieren
# Die Funktionen sollten aus dem Maple-Code generiert werden, damit dieser direkt geprüft werden kann
for f in `find codeexport/eul*jacD_matlab.m`; do
  eulstr=${f:14:3}
  echo "Erstelle Matlab euljacD für $eulstr"
  # Matlab-Funktion erstellen
  zd="../eul${eulstr}jacD.m"
  echo "% Zeitableitung der Jacobi-Matrix für ${eulstr}-Euler-Winkel" > $zd
  echo "% Zur Umrechnung zwischen zweiter Euler-Winkel-Zeitableitung und Winkelbeschleunigung" >> $zd
  echo "% Konvention: R = rot${eulstr:0:1}(phi1) * rot${eulstr:1:1}(phi2) * rot${eulstr:2:1}(phi3)." >> $zd
  echo "% (mitgedrehte Euler-Winkel; intrinsisch)" >> $zd
  echo "%"  >> $zd
  echo "% Eingabe:"  >> $zd
  echo "% phi [3x1]:"  >> $zd
  echo "%   ${eulstr}-Euler-Winkel"  >> $zd
  echo "% phiD [3x1]:"  >> $zd
  echo "%   Zeitableitung der ${eulstr}-Euler-Winkel"  >> $zd
  echo "%"  >> $zd
  echo "% Ausgabe:"  >> $zd
  echo "% JD [3x3]:"  >> $zd
  echo "%   Zeitableitung der Euler-Jacobimatrix: Zusammenhang zwischen"  >> $zd
  echo "%   Winkelbeschleunigung und zweiter Euler-Winkel-Zeitableitung"  >> $zd
  echo "%   wD = J * phiDD + JD * phiD"  >> $zd
  echo "%"  >> $zd
  echo "% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-11" >> $zd
  echo "% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover" >> $zd
  echo "%"  >> $zd
  echo "function JD = eul${eulstr}jacD(phi, phiD)" >> $zd
  echo "%% Init" >> $zd
  echo "%#codegen" >> $zd
  echo "%\$cgargs {zeros(3,1),zeros(3,1)}" >> $zd
  echo "assert(isreal(phi) && all(size(phi) == [3 1]), 'eul${eulstr}jacD: phi has to be [3x1] (double)');" >> $zd
  echo "assert(isreal(phiD) && all(size(phiD) == [3 1]), 'eul${eulstr}jacD: phiD has to be [3x1] (double)');" >> $zd
  echo "phi1_s=phi(1); phi2_s=phi(2); phi3_s=phi(3);" >> $zd
  echo "phi1D_s=phiD(1); phi2D_s=phiD(2); phi3D_s=phiD(3);" >> $zd
  echo "%% Berechnung" >> $zd
  echo "% aus $f (euler_angle_calculations.mw)" >> $zd
  cat $f >> $zd
  varname_tmp=`grep "=" $f | tail -1 | sed 's/\([a-zA-Z0-9_]*\).*/\1/'`
  echo "JD = $varname_tmp;" >> $zd
done
