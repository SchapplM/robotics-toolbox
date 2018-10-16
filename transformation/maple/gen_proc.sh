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
  echo "eul${eulstr}2r := proc (phi1, phi2, phi3)" > $zd
  echo "#Berechnung der Rotationsmatrix aus (intrinsischen) $eulstr-Euler-Winkeln" >> $zd
  echo "local RM:" >> $zd
  echo "# RM := rot${eulstr:0:1}(phi1) . rot${eulstr:1:1}(phi2) . rot${eulstr:2:1}(phi3):" >> $zd
  cat $f >> $zd
  echo "return RM:" >> $zd
  echo "end proc:" >> $zd
done

# eul2r: Matlab-Funktionen generieren
for f in `find codeexport/eul*2r_matlab.m`; do
  eulstr=${f:14:3}
  echo "Erstelle Matlab eul2r für $eulstr"
  # Prozedur erstellen
  zd="../eul${eulstr}2r.m"
  echo "% ${eulstr}-Euler-Winkel in eine Rotationsmatrix konvertieren" > $zd
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
  echo "% aus $f" >> $zd
  cat $f >> $zd
  varname_tmp=`grep "=" $f | tail -1 | sed 's/\([a-zA-Z0-9_]*\).*/\1/'`
  echo "R = $varname_tmp;" >> $zd
done

# r2eul: Matlab-Funktionen generieren
# Die Funktionen sollten aus dem Maple-Code generiert werden, damit dieser direkt geprüft werden kann
for f in `find codeexport/r2eul*_matlab.m`; do
  eulstr=${f:16:3}
  echo "Erstelle Matlab r2eul für $eulstr"
  # Prozedur erstellen
  zd="../r2eul${eulstr}.m"
  echo "% Rotationsmatrix nach ${eulstr}-Euler-Winkel konvertieren" > $zd
  echo ""  >> $zd
  echo "% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10" >> $zd
  echo "% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover" >> $zd
  echo ""  >> $zd
  echo "function phi = r2eul${eulstr}(R)" >> $zd
  echo "%% Init" >> $zd
  echo "%#codegen" >> $zd
  echo "%\$cgargs {zeros(3,3)}" >> $zd
  echo "assert(isreal(R) && all(size(R) == [3 3]), 'r2eul${eulstr}: R has to be [3x3] (double)');" >> $zd
  echo "r11=R(1,1);r12=R(1,2);r13=R(1,3);" >> $zd
  echo "r21=R(2,1);r22=R(2,2);r23=R(2,3); %#ok<NASGU>" >> $zd
  echo "r31=R(3,1);r32=R(3,2);r33=R(3,3); %#ok<NASGU>" >> $zd
  echo "%% Berechnung" >> $zd
  echo "% Konvention: R = rot${eulstr:0:1}(phi1) * rot${eulstr:1:1}(phi2) * rot${eulstr:2:1}(phi3):" >> $zd
  echo "% aus $f" >> $zd
  cat $f >> $zd
  varname_tmp=`grep "=" $f | tail -1 | sed 's/\([a-zA-Z0-9_]*\).*/\1/'`
  echo "phi = $varname_tmp;" >> $zd
done

