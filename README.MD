# Robotik-Toolbox (Matlab)

Dieses Repo enthält eine Sammlung von Matlab-Funktionen zum Thema Robotik. Darunter fallen die Gebiete Kinematik, Dynamik und (Koordinaten-)Transformationen.
Die Funktionen bieten eine Erweiterung zur Robotics Toolbox von Peter Corke insbesondere hinsichtlich der Kompilierbarkeit der Funktionen und damit der Rechengeschwindigkeit.

Moritz Schappler, schappler@imes.uni-hannover.de, 2018-03  
(C) Institut für mechatronische Systeme, Universität Hannover

## Benutzung

Für die Benutzung des Repos muss die Pfadinitialisierung durchgeführt werden mit dem Skript `robotics_toolbox_path_init.m` aus dem Hauptordner.

### Abhängigkeiten

Einige Funktionen benötigen das Repo `matlab-ext` mit externen Funktionen und Toolboxen. Die Pfad-Initialisierung von `matlab-ext` muss allerdings **vor** der Pfad-Initialisierung dieses Repos durchgeführt werden, da es gleichnamige Funktionen mit unterschiedlichem Inhalt gibt (z.B. `rpy2r`).

### Modultests

Die einzelnen Funktionen des Repos können über die Modultests aus dem Ordner `examples_tests` gestartet werden. Die Funktionsfähigkeit des Gesamt-Repos kann mit dem Gesamt-Testskript `robotics_toolbox_test_repo.m` aus dem Hauptordner geprüft werden.

## Inhalt

Hier erfolgt eine kurze Übersicht der im Repo enthaltenen Funktionen. Die Funktionsweise ist in den jeweiligen Funktionsdateien und Modultests erklärt.

* Transformationen
  * Elementar-Rotationen und -Transformationen: `rotx.m`, `roty.m`, `rotz.m`, `trotx.m`, `troty.m`, `trotz.m`, `transl.m`, `invtr.m`. Die Funktionen entsprechen der Corke-Toolbox
  * Transformationen zur Euler-XYZ-Winkelkonvention (Drehung um mitgedrehte x-, y-, z-Achse): `r2eulxyz.m`, `eulxyzjac.m`, `eulxyzjacD.m`, `eulxyzD2omega.m`, `eulxyzDD2omegaD.m`. Die Konvention wurde in der Corke-Toolbox geändert und ist nicht mehr konsistent. Daher sind die Funktionen mit eulxyz statt mit rpy benannt.
  * Integration von Winkelgeschwindigkeiten: `angvel_int_sl.m`, `angvel_int_rpy.mdl`, `angvel_int_rotmat.mdl`, `angvel_int_quat.mdl`, `angvel_body_int_rotmat.mdl`, `angvel_body_int_quat.mdl`
    * Testmodelle: Im Ordner `examples_tests/rotation_integration`
  * Quaternionen: `r2quat.m`, `r2quat_klumpp.m`, `quat2r.m`
  * Simulink-Bibliothek mit Blöcken für Transformationen: `lib_transformations.mdl`
  * Achse-Winkel-Konvention: `angvec2r.m`
* Kinematik
  * Geometrische Jacobi-Matrix für allgemeine Roboter: `robot_tree_jacobig_m.m`
  * Adjunkt-Matrix zur Umrechnung von Jacobi-Matrizen auf andere Punkte auf dem selben Segment: `adjoint_jacobian.m`, `adjointD_jacobian.m`
  * Simulink-Bibliothek: `lib_rigid_body_kinematics.mdl`
* Dynamik
  * Dynamikfunktionen für allgemeine (serielle) Roboter mit und ohne bewegte Basis: `robot_tree_invdyn_floatb_eulxyz_nnew_vp1_m.m`, `robot_tree_invdyn_floatb_eulxyz_mdh_nnew_vp1.m`
  * Massenmatrix nach dem Composite Rigid Body Algorithm von Featherstone: `inertia_nCRB_vp1_m.m`
  * Berechnungen zur Faktorisierung der Massenmatrix: `fdyn_solve_inertia_LTL.m`, `inertia_factorization_LTL.m`
  * Simulink-Bibliothek zu Starrkörperdynamik: `lib_rigid_body_dynamics.mdl`
* Mechanik-Funktionen für die Robotik
  * Speicherung des symmetrischen Trägheitstensors in einem Vektor: `inertiamatrix2vector.m`, `inertiavector2matrix.m`
  * Umrechnung des Massenträgheitstensors mit dem Steinerschen Verschiebungssatz: `inertia_steiner.m`
  * Umrechnung der Dynamikparameter auf ein anderes Koordinatensystem: `inertial_parameters_convert_par1_par2.m`
* Symbolische Herleitungen zu einigen Funktionen: Ordner `maple_codegen`
* Kontakt-Modelle
  * Hunt-Crossley-Modell (z.B. zur Modellierung von mechanischen Gelenkwinkel-Anschlägen): `joint_limit_torques_Hunt_Crossley.m`, `Hunt_Crossley_contact_model_func.m`
  * LuGre-Modell für tangentiale Reibung: `LuGre_tangential_friction_model_func.m`
  * Simulink-Bibliothek: `lib_contact_model.mdl`
  * Testmodelle: Im Ordner `examples_tests/contact_models`
* Regler
  * Gelenk-Impedanzregler für serielle Roboter (PID-ähnlicher Regler mit im Gelenkraum konstanten Eigenschaften): `lib_joint_impctrl.mdl`
  * Kartesische Regler für Position und Orientierung im Arbeitsraum: `lib_cartesian_controllers.mdl`
    * Testmodelle: Im Ordner `examples_tests/rotation_controllers`

## Erweiterung

Weitere allgemeine Funktionen sollen unter Verwendung des Benennungsschemas und grundsätzlicher Programmierkonventionen ergänzt werden und damit allen Institutsangehörigen zur Verfügung gestellt werden.

