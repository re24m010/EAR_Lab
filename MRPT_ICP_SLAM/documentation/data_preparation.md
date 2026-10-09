# Datenaufbereitung corridor1-3 (Phase C.2)

Stand: 09.10.2026. Alle Zahlen stammen aus `results/checks/data_checks.json` bzw.
`results/data/*.json` und wurden mit den Skripten in `src/` aus der Originaldatei berechnet.
In diesem Dokument werden **Beobachtungen** (direkt gemessen) und **Schlussfolgerungen**
(Interpretation, ggf. mit Literaturbezug) ausdrücklich getrennt.

> **Hinweis (öffentliches Gruppen-Repository):** Wegen der Datensatzlizenz CC BY-ND 4.0 sind `results/data/*`,
> `results/checks/revisit.npz` und die in Abschnitt 10 genannten Abbildungen (`plots/`) hier **nicht enthalten**. Sie werden
> lokal mit `src/extract_data.py`, `src/check_data.py` und `src/plot_data.py` aus der Originaldatei erzeugt.
> Enthalten ist der aggregierte Prüfbericht `results/checks/data_checks.json` / `.md`.

## 1. Eingangsdaten und Provenienz

| Eigenschaft | Wert |
|---|---|
| Datei | `dataset/corridor1-3.bag` (ROS1-Bag, unverändert) |
| Größe | 2 821 252 951 Bytes |
| SHA-256 | `1d20391f7a21d2b63c399ffa4d0cd11fe2665696891e988561f9c73c1d5732c5` |
| Dauer | 69,999 s, 81 641 Nachrichten |
| Software | Python 3.12.0, rosbags 0.11.6, numpy 2.5.3, pandas 3.0.6, matplotlib 3.11.2, Windows 11 |

Die Bag wird nur lesend geöffnet; Größe und Änderungszeitpunkt werden vor und nach der
Extraktion verglichen (`dataset_info.json: bag_unchanged = true`).

## 2. Verarbeitungskette

```
corridor1-3.bag ──extract_data.py──► results/data/   (scans.npz, odom.csv, gt.csv, tf_static.json, *_tum.txt)
                                         │
                                  check_data.py ──► results/checks/ (data_checks.json/.md, revisit.npz)
                                         │          results/data/odom_aligned.csv
                                  plot_data.py ───► plots/data_preparation/*.png
```

* **Streaming:** `bag_io.BagReader` liest die Bag chunkweise über `rosbags.highlevel.AnyReader`
  und deserialisiert nur `/scan`, `/odom`, `/gt`, `/tf_static`. Kamerabilder werden nie
  deserialisiert. Laufzeit des einen Lesedurchlaufs: ca. 80 s.
* **Nachrichtendefinitionen** werden aus der Bag selbst übernommen (der Standard-Typestore kennt
  `tf2_msgs` nicht).
* **Zeitstempel** werden als ganzzahlige Nanosekunden (`int64`) gespeichert – kein
  Rundungsverlust. TUM-Dateien schreiben die Zeit exakt als Dezimalstring.

### Windows-Workaround für rosbags (isoliert in `src/rosbags_compat.py`)

| | |
|---|---|
| Fehler | `NotImplementedError: cannot instantiate 'PosixPath' on your system` beim Öffnen der Bag |
| Ursache | `rosbags.typesys.msg.normalize_msgtype` nutzt `pathlib.PosixPath` zur String-Zerlegung von Typnamen |
| Lösung | Laufzeit-Ersatz von `PosixPath` durch `PurePosixPath` (gleiche `/`-Semantik, kein Dateisystemzugriff) |
| Absicherung | nur aktiv, wenn `PosixPath` nicht instanziierbar ist; Warnung bei ungetesteter rosbags-Version; Selbsttest nach dem Patch; Unit-Test |
| Bibliothek verändert? | Nein |

## 3. Laserscan `/scan`

**Beobachtungen**

* 2802 Scans, `frame_id = laser`, **720 Messwerte pro Scan**, Header-Felder über alle Scans konstant.
* `angle_min = −90,00°`, `angle_max = +89,75°`, `angle_increment = 0,25°`
  → **179,75° vom ersten zum letzten Strahl (≈ 180° Sichtfeld)**.
* `scan_time = 25 ms` (40 Hz), `time_increment = 17,36 µs` → 1440 Schritte pro Umdrehung → 0,25°/Schritt.
  Die 720 Strahlen werden in 12,48 ms aufgenommen.
* `range_min = 0,023 m`, `range_max = 60 m`; gültige Distanzen 0,277–38,70 m, Median 1,85 m, 99 %-Quantil 16,6 m.
* 93,0 % der Messungen gültig, 7,0 % `+inf`, keine NaN-Werte, keine Werte außerhalb `[range_min, range_max]`.
  Keine Intensitäten aufgezeichnet.
* Gültige Strahlen pro Scan: Minimum 65,3 %, Median 96,8 %; 327 Scans unter 80 %.
  Am häufigsten ungültig sind Strahlen um +9° (nur ≈ 61 % gültig), also nahezu geradeaus.

**Schlussfolgerungen**

* Das Paper (Shi et al. 2020, Tab. I) nennt für den Hokuyo UTM-30LX 270° und 1080 Messwerte.
  Winkelauflösung (0,25°) und Zeitmodell (1440 Schritte/Umdrehung) passen exakt zu diesem Sensor.
  **In der Bag wurde der Scan jedoch auf die vorderen 180° (720 Strahlen) beschnitten.**
  Für Bericht und MRPT-Konfiguration gelten 720 Strahlen / 180°.
* Die Lücken geradeaus entstehen plausibel durch Korridorenden jenseits der wirksamen Reichweite
  oder durch Glas/Fenster (Hypothese, nicht verifiziert).
* Bewegungsverzerrung pro Scan maximal 1,7 cm bzw. 0,65° (aus Odometrie-Maximalwerten). Das ist gering;
  eine Deskew-Korrektur ist vorerst nicht nötig und wird als Vereinfachung dokumentiert.
* Für MRPT relevant: Der Strahlenfächer ist nicht exakt symmetrisch (Mitte bei −0,125°).

## 4. Koordinatensysteme und Extrinsik

**Beobachtungen**

* TF-Baum (`/tf_static`): `base_link → {laser, d400_color, t265_fisheye1, marker}`,
  `d400_color → {d400_depth, d400_imu}`, `t265_fisheye1 → {t265_fisheye2, t265_imu}`. Es gibt keinen `/tf`-Topic.
* `base_link → laser`: t = (0,1435; −0,0964; 0,9980) m, RPY = (0,91°; 2,10°; 0,11°).
  Die Rotation ist orthonormal, der Laser ist aufrecht montiert (z-Achse in base_link: (0,037; −0,016; 0,999)).
* Die Scanebene ist leicht geneigt: Strahlelevation zwischen −2,29° (bei Strahlwinkel −23,5°) und +0,90°.
* Alle Quaternionen (odom, gt, tf_static) haben die Norm 1 (Abweichung < 1e-6).
* Odometrie: z, Roll und Pitch sind exakt 0, also eine rein planare Odometrie.
* GT: |Roll|, |Pitch| ≤ 0,30°, z zwischen −0,44 und −0,20 m.

**Schlussfolgerungen**

* Die 2D-Projektion (x, y, Gierwinkel) ist für Odometrie und GT gerechtfertigt.
* Läge der base_link-Ursprung auf Bodenhöhe, würde der tiefste Strahl den Boden nach ca. 25 m treffen.
  Die Höhe von base_link über dem Boden ist **nicht bekannt**. Bodentreffer bleiben daher eine Hypothese.
  Schwache, diffuse Punkte in großer Entfernung (Karte links unten) sind damit oder mit Glas vereinbar.

## 5. Odometrie `/odom`

**Beobachtungen**

* 1400 Nachrichten, 20,0 Hz (Δt 44–56 ms), Frames `base_odom → base_link`.
* Pose- und Twist-Kovarianzen sind durchgehend 0.
* Der Twist ist konsistent mit der differenzierten Pose (Korrelation v_x: 0,975, ω_z: 0,9993); v_y = 0.
  Die laterale Geschwindigkeit aus der Pose ist ≈ 0 (σ = 0,5 mm/s).
* Kein Odometrie-Zeitstempel fällt auf einen Scan-Zeitstempel; die Abstände sind zwischen −12,5 und +12,5 ms verteilt.
* 76 Schritte ohne Bewegung, keiner davon bei v_x > 0,3 m/s. 10 Schritte sind etwa doppelt so groß wie der Median.

**Schlussfolgerungen**

* Der Twist ist im base_link-Frame angegeben; das Modell ist ein Differentialantrieb (keine Querbewegung).
* Für die Konvertierung nach MRPT muss die Odometrie auf die Scanzeitpunkte **interpoliert** werden (SE(2), Winkel mit Unwrap).
* Da Kovarianzen fehlen, müssen Unsicherheitsparameter für MRPT selbst gewählt und dokumentiert werden.

## 6. Ground Truth `/gt`

### 6.1 Zeitliche Zuordnung

**Beobachtungen**

* 2797 Posen, `gt_map → base_link`.
* **Alle 2797 GT-Zeitstempel sind nanosekundengenau identisch mit Scan-Zeitstempeln.** Für 5 Scans fehlt eine GT
  (Indizes 120, 298, 828, 1603, 2490); genau dort liegen die 5 Lücken von 50 ms im GT-Strom.
* Bei allen Topics ist die Bag-Zeit exakt gleich dem Header-Stempel.
* Kreuzkorrelation der Gierraten von Odometrie und GT: Maximum bei τ = 6 ms (r = 0,9987; bei τ = 0: r = 0,9987).
  Die Kurve ist zwischen etwa −20 und +10 ms praktisch flach.

**Schlussfolgerungen (zur Frage: nur Synchronisation oder scanbasierte GT?)**

1. Identische Zeitstempel beweisen für sich genommen nur, dass jede GT-Pose **einem Scan zugeordnet** ist,
   also mit dessen Stempel versehen wurde. Aus den Stempeln allein folgt **nicht**, dass die Pose aus dem Scan *berechnet* wurde.
   Auch eine externe Quelle, die auf die Scanzeitpunkte interpoliert wurde, ergäbe dasselbe Bild.
2. Dass die Bag-Zeit bei allen Topics exakt dem Stempel entspricht, spricht dafür, dass die Bag nachträglich
   (offline) zusammengestellt wurde. Das passt zur Beschreibung im Paper: Die Software-Synchronisation zwischen den Geräten
   erfolgte pro Sequenz per Zeitversatz-Optimierung.
3. Die **Berechnung aus den Laserscans** belegt erst das Paper (Abschnitt Ground-truth): Für Szenen ohne
   Motion Capture wurde eine Variante von *hector_mapping* verwendet; zuerst wird eine Karte gebaut, dann wird
   der Roboter „with each frame of laser scan“ darin lokalisiert. Der Stempelbefund ist **damit konsistent**
   (eine GT-Pose pro Scan, fehlende Posen = Scans ohne Lokalisierungsergebnis), aber nicht der Beweis.
4. Zwischen Odometrie und GT ist kein relevanter Zeitversatz messbar (|τ| ≲ 10 ms, Auflösung durch die flache
   Korrelationskurve begrenzt).

**Bedeutung für die Evaluation:** Die GT stammt aus demselben Sensor wie unser ICP-SLAM und wurde mit einem
ähnlichen Verfahren (Scan-Matching gegen eine Karte) erzeugt. Fehler, die beide Verfahren teilen
(z. B. Mehrdeutigkeiten im langen Korridor), werden von der GT nicht aufgedeckt. LiDAR-Verfahren werden dadurch
gegenüber den RGB-D-Verfahren der Gruppe potenziell begünstigt. Laut Paper beträgt die Abweichung der laserbasierten
GT zur Motion-Capture-GT im Bürobereich der Corridor-Daten ca. 3 cm ATE; diese Größenordnung ist die untere Grenze
sinnvoll interpretierbarer Fehler.

### 6.2 Welchen Frame beschreibt `/gt`?

**Beobachtungen**

* Werden die Scans mit den GT-Posen registriert, ist die Karte am schärfsten (geringste Anzahl belegter 5-cm-Zellen),
  wenn **keine** Laser-Extrinsik angewendet wird:

  | Hypothese | belegte Zellen (Stride 2) |
  |---|---|
  | A: /gt = base_link, Extrinsik aus tf_static | 17 542 |
  | B: /gt = Laser-Frame (keine Extrinsik) | **13 399** |
  | C: Extrinsik, Strahlen gespiegelt | 108 625 |
  | D: Extrinsik, Gierwinkel + 180° | 134 430 |

* Gittersuche über den Laser-Offset (x, y ∈ [−0,3; 0,3] m in 5-cm-Schritten, Gierwinkel ∈ {−1; −0,5; 0; 0,11; 0,5; 1}°):
  Optimum bei **(0; 0; 0°)** mit 10 967 Zellen; bei der tf_static-Extrinsik sind es 14 893 Zellen (Stride 4).
* Unabhängiger Test mit der Radodometrie (beschreibt sicher base_link): RMSE der relativen Translation über 1 s

  | | gerade Abschnitte | Kurven (> 20°) |
  |---|---|---|
  | /gt als base_link | 0,057 m | 0,130 m |
  | /gt als Laser-Frame | 0,058 m | **0,084 m** |

  (2 s: 0,104 / 0,191 m gegenüber 0,105 / 0,124 m)

**Schlussfolgerungen**

* Beide Tests sind voneinander unabhängig (Kartenschärfe aus Laserdaten bzw. Kinematik aus Radodometrie) und zeigen übereinstimmend:
  **Die `/gt`-Posen beschreiben de facto die Pose des Laser-Frames, obwohl sie als `base_link` deklariert sind.**
  In geraden Abschnitten sind die Hypothesen gleich gut; in Kurven unterscheiden sie sich deutlich. Genau das erwartet man
  von einem Hebelarm (Laser 17 cm vom Drehpunkt).
* Plausible Ursache: hector_mapping schätzt die Pose des Laser-Frames. Diese wurde vermutlich ohne Rücktransformation
  als base_link veröffentlicht (Vermutung).
* Die Spiegel- und 180°-Hypothesen sind klar widerlegt. Die Strahlwinkel-Konvention (positiv = links, REP-103) ist damit bestätigt.
* **Konsequenz:** `config/dataset.json → ground_truth.child_frame_interpretation = "laser"`.
  Die base_link-GT wird berechnet als `gt ⊕ T_base_laser⁻¹`. Für die spätere ATE/RPE-Auswertung schlage ich vor,
  zusätzlich direkt die **Laserposen** zu vergleichen, weil das von der Extrinsik unabhängig ist.
  Beide Interpretationen bleiben über die Konfiguration umschaltbar. **Diese Entscheidung sollte mit der Gruppe abgestimmt werden**,
  da sie alle Verfahren betrifft.

### 6.3 Plausibilität der GT

**Beobachtung:** 13 aufeinanderfolgende GT-Schritte implizieren Geschwindigkeiten > 3 m/s (bis 7,2 m/s, Sprünge bis 18 cm in 25 ms).
Die Odometrie liegt maximal bei 1,35 m/s (Twist). Die Sprünge häufen sich bei 39,5 s, 58,8–59,2 s, 62,9–63,7 s und 67,6 s,
also überwiegend auf dem Rückweg nach der Wende.

**Schlussfolgerung:** Das sind Artefakte der GT-Erzeugung (kurzzeitige Lokalisierungsfehler), keine realen Bewegungen.
Die GT ist lokal auf einige cm bis ca. 18 cm unsicher. Bei der Evaluation sollten wir diese Stellen ausweisen und die
Robustheit der Metriken prüfen (z. B. RPE mit/ohne diese Zeitfenster).

## 7. Trajektorie, Wende und Revisits

**Beobachtungen**

* GT-Weglänge 74,5 m bei voller Rate bzw. 71,3 m bei 0,5-s-Abtastung (die volle Rate wird durch Rauschen aufgebläht).
* Start und Ende liegen 42,3 m auseinander; die Kursänderung beträgt netto **+174,8°**.
* Der Roboter fährt den Korridor nach Süden, **wendet bei y ≈ −58 m und fährt ca. 10 m zurück**.
* Revisit-Kriterium: zeitlicher Abstand ≥ 15 s **und** Abstand entlang des Pfades ≥ 10 m.
  Kleinster Revisit-Abstand: **0,35 m** (t = 44,6 s gegenüber 67,2 s, Kursdifferenz 179°).
  Anteil der Posen mit Revisit innerhalb 0,5 / 1 / 2 / 5 m: 17,9 / 19,6 / 22,2 / 30,2 %.
* Laser-Wiederbeobachtung (0,25-m-Raster): 13,0 % der belegten Zellen werden nach ≥ 10 s ohne Treffer erneut getroffen.

**Schlussfolgerungen**

* **Korrektur gegenüber der ersten Bestandsaufnahme:** Dort wurde aus dem Start-Ende-Abstand „keine Schleife“ gefolgert.
  Das war zu stark. Die Trajektorie ist zwar **keine geschlossene Schleife**, enthält aber einen **Hin-und-Rück-Abschnitt**:
  Ein ca. 10 m langer Korridorabschnitt wird in Gegenrichtung erneut befahren und erneut beobachtet.
* „Loop Closure“ ist eine Eigenschaft eines Algorithmus, nicht der Daten. Die Daten bieten **eine Revisit-Gelegenheit**
  (gleicher Ort, entgegengesetzte Blickrichtung). Ob ein Verfahren sie nutzt, ist getrennt zu prüfen.
  MRPT `icp-slam` hat keine explizite Loop-Closure-Komponente. Es gleicht aber jeden Scan gegen die akkumulierte Karte ab
  und kann bereits kartierte Wände nach der Wende daher implizit wiederverwenden. Das betrifft das spätere Experiment und ist noch nicht getestet.
* Die Rückfahrt fällt mit den GT-Sprüngen zusammen (Abschnitt 6.3). Für den Bericht ist das eine wichtige Einschränkung.

## 8. Radodometrie gegenüber GT (Charakterisierung, kein SLAM-Ergebnis)

Die Odometrie wurde auf die GT-Zeitstempel interpoliert und mit der base_link-GT (Interpretation `laser`) verglichen.

| Ausrichtung | Transformation (x, y, Gierwinkel) | Positions-RMSE | max. Abweichung | Endabweichung |
|---|---|---|---|---|
| Startpose | (−1,41 m; −21,57 m; 23,72°) | 1,90 m | 2,81 m | 2,19 m |
| SE(2)-Least-Squares (Umeyama ohne Skalierung) | (−0,67 m; −23,82 m; 26,77°) | 0,22 m | 0,45 m | 0,04 m |

Das Verhältnis der Weglängen Odometrie/GT (0,5-s-Abtastung) beträgt 0,991.

Bei der Startpose-Ausrichtung bleibt der Gierwinkelfehler klein (Mittel 0,68°, Ende 1,26°), die Positionsabweichung
wächst aber auf bis zu 2,81 m. Im Plot liegt die Odometrie nach der ersten Kurve **parallel versetzt** zur GT.

**Schlussfolgerung:** Die Positionsabweichung baut sich vor allem in den Kurvenabschnitten auf (Hinweis auf Fehler im
Kurvenradius bzw. in der Spurbreite des Differentialantriebs); ein reiner Kursfehler würde sich als Verdrehung zeigen.
Die genaue Ursache ist nicht weiter untersucht. Die Radodometrie ist als Bewegungs-Prior für ICP brauchbar, als
alleinige Lokalisierung nicht. Die Werte dienen später als **Referenz (Baseline)**, gegen die ICP-SLAM mit derselben
Ausrichtungsmethode verglichen wird.

## 9. Konsequenzen für die MRPT-Konvertierung (nächster Schritt, noch nicht umgesetzt)

1. CARMEN `ROBOTLASER1`: 720 Strahlen, Öffnungswinkel 179,75°, max. Reichweite 60 m (oder bewusst niedriger, dokumentiert).
2. Laserpose in base_link: (0,1435; −0,0964) m, Gierwinkel 0,11° − 0,125° (Asymmetrie-Korrektur, weil der Parser symmetrische Scans annimmt).
3. Odometrie auf die Scanzeitpunkte interpolieren; Kovarianzen selbst festlegen.
4. Eine Zuordnungstabelle Index → Zeitstempel ns mitschreiben (MRPT-Ausgaben enthalten keine Zeitstempel).
5. Evaluation gegen GT mit dokumentierter Frame-Interpretation (Abschnitt 6.2) und Kennzeichnung der GT-Sprünge (6.3).

## 10. Abbildungen (`plots/data_preparation/`)

| Datei | Inhalt |
|---|---|
| `scan_examples.png` | vier Scans im Laser-Frame (Start, schlechtester Scan, Mitte, nach der Wende) |
| `scan_range_profile.png` | Entfernungsprofil eines Scans; gültiger Anteil je Strahlwinkel |
| `scan_quality.png` | gültige Strahlen über die Zeit; Distanzhistogramm |
| `timestamp_intervals.png` | Δt-Verteilungen von /scan, /odom, /gt |
| `trajectory_gt_vs_odometry.png` | GT und ausgerichtete Radodometrie; Abweichung über die Zeit |
| `gt_registered_scans_map.png` | Konsistenzkarte aus GT-registrierten Scans (kein SLAM) |
| `gt_frame_hypotheses_zoom.png` | Vergleich der Frame-Hypothesen A/B im Wendebereich |
| `revisit_analysis.png` | wiederbefahrene Abschnitte und Revisit-Abstand über die Zeit |
| `odom_gt_time_lag.png` | Kreuzkorrelation der Gierraten Odometrie ↔ GT |
