# EAR-LAB-MRPT-ICP-SLAM

*Evaluation of MRPT ICP-SLAM using 2D LiDAR data from the OpenLORIS-Scene Corridor dataset on Windows.*

Masterlaborprojekt im Modul **Einsatz Autonomer Robotersysteme Lab (EAR-LAB)**, FH Technikum Wien.
Teilprojekt: Evaluierung von **MRPT ICP-SLAM** mit 2D-LiDAR-Scans der OpenLORIS-Scene-Sequenz **corridor1-3**,
nativ unter **Windows 11, ohne ROS, Ubuntu oder WSL**.

Dieser Ordner `MRPT_ICP_SLAM/` ist das Teilprojekt von Ali Bagheri im gemeinsamen Gruppen-Repository
[`re24m010/EAR_Lab`](https://github.com/re24m010/EAR_Lab), Branch `Ali`.

> **Hinweis zu den Daten:** Der OpenLORIS-Scene-Datensatz steht unter **CC BY-ND 4.0**. Dieses öffentliche Repository
> enthält deshalb **keine** Rohdaten und **keine** daraus abgeleiteten Datensätze (extrahierte Scans, Posen, Rawlogs).
> Sie werden lokal mit den Skripten aus der Originaldatei erzeugt. Siehe [Datensatz-Lizenz](#datensatz-lizenz-und-veröffentlichte-inhalte).

---

## Inhalt

1. [Projektbeschreibung](#projektbeschreibung)
2. [Ziele und Aufgabenstellung](#ziele-und-aufgabenstellung)
3. [Methode: MRPT ICP-SLAM](#methode-mrpt-icp-slam)
4. [Datensatz: OpenLORIS-Scene corridor1-3](#datensatz-openloris-scene-corridor1-3)
5. [Datensatz-Lizenz und veröffentlichte Inhalte](#datensatz-lizenz-und-veröffentlichte-inhalte)
6. [Windows-Entwicklungsumgebung ohne ROS](#windows-entwicklungsumgebung-ohne-ros)
7. [Projektstatus](#projektstatus)
8. [Installation](#installation)
9. [Ausführung](#ausführung)
10. [Projektstruktur](#projektstruktur)
11. [Wichtige Befunde der Datenprüfung](#wichtige-befunde-der-datenprüfung)
12. [Reproduzierbarkeit](#reproduzierbarkeit)
13. [Bekannte Einschränkungen](#bekannte-einschränkungen)
14. [Wissenschaftliche Quellen](#wissenschaftliche-quellen)

---

## Projektbeschreibung

Eine vierköpfige Gruppe vergleicht mehrere SLAM-Verfahren auf einer gemeinsamen Sequenz des OpenLORIS-Scene-Datensatzes.
Dieses Repository enthält das Teilprojekt **2D-LiDAR-SLAM mit dem ICP-basierten Kartierungsverfahren der
Mobile Robot Programming Toolkit (MRPT)**. Die anderen Gruppenmitglieder untersuchen RGB-D-Verfahren (Open3D, RTAB-Map).

Die Implementierung liest die ROS-Bag-Datei ohne ROS-Laufzeitumgebung, prüft die Sensordaten systematisch,
bereitet sie für MRPT auf und wird die geschätzte Trajektorie und Karte gegen die Ground Truth evaluieren.

## Ziele und Aufgabenstellung

Aufgabenstellung laut Lehrveranstaltung:

1. Mehrere SLAM-Implementierungen untersuchen und evaluieren (hier: MRPT ICP-SLAM).
2. Einen gemeinsamen Datensatz verwenden (OpenLORIS-Scene, voraussichtlich `corridor1-3`).
3. Trajektorien und Karten vergleichen.
4. Eine quantitative und qualitative wissenschaftliche Analyse durchführen.
5. Methodische Entscheidungen begründen.
6. Einen dreiseitigen wissenschaftlichen Bericht nach IMRAD-Struktur verfassen.

Geplante Metriken für dieses Teilprojekt:

| Metrik | Zweck |
|---|---|
| Absolute Pose Error (APE/ATE), RMSE | globale Genauigkeit der Trajektorie |
| Relative Pose Error (RPE) | lokale Drift, unabhängig von akkumulierten Fehlern |
| Laufzeit und Speicherbedarf | Rechenaufwand pro Scan |
| Tracking-Abdeckung, Ausfälle | Robustheit |
| Kartenqualität | geometrische Konsistenz der 2D-Belegungskarte |

Zuerst werden die dokumentierten MRPT-Standardparameter verwendet, danach werden ausgewählte ICP-Parameter systematisch variiert.

## Methode: MRPT ICP-SLAM

**Theorie.** Der *Iterative Closest Point*-Algorithmus (Besl & McKay, 1992) schätzt die Starrkörpertransformation zwischen
zwei Punktmengen. Er wechselt dabei zwischen der Zuordnung nächster Nachbarn und der Lösung des Ausrichtungsproblems in geschlossener Form.
Für 2D-Laserscans mobiler Roboter wurde das Verfahren u. a. von Martínez et al. (2006) untersucht.

**MRPT-Implementierung.** Die Anwendung `icp-slam` (MRPT 2.14.12) baut inkrementell eine metrische Karte auf
(`mrpt::slam::CMetricMapBuilderICP`). Jeder neue Scan wird per ICP gegen die bisher aufgebaute Karte ausgerichtet;
die Odometrie dient als Startschätzung. Scans werden in die Karte eingefügt, sobald sich der Roboter um mehr als
konfigurierbare Schwellen bewegt hat. Eine explizite Schleifenschluss- bzw. globale Graphoptimierung ist in
`icp-slam` nicht enthalten. Die konkreten Parameter werden aus der offiziellen Beispielkonfiguration
`icp-slam_demo_classic.ini` übernommen und hier dokumentiert, sobald sie eingesetzt werden.

**Eigene Ergebnisse.** Es liegen bisher **noch keine ICP-SLAM-Ergebnisse** vor (siehe [Projektstatus](#projektstatus)).

## Datensatz: OpenLORIS-Scene corridor1-3

Quelle: Shi et al. (2020) [1], Download über https://huggingface.co/datasets/shixuesong/openloris-scene
(Archiv `corridor1-3_5-rosbag.tar`). Die Rohdaten sind **nicht** Teil dieses Repositorys.

Aus der Bag-Datei gemessene Eigenschaften (Details: [documentation/data_preparation.md](documentation/data_preparation.md)):

| Eigenschaft | Wert |
|---|---|
| Dauer | 70,0 s, 81 641 Nachrichten, 2,82 GB |
| 2D-LiDAR `/scan` | Hokuyo UTM-30LX, 40 Hz, **720 Strahlen, −90° bis +89,75°, 0,25° (≈ 180°)**, max. 60 m |
| Radodometrie `/odom` | 20 Hz, `base_odom → base_link`, ohne Kovarianzen |
| Ground Truth `/gt` | 40 Hz, `gt_map → base_link`, laserbasiert (Variante von hector_mapping laut Paper) |
| Extrinsik `/tf_static` | `base_link → laser`: (0,144; −0,096; 0,998) m, Gierwinkel 0,11° |
| Trajektorie | ca. 71–75 m, Wende mit ca. 10 m Rückfahrt, keine geschlossene Schleife |

Das Paper nennt für den Laser 270° und 1080 Messwerte; in der Bag ist der Scan auf die vorderen 180° beschnitten.

## Datensatz-Lizenz und veröffentlichte Inhalte

Die OpenLORIS-Scene-Datensätze werden laut Datensatzseite unter der Lizenz
[**Creative Commons Attribution-NoDerivatives 4.0 International (CC BY-ND 4.0)**](https://creativecommons.org/licenses/by-nd/4.0/)
bereitgestellt. Die Nutzung ist erlaubt, **die Weitergabe abgeleiteter Datensätze jedoch nicht** (Anfragen dazu laut
Datensatzseite an die Autoren). Die Autoren bitten, das zugehörige Paper zu zitieren [1].

Für dieses **öffentliche** Repository gilt deshalb:

| Inhalt | Im Repository? | Begründung |
|---|---|---|
| Originaldatei `corridor1-3.bag` | nein | Rohdaten; Bezug über die Datensatzseite |
| `results/data/` (extrahierte Scans, GT-/Odometrie-Posen, TUM-Dateien, Extrinsik) | nein | abgeleiteter Datensatz |
| `results/checks/revisit.npz` (GT-Trajektorie, Abstandsreihen) | nein | abgeleiteter Datensatz |
| künftige Rawlogs/CARMEN-Logs | nein | abgeleiteter Datensatz |
| Abbildungen `plots/` | vorerst nein | werden lokal mit `src/plot_data.py` erzeugt |
| `results/checks/data_checks.json` / `.md` | ja | aggregierte Kennzahlen (z. B. Raten, Gültigkeitsanteile, 13 Ausreißer-Ereignisse); aus ihnen lassen sich weder Scans noch Trajektorien rekonstruieren |
| Code, Tests, Konfiguration, Dokumentation | ja | eigene Arbeit; zitiert einzelne Kennwerte des Datensatzes (z. B. Sensor-Extrinsik) zur Beschreibung |

Die `.gitignore` in diesem Ordner setzt das technisch um: Unter `results/` sind nur die beiden geprüften Statistikberichte
zugelassen (Positivliste); `*.npz`, `*.rawlog`, `*.log`, `dataset/` und `plots/` sind ausgeschlossen.

## Windows-Entwicklungsumgebung ohne ROS

| Komponente | Lösung |
|---|---|
| ROS-Bag lesen | [`rosbags`](https://gitlab.com/ternaris/rosbags) (reines Python), streamend, read-only |
| Windows-Inkompatibilität von rosbags 0.11.6 | isolierter Laufzeit-Workaround in `src/rosbags_compat.py` (`PosixPath` → `PurePosixPath`), mit Selbsttest |
| SLAM | offizieller MRPT-Windows-Build 2.14.12 (lokal unter `tools/MRPT`, nicht versioniert) |
| Rawlog-Erzeugung | geplant: CARMEN-Textdatei aus Python → offizieller RawLogViewer-Import (`carmen2rawlog.exe` fehlt im Windows-Build) |
| Auswertung | Python (numpy, pandas, matplotlib) |

Die Abweichung von der in der Lehrveranstaltung genannten ROS-Umgebung ist mit dem Lektor abzustimmen.

## Projektstatus

| Phase | Inhalt | Status |
|---|---|---|
| A | Bestandsaufnahme, Machbarkeit | abgeschlossen |
| C.1–C.3 | Datenextraktion, Datenprüfung, Visualisierung | **implementiert und getestet** (17 Tests) |
| B | MRPT-Installer geprüft (2.14.12, SHA-256 verifiziert), Installation vorbereitet | Installation ausstehend |
| C.4 | Rawlog-Konvertierung über RawLogViewer-CARMEN-Import | untersucht, noch nicht implementiert |
| C.5–C.7 | ICP-SLAM-Testlauf (200 Scans), vollständige Sequenz | geplant |
| D | ATE/RPE, Laufzeit, Kartenqualität, Parameterstudie | geplant |

## Installation

**Voraussetzungen:** Windows 11, Python 3.12, Git.

```powershell
git clone --branch Ali https://github.com/re24m010/EAR_Lab.git
cd EAR_Lab\MRPT_ICP_SLAM
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

**Datensatz:** `corridor1-3_5-rosbag.tar` von der Datensatzseite laden und `corridor1-3.bag` nach
`MRPT_ICP_SLAM\dataset\` entpacken (der Ordner ist durch `.gitignore` ausgeschlossen).
Integritätsprüfung:

```powershell
Get-FileHash dataset\corridor1-3.bag -Algorithm SHA256
# erwartet: 1D20391F7A21D2B63C399FFA4D0CD11FE2665696891E988561F9C73C1D5732C5
```

**MRPT (für die SLAM-Phasen):** Installer `mrpt-branch-develop.exe` (MRPT 2.14.12) aus
https://github.com/MRPT/mrpt/releases/tag/Windows-nightly-builds. Prüfsumme, Installationsoptionen (kein PATH, keine
Verknüpfungen, ohne Quellcode-Komponenten) und Zielordner `tools\MRPT`: siehe [documentation/mrpt_setup.md](documentation/mrpt_setup.md).

## Ausführung

Aus dem Ordner `MRPT_ICP_SLAM` (PowerShell). Die Schritte erzeugen `results/data/` und `plots/` lokal; beide werden
wegen der Datensatzlizenz nicht versioniert.

```powershell
.\.venv\Scripts\python.exe src\extract_data.py      # Bag -> results/data (ca. 1,5 min, ein Streaming-Durchlauf)
.\.venv\Scripts\python.exe src\check_data.py        # Prüfungen -> results/checks/data_checks.md
.\.venv\Scripts\python.exe src\plot_data.py         # Abbildungen -> plots/data_preparation/
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

`extract_data.py --skip-hash` überspringt die SHA-256-Berechnung der Bag. Alle Parameter stehen in `config/dataset.json`.
Ohne Bag-Datei laufen 13 der 17 Tests; die 4 Bag-Integrationstests werden dann übersprungen.

## Projektstruktur

```
EAR_Lab/                        gemeinsames Gruppen-Repository
└── MRPT_ICP_SLAM/              dieses Teilprojekt
├── config/
│   └── dataset.json           Pfade, Topics, Frames, Prüfparameter, GT-Frame-Interpretation
├── src/
│   ├── rosbags_compat.py      isolierter Windows-Workaround für rosbags 0.11.6
│   ├── bag_io.py              streamender, read-only ROS1-Bag-Reader
│   ├── transforms.py          Quaternionen, SE(2), Interpolation, Umeyama-2D
│   ├── dataset.py             Lader für results/data, statische TF, Scan-Geometrie
│   ├── project_config.py      Pfade und Konfiguration
│   ├── extract_data.py        Bag -> results/data
│   ├── check_data.py          Zeit-, Frame-, Quaternion- und Plausibilitätsprüfungen
│   └── plot_data.py           Abbildungen
├── tests/                     unittest (Standardbibliothek)
├── results/
│   └── checks/                aggregierte Prüfberichte (data_checks.json, data_checks.md)
├── documentation/
│   ├── data_preparation.md    Datenaufbereitung und -prüfung
│   └── mrpt_setup.md          Installer-Prüfung, Installationsanleitung, Konvertierungsweg
├── requirements.txt           exakte Python-Paketversionen
├── .gitignore / .gitattributes
└── README.md

Nur lokal (nicht versioniert): dataset/ (Bag), .venv/, tools/ (MRPT-Installer, -Installation),
results/data/ und results/checks/*.npz (abgeleitete Datensätze, CC BY-ND), plots/ (lokal erzeugte Abbildungen)
```

## Wichtige Befunde der Datenprüfung

Details, Messwerte und die Trennung von Beobachtung und Schlussfolgerung: [documentation/data_preparation.md](documentation/data_preparation.md).

* **Laserscan:** 720 Strahlen, ≈ 180° Sichtfeld (nicht 270°/1080 wie im Paper).
* **GT-Frame:** `/gt` ist als `base_link` deklariert. Zwei unabhängige Prüfungen (Kartenschärfe, Odometrie-Kinematik in Kurven)
  zeigen, dass die Posen dem **Laser-Frame** entsprechen. Einstellbar über `ground_truth.child_frame_interpretation`.
* **GT-Erzeugung:** Jede GT-Pose trägt den Zeitstempel eines Scans; laut Paper wurde die GT mit einer Variante von hector_mapping
  aus denselben Laserdaten erzeugt. Das ist eine methodische Abhängigkeit, die bei der Bewertung zu berücksichtigen ist.
* **Trajektorie:** keine geschlossene Schleife, aber eine Wende mit ca. 10 m Rückfahrt (Revisit bis 0,35 m).
* **GT-Ausreißer:** 13 physikalisch unplausible Sprünge (bis 18 cm in 25 ms), vor allem auf der Rückfahrt.
* **Odometrie-Referenz:** Positions-RMSE 1,90 m (Ausrichtung an der Startpose) bzw. 0,22 m (SE(2)-Least-Squares).

## Reproduzierbarkeit

* Die Originaldaten werden nur gelesen; Größe, Änderungszeit und SHA-256 werden vor und nach der Extraktion geprüft.
* Paketversionen sind fest in `requirements.txt`; die lokal erzeugte `results/data/dataset_info.json` protokolliert
  Software-Versionen und die SHA-256 der Bag.
* Zeitstempel werden als ganzzahlige Nanosekunden gespeichert.
* Ein vollständiger zweiter Durchlauf der Pipeline lieferte bitidentische Ergebnisdateien.
* `results/**` wird ohne Zeilenende-Konvertierung versioniert (`.gitattributes`), damit Prüfsummen auch nach dem Klonen gelten.
* Dieser Stand wurde als bereinigter Snapshot aus dem lokalen Entwicklungs-Repository übernommen (Quell-Commit `b803ba3`),
  ohne dessen Git-Historie, damit keine abgeleiteten Datensätze über die Historie veröffentlicht werden.

## Bekannte Einschränkungen

* rosbags 0.11.6 ist unter Windows nur mit dem Workaround in `src/rosbags_compat.py` lauffähig.
* Die Höhe von `base_link` über dem Boden ist unbekannt; mögliche Bodentreffer der leicht geneigten Scanebene (≈ 2°) sind nicht verifiziert.
* Die GT-Frame-Interpretation ist eine datenbasierte Schlussfolgerung und muss in der Gruppe abgestimmt werden.
* Die GT ist laserbasiert und lokal auf einige Zentimeter (Ausreißer bis ca. 18 cm) unsicher.
* Die Nutzung von Windows ohne ROS muss vom Lektor bestätigt werden.

## Wissenschaftliche Quellen

1. X. Shi, D. Li, P. Zhao, Q. Tian, Y. Tian, Q. Long, C. Zhu, J. Song, F. Qiao, L. Song, Y. Guo, Z. Wang, Y. Zhang, B. Qin,
   W. Yang, F. Wang, R. H. M. Chan, Q. She, „Are We Ready for Service Robots? The OpenLORIS-Scene Datasets for Lifelong SLAM“,
   *IEEE International Conference on Robotics and Automation (ICRA)*, 2020. arXiv:1911.05603.
   Datensatz: https://huggingface.co/datasets/shixuesong/openloris-scene, Lizenz CC BY-ND 4.0.
   (Autoren und Titel laut arXiv-Metadaten; Angabe „ICRA 2020“ laut arXiv-Kommentar der Autoren.)
2. J. L. Martínez, J. González, J. Morales, A. Mandow, A. J. García-Cerezo, „Mobile Robot Motion Estimation by 2D Scan Matching
   with Genetic and Iterative Closest Point Algorithms“, *Journal of Field Robotics*, 23(1), 2006. doi:10.1002/rob.20104.
3. P. J. Besl, N. D. McKay, „A Method for Registration of 3-D Shapes“, *IEEE Transactions on Pattern Analysis and Machine
   Intelligence*, 14(2), 1992. doi:10.1109/34.121791.
4. S. Kohlbrecher, O. von Stryk, J. Meyer, U. Klingauf, „A Flexible and Scalable SLAM System with Full 3D Motion Estimation“,
   *IEEE International Symposium on Safety, Security, and Rescue Robotics (SSRR)*, 2011 (hector_mapping).
5. S. Umeyama, „Least-Squares Estimation of Transformation Parameters Between Two Point Patterns“, *IEEE Transactions on Pattern
   Analysis and Machine Intelligence*, 13(4), 1991 (Trajektorienausrichtung).
6. J. Sturm, N. Engelhard, F. Endres, W. Burgard, D. Cremers, „A Benchmark for the Evaluation of RGB-D SLAM Systems“,
   *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2012 (Definition von ATE/RPE).
7. MRPT – Mobile Robot Programming Toolkit, Version 2.14.12: https://github.com/MRPT/mrpt ;
   Anwendung `icp-slam`: https://docs.mrpt.org/reference/latest/page_app_icp-slam.html

Die bibliografischen Details von [2]–[6] (Autorenreihenfolge, Seitenzahlen) werden vor der Verwendung im Bericht anhand der Originalquellen geprüft.

BibTeX für den Datensatz [1]:

```bibtex
@inproceedings{shi2020openloris,
  title     = {Are We Ready for Service Robots? The {OpenLORIS-Scene} Datasets for Lifelong {SLAM}},
  author    = {Shi, Xuesong and Li, Dongjiang and Zhao, Pengpeng and Tian, Qinbin and Tian, Yuxin and Long, Qiwei
               and Zhu, Chunhao and Song, Jingwei and Qiao, Fei and Song, Le and Guo, Yangquan and Wang, Zhigang
               and Zhang, Yimin and Qin, Baoxing and Yang, Wei and Wang, Fangshi and Chan, Rosa H. M. and She, Qi},
  booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
  year      = {2020},
  eprint    = {1911.05603},
  archivePrefix = {arXiv}
}
```

---

*Projektstatus und Ergebnisse werden fortlaufend aktualisiert. Es liegen noch keine SLAM-Ergebnisse vor.*
