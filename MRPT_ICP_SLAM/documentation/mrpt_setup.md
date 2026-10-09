# MRPT unter Windows: Installationsprüfung und Konvertierungsweg (Phase 3)

Stand: 09.10.2026. Status: **Installation vorbereitet, noch nicht ausgeführt.** Es gibt noch keine Konvertierung und keinen ICP-SLAM-Lauf.

## 1. Installer: Version, Herkunft, Integrität

| Prüfung | Ergebnis |
|---|---|
| Datei | `tools/_download/mrpt-branch-develop.exe`, 129 121 434 Bytes |
| Quelle | https://github.com/MRPT/mrpt/releases/tag/Windows-nightly-builds, hochgeladen von `jlblancoc` (MRPT-Hauptentwickler) |
| SHA-256 | `c01759449335267ef21f43b574da1290f3accea78cfa83dae52ae5bfb8715b4e`; stimmt mit dem GitHub-Asset-Digest überein und wurde vor der Installation erneut geprüft |
| Build | `2.14.12-develop-build5130`, Zeitstempel 2025-08-31, Commit `e0669f41abf9841861eacbb587e468527eb82ea4` (GitHub: *verified*) |
| Version | `version_prefix.txt` = 2.14.12; Release-Tag `2.14.12` = Merge genau dieses Commits → **MRPT 2.14.12** |
| Digitale Signatur | **keine** (Authenticode: NotSigned) |
| Typ | NSIS (CMake/CPack), Manifest `requireAdministrator` → UAC-Abfrage nötig |
| Mark-of-the-Web | nicht vorhanden (Download per curl) |

Der Inhalt wurde **ohne Ausführung** ermittelt: Der NSIS-Header-Block wurde mit Pythons `lzma` (LZMA1, raw) dekomprimiert,
und die Zeichenketten wurden ausgewertet. Das Analyseartefakt liegt in `tools/_download/mrpt-branch-develop.exe.nsis_header.bin`.

**Enthalten:** 33 Programme, u. a. `icp-slam.exe`, `rawlog-edit.exe`, `RawLogViewer.exe`. Dazu alle MRPT-, wxWidgets-3.1.3- und
OpenCV-4.11-DLLs, eine eigene VC++-Runtime in `bin\` und die Beispielkonfigurationen
`share\mrpt\config_files\icp-slam\*.ini` (u. a. `icp-slam_demo_classic.ini`).

**Nicht enthalten:** `carmen2rawlog.exe` (nur Quellcode) und Python-Bindings (`pymrpt`, nur Dokumentation).

## 2. Systemänderungen durch die Installation

| Änderung | Details |
|---|---|
| Rechte | Administrator (UAC), Herausgeber „Unbekannt“ |
| Dateien | nur im gewählten Zielordner |
| Registry | `...\CurrentVersion\Uninstall\mrpt-2.14.12` (+ `\Components\*`), `Software\Jose Luis Blanco Claraco\mrpt-2.14.12` |
| PATH | optional (all users / current user), **gewählt: nicht hinzufügen** |
| Startmenü | optional, **gewählt: „Do not create shortcuts“** |
| Dienste, Treiber, Sicherheitseinstellungen | keine |
| Deinstallation | `tools\MRPT\Uninstall.exe` |

Vorzustand gesichert in `tools/install_logs/pre_install_state.json`: keine MRPT-Einträge in PATH (Machine/User), Registry oder Startmenü;
Zielordner existierte nicht.

## 3. Installationsanleitung (manuell, durch den Nutzer)

1. `tools\_download\mrpt-branch-develop.exe` starten → UAC mit *Ja* bestätigen.
   **Falls SmartScreen blockiert: abbrechen, nicht umgehen.**
2. Welcome → *Next*; License (BSD-3) → *I Agree*.
3. Install Options: **„Do not add mrpt to the system PATH“**, kein Desktop-Icon.
4. Install Location: `D:\Meine Studium\Semester_3\EAR_LAB\tools\MRPT`.
5. Start Menu Folder: **„Do not create shortcuts“**.
6. Components: **Apps, Libraries, Unspecified** behalten; **App_sources, Library_sources** abwählen.
7. *Install* → *Finish*.

**Nach der Installation geplant:** `icp-slam.exe --help`, `rawlog-edit.exe --help`, Start von RawLogViewer;
rekursive DLL-Abhängigkeitsprüfung (Python-PE-Parser, nur Standardbibliothek); Vergleich von PATH, Registry und Startmenü gegen den Vorzustand.

## 4. Konvertierung: offizielle Import-Möglichkeiten

| Programm | Import | Bewertung |
|---|---|---|
| `carmen2rawlog.exe` | CARMEN → Rawlog | nicht im Windows-Installer |
| `rawlog-edit.exe` | keine (nur Export, Filter, Info) | ungeeignet |
| `RawLogViewer.exe` | File → Import → „a CARMEN log…“ (außerdem MOOS-alog, RTL, Bremen-DLR) | **geeignet, mit Gegenmaßnahmen** |
| `gps2rawlog`, `ros-map-yaml2mrpt` | GPS bzw. Karten | ungeeignet |

### RawLogViewer-CARMEN-Import (Quellcode 2.14.12, `apps/RawLogViewer/main_imports_exports.cpp`, `OnImportCARMEN`)

| Verhalten im Code | Problem für corridor1-3 | Gegenmaßnahme (in der CARMEN-Datei) |
|---|---|---|
| nur `FLASER`/`RLASER`; `aperture = π` fest | 720 Strahlen umfassen 179,75° → Winkelskalierung 0,14 % falsch (bis 0,25° am Rand) | **721 Strahlen** schreiben, Strahl 721 (+90°) ungültig → Abstand π/720 = 0,25° exakt |
| `sensorPose = (0,0,0)` fest | Extrinsik nicht angebbar | Odometrie in den **Laser-Frame** transformieren: `odom_laser = odom_base ⊕ T_base_laser` (2D exakt) → ICP schätzt Laserposen, vergleichbar mit `/gt` (Interpretation `laser`) |
| Zeitstempel als `float` gelesen (`mrpt::Clock::fromDouble`) | bei Unix-Zeit ≈ 1,56·10⁹ s nur **128 s Auflösung** | **relative Zeit** t − t₀ schreiben (≈ 8 µs Auflösung bei 70 s); t₀ in der Zuordnungstabelle |
| gültig nur, wenn r < max. Reichweite (Dialog, Standard 30) | `+inf` ist in CARMEN nicht darstellbar | ungültige Strahlen als Wert ≥ max. Reichweite; im Dialog **60** eingeben |
| erstes Odometrie-Tripel nach den Distanzen = Odometrie; letztes Token = Zeitstempel | – | beide Posen-Tripel = Laser-Frame-Odometrie; beide Zeitstempel = relative Zeit |
| erzeugt Actions/SensoryFrames (`CActionRobotMovement2D`, Standard-Rauschmodell) | – | von `icp-slam` unterstützt; Rauschmodell dokumentieren |
| nur GUI | manueller Schritt | Klickfolge dokumentieren; SHA-256 jeder erzeugten Datei festhalten |

**Empfehlung:** Primärer Weg ist der RawLogViewer-Import. Python erzeugt nur die CARMEN-Textdatei; das Binärformat schreibt MRPT selbst.
Ein eigener Python-Rawlog-Writer bleibt die Rückfallebene (Format im Quellcode 2.14.12 geprüft: `CArchive::WriteObject`,
`CObservation2DRangeScan` v7, `CObservationOdometry` v2, `CPose3D` v3, `CPose2D` v1). **Noch nicht freigegeben.**

### Geplante Validierung (zuerst Mini-Datensatz mit 20 Scans, dann 200)

| Prüfgegenstand | Werkzeug | Kriterium |
|---|---|---|
| Format, Anzahl, Typen | `rawlog-edit --info` | 20 Actions + 20 SensoryFrames mit je 1 Scan |
| Reihenfolge, Zeit | `rawlog-edit --list-timestamps` | monoton, = t − t₀ (< 10 µs) |
| 721 Strahlen, Gültigkeit | `rawlog-edit --export-2d-scans-txt` | Distanzen float32-genau, Ungültig-Markierungen identisch, Strahl 721 ungültig |
| Odometrie, Zuordnung | RawLogViewer „Generate odometry and laser text files…“ bzw. `rawlog-edit`-Export | Inkremente = Laser-Frame-Odometrie-Inkremente (< 1e-6) |
| Geometrie | eigener Vergleich | MRPT-Scanpunkte gegen eigene Punkte im Laser-Frame (< 1 mm) |

Danach: 200 Scans (0–5 s; kein GT-Sprung, aber ein doppelter Odometrie-Schritt bei 4,52 s) und der erste `icp-slam`-Lauf mit der
offiziellen `icp-slam_demo_classic.ini` (nur Pfade angepasst, Kopie in `config/`).

## 5. Offene Entscheidungen

1. Installation durch den Nutzer (Anleitung Abschnitt 3), danach die Verifikation.
2. Freigabe des RawLogViewer-Wegs als primärer Konvertierungsweg.
3. GT-Frame-Interpretation (`laser`) in der Gruppe abstimmen (siehe `data_preparation.md` 6.2).

## Quellen

* Windows-Release: https://github.com/MRPT/mrpt/releases/tag/Windows-nightly-builds
* Quellcode 2.14.12: https://github.com/MRPT/mrpt/tree/2.14.12 (`apps/RawLogViewer/main_imports_exports.cpp`,
  `libs/serialization/src/CArchive.cpp`, `libs/obs/src/CObservation2DRangeScan.cpp`, `libs/obs/src/CObservationOdometry.cpp`,
  `libs/poses/src/CPose3D.cpp`, `libs/poses/src/CPose2D.cpp`, `libs/obs/src/carmen_log_tools.cpp`)
* icp-slam: https://docs.mrpt.org/reference/latest/page_app_icp-slam.html
