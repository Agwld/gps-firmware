# GPS Desktop

Desktop app that shows everything the STAG 12 GPS node sends on CAN, written in Python with Qt
(PySide6). It is built on the same framework as [Equilibrium](https://github.com/espitt/BMS.Equilibrium),
the BMS app: the same layout, theme, installer and self-update.

| Screen    | Shows                                                                                   |
| --------- | --------------------------------------------------------------------------------------- |
| Dashboard | Fix, position, motion, attitude, lap timing and race time, inertial sensors and node health at a glance |
| Track     | A map or satellite view with the car's path, saved locations, track lines drawn over the map, and the timing gates. Gates are placed by clicking the map |
| Laps      | Race time, the lap in progress, every lap with its sector times, and the node's event log |
| Plots     | Any signals against time                                                                |
| Signals   | Every message and signal in the CAN database with its live value and rate                |
| Node      | Every status flag (including whether gate changes have reached flash), and the magnetometer, flash and NMEA commands |

Messages and signals are read from `tools/GPS.dbc`, the firmware's own CAN database, so a signal
added to the firmware appears on the Signals and Plots screens without a change here.

Data comes from a CAN adapter (through [python-can](https://python-can.readthedocs.io/), with the
adapter maker's driver installed), from a recorded log, or from a built-in simulated node that needs
no hardware. Any session can be recorded to a candump log and replayed later.

## Installing

Download and run the setup program for your system from the
[GPS Desktop (latest)](https://github.com/Agwld/gps-firmware/releases/tag/desktop-latest) release.
It fetches the latest version of the app and installs it for your user account, with no admin
rights needed.

| System  | Setup file                           | Installs to                            | To uninstall                   |
| ------- | ------------------------------------ | -------------------------------------- | ------------------------------ |
| Windows | `GPSDesktopSetup-windows-x64.exe`    | `%LOCALAPPDATA%\Programs\GPS Desktop`  | Settings > Apps                |
| macOS   | `GPSDesktopSetup-macos-arm64.zip`    | `~/Applications/GPS Desktop.app`       | Move to Trash                  |
| Linux   | `GPSDesktopSetup-linux-x64.tar.gz`   | `~/.local/share/GPS Desktop`           | Run `uninstall` in that folder |

The programs are not code signed, so Windows SmartScreen and macOS Gatekeeper will warn the first
time the setup program is run.

## Locations, maps and gates

On the Track screen:

- **Map**: choose None, Map (OpenStreetMap) or Satellite (Esri World Imagery). Tiles are kept on
  disk once viewed, so a track you looked at with internet access still has its map at the track.
  Find a place by name, or paste a latitude and longitude.
- **Save as a location** names the place the map shows. A location keeps the lines drawn over its
  map and a gate layout. Locations are saved per user, and can be exported to a file and imported
  on another laptop. When the node reports a position near a saved location, it opens by itself.
- **Draw a track line**: click points along the track; double-click or press Enter to finish.
- **Place start/finish** and **Place a sector**: click where the line crosses the track, then click
  the direction the car drives through it. The gate is saved to the location's layout and, when
  connected, sent straight to the node (the `GPS_Gate_Set` message, 0x6C0). The buttons marked
  "at the car" work like the steering-wheel lap button.
- **Send this layout to the node** replaces the node's gates with the location's, then reads them
  back from the node's broadcast to confirm. **Save the node's gates to this location** does the
  reverse. The panel shows whether the node matches the layout, and whether the node has written
  its gates to flash.

The node writes gates to its flash as soon as they change. It needs firmware with `GPS_Gate_Set`
support to place gates from the map; older firmware ignores that message, and the read-back will
report that the gates did not arrive.

## Updates

A few seconds after opening, and from **Help > Check for Updates**, the app looks for a newer
release. If there is one, the app asks before doing anything. Choosing **Install now** downloads the
setup program, closes the app, installs the update and reopens the app. If the update fails or is
cancelled, the previous version reopens.

Only an installed copy updates itself. A copy run from source does not.

## Project layout

- `app/gps_desktop/`: the application (`updater.py` and `update_ui.py` handle updates)
  - `gps/`: the CAN database reader, live data store, CAN sources, recorder and the simulated node
  - `ui/`: the main window, start screen and one module per screen in `ui/screens/`
  - `theme.py`: every colour
- `installer/gps_desktop_setup/`: the setup wizard (standard library and `certifi` only, to stay small)
- `packaging/build.py`: builds release files with PyInstaller
- `../.github/workflows/gps-desktop-release.yml`: builds every platform and publishes the release
- `tests/`: installer, update, GPS data and screen tests

The install folder belongs to the installer and is replaced on update and deleted on uninstall.
**The app must keep user data and settings elsewhere**, for example in Qt's `QStandardPaths.AppDataLocation`.

## Development

Requires Python 3.12. Run these from this folder.

```
python -m venv .venv
.venv\Scripts\activate          (Windows)
source .venv/bin/activate       (macOS, Linux)
pip install -r requirements-build.txt

python -m unittest discover -s tests -v

cd app
python -m gps_desktop                 (pick "Simulated GPS node" to try it without hardware)
python -m gps_desktop --simulate      (start already connected to the simulator)
```

`python -m gps_desktop --screenshots FOLDER` connects to the simulator, saves a picture of every
screen and quits, which is a quick way to check a change or a build.

## Publishing a release

Push a tag that starts with `desktop-v`. GitHub Actions tests and builds all three platforms, then
publishes the release.

```
git tag desktop-v0.1.0
git push origin desktop-v0.1.0
```

Firmware releases use plain `v` tags and do not affect the app. Because both kinds of release live
in this repository, GitHub's "latest" release may be a firmware one. The workflow therefore also
keeps a release tagged `desktop-latest`, holding the newest setup programs and `manifest.json`.
The setup program and installed apps read from that tag.
