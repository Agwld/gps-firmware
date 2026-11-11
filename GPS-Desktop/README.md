# GPS Desktop

Desktop app that shows everything the STAG 12 GPS node sends on CAN, written in Python with Qt
(PySide6). It is built on the same framework as [Equilibrium](https://github.com/espitt/BMS.Equilibrium),
the BMS app: the same layout, theme, installer and self-update.

| Screen    | Shows                                                                                   |
| --------- | --------------------------------------------------------------------------------------- |
| Dashboard | Fix, position, motion, attitude, lap timing, inertial sensors and node health at a glance |
| Track     | The car's path, its position and the timing gates. Gates are set and cleared here        |
| Laps      | The lap in progress, every lap with its sector times, and the node's event log           |
| Plots     | Any signals against time                                                                |
| Signals   | Every message and signal in the CAN database with its live value and rate                |
| Node      | Every status flag, and the magnetometer, save and NMEA commands                          |

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
