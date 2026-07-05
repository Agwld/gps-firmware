# STAG 12 GPS dashboard

A PyQt6 desktop app that visualises the GPS node's CAN traffic (decoded
against `tools/GPS.dbc`): fix/position/attitude status, rolling
speed/attitude/accel/gyro plots, and a 2D track map.

It can read frames from a live CAN interface, replay a recorded log, or
generate synthetic traffic - all three go through the same decode/UI
pipeline, so what you see in simulate mode is what you'll see on the
bench.

## Setup

Requires [uv](https://docs.astral.sh/uv/) and Python 3.12+ (uv will
fetch 3.12 automatically if it's not already installed).

```sh
cd tools/dashboard
uv sync
```

## Running

```sh
uv run gps-dashboard
```

or, without installing the entry point:

```sh
uv run python -m gps_dashboard
```

Useful flags for scripted/headless verification:

- `--simulate` - start immediately in simulate mode instead of waiting
  for Connect to be clicked.
- `--run-seconds N` - exit automatically after N seconds.
- `--screenshot PATH` - save a PNG of the window before exiting (implies
  `--run-seconds 2` if not given explicitly).

## Data sources

Pick a source in the connection panel, then click **Connect**.

### Live

Reads frames from a real CAN interface via
[python-can](https://python-can.readthedocs.io/). The GPS node's bus
runs at 1 Mbit/s (pre-filled as the default bitrate) - only change it if
you're deliberately testing at a different rate.

- **SocketCAN** (Linux, e.g. a USB-CAN adapter bound to `can0`):
  interface `socketcan`, channel `can0` (or `vcan0` for a virtual bus).
  Bring the interface up first: `sudo ip link set can0 up type can
  bitrate 1000000`.
- **Vector** (XL Driver Library): interface `vector`, channel `0` for
  the first configured channel (set up channels via Vector Hardware
  Config first). Requires the Vector XL Driver Library to be installed
  on the host - Vector hardware isn't supported natively on Linux
  without it.
- **Kvaser** (CANlib): interface `kvaser`, channel `0` for the first
  detected Kvaser channel. Requires Kvaser's CANlib SDK/drivers to be
  installed.

If the bus fails to open (wrong channel, driver not installed, device
unplugged), the error from the underlying library is shown in the
connection panel rather than crashing the app.

### Replay

Plays back a `candump -l`-format log file (the standard `can-utils`
capture format), paced by the log's own recorded timestamps. Set
**Speed** to `0` to play back as fast as possible with no inter-frame
delay, or leave **Loop** checked to repeat once the file ends.

Logs recorded by this app's own **Record to file** option (see below)
are in the same format, so they round-trip directly.

### Simulate

Drives a synthetic car around a circular track, with no hardware
required - useful for developing the dashboard itself or for demos.
**Track radius** and **Lap period** control the circuit; every message
type is generated at its real nominal CAN rate.

## Recording

Check **Record to file** and pick an output path before connecting (in
any of the three modes) to save every raw frame as it arrives, in
`candump -l` format. This works with Live and Simulate too, not just
Replay - handy for capturing a bench/track session to replay later.

## Tests

```sh
uv run pytest
```

Covers the DBC decoder, replay log parsing, the recorder's round-trip
format, and the simulator (including that it never produces a frame the
DBC can't encode).
