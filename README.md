# STAG 12 GPS Node — Live Lap Timing System

**Real-time lap and segment timing for Formula Student, broadcast live to the in-car dash.**

The SUFST STAG 12 GPS node is a **standalone sensor module** that transforms raw GNSS, IMU, and magnetometer data into precision-engineered lap/segment times, on the car, as you drive. Gates are persistent and reproducible across power cycles, broadcast live to the dashboard, and accurate to sub-millisecond resolution via GPS-time synchronization.

| Feature | Spec |
|---------|------|
| **Timing resolution** | ~1 ms (crossings interpolated in the fused 104 Hz state, stamped in GPS time) |
| **Lap detection** | Automatic gate crossing, steering-wheel programmable (short/long/very-long press) |
| **Gate persistence** | Stored as absolute lat/lon, reproducible after power-cycle-and-move |
| **Broadcast** | 20 Hz position/velocity, 50 Hz attitude, 100 Hz IMU, 10 Hz pitot airspeed + wind, ~5 Hz gates, 1 Hz GPS/UTC time, CAN 1 Mbps |
| **GNSS** | u-blox ZED-F9P (GPS + Galileo, 20 Hz) |
| **IMU** | LSM6DSO32 accel+gyro + IIS2MDC magnetometer, 104 Hz |
| **Hardware timing** | Steering-wheel button edge-stamped by GPS receiver (nanosecond precision) |
| **MCU** | STM32G431 (170 MHz Cortex-M4F, 128 KB flash, FreeRTOS) |

## What's included

- **Firmware** — STM32G431 application with sensor fusion (Mahony AHRS + 6-state Kalman filter), lap-gate crossing detection, CAN broadcast, and persistent gate storage
- **Dashboard tool** — Python/PyQt6 desktop reference implementation: live telemetry plot, track map with gates, lap timing display, remote gate control
- **CAN database** — GPS.dbc message definitions for the car's CAN network
- **Tests** — Host-side unit tests for fusion, protocol, timing logic (no hardware needed)

## For drivers

→ See [**DRIVER_GUIDE.md**](docs/DRIVER_GUIDE.md) for how to set/clear gates and use the timing system.

## For engineers

- **[QUICK_START.md](docs/QUICK_START.md)** — Get the firmware building in 5 minutes
- **[DEVELOPER.md](docs/DEVELOPER.md)** — Detailed development environment, build system, testing, debugging
- **[ARCHITECTURE.md](docs/ARCHITECTURE.md)** — Technical deep dive: sensor fusion, lap timing, CAN protocol, persistence
- **[DATASHEET.md](docs/DATASHEET.md)** — Component specs, pinouts, electrical, CAN matrix, firmware metrics

## Hardware

The GPS mainboard ([schematic](gps-mainboard.pdf)) is a 4-layer board that:
- Talks to the car via CAN-S (1 Mbps sensor bus), RS232 (NMEA out to the MoTeC datalogger, RTCM corrections in for RTK) and the steering-wheel lap button
- Synchronises to GPS time via a 1 PPS input from the ZED-F9P
- Runs 104 Hz IMU samples through a real-time fusion pipeline
- Persists gate configuration in the STM32's internal flash

See [**DATASHEET.md**](docs/DATASHEET.md) for pinouts, interfaces and the CAN matrix.

## Status

**v1.0.0 — release candidate, not yet validated on hardware.** The firmware builds warning-free (Release and Debug) and all host unit tests pass, but it has not yet been run on the board: sensor bring-up, timing accuracy and gate persistence still need confirming on the bench and on track. See [CHANGELOG.md](CHANGELOG.md) for what changed and what is still open — and the bring-up checklist in [DEVELOPER.md](docs/DEVELOPER.md#bring-up-checklist) for first power-on.

## Building

### Firmware (target hardware)

Requires `arm-none-eabi` toolchain (GCC 13.x+ recommended):

```bash
cmake --preset Release
cmake --build --preset Release
# Output: build/Release/gps-firmware.{elf,hex,bin}, ~68 KB of 124 KB
```

Debug build (with `-O0`):
```bash
cmake --preset Debug
cmake --build --preset Debug
# Output: build/Debug/, ~114 KB (still fits)
```

### Unit tests (host machine)

Runs on native compiler (no ARM toolchain needed):

```bash
cmake --preset Test
cmake --build --preset Test
ctest --preset Test
# 12/12 tests: UBX parsing, CAN pack/unpack, fusion, air data, timing, persistence
```

### Dashboard (reference UI)

```bash
cd tools/dashboard
uv sync
uv run gps-dashboard
```

## Contributing

Contributions follow SUFST style: read [DEVELOPER.md](docs/DEVELOPER.md) for naming, testing, and commit conventions before opening a PR.

---

**Questions?** Check [ARCHITECTURE.md](docs/ARCHITECTURE.md) for the big picture and [DEVELOPER.md](docs/DEVELOPER.md) for build/test details; the source comments explain the reasoning behind most decisions.

*SUFST STAG 12 GPS Node Firmware v1.0.0.*
