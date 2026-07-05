# STAG 12 GPS Node — Live Lap Timing System

**Real-time lap and segment timing for Formula Student, broadcast live to the in-car dash.**

The SUFST STAG 12 GPS node is a **standalone sensor module** that transforms raw GNSS, IMU, and magnetometer data into precision-engineered lap/segment times, on the car, as you drive. Gates are persistent and reproducible across power cycles, broadcast live to the dashboard, and accurate to sub-millisecond resolution via GPS-time synchronization.

| Feature | Spec |
|---------|------|
| **Timing accuracy** | ~1 ms (fused 104 Hz state with GPS-time stamping) |
| **Lap detection** | Automatic gate crossing, steering-wheel programmable (short/long/very-long press) |
| **Gate persistence** | Stored as absolute lat/lon, reproducible after power-cycle-and-move |
| **Broadcast** | 20 Hz position/velocity, 50 Hz attitude, 100 Hz IMU, ~5 Hz gates, CAN 1 Mbps |
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

→ See [**DRIVER_GUIDE.md**](DRIVER_GUIDE.md) for how to set/clear gates and use the timing system.

## For engineers

- **[QUICK_START.md](QUICK_START.md)** — Get the firmware building in 5 minutes
- **[DEVELOPER.md](DEVELOPER.md)** — Detailed development environment, build system, testing, debugging
- **[ARCHITECTURE.md](ARCHITECTURE.md)** — Technical deep dive: sensor fusion, lap timing, CAN protocol, persistence
- **[DATASHEET.md](DATASHEET.md)** — Component specs, pinouts, electrical, CAN matrix, firmware metrics

## Hardware

The GPS mainboard (`pcb/gps-mainboard`) is a 2-layer, 65 mm × 85 mm board that:
- Talks to the car via 1× CAN-S (1 Mbps sensor bus), 1× RS232 (MoTeC datalogger), 1× steering-wheel button
- Synchronises to GPS time via a 1 PPS input from the ZED-F9P
- Runs 104 Hz IMU samples through a real-time fusion pipeline
- Persists gate configuration in the STM32's internal flash

See [**DATASHEET.md**](DATASHEET.md) for schematics, layout, pinouts, and electrical specs.

## Status

**Production-ready firmware.** Every module builds and links correctly; all unit tests pass. The system has been validated in simulation (realistic car dynamics, synthetic tracks) and field-tested for timing accuracy, lap detection, and gate persistence. Ready for track deployment in STAG 12.

## Building

### Firmware (target hardware)

Requires `arm-none-eabi` toolchain (GCC 13.x+ recommended):

```bash
cmake --preset Release
cmake --build --preset Release
# Output: build/gps_firmware.elf, ~58 KB
```

Debug build (with `-O0`):
```bash
cmake --preset Debug
cmake --build --preset Debug
# Output: ~94 KB (still fits in 128 KB CB variant)
```

### Unit tests (host machine)

Runs on native compiler (no ARM toolchain needed):

```bash
cmake -B build-host -DGPS_HOST_TESTS=ON -G Ninja
cmake --build build-host
ctest --test-dir build-host
# 11/11 tests: UBX parsing, CAN pack/unpack, fusion, timing, persistence
```

### Dashboard (reference UI)

```bash
cd tools/dashboard
pip install -e .
gps-dashboard
```

## Contributing

Contributions follow SUFST style: read [DEVELOPER.md](DEVELOPER.md) for naming, testing, and commit conventions before opening a PR.

---

**Questions?** Check [ARCHITECTURE.md](ARCHITECTURE.md) for the big picture, [DEVELOPER.md](DEVELOPER.md) for build/test details, or grep the codebase—comments are sparse by design (signal-to-noise, not mystique).

*Last updated: 2026-07-05. SUFST STAG 12 GPS Node Firmware.*
