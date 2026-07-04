# SUFST GPS Node Firmware

Firmware for Southampton University Formula Student Team's STAG 12 **GPS
mainboard** (`pcb/gps-mainboard`, rev 1.0.0): an STM32G4-based node that
turns raw GNSS + IMU data into on-car lap/segment timing, precise event
time-marking, and a CAN telemetry stream for the rest of the car.

## What this node does

1. **Live lap and segment timing**, computed on the car rather than
   post-processed afterwards, so the driver/dash can show it in real
   time. Timing is derived from a fused position/attitude estimate, not
   raw GPS fixes alone, so it stays smooth and accurate between the
   20 Hz GPS updates.
2. **Hardware event time-marking**: a steering-wheel button is wired
   through to the GPS receiver's own timestamp hardware, so "when did
   the driver press this" is answered with nanosecond-class GPS-time
   accuracy rather than "whenever the MCU next polled a GPIO".
3. **A CAN broadcast** of position, attitude, raw IMU data and status, so
   other cars systems (dash, data logger, VCU) can consume it.
4. **NMEA output to the team's MoTeC datalogger**, synthesised by this
   firmware rather than passed through from the GPS module directly,
   because the GPS module's own UART is busy running the higher-rate
   binary protocol this firmware needs for fusion and timing.

## Hardware

| Peripheral | Pins | Connects to |
|---|---|---|
| USART3 + DMA | PB10 TX, PB11 RX | u-blox ZED-F9P UART1 |
| USART2 + DMA | PA2 TX, PA15 RX | RS232/MAX3232 -> MoTeC datalogger |
| USART1 | PB6/PB7 | Debug header |
| I2C2 | PA8 SDA, PA9 SCL | F9P I2C, MCP9800 temp sensor, EEPROM |
| SPI1 + DMA | PB3/4/5, CS PB9 | LSM6DSO32 IMU (+ IIS2MDC mag via sensor-hub) |
| FDCAN1 | PA11 RX, PA12 TX | CAN-S (car's sensor CAN bus, 1 Mbps) |
| TIM3_CH2 capture | PA7 | GPS PPS (timebase discipline) |
| GPIO | PC13/PC14/PA5/... | Lap button, user button, GPS EXTINT tap, status/fault lines |

- **MCU**: STM32G431 (170 MHz Cortex-M4F). The fitted part is confirmed
  as the **CB** (128 KB flash). The full firmware currently links at
  ~58 KB / 62 KB (92%) against the smaller **C8** variant it was
  originally sized for — comfortably inside CB's budget with ~2x
  headroom, tight but real headroom if the team ever wants C8
  interchangeability.
- **GNSS**: u-blox ZED-F9P (SparkFun MicroMod GNSS function board),
  configured for GPS+Galileo only at 20 Hz (25 Hz needs a harsher
  constellation cut than this firmware wants).
- **IMU**: LSM6DSO32 (accel+gyro) on SPI1, with an IIS2MDC magnetometer
  wired behind the LSM6DSO32's sensor-hub I2C master (so the MCU never
  talks to the magnetometer directly).
- **CAN**: classic CAN, 1 Mbps on the car's CAN-S (sensor) bus, 500 kbps
  fallback tolerated. This node owns ID block `0x6B0-0x6BF`.

## Why fusion instead of raw GPS

A raw 20 Hz GPS fix stream is too coarse and too latent (the F9P reports
fixes 20-60 ms after they physically happened) for sub-metre lap-gate
crossing detection at racing speeds. This firmware instead runs a
sensor-fusion pipeline at IMU rate (104 Hz) and treats each GPS fix as a
*correction* to that continuous estimate rather than as the position
itself:

```
IMU (104 Hz)              GPS (20 Hz)
accel + gyro                  NAV-PVT
     |                           |
     v                           v
 Mahony AHRS  ---attitude--->  ENU conversion (geodesy.c)
 (ahrs.c)                        |
     |                           v
     +----------------->  6-state KF predict/correct (kf6.c)
                                  |
                                  v
                        fused position/velocity/attitude
                                  |
                    +-------------+-------------+
                    v             v             v
              lap timing     CAN broadcast   NMEA (MoTeC)
             (laptimer.c)     (can_pack.c)   (nmea_out.c)
```

- **Attitude** comes from a Mahony 9-DOF complementary filter
  ([SUFST/Src/fusion/ahrs.c](SUFST/Src/fusion/ahrs.c)) running on every
  IMU sample: gyro integration corrected by accel (gravity reference) and
  magnetometer (heading reference), with an integral term that estimates
  and cancels gyro bias.
- **Position/velocity** come from a 6-state linear Kalman filter
  ([SUFST/Src/fusion/kf6.c](SUFST/Src/fusion/kf6.c)): state is
  `[pos_e, pos_n, pos_u, vel_e, vel_n, vel_u]` in a local East-North-Up
  frame set up once from the first GPS fix
  ([SUFST/Src/fusion/geodesy.c](SUFST/Src/fusion/geodesy.c)). It predicts
  from IMU accel (rotated into ENU by the AHRS attitude) at 104 Hz and
  corrects from GPS NAV-PVT at 20 Hz. Quaternions were deliberately left
  out of the CAN matrix — Euler angles plus raw IMU data is what every
  downstream consumer (dash, data analysis) actually wants, and a car
  never gets close to gimbal lock.
- **Delayed-state correction**: because a GPS fix describes where the car
  *was* 20-60 ms ago, applying it to the *current* filter state would
  smear the correction across the wrong instant (worth over a metre at
  speed). Instead, `kf6.c` keeps a 16-entry ring of past predict steps;
  a correction rewinds to the history entry matching the fix's true
  epoch, applies it there, then replays every later predict step so the
  final state reflects the fix at the right point in time. The same
  mechanism is designed to take a wheelspeed-derived scalar speed
  correction (`kf6_correct_speed()`) once that CAN signal is wired up, to
  keep velocity aided through GPS dropout (tunnels, tight corners).
- No orientation/velocity bias states: at 50 ms GPS update intervals they
  wouldn't have time to converge to anything useful, so the filter stays
  a plain 6-state linear KF rather than a full EKF.

## Lap timing

A gate is a stored ENU point + heading, treated as a finite-width line
perpendicular to that heading. The fused 104 Hz position stream is
checked every step for a forward crossing (sign change in the
along-heading projection, direction-checked so a crossing the wrong way
doesn't trigger it), with linear interpolation between samples for
~1 ms-class timing resolution. All lap/segment times are expressed in
GPS time-of-week via a PPS-disciplined tick<->iTOW mapping
([SUFST/Src/fusion/timebase.c](SUFST/Src/fusion/timebase.c)), not the
MCU's free-running clock, so times are meaningful across resets and
comparable to external GPS-timestamped data.

The steering wheel's lap button is a short/long/very-long press FSM:
short = new segment gate, long (>1 s) = new start/finish line (clears
segments), very long (>5 s) = clear all gates. Gate *sets* persist across
power cycles in the last flash page
([SUFST/Src/persist/flash_store.c](SUFST/Src/persist/flash_store.c)), as
an append-only, CRC-checked log; the newest record for a given gate wins
on restore, and the page is only erased-and-compacted when explicitly
requested (`CAN_CMD_CONFIG_SAVE`) and the car is judged stationary. Gate
*clears* are not yet persisted (a power cycle after clearing without
setting a new gate over it would restore the old one) — a known gap, not
a silent one.

A Formula Student track is ~3 m wide, so gates are deliberately narrower
than that (2 m half-width) rather than the much wider default an earlier
pass of this firmware shipped with — a gate wider than the track itself
risks false-triggering on an adjacent section at a hairpin or chicane.

## Event time-marking

The event button doesn't go to a plain GPIO interrupt — it's wired
(hardware-debounced) into the ZED-F9P's `EXTINT` pin, so the GPS module
itself timestamps the edge in its own oscillator-disciplined time domain
and reports it as a **UBX-TIM-TM2** message (binary UBX, not NMEA — this
was one of the earlier assumptions this project corrected). The firmware
dedupes on TM2's `count` field and forwards it as a `Lap_Event` CAN frame
carrying the GPS-time timestamp.

## CAN matrix (CAN-S, base `0x6B0`)

Little-endian Intel signal packing, matching the team's existing DBC
convention. See [tools/GPS.dbc](tools/GPS.dbc) for the authoritative
definition; summary:

| ID | Message | Rate | Contents |
|---|---|---|---|
| 0x6B0 | GPS_Position | 20 Hz | fused lat/lon |
| 0x6B1 | GPS_Velocity | 20 Hz | speed, course, altitude, fix status |
| 0x6B2 | GPS_Attitude | 50 Hz | yaw/pitch/roll, fusion status |
| 0x6B3 | Lap_Status | 10 Hz | lap number, running time, sector, flags |
| 0x6B4 | Lap_Event | event | button/TM2 event with GPS-time timestamp |
| 0x6B5 | GPS_Quality | 5 Hz | accuracy estimates, RTK/fix flags |
| 0x6B6 | GPS_IMU_Accel | 100 Hz | raw calibrated accel |
| 0x6B7 | GPS_IMU_Gyro | 100 Hz | raw calibrated gyro |
| 0x6B8 | GPS_Temp | 1 Hz | board/IMU/MCU temperatures |
| 0x6B9 | GPS_Status | 1 Hz | uptime, fault bits, CPU load |
| 0x6BA | GPS_Mag | 10 Hz | magnetometer + calibration status |
| 0x6BF | GPS_Command | RX | gate set/clear, mag-cal, config commands |

`0x251` (`Wheel_Speeds`, sent by the VCU) is consumed as an RX aiding
input — see the fusion section above. Byte layout is transcribed from
memory of the `sufst/can-defs` DBC and should be spot-checked against it
(or a bench candump) before trusting it; the fallback if it's wrong is
just a bad speed aiding value, not a crash, since it only ever nudges the
KF's velocity estimate.

## Magnetometer calibration

Hard-iron (constant bias) and soft-iron (per-axis gain imbalance)
distortion are calibrated via `CAN_CMD_MAG_CAL_START`/`STOP`
([SUFST/Src/imu/mag_cal.c](SUFST/Src/imu/mag_cal.c)): while a pass is
running, `imu_task` feeds every raw magnetometer sample into a min/max
accumulator per axis (so the car needs to actually be rotated through as
much of a full sphere of orientations as practical); on stop, the
per-axis midpoint becomes the bias and the per-axis range is normalised
against the mean range to become the scale. This is the "basic"
axis-aligned calibration, not a full 9-parameter ellipsoid/skew fit — it
won't correct a distortion ellipsoid that's rotated relative to the
sensor's own axes, only a constant offset plus unequal axis gain. Good
enough as a first pass; a full ellipsoid fit is a natural upgrade if
bench data shows it's needed. The result is written to flash
immediately, and applied to every reading before it reaches the AHRS.

## Task architecture (target build)

FreeRTOS, fully static allocation (no heap, no dynamic task/queue
creation), mirroring the rest of the team's firmware fleet:

| Task | Role |
|---|---|
| `imu_task` | SPI read @104 Hz -> AHRS -> KF predict -> apply pending GPS/wheelspeed corrections -> lap-gate check -> mag calibration |
| `gps_task` | Parses the UBX stream from the F9P; publishes NAV-PVT and TIM-TM2, runs the boot configuration state machine |
| `can_task` | Staggered periodic CAN broadcast + command/event queues, dispatches RX commands to the owning task |
| `aux_task` | Synthesises NMEA for the MoTeC datalogger, forwards RTCM correction data to the GPS |
| `sys_task` | Buttons, LEDs, temperature sensor, flash-compact trigger, watchdog |

`imu_task` owns gates/laptimer/kf6/ahrs/mag-cal state because it's the
only task with fused position and raw sensor samples each cycle; other
tasks reach it only through queues (`app.h`), never shared memory,
except two mutex-guarded exceptions: the CAN broadcast state
(`canbc.c`) and USART3 TX (shared between `gps_task`'s boot config and
`aux_task`'s RTCM forwarding).

## Repository layout

```
Core/, Drivers/, Middlewares/     CubeMX-style HAL/FreeRTOS glue and vendor code
SUFST/Inc/, SUFST/Src/            hand-written application code, by subsystem:
  gps/        UBX protocol parsing
  fusion/     AHRS, Kalman filter, geodesy, timebase
  laptimer/   gate storage/crossing detection, lap state machine
  canbus/     CAN message definitions, pack/unpack, broadcast task
  imu/        LSM6DSO32 + sensor-hub driver, fusion task, mag calibration
  nmea/       NMEA sentence synthesis + RTCM forwarding for the MoTeC link
  persist/    flash-backed gate/mag-cal persistence
  sys/        app-level task/queue wiring, event flags, buttons/LEDs/IWDG
  board/      compile-time rates/scalings/mounting configuration
tools/GPS.dbc                     CAN database for this node's messages
tests/                            host-side unit tests (see below)
cmake/, CMakeLists.txt            build configuration
```

## Building

Target firmware (requires the `arm-none-eabi` toolchain):

```sh
cmake --preset Debug      # or Release
cmake --build --preset Debug
```

Host unit tests (native compiler, no target toolchain needed) — most
fusion/protocol/timing logic is written to be testable this way before
it ever touches hardware:

```sh
cmake -B build-host -DGPS_HOST_TESTS=ON -G Ninja
cmake --build build-host
ctest --test-dir build-host
```

Each `tests/test_<module>.c` builds against the real
`SUFST/Src/<...>.c` it exercises (declared in a matching
`tests/test_<module>.deps` file) — no mocking framework, no HAL
dependency for anything host-testable.

## Status

**The full firmware builds and links** against the real STM32G431 target
(`cmake --preset Release && cmake --build --preset Release`) — every
module described above exists and is wired together, not just the
fusion/protocol core. 11/11 host tests pass (`ctest`).

That said, **none of it has been run on real hardware yet.** Compiling
and linking proves internal consistency, not correctness against actual
silicon/sensors/bus traffic — treat everything below as "should work,
needs a bench to confirm," not "done":

- **Confirmed correct by host tests** (synthetic data, no hardware):
  UBX parsing, CAN pack/unpack, geodesy (incl. the ENU inverse used to
  publish fused position), timebase (incl. the iTOW->tick inverse used
  for delayed-state corrections), AHRS, the 6-state KF with
  delayed-state rewind, lap gates/timing, NMEA synthesis, magnetometer
  calibration, and flash record persistence.
- **Written but only compile-checked against the target toolchain**
  (i.e. plausible C, never executed): the LSM6DSO32 driver and
  sensor-hub magnetometer bring-up, the ZED-F9P boot configuration
  (UBX-CFG-VALSET), and all five FreeRTOS tasks (`imu_task`, `gps_task`,
  `can_task`, `aux_task`, `sys_task`) and their wiring in `app.c`.
- **Explicitly flagged as needing datasheet/bench verification**, since
  they were written from memory rather than the source documents: the
  LSM6DSO32 register map and sensor-hub bring-up sequence
  ([lsm6dso32.h](SUFST/Inc/imu/lsm6dso32.h)), the u-blox CFG-\* key IDs
  used for boot configuration ([gps_config.h](SUFST/Inc/gps/gps_config.h)
  — mitigated by verifying via VALGET readback rather than trusting the
  VALSET ACK, which the design already does regardless of who wrote it),
  the MCP9800 register/resolution assumption, and the `Wheel_Speeds`
  (0x251) byte layout.
- **Known gaps, not silent ones**: gate *clears* aren't persisted to
  flash (only sets); CPU load reporting in `GPS_Status` is a placeholder
  zero (`configGENERATE_RUN_TIME_STATS` isn't wired up); the IMU SPI path
  is blocking HAL calls rather than the DMA pipeline originally planned;
  no host-side SIL (software-in-the-loop) harness exists yet to replay a
  synthetic or recorded track through the full fusion+timing pipeline
  and check lap times against ground truth — the per-module unit tests
  above cover each piece in isolation, not the integrated behaviour.

Along the way, fixed two pre-existing bugs unrelated to any of the above:
`tests/CMakeLists.txt` wasn't linking `libm` (broke 3 of the original 7
host tests), and this toolchain's prebuilt `libm.a` ships a genuinely
broken `__errno` (a packaging defect, not a project bug) worked around
with a local stub in
[Core/Src/syscalls_errno.c](Core/Src/syscalls_errno.c).
