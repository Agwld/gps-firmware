# Architecture — How the GPS firmware works

Design and implementation of the STAG 12 GPS node firmware (v1.0.0). For pins,
CAN signals and memory figures see [DATASHEET.md](DATASHEET.md); for the
reasoning behind the bigger choices see [NOTES.md](NOTES.md).

## System overview

The firmware runs on an STM32G431 under FreeRTOS, with every task, queue and
mutex allocated statically at start-up. It does four things:

1. **Acquire** GNSS (u-blox ZED-F9P, 20 Hz) and inertial data (LSM6DSO32 +
   IIS2MDC, 104 Hz).
2. **Fuse** them into a position, velocity and attitude estimate at 104 Hz.
3. **Time laps** by detecting crossings of driver-placed gates in that estimate,
   stamped in GPS time.
4. **Publish** everything on CAN (and a NMEA stream to the MoTeC logger).

```
 LSM6DSO32 ─SPI/DMA─┐                       ┌─► CAN broadcast (can_task)
 IIS2MDC (via hub) ─┤                       │
                    ▼                       │
              imu_task (104 Hz) ────────────┤
   AHRS ─► KF predict ─► KF correct ◄─ fix  │
          ─► gates / lap timer ─► flash     │
                    ▲                       │
 ZED-F9P ─UART/DMA─► gps_task ──NAV-PVT─────┼─► aux_task ─► NMEA ─► MoTeC
         ─PPS──────► TIM3 capture (time)    │
 lap button ───────► sys_task ──gate cmds───┘   (LEDs, temps, watchdog)
```

## Tasks

| Task | Prio | Driven by | Owns |
|------|------|-----------|------|
| `imu_task` | 6 | IMU data-ready interrupt (104 Hz), 20 ms timeout fallback | AHRS, KF, geodesy origin, gates, lap timer, mag calibration, flash store, pitot air data |
| `gps_task` | 5 | USART3 idle-line / DMA events | F9P configuration, UBX parser, PPS pairing, GPS link supervision |
| `can_task` | 3 | 10 ms slots | FDCAN1: broadcast rota, Lap_Event TX, command dispatch, bus-off recovery |
| `aux_task` | 2 | NMEA rate (10 Hz default) | USART3 TX |
| `sys_task` | 1 | 20 ms | Lap button, LEDs, temperatures, watchdog refresh, status frame |

Single ownership is the main concurrency rule: each piece of state is touched
by exactly one task, and the others talk to it through queues. The only shared
data structures are:

- **`canbc` state** — a mutex-guarded struct every task publishes its latest
  values into; `can_task` takes a snapshot each slot and packs frames from it,
  so no producer ever touches FDCAN.
- **The I2C2 mutex** — the bus is shared by `gps_task` (F9P configuration) and
  `sys_task` (MCP9800).
- **The system event group** — ready/fault flags (`SYS_EVT_*`), broadcast as
  `GPS_Status.fault_bits` and used to drive the LEDs.

Queues ([`app.h`](../SUFST/Inc/sys/app.h)): CAN RX → `can_task`; gate,
config-save and mag-cal commands → `imu_task`; latest NAV-PVT → `imu_task` and
`aux_task` (separate length-1 overwrite queues, so neither steals the other's
copy); Lap_Event → `can_task`; NMEA config → `aux_task`; wheel speed and
pitot samples (from the CAN RX interrupt) → `imu_task`.

## Time base

All lap times are in **GPS time-of-week** (iTOW, ms), not MCU time.

- TIM3 counts at 1 MHz; its rollover interrupt extends it to a 32-bit
  microsecond tick ([`timebase.c`](../SUFST/Src/fusion/timebase.c)). The tick
  read and the PPS capture both account for a rollover that is pending but not
  yet counted.
- The F9P's 1 PPS edge is captured on TIM3_CH2. `gps_task` pairs each captured
  edge with the GPS second it marks (the next NAV-PVT's iTOW rounded to the
  second, once the receiver reports time as valid).
- The last two pairs give both an offset and the MCU clock's actual rate, so
  `tick → iTOW` (and back) tracks crystal drift.

Until the first PPS edge is paired the node is not *time locked*: lap timing
is suspended and the KF treats each fix as current rather than back-dating it.

## Sensor fusion

### Attitude — Mahony AHRS ([`ahrs.c`](../SUFST/Src/fusion/ahrs.c))

A complementary filter on the unit quaternion: gyro rates are integrated every
sample; the accelerometer (gravity) corrects roll/pitch and the calibrated
magnetometer corrects yaw, through a proportional term plus an integral term
that doubles as gyro-bias estimation. If the magnetometer stops delivering
samples the filter carries on from gyro + accel alone (yaw then drifts, and the
fusion status shows it).

### Position/velocity — 6-state Kalman filter ([`kf6.c`](../SUFST/Src/fusion/kf6.c))

State `[e, n, u, ve, vn, vu]` in a local East-North-Up frame.

- **Predict** (104 Hz): body acceleration rotated into ENU by the AHRS
  quaternion, gravity removed, integrated into velocity and position.
- **Correct** (each valid GPS fix): position and velocity from NAV-PVT, with
  noise from the receiver's own hAcc/sAcc (floored at 0.5 m and 0.1 m/s).
- **Delayed-state correction**: a fix describes the car 20–60 ms in the past.
  The filter keeps the last 16 predict steps; a fix is applied at the history
  entry matching its GPS time and every later step is replayed.
- **Outlier gate**: a fix whose innovation fails a 99.9 % chi-squared test is
  rejected.
- **Divergence recovery**: if 10 position fixes in a row are rejected, the
  filter is taken to be the one that is wrong (e.g. after dead-reckoning
  through a GPS outage) — it re-opens its covariance and snaps back onto GPS.
  `imu_task` treats that jump as a break in the trajectory for gate crossing.
- **Wheel-speed aiding** (`kf6_correct_speed`, front-left wheel only, along the
  AHRS heading) exists but is compiled out unless `KF_WHEELSPEED_AIDING` is
  set in [`board_config.h`](../SUFST/Inc/board/board_config.h).

If the IMU fails, the loop keeps running on its 20 ms timeout with a
zero-acceleration sample, so the KF becomes a constant-velocity model between
fixes: position, speed and lap timing continue at GPS quality.

### Frames ([`geodesy.c`](../SUFST/Src/fusion/geodesy.c))

The ENU origin is anchored on the first fix with hAcc ≤ 5 m after each boot;
the KF is restarted at that moment so it starts at (0, 0, 0). Conversions use a
flat-Earth tangent plane scaled by the WGS84 meridian and normal radii at the
origin — well under a metre of error across a circuit. Latitude/longitude stay
in double precision; ENU offsets are float.

## Air data ([`airdata.c`](../SUFST/Src/fusion/airdata.c))

The EvoScann P4-A pitot scanner sends four absolute pressures on CAN at
100 Hz. CH3 is the pitot tube's total-pressure port; the static reference is
the mean of CH1, CH2 and CH4. Its airspeed and a wind estimate are broadcast
on `GPS_Airspeed` for logging. **Nothing here feeds the position filter or lap
timing**: a pitot measures airspeed, and wind makes that a biased measure of
ground speed.

- **Airspeed**: incompressible Bernoulli, v = √(2q/ρ), with ρ from the static
  pressure and the P4-A's temperature (20 °C if it isn't reporting).
- **Tare**: each channel has its own zero error (~50 Pa between channels, a
  phantom ~9 m/s). Whenever the GPS-aided ground speed has been below
  0.3 m/s for more than a second, the total-minus-static difference is taken
  as zero; airspeed is reported from the first tare after boot (3 s
  stationary). Wind present during the stop is tared away too, so a stop in
  shelter gives the best zero.
- **Wind and scale factor**: the pitot sits in the car's own flow, so it reads
  k·|v_ground − wind|. A 3-state extended Kalman filter on [k, wind_e,
  wind_n] fits that model against the fused ground velocity above 8 m/s. The
  wind is observable because the car points in every direction over a lap; k
  because its speed varies. A 3σ innovation gate rejects gusts and other cars'
  wakes. The broadcast airspeed is the measured value divided by k.

## Lap timing

### Gates ([`gates.c`](../SUFST/Src/laptimer/gates.c))

Eight slots: 0 is start/finish, 1–7 are sectors. A gate is a point plus a
heading; the gate line runs perpendicular to the heading and is
`LAP_GATE_HALF_WIDTH_M` (2 m) either side of the point.

A gate is placed at the current fused position. Its heading is the direction
of travel when moving faster than 2 m/s, otherwise the vehicle's AHRS heading
(so a gate can be set with the car parked, nose down the track — which needs a
calibrated magnetometer). Setting the start/finish clears every sector gate.

A crossing is a sign change of the signed distance to the line, in the forward
direction only, inside the half-width. The crossing instant is linearly
interpolated between the two 104 Hz samples that bracket it.

### Lap state machine ([`laptimer.c`](../SUFST/Src/laptimer/laptimer.c))

- The first start/finish crossing starts lap timing.
- Each later start/finish crossing completes a lap, provided at least
  `LAP_MIN_LAP_TIME_MS` (10 s) has passed — closer re-crossings are ignored.
  Completing a lap also completes its final sector (last sector gate → finish).
- Each sector gate counts once per lap; crossing order is not enforced.
- Moving or clearing the start/finish gate resets lap timing and the lap count.

`imu_task` runs this only while time locked and with the frame anchored, and
never across a discontinuity in the estimate (frame anchoring, time lock, KF
divergence recovery). Completed laps and sectors go out as `Lap_Event` frames;
the running state as `Lap_Status`, whose `DEGRADED` flag is set while there is
no recent fix or no time lock.

### Gate persistence ([`flash_store.c`](../SUFST/Src/persist/flash_store.c))

Gates are stored as **absolute** latitude/longitude (i32, 1e-7°) plus heading,
because the ENU origin moves every boot. At boot they are loaded into a table;
when the first fix anchors the frame they are converted to ENU and placed.
Clears before that act on the table directly, so a restored gate can be removed
before a fix.

The store is an append-only log of 32-byte CRC-protected records in the top
two 2 KB flash pages:

- Every gate set, gate clear and clear-all appends a record; the latest record
  per slot wins on restore. The magnetometer calibration is stored the same way.
- **Compaction** rewrites only the current state into the spare page, then
  writes a completion marker carrying a generation number, and only then erases
  the old page. At boot, if a reset left both pages in use, the page with the
  newer completion marker wins (a marker-less page is a partial rewrite), so an
  interruption at any point leaves one complete copy.
- Compaction erases flash, stalling the CPU for tens of milliseconds, so it only
  runs at boot (if fewer than 32 slots are free) or, when requested by
  `CONFIG_SAVE` or a full page, while the car is stationary. A record that found
  the page full is not lost: the whole current state is re-saved after the
  compaction.

Only `imu_task` touches the store.

## Magnetometer calibration ([`mag_cal.c`](../SUFST/Src/imu/mag_cal.c))

Runs continuously; no driver action needed.

1. Every raw sample feeds a window that tracks how much of the heading circle
   (12 sectors) it has covered.
2. When a window has 200 samples across ≥ 9 sectors it commits a new
   horizontal hard-iron offset and soft-iron scale (the vertical axis can't be
   learned from a level car and keeps its stored value), then starts again — so
   a magnetic disturbance taints at most one window.
3. While GPS is aiding and the car is moving, the AHRS heading is compared with
   GPS course; consistent agreement promotes the calibration to *validated*.
4. Improved calibrations are saved to flash at most once a minute and seed the
   next boot.

`GPS_Mag.cal_status`: 0 uncalibrated, 1 collecting, 2 calibrated, 3 validated.
`CAN_CMD_MAG_CAL_START` discards the calibration and re-learns it.

## GPS link ([`gps_task.c`](../SUFST/Src/gps/gps_task.c), [`gps_config.c`](../SUFST/Src/gps/gps_config.c))

- **Configuration** goes over I2C (the board has no MCU→GPS UART path): probe
  until the receiver answers (it NACKs for ~1 s after power-up), then one
  CFG-VALSET to the RAM layer (115200 baud UBX on UART1, RTCM3 input, 20 Hz,
  automotive model, GPS + Galileo, NAV-PVT and TIM-TM2), confirmed by reading the
  baud rate back.
- **Reception**: USART3 RX in circular DMA with idle-line events; a byte-wise
  UBX parser, so frames may straddle the ring wrap. UART errors re-arm reception.
- **Supervision**: NAV-PVT arrives every 50 ms with or without a fix. If none
  arrives for 2 s the GPS fault flag is set, and every 5 s the configuration is
  re-sent and reception restarted — covering a receiver that reset itself (it
  comes back at 38400 baud talking NMEA).
- Every NAV-PVT is forwarded, fix or not; consumers check `gnssFixOK`
  themselves, and anything older than 1 s is treated as lost.

## CAN ([`canbc_task.c`](../SUFST/Src/canbus/canbc_task.c))

A 10 ms slot loop walks a staggered rota (each message has a period and phase in
slots, so rates from 100 Hz down to 1 Hz don't pile into the same slot). The
3-deep hardware TX FIFO is allowed a few milliseconds to drain before a frame is
dropped; if the loop overruns (nothing on the bus to acknowledge), the backlog
is discarded rather than replayed. Bus-off is cleared once a second.

Received frames: `GPS_Command` (dispatched to the owning task's queue),
`GPS_Gate_Set` (a gate at a given position, straight to imu_task's
`g_gate_set_queue`) and the VCU's `Wheel_Speeds` (front-left wheel only).

## Health and fault handling

| Fault | Detection | Response |
|-------|-----------|----------|
| Task hang / boot hang | IWDG, ~1.25 s, refreshed only by the lowest-priority task | Reset |
| Stack overflow | FreeRTOS check (method 2) | Immediate reset |
| GPS link lost | No NAV-PVT for 2 s | Fault flag, periodic re-configure |
| Loss of fix | `gnssFixOK` clear, or no fix for 1 s | GPS-aided / degraded flags; KF dead-reckons |
| IMU read failures | 10 consecutive failed reads, or config read-back mismatch (1 Hz) | Fault flag, GPS-only mode, re-init attempt every second |
| Magnetometer silent | No new sample for 1 s | Fault flag; AHRS continues without it |
| CAN bus-off | Protocol status, 1 Hz | Leave INIT mode to rejoin |
| Flash page full | Append returns full | Compact when stationary, then re-save |

## Testing

Host-side unit tests (no hardware needed) cover the UBX parser, CAN packing,
air data (tare, wind and scale convergence on a simulated session), AHRS, KF (including delayed correction, outlier rejection and divergence
recovery), geodesy, gates, lap timer, magnetometer calibration, NMEA formatting,
time base and the flash store's record, restore and page-selection logic. The
FreeRTOS tasks and hardware drivers are not covered and have not yet been run on
the board.
