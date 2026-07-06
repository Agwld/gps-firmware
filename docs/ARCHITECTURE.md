# Architecture — How the GPS firmware works

This document covers the design and implementation of the STAG 12 GPS node firmware.

## System overview

The firmware runs on an STM32G431 and orchestrates three main data flows:

1. **Sensor acquisition** — GNSS (20 Hz) and IMU (104 Hz) data streams
2. **Fusion pipeline** — Real-time blending of GPS and IMU into a smooth position/attitude estimate
3. **Lap timing** — Gate crossing detection and time-stamping on the fused estimate
4. **CAN broadcast** — Position, attitude, IMU, lap times, status at various rates

All tasks run on FreeRTOS with static allocation (no heap), mirroring the team's firmware style.

## Sensor fusion

### Why fusion?

Raw GPS at 20 Hz is too slow and too latent (20–60 ms post-facto) for sub-metre gate crossing at racing speeds. The solution is to run a **continuous estimate at IMU rate (104 Hz)** and treat each GPS fix as a *correction* to that estimate, not as the position itself.

```
IMU (104 Hz)              GPS (20 Hz)
 ↓                         ↓
Mahony AHRS     →←        ENU conversion
(attitude)       |         ↓
                 v         ↓
            6-state Kalman filter
                 ↓
        fused position/velocity
                 ↓
         ← lap timing, CAN, NMEA
```

### Mahony AHRS (`ahrs.c`)

A 9-DOF complementary filter that computes attitude (yaw/pitch/roll) from:
- **Gyro** — primary attitude rate (integrated)
- **Accel** — gravity reference (corrects pitch/roll drift)
- **Mag** — heading reference (corrects yaw drift)

```
Quaternion update every IMU sample:
q_new = q_old + (0.5 * w) * q_old * dt + Kp * accel_error + Ki * integral(accel_error)
```

**Key properties:**
- Integral term estimates and cancels gyro bias (no separate bias state needed)
- Runs at 104 Hz (every IMU sample), so attitude is always up-to-date
- No gimbal lock (quaternions handle all orientations)
- Output: unit quaternion `q = [qx, qy, qz, qw]`

### 6-state Kalman filter (`kf6.c`)

State vector: `[pos_e, pos_n, pos_u, vel_e, vel_n, vel_u]` (East-North-Up frame set at first GPS fix)

**Predict step** (every 104 Hz IMU sample):
- Rotate IMU accel into ENU using AHRS attitude
- Integrate accel to update velocity: `vel_new = vel_old + accel * dt`
- Integrate velocity to update position: `pos_new = pos_old + vel * dt`

**Correct step** (every GPS fix, ~20 Hz):
- Convert GPS lat/lon to ENU relative to the frame origin
- Compare fused position to GPS position → position error
- Kalman gain blends this error back into the state (velocity damping included)
- **Delayed-state rewind**: because a GPS fix is 20–60 ms old, the filter keeps a 16-entry ring of past predict steps; a correction rewinds to the matching history entry, applies the correction there, then replays all subsequent predicts to get the final state right

**Result:** A smooth position estimate at 104 Hz with GPS as the long-term truth and IMU providing short-term coherence and inter-sample resolution.

### Coordinate frames

| Frame | Definition | Used for |
|-------|-----------|----------|
| **ECEF** | Earth-Centered-Earth-Fixed (WGS84) | GPS receiver coordinates |
| **Geodetic** | lat/lon/height (degrees + metres) | User-facing gate positions, CAN broadcast (absolute) |
| **ENU** | Local East-North-Up, origin at first GPS fix | Fusion state, gate crossing detection, CAN broadcast (relative) |
| **Body** | Aircraft convention (roll/pitch/yaw) | Accel/gyro/mag interpretation, vehicle dynamics |

Conversions:
- **GPS lat/lon → ENU**: `geodesy.c` (flat-Earth approximation: valid for ~10 km tracks)
- **ENU → lat/lon**: `geodesy.c` inverse (used when broadcasting gates)
- **Body accel → ENU accel**: `ahrs.c` rotates via attitude quaternion

## Lap timing

### Gate representation

A gate is a **point + heading in ENU**, treated as an infinite-width line perpendicular to that heading:

```
Heading = 45° (northeast)
    ↑
    │
  ─ ─ ─ ← gate line (perpendicular to heading)
    │
    → car trajectory
```

Stored in flash as **absolute lat/lon** (int32_t 1e-7 degree) + heading (float rad), so gates survive power cycles even if the car moves. On origin-set (first GPS fix), all persisted gates are re-projected into the current ENU frame.

### Crossing detection (`laptimer.c`)

Every 104 Hz iteration:

1. Compute the projection of the fused position along the heading vector:
   ```
   projection = (pos - gate_pos) · heading_unit_vector
   ```
2. Check for a sign change (position crossed the line)
3. Check direction (must cross forward, not backward)
4. Use linear interpolation between the current and previous sample for ~1 ms-class timing resolution

```c
// Pseudocode
float proj_now = dot(pos_now - gate_pos, heading_dir);
float proj_prev = dot(pos_prev - gate_pos, heading_dir);

if ((proj_prev <= 0 && proj_now > 0) || sign change with valid direction) {
    // Interpolate to find exact crossing time
    float t_cross = -proj_prev / (proj_now - proj_prev);
    timestamp = prev_timestamp + t_cross * dt;
    on_crossing(gate_idx, timestamp);
}
```

### Lap state machine

Gates are organized in slots:
- **Slot 0**: Start/finish line
- **Slots 1–7**: Sector gates (up to 7 segments per lap)

On each crossing:
1. If **slot 0 (start/finish)** is crossed:
   - If sector counter is 0, start a new lap
   - If sector counter > 0, end the current lap and record total time
   - Reset sector counter to 0
2. If **slot 1–7** is crossed:
   - Record the segment time (delta from previous gate)
   - Increment sector counter
   - Broadcast the segment time to the dashboard

### Time-stamping

All crossing times are expressed in **GPS time-of-week** (iTOW, 0–604799999 ms per week) via a PPS-disciplined mapping ([`timebase.c`](../SUFST/Src/fusion/timebase.c)):

- TIM3_CH2 captures the rising edge of the 1 PPS signal from the ZED-F9P
- A `tick ↔ iTOW` mapping is maintained: when PPS fires, we know the exact GPS time for that tick count
- All fused positions are stamped with the TIM3 tick at which they were computed
- On crossing, the tick is converted to iTOW (GPS time) using this mapping

**Why GPS time, not MCU ticks?**
- Times are meaningful across resets (reboot during a lap doesn't lose the crossing timestamp)
- Times are comparable to external GPS-timestamped data (data logger, other vehicles)
- PPS is nanosecond-accurate; we inherit that precision

## Gate persistence

### Flash storage format

Gates are stored in the **last 2 KB page of flash** (address depends on variant: 0x1F800 for CB, 0x0F800 for C8) as an **append-only log**:

```c
struct flash_store_record {
    uint8_t kind;              // GATE, GATE_CLEAR_ALL, or MAG_CAL
    uint8_t key;               // gate index (0–7)
    uint8_t gate_valid;        // 1 = valid gate, 0 = cleared slot
    uint8_t reserved;
    int32_t payload[2];        // gate: [lat_1e7, lon_1e7]; mag_cal: [bias_xyz, scale_xyz]
    float heading_rad;         // or scale/bias floats for mag_cal
    uint32_t crc;              // CRC32 of all fields except crc itself
};
```

Each record is 24 bytes. The page holds ~85 records before compaction is needed.

### Restore sequence

At boot, `flash_store_init()` scans the page:

1. **For each gate record**: update the in-memory gate table (newest wins; cleared slots have `valid=0`)
2. **For each clear-all record**: void all slots
3. **Result**: in-memory gate table reflects the latest state

Gates are then **deferred**: they're stored as absolute lat/lon until the first GPS fix anchors the frame origin. Once the origin is known, all valid gates are re-projected into ENU and placed into the gate crossing detector.

### Persist on write

When a gate is set or cleared (via steering wheel or CAN command):

```
set_gate(index, heading) →
  ├─ place in ENU (for crossing detection)
  ├─ compute absolute lat/lon from fused pos + origin
  └─ append record to flash (with valid=1 or valid=0)
```

Flash writes are non-blocking on this MCU (using HAL DMA + CCM RAM for program ops).

### Compaction

The append-only log grows with every gate edit. To reclaim space, `flash_store_erase_and_compact()`:

1. **Replay** the entire log to recover the current state
2. **Erase** the flash page
3. **Rewrite** only the latest valid gates (cleared/superseded entries vanish)

Compaction is triggered by `CAN_CMD_CONFIG_SAVE` (via dashboard or the `sys_task` button) and waits for the car to be **judged stationary** to avoid disrupting real-time loops.

## CAN broadcast

### Message schedule

The `can_task` runs at ~100 Hz (every 10.4 ms) and broadcasts staggered periodic messages:

| ID | Name | Rate | Contents |
|----|------|------|----------|
| 0x6B0 | GPS_Position | 20 Hz | fused lat/lon |
| 0x6B1 | GPS_Velocity | 20 Hz | speed, course, altitude, fix status |
| 0x6B2 | GPS_Attitude | 50 Hz | yaw/pitch/roll (Euler), fusion status |
| 0x6B3 | Lap_Status | 10 Hz | lap number, running time, sector, crossing status |
| 0x6B4 | Lap_Event | event | steering wheel or TIM-TM2 button timestamp (GPS time) |
| 0x6B5 | GPS_Quality | 5 Hz | accuracy (HDOP, VDOP), fix/RTK flags |
| 0x6B6 | GPS_IMU_Accel | 100 Hz | raw calibrated accel (x, y, z) |
| 0x6B7 | GPS_IMU_Gyro | 100 Hz | raw calibrated gyro (x, y, z) |
| 0x6B8 | GPS_Temp | 1 Hz | board temp, IMU temp, MCU temp |
| 0x6B9 | GPS_Status | 1 Hz | uptime, fault bits, CPU load |
| 0x6BA | GPS_Mag | 10 Hz | magnetometer + calibration status |
| 0x6BB | GPS_Frame_Origin | 1 Hz | ENU origin as absolute lat/lon |
| 0x6BC | GPS_Gate | round-robin ~5 Hz | one gate per frame (index, east_m, north_m, heading_deg, valid) |
| 0x6BD | GPS_Time | 1 Hz | GPS iTOW + UTC wall clock + validity flags (bus-wide time reference) |
| 0x6BF | GPS_Command | RX | gate set/clear, mag-cal, config commands |

### Signal scaling and precision

Example: `GPS_Position` (0x6B0):
```
lat_deg = (int32_t raw_lat) * 1e-7   # ±90° in 10 cm increments
lon_deg = (int32_t raw_lon) * 1e-7   # ±180° in 10 cm increments
```

Gate positions:
```
gate_east_m = (int16_t raw) * 0.1   # ±3276.7 m in 0.1 m increments
gate_north_m = (int16_t raw) * 0.1
gate_heading_deg = (uint16_t raw) * 0.1  # 0–360° in 0.1° increments
```

All scaling is done at pack time (firmware) and unpack time (dashboard/logger), preserving raw precision in CAN.

## Task architecture (FreeRTOS)

Five tasks run statically allocated, priority-based scheduling:

### `imu_task` (priority 3, highest real-time)

- **Rate**: 104 Hz (every 9.6 ms)
- **Triggered**: SPI1 DMA completion interrupt (LSM6DSO32 burst read)
- **Logic**:
  1. Read 6 accel + gyro values from DMA buffer
  2. Apply calibration (bias subtraction, scale division)
  3. Feed accel/gyro to Mahony AHRS → quaternion
  4. Feed IMU to KF predict step
  5. Apply pending GPS/wheelspeed corrections (from queues)
  6. Check gate crossings
  7. Update lap state machine
  8. Feed the magnetometer sample to the continuous background calibrator (and cross-check heading vs GPS course when moving)
- **Output**: fused position/velocity to `canbc` state (mutex-guarded)
- **Stack**: 2 KB

### `gps_task` (priority 2)

- **Rate**: Variable (depends on UBX frame boundaries)
- **Triggered**: USART3 RX DMA (u-blox UBX protocol)
- **Logic**:
  1. Parse UBX frame (header, payload, checksum)
  2. On NAV-PVT: extract lat/lon/height/velE/velN/velD → queue to `imu_task`
  3. On TIM-TM2: extract button edge timestamp (GPS time) → Lap_Event CAN message
  4. Handle boot-time UBX-CFG-VALSET/VALGET responses (configuration state machine)
- **Output**: GPS fixes and TM2 events queued to `imu_task`
- **Stack**: 2 KB

### `can_task` (priority 1)

- **Rate**: ~100 Hz (staggered periodic)
- **Logic**:
  1. Broadcast periodic messages (Position @ 20 Hz, Attitude @ 50 Hz, etc.)
  2. Dequeue inbound CAN commands (gate set/clear, mag-cal, config)
  3. Dispatch commands to `imu_task` or `sys_task` via queues
- **Output**: CAN frames on FDCAN1
- **Stack**: 1.5 KB

### `aux_task` (priority 1)

- **Rate**: ~1 Hz (or on-demand)
- **Logic**: Synthesize NMEA sentences from latest fused state → USART3 TX (→ MAX3232 → MoTeC datalogger, via solder jumper JP6)
- **Output**: NMEA on RS232 link at 115200 (USART3's TX shares the baud register with the UBX stream on RX)
- **Stack**: 1.5 KB
- Note: RTCM corrections never pass through the MCU — the RS232 input is routed straight to the F9P's UART1 RX in hardware (JP7 bridged 2‑3)

### `sys_task` (priority 0, lowest)

- **Rate**: 10 Hz
- **Logic**:
  1. Read buttons (lap button, user button) → queue to `imu_task` or trigger actions
  2. Update LEDs (status, fault indicators)
  3. Read temperature sensor (MCP9800) → CAN broadcast
  4. Reset watchdog (IWDG)
  5. On user request: trigger flash compaction (gate cleanup)
- **Output**: Button events, LED state, temperature readings
- **Stack**: 1.5 KB

### Queues and synchronization

- **`app_event_queue`** — Button/command events from `sys_task`/`can_task` to `imu_task`
- **`gps_nav_queue`** — GPS fixes from `gps_task` to `imu_task`
- **`canbc` state** — Mutex-guarded FreeRTOS state (fused position/attitude for `can_task` to broadcast)
- **I2C2 bus mutex** — arbitrates the shared bus between `gps_task`'s boot config (F9P DDC port) and `sys_task`'s MCP9800 temperature reads
- **DMA completion semaphore** — `imu_task` blocks waiting for SPI DMA, unblocks on completion

All queues are **static allocation** with FIFO overflow handling (oldest message dropped if full).

## Boot sequence

1. **HAL init** — Reset handler, clock tree, peripheral clocks, GPIO
2. **FreeRTOS init** — Scheduler setup, static tasks created
3. **Peripheral drivers** — USART, SPI, CAN, I2C, TIM configured
4. **Task startup**:
   - `sys_task` — Reads button state, initializes LEDs
   - `gps_task` — Configures the ZED-F9P via UBX-CFG-VALSET **over I2C** (the board has no MCU→GPS UART path): UART1 baud, UBX-only output, RTCM3X input, 20 Hz rate, constellations, NAV-PVT/TIM-TM2 enables
   - `imu_task` — Initializes LSM6DSO32, Mahony AHRS, KF state
   - `can_task` — Initializes CAN transceiver, schedules first broadcast
   - `aux_task` — Starts NMEA synthesis to the MoTeC (USART3 TX)
5. **Restore state** — `flash_store_init()` loads gates from flash (deferred)
6. **Scheduler starts** — `vTaskStartScheduler()` begins task switching

When the first GPS fix arrives:
- `gps_task` queues a GPS fix to `imu_task`
- `imu_task` anchors the ENU frame origin at the first position
- All deferred gates are projected into ENU and armed for crossing detection

## Magnetic calibration

The magnetometer experiences hard-iron (constant bias) and soft-iron (scale imbalance) distortion. Calibration runs **continuously in the background** — no driver action needed:

1. **Feed**: every raw sample flows into a rolling window (`mag_cal_cont_feed` in [`mag_cal.c`](../SUFST/Src/imu/mag_cal.c)), which tracks how much of the heading circle has been swept (12 sectors).
2. **Finalise**: once a window covers ≥9/12 sectors, it commits a new **horizontal** (x/y) bias/scale — all a heading needs. The vertical axis can't be calibrated from a level car (it sees a constant field), so its terms are carried over from flash. The window then resets, so a magnetic transient taints at most one window (self-healing).
3. **Validate**: whenever the car is moving above the course-valid speed, the fused heading is cross-checked against GPS course-over-ground. Tight circular agreement (constant offset = declination + mounting) promotes the state to *validated*.
4. **Quality flag** (`GPS_Mag.cal_status`): 0 uncalibrated → 1 collecting → 2 calibrated → 3 validated. A consumer wanting a trustworthy heading gates on ≥2.
5. **Persist**: an improved calibration is written to flash (rate-limited to bound wear), and restored at boot as the seed — so heading works immediately, starting at *calibrated*, before re-validating against course.

`CAN_CMD_MAG_CAL_START` remains as a "force a fresh rebuild" override (discard the current cal and re-sweep, e.g. after new hardware is fitted near the sensor); `CAN_CMD_MAG_CAL_STOP` is now a no-op.

The underlying model is the "basic" axis-aligned min/max method, not a full 9-parameter ellipsoid fit. Good enough for a heading; a full ellipsoid fit is a natural upgrade.

## Error handling & robustness

### CRC validation

- UBX frames: checksum validated on every parse
- Flash records: CRC32 validated on every restore
- Corrupted records are skipped (logged if debug enabled)

### Watchdog

- IWDG (Independent Watchdog) set to 5 s timeout
- `sys_task` resets it every 100 ms
- If the firmware hangs, IWDG fires a hard reset

### Bus errors

- SPI DMA: errors logged; transfer is retried
- CAN: errors logged; transceiver stays alive (CAN is fail-silent anyway)
- USART: parity/framing errors logged; next frame is re-synced

### GPS loss of fix

- KF continues predicting from IMU accel
- Lap timing still works (position estimate is continuous)
- CAN broadcast flags indicate "no fix" (downstream consumers handle it)

## Performance & optimization

See [DEVELOPER.md](DEVELOPER.md) for CPU load breakdown. Key optimizations:

- **DMA for SPI** — SPI reads yield the CPU instead of spinning
- **Staggered CAN broadcast** — Not all messages every cycle; spreads load
- **Ring buffers** — KF history ring, lap history ring; no malloc
- **Inline math** — Common operations (dot product, quaternion ops) are inlined
- **Fixed-point optional** — Some paths could use fixed-point; currently all float for precision and simplicity

Flash memory:
- **Release build**: ~58 KB (45% of 128 KB CB)
- **Debug build**: ~94 KB (73% of 128 KB CB)

## Testing strategy

### Host-side tests

- **UBX parsing** ([`test_ubx.c`](../tests/test_ubx.c)) — Checksum, frame sync, payload extraction
- **CAN pack/unpack** ([`test_can_pack.c`](../tests/test_can_pack.c)) — All message types encoded/decoded correctly
- **Fusion** ([`test_kf6.c`](../tests/test_kf6.c), [`test_ahrs.c`](../tests/test_ahrs.c)) — Kalman filter and AHRS with synthetic data
- **Lap timing** ([`test_laptimer.c`](../tests/test_laptimer.c)) — Gate crossing detection, sector times, lap state machine
- **Persistence** ([`test_flash_store.c`](../tests/test_flash_store.c)) — Record write/read, CRC, restore, compaction

All tests are **deterministic** and pass/fail on assertions. No mocking framework; tests exercise real code paths.

### Hardware testing

Not yet performed. Pending items:
- Gate placement accuracy and reproducibility across power cycles
- Lap time accuracy under real GPS + IMU conditions (compare to external timing reference)
- CAN broadcast timing and signal integrity
- Thermal stability of the fusion filter

## Future enhancements

See [NOTES.md](NOTES.md) for architectural notes and planned improvements.

---

**Questions about a specific subsystem?** Check the header file (e.g., `SUFST/Inc/fusion/ahrs.h`) for API docs and usage examples.
