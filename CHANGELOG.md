# Changelog

## 1.0.0 (unreleased)

First versioned build. The version is set in `CMakeLists.txt` and embedded in
the image (`strings gps-firmware.bin | grep gps-firmware`).

**Status: builds clean, host tests pass, not yet run on the board.** Every
hardware-facing fix below is derived from datasheets and code review and still
needs confirming on the bench.

### Fixed — would have broken operation on the car

- **IMU data-ready interrupt was never enabled** (`INT1_CTRL`), so the fusion
  loop ran off its 20 ms fallback timeout (~50 Hz) instead of 104 Hz.
- **Gyro scale was 12.8 % low**: ±2000 dps is 70 mdps/LSB, not FS/32768.
- **GPS boot configuration had no delay between probe retries**, so a receiver
  still booting (it NACKs for ~1 s) was never configured after a cold start.
- **Kalman filter could lock GPS out permanently** after an outage: once the
  state drifted, every fix failed the outlier gate. It now re-opens its
  covariance after 10 consecutive rejections and snaps back.
- **Loss of fix was invisible**: the last good fix kept being fused status
  (`fix_type`), `GPS_READY` never cleared, and NMEA replayed the stale position
  forever. All NAV-PVTs are now forwarded and checked; stale data expires.
- **Lap timer kept timing against a moved/removed start/finish line**, and
  could start a lap before GPS time was locked (yielding a multi-day "lap").
- **Flash store could pick the half-written page after an interrupted
  compaction** and erase the good one. Compactions are now sealed with a
  generation marker; page selection is host-tested.
- **Flash store was used from two tasks** (gate saves vs. compaction). It is
  now owned by `imu_task` alone; a full page triggers compact-and-resave
  instead of silently dropping records, and the store compacts at boot.
- **CAN bus-off was never recovered** — one wiring glitch silenced the node
  until power-cycle.
- **IMU task stack overflowed in Debug builds** (and had ~200 B spare in
  Release); IMU/GPS stacks enlarged from measured frame sizes.
- I2C2 timing constant was for a 64 MHz clock (~265 kHz bus); now 100 kHz.
- PPS capture / timer rollover race (65 ms timestamp error, rare).
- Target build used `-fsingle-precision-constant`, making the 1e-7° lat/lon
  scaling single-precision on target only (~7 cm bias, and gates shifting by
  that much across a reboot). Removed; target now matches the host tests.
- Lap-button sector slot was a press counter that reset on reboot, so a short
  press after power-up overwrote sector 1. It now picks the first free slot.
- Out-of-range gate slots from CAN were written to flash as junk records.
- A gate cleared before the first fix was resurrected when the fix arrived.

### Added

- **Pitot airspeed and wind** (`GPS_Airspeed`, 0x6BE, 10 Hz) from the
  EvoScann P4-A on 0x190/0x191: CH3 total pressure against the mean of CH1,
  CH2 and CH4, zeroed at every standstill, with an EKF fitting the wind and
  the pitot's installation scale factor against the GPS ground velocity.
  Broadcast only — not used for position or lap timing. Dashboard simulator
  and DBC updated.

- Runtime health: GPS link supervision with automatic re-arm/re-configure,
  IMU config read-back with re-init, magnetometer liveness, GPS-only degraded
  mode if the IMU dies.
- `GPS_Attitude.fusion_status` and `Lap_Status.flags` now carry defined bits
  (`CAN_FUSION_STATUS_*`, `CAN_LAP_FLAG_*`); `GPS_Status` retry counters,
  IMU and MCU temperatures, and the `diffSoln` quality flag are populated
  (all were hard-coded to zero).
- `GATE_SET` slot `0xFE` = next free sector.
- NMEA output via DMA (no CPU busy-wait), rate clamped to the fix rate.
- LSM6DSO32 is software-reset before configuration (safe across warm resets).
- Host tests for page selection, KF divergence recovery, lap events and
  per-lap sector de-duplication. `-Wextra` on project sources.

### Changed

- **Wheel speed now comes from the front-left wheel only** — the other three
  channels of the VCU's `Wheel_Speeds` frame are unreliable on the car, and a
  dead channel reading 0 would have dragged the old 4-wheel average down.
- **Wheel-speed aiding is now off by default** (`KF_WHEELSPEED_AIDING` in
  `board_config.h`). It was wired in but unvalidated, fused ungated along the
  magnetometer heading. Enable it once FL speed has been checked against GPS
  speed on track.
- Setting or clearing the start/finish gate resets the lap counter.
- `Lap_Event` SECTOR frames carry the lap count before that lap's completion
  consistently (the final sector previously carried the next number).
- `SYS_EVT_PVT_TICK` (never set) removed from the status bits.
- Dashboard: "Set next sector gate" sends slot `0xFE` instead of keeping its
  own counter (which drifted from the node's real gate table); the simulator
  handles `0xFE`, reports the new status bits, and resets the lap count on a
  new start/finish. Status panel decodes `fusion_status` and lap flags.
- Documentation rewritten against the code: DATASHEET (pins, clocks, CAN
  matrix, status bits, flash store), ARCHITECTURE, DEVELOPER (flashing,
  bring-up checklist), QUICK_START, DRIVER_GUIDE, NOTES.

### Known gaps

- No on-hardware validation yet: follow the bring-up checklist in
  `docs/DEVELOPER.md`. CPU load and stack high-water marks are unmeasured.
- `LAP_MIN_LAP_TIME_MS` (10 s) rejects skidpad-length laps.
- The Debug build uses 90 % of code flash.
- The pitot channel mapping (CH3 total, CH1/2/4 static) and the P4-A scaling
  (from `can-defs`, itself inferred) need confirming on the car.
