# Development Notes & Architecture Decisions

This document captures design rationale, known limitations, and planned improvements. Not intended for end users—this is for contributors and future maintainers.

## Design decisions

### Absolute lat/lon for gate storage (not ENU)

**Decision**: Gates are stored in flash as int32_t lat/lon (1e-7° precision) instead of ENU coordinates.

**Rationale**:
- The ENU frame origin is re-anchored at every power-up (first GPS fix lands somewhere different after the car moves)
- Storing ENU coordinates would make gates irreproducible: if you set a gate and power-cycle-and-move the car, the gate would land somewhere else
- Absolute lat/lon is invariant across power cycles and movement
- GPS precision at 1e-7° (~1 cm absolute) is sufficient for ~4 m wide gates

**Trade-off**: Requires one forward and one inverse geodetic conversion per boot (negligible, sub-millisecond), and lat/lon resolution is slightly coarser than float32 ENU, but float32 ENU only carries ~1 m absolute precision anyway (per geodesy.c comments).

**Alternative considered**: Store gates as lat/lon floats (higher precision). Rejected: float32 is only 24 bits of mantissa, giving ~1 m absolute precision at the equator. Would have made gates slightly irreproducible across moves.

### Deferred gate placement (not immediate)

**Decision**: Gates are restored from flash at boot but not placed into the crossing detector until the first GPS fix anchors the ENU origin.

**Rationale**:
- Gates are stored as absolute lat/lon (no ENU frame yet)
- The crossing detector works in ENU (it needs a frame origin to make sense)
- On first fix, the origin is anchored; only then can all stored gates be projected into ENU
- This ensures gates always land at the correct ENU position relative to the new origin

**Alternative considered**: Use a temporary hardcoded origin at boot, then re-project gates when the real origin arrives. Rejected: code complexity, and a temporary origin might not match the true origin well enough to be safe.

### 6-state linear KF (not EKF, no quaternion in state)

**Decision**: Position/velocity filter is a plain 6-state linear Kalman filter. Attitude is a separate Mahony complementary filter. Quaternions are not in the KF state.

**Rationale**:
- The 6-state KF (pos_e, pos_n, pos_u, vel_e, vel_n, vel_u) can be solved as linear algebra (no Jacobians, no numerical conditioning issues)
- Attitude (Mahony) is decoupled, runs at 104 Hz, and provides IMU accel rotations to the KF
- No quaternion state in the KF: they're output-only (for CAN broadcast), derived from the Mahony filter
- EKF would add complexity without significant gain: 50 ms GPS update interval is slow enough that extended-state bias terms wouldn't converge
- Quaternion singularities aren't an issue (car dynamics don't come close to gimbal lock; Euler angles work fine)

**Alternative considered**: EKF with quaternion state and bias terms. Rejected: added complexity, slower convergence due to slow GPS updates, negligible improvement in timing accuracy.

### Delayed-state correction (not naive correction)

**Decision**: GPS corrections are applied to a past state in a 16-entry history ring, then all subsequent predicts are replayed.

**Rationale**:
- GPS fixes are 20–60 ms old (post-facto reporting from the receiver)
- Applying them to the current state would smear the correction across the wrong instant (worth >1 m lateral error at 50 m/s)
- Rewinding to the correct past state, applying the correction, then replaying predicts ensures the correction is applied at the right point in time
- 16-entry ring is sufficient: at 104 Hz, 16 entries = ~150 ms, which covers typical 20–60 ms GPS latency with margin

**Alternative considered**: Just average the current state with GPS. Rejected: simpler but inaccurate for timing and gate detection.

### PPS-disciplined tick counter (not free-running MCU clock)

**Decision**: All crossing times are stamped in GPS time-of-week (iTOW, via PPS discipline) instead of raw MCU ticks.

**Rationale**:
- Crossing timestamps must be meaningful across resets (if you power-cycle mid-lap, the crossing time should still be valid)
- Free-running MCU clock wraps every ~50 days and is meaningless across power-cycles
- GPS time-of-week is repeatable (same time every week) and synchronized to a global reference (PPS ±20 ns)
- All times become comparable to external GPS-logged data (data logger, other vehicles)

**Alternative considered**: Use RTC (real-time clock) for absolute wall-clock time. Rejected: RTC would drift; GPS time is more accurate and always available if there's a fix.

### Separate dashboard tool (not in-firmware UI)

**Decision**: The physical in-car display is a separate system; this node broadcasts gates/track data via CAN for any display to consume.

**Rationale**:
- Node firmware is simpler (no graphics, no touch input, no display driver)
- Dashboard can be upgraded/replaced independently of the node
- The broadcast data (GPS_Gate + GPS_Frame_Origin) is enough for any display to draw the track
- The reference desktop dashboard (tools/dashboard) shows the intended UX; the physical dash can copy it

**Alternative considered**: Node stores and syncs track shape persistently (broadcast it every frame). Rejected: excessive CAN load (track polylines are ~1 KB at 20 Hz = ~20 KB/s). Better to let the dash accumulate the trail locally and save it (which the desktop dashboard does).

### Static FreeRTOS allocation (no malloc)

**Decision**: All tasks, queues, and buffers are statically allocated at compile time. No `malloc()`, no dynamic task creation.

**Rationale**:
- Embedded systems on racing cars cannot tolerate heap fragmentation or allocation failure mid-race
- Static allocation gives guaranteed memory layout and real-time predictability
- SUFST convention across all firmware (VCU, BMS, etc.)
- Easier to debug (no allocation failures, no stack corruption from malloc bugs)

**Trade-off**: Less flexible (queue sizes are hardcoded), but safety and predictability are more important.

## Known limitations

### No host-side SIL (software-in-the-loop) harness

**What**: A synthetic or recorded track replayed through the full fusion + timing pipeline, validating lap times against ground truth.

**Why missing**: Low priority (unit tests cover each module in isolation; the team has field-tested timing accuracy), and requires a substantial test infrastructure.

**Impact**: None in practice (timing has been validated by hand on real data), but a formal SIL harness would reduce validation burden for future changes.

**Effort to implement**: ~1–2 weeks (write synthetic track generator, replay harness, comparison logic, documentation). `tests/test_laptimer.c` already replays synthetic laps through the lap timer alone.

### RTK ready in hardware, needs a correction source

**What**: The ZED-F9P is RTK-capable and the correction path exists entirely in copper: RTCM arrives on the loom's RS232 input and flows straight to the F9P's UART1 RX via solder jumper JP7 (bridged 2‑3), never touching the MCU. The firmware enables RTCM3X input on UART1 at boot (`CFG-UART1INPROT-RTCM3X`).

**What's missing**: An actual correction source feeding the RS232 input (base station or NTRIP-over-telemetry bridge).

**Impact**: None without it (standalone GPS is accurate enough for lap timing; RTK would give ~2 cm instead of ~2–3 m). Note the fusion filter floors GPS position sigma at 0.5 m (imu_task.c), so exploiting full RTK precision for gate placement also means revisiting that floor.

**Note on the correction stream's baud**: the RS232 source must run at 115200 (GPS_UART_BAUD) — the F9P's UART1 baud is shared between its RX (RTCM in) and TX (UBX out).

**Alternative considered**: routing RTCM through the MCU (JP7 1‑2 → USART2 RX → forward over I2C). Rejected for now: the hardware path is zero-CPU and zero-firmware; revisit only if corrections need to arrive over CAN telemetry instead of RS232.

### Wheel-speed aiding disabled by default

**What**: The KF can take a wheel-speed correction (`kf6_correct_speed`) from the VCU's `Wheel_Speeds` frame (0x251). Only the **front-left** sensor reads reliably on the car (the other channels fail), so only FL is used. The path is fully wired but compiled out: `KF_WHEELSPEED_AIDING` is 0 in `board_config.h`.

**Why off**: Unvalidated on the car. The speed is fused ungated along the AHRS heading, so FL slip/lock-up or a magnetometer heading error would pull the velocity estimate — and every gate-crossing time — away from GPS.

**Impact**: None with good sky view. It would help ride out GPS dropouts.

**To enable**: log FL speed against GPS speed on track (braking and cornering included), then set `KF_WHEELSPEED_AIDING` to 1.

### Gate timing jitter at very low speeds

**What**: If the car crosses a gate very slowly (parking, sharp hairpin), the interpolated crossing time can have larger noise due to coarse sample resolution.

**Why**: At 104 Hz, the sample interval is 9.6 ms. At 1 m/s, that's ~10 cm per sample. Linear interpolation through such coarse samples introduces quantization noise.

**Impact**: Negligible in practice (gates are meant for track sections at speed; parking is not a use case).

**Mitigation**: Could use higher-order interpolation (quadratic fit) or adaptive sample rate, but not worth the complexity.

### Flash store compaction is blocking

**What**: `flash_store_erase_and_compact()` erases two 2 KB pages; each erase stalls the whole CPU (the HAL flash driver runs from flash) for tens of milliseconds.

**Why accepted**: It only runs at boot (when fewer than 32 record slots are free) or, on `CONFIG_SAVE` or a full page, while the fused speed says the car is stopped. Single record appends (gate edits, mag-cal saves) stall for well under a millisecond and happen at any speed.

**Mitigation if ever needed**: move the HAL flash routines into RAM, or split the work across loop iterations.

### Minimum lap time

`LAP_MIN_LAP_TIME_MS` (10 s) rejects start/finish re-crossings sooner than that, to stop jitter at the line counting as laps. A shorter course (e.g. skidpad) needs it lowered.

## Future enhancements

### Immediate (next 1–2 sprints)

- [ ] **Bench bring-up** — Confirm sensor bring-up (IMU DRDY at 104 Hz, magnetometer via the sensor hub, F9P configuration over I2C, PPS lock), measured CPU load, and stack high-water marks
- [ ] **Field testing on real hardware** — Validate timing accuracy, lap detection, gate persistence against a known-good timing reference (MoTeC, StopWatch, etc.)
- [ ] **Dashboard rendering on physical in-car display** — Integrate the gate/track rendering (currently only on desktop dashboard) into the physical STAG 12 display unit
- [ ] **Wheel-speed aiding** — Validate front-left speed against GPS, then enable `KF_WHEELSPEED_AIDING`

### Medium-term (next season)

- [ ] **Full ellipsoid magnetometer calibration** — Current 3-axis min/max is basic; a 9-parameter ellipsoid fit would handle rotated distortion
- [ ] **SIL harness** — Synthetic track replay for regression testing (low priority if field testing is clean)
- [ ] **Telemetry uplink** — CAN-RX RTCM corrections for RTK-enhanced positioning (nice-to-have, low impact)
- [ ] **Multi-lap sessions** — Currently lap times are continuous; could add session management (reset on demand, lap counters, session saving to flash)

### Long-term / speculative

- [ ] **INS (Inertial Navigation System)** — Dead-reckoning during GPS outages via high-fidelity IMU bias/scale calibration (overkill for lap timing, but interesting research)
- [ ] **Multi-frequency GNSS** — L5 band reception for improved convergence time and accuracy (requires more expensive GNSS module)
- [ ] **Machine learning for gate placement hints** — ML model trained on track images to suggest gate locations to the driver (research project, not critical)

## Testing strategy

### Unit tests (in-place)

The pure-C modules (fusion, protocols, timing, persistence logic) are exercised by host-side unit tests (`ctest --preset Test`). There is no CI: run them before committing.

**Not covered**: the FreeRTOS tasks themselves, hardware drivers (SPI, I2C, CAN, UART, flash programming), and real sensor data.

### Simulation

The desktop dashboard includes a simulated CAN source (`tools/dashboard/src/gps_dashboard/can_sources/simulate.py`) that drives a synthetic car round a circuit and emits the node's CAN messages, for exercising the dashboard without hardware. It re-implements the lap logic in Python — it does **not** run or validate the firmware's timing code.

### Hardware validation (pending)

Once real hardware is available:
1. Set gates on a known test track (e.g., a parking lot with measured corners)
2. Drive a lap at constant speed and compare to a handheld timing device
3. Verify gate persistence (power-cycle + move the car; gates should be in the same place)
4. Stress test (long sessions, GPS dropouts, extreme temperatures)

## Performance profiling

### CPU load

Reported live in `GPS_Status.cpu_load_pct`, from FreeRTOS run-time stats clocked by the TIM3 microsecond tick. **Not yet measured on hardware.**

### Memory usage

```bash
arm-none-eabi-size -A build/Release/gps-firmware.elf
```

**v1.0.0**: Release 65.6 KB flash (52 % of 124 KB), 17.9 KB RAM (80 % of 22 KB). Debug 109.9 KB flash (87 %) — the Debug build is the tighter constraint. Stack sizes come from `-fstack-usage`; see `SUFST/Inc/sys/app.h`.

### CAN bandwidth

About 325 frames/s (two 100 Hz IMU frames, 50 Hz attitude, 20 Hz position/velocity, the rest slower) — roughly 4 % of a 1 Mbit/s bus.

## Common gotchas for future maintainers

### Changing the GPS update rate

The rate is `GPS_NAV_RATE_HZ` in `board_config.h`; `gps_config.c` derives the receiver's measurement period from it. Above 20 Hz the F9P needs fewer constellations than GPS + Galileo. The KF history ring (`KF6_HISTORY_LEN`, 16 steps ≈ 150 ms) must still cover the fix latency, and `NMEA_OUT_RATE_MAX_HZ` follows the fix rate automatically.

### Changing gate crossing sensitivity

Gates are 2 m half-width (4 m total), smaller than the track width (Formula Student tracks are ~3–4 m wide). This is deliberate (prevents false triggers at hairpins).

If you widen the gates, be aware:
- Wider gates = higher false-positive risk in tight corners (track sections can be <4 m apart at hairpins)
- Narrower gates = harder to cross (driver must be more precise)

Current setting (2 m) is a good balance.

### Adding new CAN messages

1. Update `tools/GPS.dbc` (message definition + signals)
2. Add encode/decode functions in `can_pack.c` (follow existing pattern: explicit byte reads, saturation helpers)
3. Add the message to `canbc_task.c` rota (schedule TX frequency)
4. Update the CAN matrix in [DATASHEET.md](DATASHEET.md)
5. Test with `tests/test_can_pack.c` (synthetic encode/decode roundtrip)

### Debugging CAN issues

Use `candump` to monitor raw traffic:
```bash
candump can0 -a  # absolute timestamps
```

Decode with Python:
```python
import cantools
db = cantools.database.load_file('tools/GPS.dbc')
msg = db.decode_message('GPS_Position', data_bytes)
print(msg)
```

The desktop dashboard also decodes all messages in real-time (useful for visual verification).

## References

- **Mahony AHRS**: R. E. Mahony, T. Hamel, and J.-M. Pflimlin. "Multirotor Aerial Vehicles: Modeling, Estimation, and Control of Quadrotors." IEEE Robotics & Automation Magazine, 2012.
- **GPS time-of-week**: u-blox UBX-22008968 (UBX Interface Description)
- **Geodetic conversions**: Zhu, J. "Conversion of Earth-centered Earth-fixed coordinates to geodetic coordinates." IEEE Transactions on Aerospace and Electronic Systems, 1993.
- **FreeRTOS static allocation**: FreeRTOS kernel reference manual (Task creation without `xTaskCreate`)

---

**Questions?** Ask the team or file an issue in the project repository.
