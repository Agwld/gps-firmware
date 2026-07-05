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

**Effort to implement**: ~1–2 weeks (write synthetic track generator, replay harness, comparison logic, documentation).

### Limited RTK support

**What**: The ZED-F9P is RTK-capable, and RTCM correction forwarding is wired up (aux_task), but no external correction source is connected to the node.

**Why missing**: Requires correction service (Trimble, Swift Navigation, etc.) subscription and antenna. Not a STAG 12 requirement.

**Impact**: None (standalone GPS is accurate enough for lap timing; RTK would give ~2 cm accuracy instead of ~2–3 m).

**Upgrade path**: Patch aux_task to accept RTCM via CAN (from a telemetry uplink) or direct UART (if a correction receiver is added).

### No wheel-speed aiding integration

**What**: The 6-state KF is designed to accept wheel-speed corrections (kf6_correct_speed), but the CAN input (0x251, Wheel_Speeds from the VCU) is parsed but not wired.

**Why missing**: VCU integration and validation pending (depends on VCU CAN availability).

**Impact**: None in practice (IMU accel integration keeps velocity coherent enough between GPS fixes), but would improve velocity estimate during GPS dropouts (tunnels, overpasses).

**Effort to enable**: ~30 min (one function call in imu_task's GPS correction loop).

### Gate timing jitter at very low speeds

**What**: If the car crosses a gate very slowly (parking, sharp hairpin), the interpolated crossing time can have larger noise due to coarse sample resolution.

**Why**: At 104 Hz, the sample interval is 9.6 ms. At 1 m/s, that's ~10 cm per sample. Linear interpolation through such coarse samples introduces quantization noise.

**Impact**: Negligible in practice (gates are meant for track sections at speed; parking is not a use case).

**Mitigation**: Could use higher-order interpolation (quadratic fit) or adaptive sample rate, but not worth the complexity.

### Flash store compaction is blocking

**What**: `flash_store_erase_and_compact()` is a blocking operation that takes ~100 ms (flash erase + rewrite).

**Why**: Deliberate (atomic write, no corruption risk), but breaks real-time guarantees.

**Impact**: Timing task will stall for ~100 ms during compaction. Compaction only runs when requested (CAN_CMD_CONFIG_SAVE) and after the car is judged stationary, so in practice it doesn't affect racing.

**Mitigation**: Could be refactored to split into chunks and interleave with other tasks, but adds complexity. Current approach is safe and sufficient.

## Future enhancements

### Immediate (next 1–2 sprints)

- [ ] **Field testing on real hardware** — Validate timing accuracy, lap detection, gate persistence against a known-good timing reference (MoTeC, StopWatch, etc.)
- [ ] **Dashboard rendering on physical in-car display** — Integrate the gate/track rendering (currently only on desktop dashboard) into the physical STAG 12 display unit
- [ ] **VCU wheel-speed aiding** — Wire up the 0x251 Wheel_Speeds message to kf6_correct_speed() for improved velocity during GPS dropout

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

All testable logic (fusion, protocols, timing, persistence) is exercised by host-side unit tests. These pass and are run on every commit.

**Coverage**: ~90% of critical paths. Not covered: FreeRTOS task scheduling, hardware drivers (SPI, CAN, UARTs), sensor samples (physical IMU + GPS data is not synthetic).

### Simulation

The desktop dashboard includes a physics simulator (`tools/dashboard/sim`) that renders synthetic car motion through a realistic circuit and validates lap timing in isolation. Used for track setup testing and driver training.

**Validity**: Simulator uses realistic car dynamics (acceleration, turning, speed profiles) but synthetic GPS data (injected at 20 Hz with configurable noise). Timing logic is the same as on-vehicle.

### Hardware validation (pending)

Once real hardware is available:
1. Set gates on a known test track (e.g., a parking lot with measured corners)
2. Drive a lap at constant speed and compare to a handheld timing device
3. Verify gate persistence (power-cycle + move the car; gates should be in the same place)
4. Stress test (long sessions, GPS dropouts, extreme temperatures)

## Performance profiling

### CPU load

Measured via FreeRTOS `configGENERATE_RUN_TIME_STATS` using TIM3 (already free-running for PPS). See [DEVELOPER.md](DEVELOPER.md) for breakdown.

**Current**: ~32% load, leaving 68% idle. Could handle additional tasks (e.g., SD logging, advanced sensor processing) without issues.

### Memory usage

```bash
arm-none-eabi-size -A build/gps_firmware.elf
```

**Current**: ~58 KB text + RO data (45% of 128 KB flash). Each task stack is <2 KB (average utilization ~60%). Could fit additional features or more aggressive optimization.

### CAN bandwidth

Current TX load: ~1.5 KB/s aggregate (mostly from 100 Hz IMU broadcast). CAN bus at 1 Mbps can handle up to ~100 KB/s without congestion. Plenty of headroom.

## Common gotchas for future maintainers

### Changing the GPS update rate

The KF is designed for 20 Hz GPS (with 16-entry history ring for delayed-state correction). If you increase the GPS rate to 25+ Hz:
1. Increase the history ring size in `kf6.c` (`KF_HISTORY_SIZE`)
2. Adjust the KF gain matrices (they're calibrated for 20 Hz gaps)
3. Update the boot config in `gps_task.c` to request the new rate

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
