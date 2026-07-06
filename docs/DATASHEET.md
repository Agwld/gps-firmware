# Datasheet — GPS Mainboard Hardware & Firmware Specifications

Complete technical specifications for the STAG 12 GPS mainboard and firmware.

## Hardware specifications

### Microcontroller

| Component | Spec |
|-----------|------|
| Part | STM32G431CB (128 KB flash, 32 KB SRAM) or C8 (64 KB flash) variant |
| Clock | 170 MHz internal RC oscillator (calibrated) |
| Flash | 126 KB usable (last 2 KB reserved for gate/mag-cal storage) |
| SRAM | 32 KB total (16 KB for task stacks + queues, 16 KB for buffers) |
| Floating-point | Cortex-M4F with FPU (single-precision, hardware accelerated) |
| HSE | 25 Mhz Crystal Oscillator |
| RTC | 32 kHz LSI (low-speed internal oscillator) for watchdog |
| Watchdog | Independent Watchdog (IWDG) with 5 s timeout |
| **Package** | 48-pin LQFP |

### GNSS receiver

| Component | Spec |
|-----------|------|
| Part | u-blox ZED-F9P (multi-constellation GNSS) |
| Form factor | SparkFun MicroMod function board (M.2 slot on mainboard) |
| Constellations | GPS + Galileo (other constellations available via config) |
| Update rate | 20 Hz (5 Hz optional; 25 Hz supported with harsher cuts) |
| Time-to-first-fix | ~30 s cold start, ~1–5 s warm start |
| **Accuracy** (standalone) | — |
| Horizontal (CEP50) | 2.5 m RMS typical |
| Vertical (RMS) | 4 m RMS typical |
| **Time accuracy** (PPS) | 20 ns RMS (disciplined 1 PPS output) |
| Message rates | NAV-PVT @ 20 Hz, TIM-TM2 on EXTINT event |
| Power | 150 mA typical @ 3.3 V |

### Inertial measurement unit

| Component | Spec |
|-----------|------|
| Accel + Gyro | LSM6DSO32 (ST MEMS, SPI interface) |
| **Accel specs** | — |
| Ranges | ±16 g, ±8 g, ±4 g, ±2 g (firmware set to ±16 g) |
| ODR | Up to 6.66 kHz; firmware set to **104 Hz** |
| Sensitivity | ~0.488 mg/LSB @ ±16 g, 16-bit output |
| Noise | 80 mg/√Hz typical |
| **Gyro specs** | — |
| Range | ±4000 °/s (firmware set to ±2000 °/s full scale) |
| ODR | Up to 6.66 kHz; firmware set to **104 Hz** |
| Sensitivity | ~70 mdps/LSB @ ±2000 °/s, 16-bit output |
| Noise | 4.4 °/s/√Hz typical |
| **Magnetometer** | IIS2MDC via sensor-hub I2C |
| Range | ±50 mGauss |
| ODR | Up to 100 Hz; firmware set to 10 Hz |
| Interface | I2C behind LSM6DSO32 sensor-hub master (MCU never talks to mag directly) |

### Temperature sensor

| Component | Spec |
|-----------|------|
| Part | MicroChip MCP9800 (I2C, ±5 °C accuracy) |
| I2C address | 0x4D (A5 variant, fixed address) |
| Resolution | 0.0625 °C (9-bit mode) |
| Conversion time | 240 ms |
| Range | −55 to +125 °C |
| Mounted | On IMU bus (measures board temperature) |

### CAN interface

| Spec | Value |
|------|-------|
| Transceiver | TI TCAN3404 |
| Bus | FDCAN1 on STM32 (CAN-FD capable, running classic CAN mode) |
| Bit rate | 1 Mbps (nominal), fallback 500 kbps if auto-negotiation needed |
| Termination | 120 Ω resistor on both ends of the bus (external to this board) |

### Power

| Spec | Value |
|------|-------|
| Supply voltage | +12 V nominal (10–16 V tolerance, car battery) |
| Quiescent draw | ~200 mA (GNSS + IMU active) |
| Peak draw | ~250 mA (SPI/CAN peaks) |

### UART interfaces

| UART | Use | Pins | Baud | Notes |
|------|-----|------|------|-------|
| USART3 RX | GPS UBX stream in | PB11 | 115200 | F9P UART1 TX hard-wired via net-tie NT1; F9P switched from its 38400 default over I2C at boot |
| USART3 TX | NMEA to MoTeC | PB10 | 115200 | Via solder jumper JP6 (2‑3) → MAX3232 → RS232 out. Shares USART3's single baud register with RX |
| USART1 | Debug output | PB6 TX, PB7 RX | 115200 | Not used in stock firmware; reserved for debugging |
| USART2 | Unused | PA2, PA15 | — | PA2 is unrouted on the board; PA15 only reaches JP7, which is bridged 2‑3 (RTCM → GPS direct) |

**GPS configuration goes over I2C, not UART**: the board has no MCU→GPS UART path, so all UBX-CFG-VALSET traffic rides I2C2 to the F9P's DDC port (address 0x42). The high-rate UBX stream still arrives on USART3 RX.

### UART direction solder jumpers

| Jumper | Setting | Effect |
|--------|---------|--------|
| NT1 | fixed (net tie) | F9P UART1 TX → USART3 RX, always |
| **JP6** | **bridge 2‑3** | MCU NMEA (USART3 TX) → RS232 out → MoTeC. *Do not bridge 1‑2*: the F9P's UART1 carries binary UBX in this firmware, which would feed garbage to the MoTeC |
| **JP7** | **bridge 2‑3** | RS232 in → F9P UART1 RX directly — the RTK/RTCM correction path, no MCU involvement. (1‑2 routes RS232 in to PA15 instead; unsupported by stock firmware) |

### GPIO & external interfaces

| Function | Pin | Notes |
|----------|-----|-------|
| **Time sync** | — | — |
| GPS PPS input | PA7 (TIM3_CH2) | Rising edge capture, fed to timebase.c for tick ↔ iTOW mapping |
| **Steering wheel** | — | — |
| Lap button | PC13 | Active high, debounced in sys_task (20 ms hysteresis) |
| **Status signals** | — | — |
| Status LED | PA5 | GPIO output, indicates system state |
| Fault LED | PC14 | GPIO output, indicates error condition |


## Firmware specifications

### Build variants

| Variant | Flash | SRAM | Release size | Debug size | Notes |
|---------|-------|------|--------------|------------|-------|
| **CB (default)** | 128 KB | 32 KB | ~58 KB (45%) | ~94 KB (73%) | Full feature set, recommended |
| **C8** | 64 KB | 32 KB | ~58 KB (92%) | N/A | Minimal feature set, Debug doesn't fit |

Selected at compile time:
```bash
cmake --preset Release -DGPS_MCU_VARIANT=CB   # or C8
```

### Timing accuracy

| Metric | Spec |
|--------|------|
| Position update rate | 104 Hz (IMU-based estimate) |
| Gate crossing detection resolution | ~1 ms (linear interpolation between samples) |
| Lap time precision | ±1 ms (depends on GPS fix latency, typically 20–60 ms post-facto) |
| Time-stamping | GPS time-of-week (iTOW) via PPS-disciplined tick counter |
| Synchronization accuracy | ±20 ns (GPS PPS clock) |

### Real-time performance

| Task | Rate | CPU load | Max latency | Stack |
|------|------|----------|-------------|-------|
| `imu_task` | 104 Hz | ~18% | 9.6 ms | 2 KB |
| `gps_task` | variable | ~3% | <50 ms | 2 KB |
| `can_task` | ~100 Hz | ~8% | <10 ms | 1.5 KB |
| `aux_task` | ~1 Hz | ~1% | <1 s | 1.5 KB |
| `sys_task` | 10 Hz | ~2% | <100 ms | 1.5 KB |
| **Total** | — | **~32%** | — | — |
| **Idle** | — | **~68%** | — | — |

Measured via FreeRTOS `ulTaskGetIdleRunTimePercent()` on the 170 MHz STM32G431.

### Memory usage (CB variant)

| Region | Used | Available | % |
|--------|------|-----------|---|
| Flash (text + RO data) | 57 KB | 126 KB | 45% |
| SRAM (stacks + queues + buffers) | ~24 KB | 32 KB | 75% |
| Flash store (gates + mag-cal) | 0–2 KB | 2 KB | 0–100% |

Stack sizes are conservative; each task could be reduced by ~500 bytes if needed (currently using <60% of allocated stack per task).

### CAN message matrix (ID 0x6B0–0x6BF)

All little-endian Intel-style packing per SUFST convention. See [tools/GPS.dbc](../tools/GPS.dbc) for authoritative definitions.

#### TX messages (broadcasts from node)

| ID | Name | Period | Signals | Bytes |
|----|------|--------|---------|-------|
| 0x6B0 | GPS_Position | 50 ms (20 Hz) | lat_deg, lon_deg | 8 |
| 0x6B1 | GPS_Velocity | 50 ms (20 Hz) | speed_mps, course_deg, altitude_m, fix_ok | 8 |
| 0x6B2 | GPS_Attitude | 20 ms (50 Hz) | yaw_deg, pitch_deg, roll_deg, ahrs_ok | 8 |
| 0x6B3 | Lap_Status | 100 ms (10 Hz) | lap_num, running_time_ms, sector_num, flags | 8 |
| 0x6B4 | Lap_Event | event | event_type, gps_time_ms, button_count | 8 |
| 0x6B5 | GPS_Quality | 200 ms (5 Hz) | hdop, vdop, fix_type, rtk_status | 8 |
| 0x6B6 | GPS_IMU_Accel | 10 ms (100 Hz) | accel_x_g, accel_y_g, accel_z_g | 7 |
| 0x6B7 | GPS_IMU_Gyro | 10 ms (100 Hz) | gyro_x_dps, gyro_y_dps, gyro_z_dps | 7 |
| 0x6B8 | GPS_Temp | 1000 ms (1 Hz) | board_temp_c, imu_temp_c, mcu_temp_c | 6 |
| 0x6B9 | GPS_Status | 1000 ms (1 Hz) | uptime_s, fault_bits, cpu_load_pct | 8 |
| 0x6BA | GPS_Mag | 100 ms (10 Hz) | mag_x_mgauss, mag_y_mgauss, mag_z_mgauss, cal_status | 8 |
| 0x6BB | GPS_Frame_Origin | 1000 ms (1 Hz) | origin_lat_deg, origin_lon_deg, origin_valid | 8 |
| 0x6BC | GPS_Gate | ~200 ms (5 Hz agg) | gate_index, gate_flags, east_m, north_m, heading_deg | 8 |
| 0x6BD | GPS_Time | 1000 ms (1 Hz) | itow_ms, utc_hour, utc_min, utc_sec, time_flags | 8 |

`GPS_Time` is the bus-wide time reference: `itow_ms` is GPS time-of-week in the same time domain as `Lap_Event` timestamps (for aligning other nodes' data), and the UTC fields drive a human-readable clock. UTC date is not broadcast (the MoTeC datalogger receives it via NMEA RMC). Note GPS time leads UTC by the current leap-second count, so the two fields are not redundant.

#### RX messages (commands to node)

| ID | Name | Signals | Purpose |
|----|------|---------|---------|
| 0x6BF | GPS_Command | cmd_id, arg0, arg1 | Gate set/clear, mag-cal, config commands |

Command codes (in GPS_Command.cmd_id):
```
0x01 = CAN_CMD_GATE_SET           (set gate at slot arg0 via CAN)
0x02 = CAN_CMD_GATE_CLEAR         (clear gate at slot arg0)
0x20 = CAN_CMD_CONFIG_SAVE        (trigger flash compaction)
0xFF = CAN_CMD_GATE_CLEAR_ALL     (clear all gates)
0x10 = CAN_CMD_MAG_CAL_START      (force a fresh mag recalibration; cal is
                                   otherwise continuous/automatic)
0x11 = CAN_CMD_MAG_CAL_STOP       (no-op, retained for compatibility)
```

#### Signal scaling

| Signal | Format | Scale | Offset | Range |
|--------|--------|-------|--------|-------|
| lat_deg, lon_deg | int32_t | 1e-7 | 0 | ±90 / ±180 |
| speed_mps | uint16_t | 0.01 | 0 | 0–655 m/s |
| course_deg | uint16_t | 0.01 | 0 | 0–360° |
| altitude_m | int16_t | 1 | 0 | ±32768 m |
| yaw/pitch/roll_deg | int16_t | 0.01 | 0 | ±327.68° |
| hdop, vdop | uint8_t | 0.1 | 0 | 0–25.5 |
| accel_*_g | int16_t | 0.001 | 0 | ±32.768 g |
| gyro_*_dps | int16_t | 0.1 | 0 | ±3276.8 °/s |
| temp_c | int8_t | 1 | 0 | −128 to +127 °C |
| gate_east/north_m | int16_t | 0.1 | 0 | ±3276.7 m |
| gate_heading_deg | uint16_t | 0.1 | 0 | 0–6553.5° |
| cpu_load_pct | uint8_t | 1 | 0 | 0–100 % |
| itow_ms | uint32_t | 1 | 0 | 0–604,799,999 ms |
| utc_hour/min/sec | uint8_t | 1 | 0 | 0–23 / 0–59 / 0–60 |

### Flash storage

| Region | Address | Size | Purpose |
|--------|---------|------|---------|
| Firmware (text) | 0x08000000 | ~58 KB | STM32 application |
| Firmware (RO data) | 0x0800E800 | varies | Constants, lookup tables |
| Reserved gap | varies | — | Linker script gap |
| **Gate store** | **0x1FC00** (CB) or **0x0FC00** (C8) | **2 KB** | Append-only gate + mag-cal log |
| **Blank flash** | varies | remainder | Available for future use |

Gate store records:
```
Layout: [kind(1)] [key(1)] [gate_valid(1)] [reserved(1)] [payload(12)] [heading(4)] [crc(4)] = 24 bytes
Max records: ~85 per page before compaction needed
Clear marker: kind=GATE_CLEAR_ALL wipes all gates
```



## Firmware feature matrix

| Feature | Status | Notes |
|---------|--------|-------|
| **Core timing** | ✓ Complete | Gate crossing detection, lap/sector times |
| **Sensor fusion** | ✓ Complete | Mahony AHRS + 6-state Kalman filter |
| **Gate persistence** | ✓ Complete | Absolute lat/lon, survives power-cycle-and-move |
| **CAN broadcast** | ✓ Complete | 15 message types, 1 Hz to 100 Hz sampling |
| **Magnetic calibration** | ✓ Complete | Continuous/automatic; hard-iron + soft-iron (horizontal), GPS-course validated |
| **NMEA synthesis** | ✓ Complete | MoTeC datalogger compatibility |
| **Event time-marking** | ✓ Complete | GPS TIM-TM2 for button edge stamping |
| **Wheel-speed aiding** | ✓ Designed | Code in place; needs VCU integration (not in critical path) |
| **RTK support** | ⚠ Capable | RTCM corrections flow RS232-in → F9P UART1 in hardware (JP7 2‑3); firmware enables RTCM3X input at boot. Needs an external correction source on the loom |
| **SIL harness** | ❌ Pending | Synthetic track replay for validation testing |

✓ = production-ready | ⚠ = partially implemented | ❌ = not started

## Compliance & standards

- **CAN**: ISO 11898-1 (CAN 2.0B)
- **GNSS**: GPS L1, Galileo E1 (u-blox standard implementations)
- **Time**: GPS time-of-week per u-blox UBX spec
- **IMU**: LGA package, MEMS standard operating conditions

---

**For complete schematic & layout, see:** [`gps-mainboard.pdf`](../gps-mainboard.pdf) (KiCad project at `/pcb/gps-mainboard/`)
