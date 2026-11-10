# Datasheet — GPS Mainboard Hardware & Firmware Specifications

Technical reference for the STAG 12 GPS mainboard and firmware v1.0.0.
Where this page and the code disagree, the code wins: pins are in
[`Core/Inc/main.h`](../Core/Inc/main.h), rates and constants in
[`SUFST/Inc/board/board_config.h`](../SUFST/Inc/board/board_config.h), and the
CAN interface in [`tools/GPS.dbc`](../tools/GPS.dbc) /
[`SUFST/Inc/canbus/can_defs.h`](../SUFST/Inc/canbus/can_defs.h).

## Hardware specifications

### Microcontroller

| Item | Spec |
|------|------|
| Part | STM32G431CBT (128 KB flash, 32 KB SRAM, 48-pin LQFP) — required |
| Core | Cortex-M4F, single-precision FPU (double is software-emulated) |
| Clock | 25 MHz HSE crystal → PLL → 170 MHz SYSCLK (boost mode); APB1/APB2 at 170 MHz |
| Flash | 124 KB for code; the top 4 KB (two 2 KB pages, 0x0801F000–0x0801FFFF) hold the gate/mag-cal store |
| SRAM | 22 KB main SRAM (SRAM1 + SRAM2) + 10 KB CCM RAM |
| Watchdog | IWDG on the 32 kHz LSI, ~1.25 s timeout, started before the scheduler and refreshed by `sys_task` every 20 ms |

### GNSS receiver

| Item | Spec |
|------|------|
| Part | u-blox ZED-F9P on a SparkFun MicroMod GNSS function board |
| Configuration | Over I2C (DDC port, address 0x42) at every boot, RAM layer only; re-sent automatically if the NAV-PVT stream stops |
| Constellations | GPS + Galileo (GLONASS, BeiDou, QZSS disabled — needed for 20 Hz) |
| Navigation rate | 20 Hz (`GPS_NAV_RATE_HZ`) |
| Dynamic model | Automotive |
| Messages used | UBX-NAV-PVT every epoch, UBX-TIM-TM2 on an EXTINT edge |
| Data link | F9P UART1 TX → MCU USART3 RX, 115200 baud, UBX only |
| Time pulse | 1 PPS → PA7 (TIM3_CH2 input capture) |
| RTK | Receiver accepts RTCM3 on UART1 RX, wired straight from the RS232 input (JP7 2‑3); needs an external correction source |

Accuracy and time-to-first-fix are as specified in the u-blox ZED-F9P datasheet.

### Inertial sensors

| Item | Spec |
|------|------|
| Accel + gyro | ST LSM6DSO32 on SPI1 (mode 3, ~5.3 MHz, DMA) |
| Accel | ±16 g full scale (part supports ±4/8/16/32 g), 0.488 mg/LSB, 104 Hz ODR |
| Gyro | ±2000 dps full scale, 70 mdps/LSB, 104 Hz ODR |
| Data-ready | Gyro DRDY on INT1 → PC15 (EXTI, rising edge) paces the fusion loop |
| Magnetometer | ST IIS2MDC (±50 gauss, 1.5 mG/LSB) on the LSM6DSO32's sensor-hub I2C master; continuous mode, 100 Hz; the MCU never talks to it directly |
| Mounting | Fixed in `IMU_MOUNT_MATRIX` / `MAG_MOUNT_MATRIX` (`board_config.h`) — the two parts are mounted differently |

### Temperature

| Source | Spec |
|--------|------|
| Board ambient | Microchip MCP9800A5 on I2C2, address 0x4D, power-up 9-bit mode (0.5 °C/LSB), read at 1 Hz |
| IMU die | LSM6DSO32 internal sensor, read at 1 Hz |
| MCU die | STM32G431 internal sensor via ADC1 and the factory calibration, whole degrees, read at 1 Hz |

### CAN interface

| Item | Value |
|------|-------|
| Controller | FDCAN1 in classic CAN mode, PA11 RX / PA12 TX |
| Bit rate | 1 Mbit/s, 88 % sample point (500 kbit/s by building with `-DCAN_BITRATE_500K`) |
| Identifiers | 11-bit standard; node transmits 0x6B0–0x6BD, receives 0x6BF and 0x251 |
| Bus-off | Recovered automatically (checked once a second) |
| Termination | Not on this board |

### UART interfaces

| UART | Use | Pins | Baud | Notes |
|------|-----|------|------|-------|
| USART3 RX | GPS UBX stream in | PB11 | 115200 | F9P UART1 TX via net tie NT1; circular DMA |
| USART3 TX | NMEA out to MoTeC | PB10 | 115200 | Via JP6 (2‑3) → MAX3232 → RS232. Shares the baud register with RX; DMA |
| USART1 | Debug | PB6 TX, PB7 RX | 115200 | Initialised, unused by the stock firmware |
| USART2 | Unused | PA2, PA15 | — | Not initialised. PA2 unrouted; PA15 only reaches JP7 |

### Solder jumpers

| Jumper | Setting | Effect |
|--------|---------|--------|
| NT1 | fixed | F9P UART1 TX → USART3 RX |
| **JP6** | **bridge 2‑3** | MCU NMEA (USART3 TX) → RS232 out. Don't bridge 1‑2: the F9P's UART1 carries binary UBX |
| **JP7** | **bridge 2‑3** | RS232 in → F9P UART1 RX (RTCM corrections). 1‑2 routes it to PA15, which the firmware doesn't use |

### GPIO

| Function | Pin | Notes |
|----------|-----|-------|
| Lap button | PC13 | Active high, RC + Schmitt conditioned on the board; polled every 20 ms |
| User button | PC14 | Active low; configured, unused |
| User LED | PB15 | On when GPS has a fix **and** the IMU is running |
| Error LED | PA10 | On while any GPS/IMU/mag fault flag is set; solid with interrupts off = `Error_Handler()` |
| GPS PPS | PA7 | TIM3_CH2 capture, 1 MHz timebase |
| GPS EXTINT tap | PA5 | Input; the event button edge is time-stamped by the F9P (TIM-TM2) |
| GPS reset / power | PB1 (open drain) / PA1 | Held released / on |
| IMU chip select | PB9 | Active low |
| I2C2 | PA8 SDA / PA9 SCL | 100 kHz; F9P DDC, MCP9800, MicroMod EEPROM |
| SPI1 | PB3 SCK / PB4 MISO / PB5 MOSI | LSM6DSO32 |

## Firmware specifications

### Memory (v1.0.0, GCC 13.3)

| Build | Flash | RAM | CCM |
|-------|-------|-----|-----|
| Release (`-Os`) | 65.6 KB of 124 KB (52 %) | 17.9 KB of 22 KB (80 %) | 0.6 KB of 10 KB |
| Debug (`-O0 -g3`) | 109.9 KB of 124 KB (87 %) | 17.9 KB of 22 KB | 1.1 KB of 10 KB |

The Debug build is close to the flash limit; watch it when adding code.

### Tasks

All static (no heap). Stack sizes are derived from `-fstack-usage`; see the
comment in [`SUFST/Inc/sys/app.h`](../SUFST/Inc/sys/app.h) before changing them.

| Task | Priority | Rate | Stack | Role |
|------|----------|------|-------|------|
| `imu_task` | 6 | 104 Hz (IMU DRDY) | 3 KB | IMU read, AHRS, KF, gates, lap timing, flash store |
| `gps_task` | 5 | per UBX frame | 2 KB | F9P configuration, UBX parsing, PPS pairing, link supervision |
| `can_task` | 3 | 100 Hz slots | 1 KB | Broadcast rota, Lap_Event TX, command dispatch |
| `aux_task` | 2 | 10 Hz (configurable) | 1 KB | NMEA GGA/RMC/VTG to the MoTeC |
| `sys_task` | 1 | 50 Hz | 1.25 KB | Lap button, LEDs, temperatures, watchdog, status |

CPU load has not yet been measured on hardware; the node reports it live in
`GPS_Status.cpu_load_pct`.

### Timing

| Metric | Value |
|--------|-------|
| Fused state rate | 104 Hz |
| Crossing time resolution | Linear interpolation between 9.6 ms samples |
| Time base | TIM3 at 1 MHz, mapped to GPS time-of-week by PPS (rate re-measured every second) |
| Lap timing prerequisite | PPS time lock **and** an anchored ENU frame; flagged `DEGRADED` without a recent fix |

End-to-end lap-time accuracy has not yet been measured against a reference.

### CAN message matrix

Little-endian, 11-bit IDs. Rates are from the broadcast rota in
[`canbc_task.c`](../SUFST/Src/canbus/canbc_task.c).

| ID | Name | Rate | DLC | Contents |
|----|------|------|-----|----------|
| 0x6B0 | GPS_Position | 20 Hz | 8 | Fused lat, lon (i32, 1e-7°) |
| 0x6B1 | GPS_Velocity | 20 Hz | 8 | Speed (0.01 m/s), course (0.01°), altitude (0.1 m), fix type (4 bit), satellites (4 bit, saturates at 15), counter |
| 0x6B2 | GPS_Attitude | 50 Hz | 8 | Yaw (u16, 0.01°), pitch, roll (i16, 0.01°), fusion status, counter |
| 0x6B3 | Lap_Status | 10 Hz | 8 | Lap count (u16), running time (u32 ms), sector (u8), flags |
| 0x6B4 | Lap_Event | event | 8 | Type, lap (u16), time (u32 ms), counter |
| 0x6B5 | GPS_Quality | 5 Hz | 7 | hAcc (mm), sAcc (0.1 mm/s), PDOP (0.01), flags |
| 0x6B6 | GPS_IMU_Accel | 100 Hz | 7 | x, y, z (i16, 1 mg, vehicle frame), counter |
| 0x6B7 | GPS_IMU_Gyro | 100 Hz | 7 | x, y, z (i16, 0.0625 dps, vehicle frame), counter |
| 0x6B8 | GPS_Temp | 1 Hz | 6 | Board, IMU, MCU temperature (i16, 0.01 °C) |
| 0x6B9 | GPS_Status | 1 Hz | 7 | Uptime (s), status bits, GPS link retries, IMU read errors, CPU load (%) |
| 0x6BA | GPS_Mag | 10 Hz | 7 | x, y, z (i16, 0.1 µT, calibrated), calibration state |
| 0x6BB | GPS_Frame_Origin | 1 Hz | 8 | ENU origin lat, lon (i32, 1e-7°); not sent until a fix anchors it |
| 0x6BC | GPS_Gate | 5 Hz (one slot per frame) | 8 | Index, flags (bit0 valid), east, north (i16, 0.1 m), heading (u16, 0.1°) |
| 0x6BD | GPS_Time | 1 Hz | 8 | iTOW (u32 ms), UTC h/m/s, flags (bit0 UTC valid, bit1 fully resolved) |

#### Status bit fields

| Field | Bits |
|-------|------|
| `GPS_Attitude.fusion_status` | 0 IMU OK · 1 mag OK · 2 mag calibrated · 3 origin set · 4 GPS-aided (fix in the last 1 s) · 5 time locked |
| `Lap_Status.flags` | 0 running · 1 armed (start/finish placed) · 2 degraded (no recent fix or no time lock) |
| `Lap_Event.type` | 0 lap complete · 1 sector complete · 2 TM2 time mark (time = iTOW) |
| `GPS_Quality.flags` | 0 gnssFixOK · 1–2 carrier solution (0 none, 1 float, 2 fixed) · 3 differential |
| `GPS_Mag.cal_status` | 0 uncalibrated · 1 collecting · 2 calibrated · 3 validated against GPS course |
| `GPS_Status.fault_bits` | The whole status event group: 0 GPS fix · 1 IMU ready · 2 mag ready · 3 time locked · 4 origin set · 5 GPS fault (no NAV-PVT stream) · 6 IMU fault · 7 mag fault |

`GPS_Time` is the bus-wide time reference: `itow_ms` is in the same time
domain as `Lap_Event` TM2 marks. GPS time leads UTC by the leap-second count,
so the two fields are not redundant. The UTC date is not broadcast (the MoTeC
gets it via NMEA RMC).

#### Received messages

| ID | Name | Source | Use |
|----|------|--------|-----|
| 0x6BF | GPS_Command | Dash / laptop | Commands, below |
| 0x251 | Wheel_Speeds | VCU | Front-left wheel only (the other channels are unreliable); used only if `KF_WHEELSPEED_AIDING` is enabled, which it is not by default |

`GPS_Command` (byte 0 = cmd, byte 1 = arg0, byte 2 = arg1):

| cmd | Name | Arguments |
|-----|------|-----------|
| 0x01 | GATE_SET | arg0 = slot 0 (start/finish) – 7, or 0xFE = next free sector. Placed at the current fused position. Setting slot 0 clears all sectors and restarts lap timing |
| 0x02 | GATE_CLEAR | arg0 = slot, or 0xFF = all |
| 0x10 | MAG_CAL_START | Discard the magnetometer calibration and re-learn it |
| 0x11 | MAG_CAL_STOP | Accepted, no effect (calibration is continuous) |
| 0x20 | CONFIG_SAVE | Compact the flash store once the car is stationary |
| 0x30 | NMEA_CFG | arg0 = rate 1–20 Hz (0 = unchanged), arg1 = enable (0/1) |

### NMEA output

GGA, RMC and VTG (talker `GP`) at 10 Hz by default, built from the raw NAV-PVT
(not the fused state), one burst per cycle. A cycle without a fresh NAV-PVT
sends nothing; a NAV-PVT without a valid fix goes out flagged invalid (GGA
quality 0, RMC status V).

### Flash store

| Item | Value |
|------|-------|
| Location | Two 2 KB pages, 0x0801F000 and 0x0801F800 (A/B) |
| Record | 32 bytes: kind, key, valid flag, reserved, 6 × float payload, CRC32 |
| Capacity | 64 records per page |
| Kinds | 1 gate (absolute lat/lon i32 1e-7° + heading), 2 mag calibration, 3 clear-all marker, 4 compaction-complete marker (with generation) |
| Compaction | At boot when fewer than 32 slots are free; on `CONFIG_SAVE`; or when a write finds the page full — always only while stationary |

## Firmware feature status

| Feature | Status |
|---------|--------|
| Lap and sector timing | Implemented, host-tested; not yet validated on track |
| Sensor fusion (Mahony AHRS + 6-state KF) | Implemented, host-tested |
| Gate persistence across power cycles | Implemented, host-tested (store logic); not yet validated on hardware |
| CAN broadcast and commands | Implemented |
| Continuous magnetometer calibration | Implemented (horizontal hard/soft iron, validated against GPS course) |
| NMEA to MoTeC | Implemented |
| TIM-TM2 event time-marks | Implemented |
| Fault handling (GPS link, IMU, CAN bus-off, watchdog) | Implemented |
| Wheel-speed aiding (front-left) | Implemented, **disabled by default** pending validation |
| RTK | Receiver-side only; needs a correction source |

---

Schematic: [`gps-mainboard.pdf`](../gps-mainboard.pdf).
