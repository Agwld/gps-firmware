# Developer Guide

Building, testing, flashing and changing the GPS firmware.

## Development environment

### Required tools

| Tool | Version | Purpose |
|------|---------|---------|
| Arm GNU toolchain (`arm-none-eabi-gcc`) | 13.x (13.3 used for v1.0.0) | Cross-compiler |
| CMake | ≥ 3.22 | Build system (uses presets) |
| Ninja | any recent | Generator used by the presets |
| A host C compiler (GCC/MinGW or Clang) | any C11 | Host unit tests |

Optional: STM32CubeProgrammer or `st-flash` (flashing), OpenOCD + `arm-none-eabi-gdb` (debugging), `clang-format` (config in `.clang-format`), [uv](https://docs.astral.sh/uv/) (the dashboard, see [its README](../tools/dashboard/README.md)).

### Toolchain on PATH

The presets expect `arm-none-eabi-gcc` on `PATH`.

- **Linux**: `sudo apt-get install gcc-arm-none-eabi cmake ninja-build`
- **macOS**: `brew install --cask gcc-arm-embedded && brew install cmake ninja`
- **Windows**: install the Arm GNU toolchain, or reuse the one bundled with STM32CubeIDE — add its `…/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.<version>/tools/bin` directory to `PATH`.

## Building

```bash
cmake --list-presets          # Debug, Release, Test

cmake --preset Release
cmake --build --preset Release
```

Outputs land in `build/<Preset>/`:

```
build/Release/
├── gps-firmware.elf   # ELF (for the debugger)
├── gps-firmware.hex   # Intel HEX
├── gps-firmware.bin   # Raw image, flashed at 0x08000000
└── gps-firmware.map   # Linker map
```

The build prints the memory use at the end. v1.0.0: Release 65.6 KB / 124 KB
flash, Debug 109.9 KB — Debug is the one that will run out first.

### Configuration

There are no build-time options beyond the build type, except
`-DCAN_BITRATE_500K` for a 500 kbit/s bus. Everything else is a constant in
[`SUFST/Inc/board/board_config.h`](../SUFST/Inc/board/board_config.h): GPS rate,
gate width, button timings, minimum lap time, NMEA rate, wheel-speed aiding
(`KF_WHEELSPEED_AIDING`, off), fix staleness, and the IMU/magnetometer mounting
matrices. The flash layout is fixed to the STM32G431CB.

### Version

The firmware version is `project(... VERSION x.y.z)` in `CMakeLists.txt`. It
is compiled into the image as `SUFST gps-firmware vx.y.z`:

```bash
strings build/Release/gps-firmware.bin | grep gps-firmware
```

To release: bump the version, add a [CHANGELOG.md](../CHANGELOG.md) entry,
rebuild both presets and run the tests.

## Testing

### Host unit tests

```bash
cmake --preset Test
cmake --build --preset Test
ctest --preset Test
```

Each `tests/test_<module>.c` is its own executable, discovered automatically.
Its firmware sources are listed in `tests/test_<module>.deps`, one per line,
relative to `SUFST/Src` (e.g. `laptimer/laptimer.c`). Tests are plain C: they
print failures and return non-zero, no framework.

| Test | Covers |
|------|--------|
| `test_ubx` | UBX parser (sync, checksum, truncation), frame builder, decoders |
| `test_can_pack` | Pack/unpack of every CAN message, scaling and saturation |
| `test_ahrs` | Mahony AHRS convergence and conventions |
| `test_kf6` | KF predict/correct, delayed correction, outlier rejection, divergence recovery, speed aiding |
| `test_geodesy` | ENU ↔ lat/lon |
| `test_timebase` | PPS-disciplined tick ↔ iTOW mapping, week rollover |
| `test_gates` | Crossing detection, direction, half-width |
| `test_laptimer` | Lap/sector times on synthetic laps, events, once-per-lap sectors, resets |
| `test_mag_cal` | Batch and continuous magnetometer calibration |
| `test_nmea_out` | GGA/RMC/VTG formatting against hand-computed sentences |
| `test_flash_store` | Records, CRC, restore order, clear-all, A/B page selection after an interrupted compaction |

Not covered by tests: the FreeRTOS tasks and anything touching hardware.

### Dashboard tests

```bash
cd tools/dashboard
uv run pytest
```

## Flashing

```bash
# STM32CubeProgrammer CLI
STM32_Programmer_CLI -c port=SWD -w build/Release/gps-firmware.hex -v -rst

# or stlink-tools (raw binary needs the address)
st-flash write build/Release/gps-firmware.bin 0x08000000
```

Flashing does not touch the gate store (the top 4 KB) unless you mass-erase.
A full chip erase clears saved gates and the magnetometer calibration.

## Bring-up checklist

After flashing a board for the first time:

1. **LEDs** — the error LED (PA10) should stay off (it lights on any GPS/IMU/mag fault flag; the GPS one can take ~5 s to appear if the receiver never answers); the user LED (PB15) comes on once there is a GPS fix and the IMU is running.
2. **CAN** — connect the dashboard or `candump`; `GPS_Status` (0x6B9) at 1 Hz with uptime counting. `fault_bits` should show IMU ready + mag ready, and no faults.
3. **IMU** — `GPS_IMU_Accel` z ≈ +1000 mg with the board flat. `GPS_Attitude.fusion_status` bit0 set. If the loop is running off its fallback timer instead of the data-ready interrupt, attitude still updates but at ~50 Hz — check with a logic analyser on PC15 (expect 104 Hz).
4. **GPS** — `GPS_Time` flags go valid, then `fault_bits` bit3 (time locked), bit0 (fix) and bit4 (origin set). `gps_retry_count` should stay at 0 on a healthy link.
5. **Temperatures** — `GPS_Temp`: three plausible values.
6. **Gates** — place one with the lap button, power-cycle, confirm it reappears after the first fix (`GPS_Gate`, valid flag).

## Debugging

### GDB

```bash
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg
arm-none-eabi-gdb build/Debug/gps-firmware.elf
(gdb) target extended-remote :3333
(gdb) load
(gdb) monitor reset halt
(gdb) continue
```

Note the watchdog (~1.25 s) keeps running while halted unless frozen:
`(gdb) monitor mww 0xE0042008 0x00001000` sets `DBGMCU_APB1FZR1.DBG_IWDG_STOP`.

### Useful places to look

- `app_get_events()` — the `SYS_EVT_*` ready/fault bits (also on CAN).
- `uxTaskGetStackHighWaterMark()` per task — check headroom on hardware.
- A reset loop with no fault LED is usually the watchdog or a stack overflow
  (`vApplicationStackOverflowHook` resets immediately).
- The fault handlers (`HardFault_Handler` etc.) spin, and the watchdog then resets.

USART1 (PB6/PB7, 115200) is initialised but nothing is printed on it by default.

### CAN inspection

```bash
candump -ta can0
python -c "import cantools; db = cantools.database.load_file('tools/GPS.dbc'); print(db.get_message_by_name('GPS_Status'))"
```

The dashboard decodes everything live.

## Code style & conventions

- C11, 4-space indent, `.clang-format` in the root.
- snake_case, `UPPER_CASE` macros, `module_function()` prefixes, `s_` for file statics, `g_` for globals, `k_` for constant tables.
- No heap: everything static. One owner per piece of state (see [ARCHITECTURE.md](ARCHITECTURE.md#tasks)).
- Comments explain *why*.
- Project sources build with `-Wall -Wextra -Wdouble-promotion` and should stay warning-free. Float literals need an `f` suffix — doubles are software-emulated on this core.

### Commit messages

```
<subsystem>: <short summary under 50 chars>

<why, wrapped at 72 chars>
```

e.g. `canbus: use front-left wheel speed only`.

## Repository layout

```
.
├── README.md, CHANGELOG.md
├── CMakeLists.txt, CMakePresets.json, cmake/   # build
├── STM32G431XB_FLASH.ld                        # linker script (reserves the gate store)
├── Core/                  # MCU glue: clocks, peripheral init, interrupts, FreeRTOS config
├── SUFST/
│   ├── Inc/<module>/*.h   # + Inc/board/board_config.h (tunables)
│   └── Src/
│       ├── canbus/        # CAN packing, broadcast state, can_task
│       ├── fusion/        # AHRS, KF, geodesy, time base
│       ├── gps/           # UBX, F9P configuration over I2C, gps_task
│       ├── imu/           # LSM6DSO32/IIS2MDC driver, mag calibration, imu_task
│       ├── laptimer/      # gates, lap state machine
│       ├── nmea/          # NMEA formatting, aux_task
│       ├── persist/       # flash store
│       └── sys/           # app.c (task/queue creation), sys_task
├── Drivers/, Middlewares/ # STM32 HAL, CMSIS, FreeRTOS (vendor code)
├── tests/                 # host unit tests
├── tools/GPS.dbc          # CAN database (authoritative)
├── tools/dashboard/       # PyQt6 reference dashboard + CAN simulator
└── docs/
```

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `arm-none-eabi-gcc: command not found` | Toolchain not on `PATH` |
| `undefined reference to '__errno'` | Already handled by `Core/Src/syscalls_errno.c` and `-fno-math-errno`; check neither was removed |
| Debug build overflows flash | It is at 87 %: build Release, or trim |
| Host tests won't link a module | Add its source to `tests/test_<module>.deps` |
| Board resets every ~1.25 s | Watchdog: something blocks `sys_task` (lowest priority) |
| No CAN traffic | Transceiver power, 120 Ω termination at both bus ends, bit rate |
| GPS fault flag set | F9P not answering on I2C (address 0x42), or no UBX on USART3 — check JP/NT1 and the MicroMod board |
