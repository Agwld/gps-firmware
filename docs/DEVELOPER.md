# Developer Guide

Comprehensive guide to building, testing, debugging, and contributing to the GPS firmware.

## Development environment

### Required tools

| Tool | Min version | Purpose |
|------|-------------|---------|
| GCC ARM | 13.x | STM32G431 cross-compiler |
| CMake | 3.27 | Build system |
| Ninja | 1.12 | Fast parallel build |
| Git | 2.30 | Version control |
| Python | 3.12 | Dashboard, test scripts |

### Optional tools

| Tool | Purpose |
|------|---------|
| STM32CubeProgrammer | Flash & debug (GUI) |
| OpenOCD | JTAG debug server |
| arm-none-eabi-gdb | GDB debugger for ARM |
| clang-format | Code formatting (config: `.clang-format`) |
| black | Python formatter (dashboard) |

### Setup (Ubuntu/Debian)

```bash
# Install cross-compiler
sudo apt-get update
sudo apt-get install gcc-arm-none-eabi binutils-arm-none-eabi \
  gdb-arm-none-eabi build-essential cmake ninja-build

# Install STM32 tools
sudo apt-get install stlink-tools openocd

# Install Python 3.12 (if needed)
sudo apt-get install python3.12 python3.12-venv
python3.12 -m venv venv
source venv/bin/activate

# Install Python dependencies (for dashboard, scripts)
pip install cantools pyqt6 pyqtgraph python-can pytest
```

### Setup (macOS)

```bash
# Install toolchain via Homebrew
brew install arm-none-eabi-gcc cmake ninja python@3.12 stlink openocd

# Create Python venv
python3.12 -m venv venv
source venv/bin/activate
pip install -r tools/dashboard/requirements.txt
```

## Building

### CMake presets

The build system uses CMake presets for easy configuration. Available presets:

```bash
cmake --list-presets
```

#### Release build (optimized, ~58 KB)

```bash
cmake --preset Release
cmake --build --preset Release
# Output: build/gps_firmware.elf
```

#### Debug build (symbols, -O0, ~94 KB)

```bash
cmake --preset Debug
cmake --build --preset Debug
```

#### Host tests (native machine)

```bash
cmake -B build-host -DGPS_HOST_TESTS=ON -G Ninja
cmake --build build-host
ctest --test-dir build-host -V
```

### Custom build options

The board requires the **STM32G431CB** (128 KB flash); the linker script and flash layout are fixed to that part.

To set the GPS update rate (default 20 Hz):

```bash
cmake --preset Release -DGPS_NAV_RATE=25
# Note: 25 Hz is supported; >25 Hz requires harsher constellation cuts
```

### Build outputs

```
build/
├── gps_firmware.elf          # ELF binary (full debug symbols)
├── gps_firmware.bin          # Raw binary for flashing
├── gps_firmware.hex          # Intel HEX format
└── CMakeFiles/               # CMake intermediates
```

### Binary size analysis

Check the firmware size breakdown:

```bash
# Total size (text + data)
arm-none-eabi-size -A build/gps_firmware.elf

# Detailed section breakdown
arm-none-eabi-objdump -h build/gps_firmware.elf

# Symbol sizes (to find bloat)
arm-none-eabi-nm -S --size-sort build/gps_firmware.elf | tail -20
```

Current firmware (STM32G431CB, 128 KB flash):
- **Release**: ~58 KB text + data (45% of 128 KB)
- **Debug**: ~94 KB (73% of 128 KB)

## Testing

### Unit tests (host-side)

All testable code (fusion, protocols, persistence) runs on the native machine with no hardware dependencies:

```bash
cmake -B build-host -DGPS_HOST_TESTS=ON -G Ninja
cmake --build build-host
ctest --test-dir build-host -V   # Run all tests with verbose output
ctest --test-dir build-host -R <pattern>  # Run tests matching pattern
```

Current test coverage (11 tests):

| Module | Tests | Notes |
|--------|-------|-------|
| `ubx.c` | 3 | UBX protocol parsing, frame sync, checksum validation |
| `can_pack.c` | 2 | CAN message encode/decode (position, velocity, attitude, etc.) |
| `kf6.c` | 2 | Kalman filter predict/correct, delayed-state rewind |
| `ahrs.c` | 1 | AHRS quaternion update |
| `laptimer.c` | 1 | Gate crossing detection, lap timing logic |
| `flash_store.c` | 1 | Gate/mag-cal persistence, CRC validation, restore |
| `timebase.c` | 1 | GPS time ↔ tick mapping, iTOW inversion |

### Adding a new unit test

1. Create `tests/test_<module>.c`:
   ```c
   #include <assert.h>
   #include "<module>.h"
   
   static void test_feature() {
       // Arrange
       int result = function_under_test();
       
       // Assert
       assert(result == expected_value);
   }
   
   int main() {
       test_feature();
       return 0;  // 0 = pass, non-zero = fail
   }
   ```

2. Create `tests/test_<module>.deps` listing source files to compile:
   ```
   SUFST/Src/<module>.c
   ```

3. Rebuild and run:
   ```bash
   cmake --build build-host
   ctest --test-dir build-host -V
   ```

### Hardware testing

For testing on real hardware:

1. **Program the board**:
   ```bash
   st-flash write build/gps_firmware.elf 0x08000000
   ```

2. **Open a serial terminal** to the debug UART (USART1, 115200 baud):
   ```bash
   picocom /dev/ttyUSB0 -b 115200  # Linux
   screen /dev/tty.usbserial 115200  # macOS
   ```

3. **Monitor CAN traffic**:
   ```bash
   candump can0  # if the CAN interface is already up
   # Or use the dashboard tool (see below)
   ```

4. **Check GPS status**:
   - Look for UBX-NAV-PVT messages on the debug UART
   - Watch the in-car display for "FIX: OK"

## Debugging

### On-target GDB debugging

If you have an ST-LINK or similar:

```bash
# Terminal 1: Start OpenOCD server
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg

# Terminal 2: GDB client
arm-none-eabi-gdb build/gps_firmware.elf
(gdb) target remote localhost:3333
(gdb) load
(gdb) break imu_task
(gdb) continue
(gdb) step
```

### Logging and assertions

The firmware uses `assert()` liberally in host tests. On target, assertions are stripped (compile-time `NDEBUG`). For runtime debugging on hardware, add debug output to the USART1 debug port:

```c
// In sys_task.c or similar
void debug_log(const char *fmt, ...) {
    // Use HAL_UART_Transmit() to USART1
}
```

### CAN message inspection

Use `candump` or the dashboard:

```bash
# Raw CAN traffic
candump can0 -a  # absolute timestamps

# Python script to decode messages
python3 -c "
import cantools
db = cantools.database.load_file('tools/GPS.dbc')
msg = db.get_message_by_name('GPS_Position')
print(msg)
"
```

## Code style & conventions

### C code (firmware)

- **Naming**: snake_case for functions/variables, UPPER_CASE for macros, lowercase for typedefs
- **Pointers**: `int *p;` (pointer on right), not `int* p;`
- **Indentation**: 4 spaces (enforced by `.clang-format`)
- **Line length**: 80 columns preferred, 100 max
- **Comments**: Minimal (signal-to-noise). Only explain *why*, not *what*.
  ```c
  // Bad: "increment i"
  i++;
  
  // Good: "history wraps at LAP_HISTORY_SIZE (ring buffer)"
  i = (i + 1) % LAP_HISTORY_SIZE;
  ```

### Python code (dashboard, scripts)

- **Style**: black formatter (line length 100)
- **Type hints**: Encouraged (Python 3.12+)
- **Naming**: snake_case for functions/variables, PascalCase for classes

### Commit messages

Keep commits atomic (one logical change per commit). Message format:

```
<subsystem>: <short summary under 50 chars>

<longer explanation if needed, wrapped at 72 chars>
<blank line>
<reference issues or related commits>
```

Examples:
```
fusion: fix delayed-state KF history ring buffer wrap

The history index wasn't wrapping correctly after reaching the end of
the 16-entry ring, causing the wrong past state to be rewound and
applied. Now uses modulo arithmetic consistently.

Fixes #42
```

```
canbus: add GPS_Gate broadcast message (0x6BC)

Gates are now sent once per 8 CAN cycles (~5 Hz aggregate) for the
dashboard to draw the track live. Gate index round-robins; cleared
slots are broadcast with valid=0.
```

## Repository layout

```
.
├── README.md                  # Main product documentation
├── docs/
│   ├── QUICK_START.md        # 5-minute build guide
│   ├── DEVELOPER.md          # This file
│   ├── ARCHITECTURE.md       # Technical deep dive
│   ├── DATASHEET.md          # Hardware specs & CAN matrix
│   ├── DRIVER_GUIDE.md       # User guide (drivers)
│   ├── NOTES.md              # Dev notes, decisions, future work
│   └── DOCUMENTATION.md      # Navigation guide
├──
├── CMakeLists.txt            # Top-level build config
├── CMakePresets.json         # Build presets (Release, Debug, etc.)
├── cmake/                    # CMake modules
├──
├── SUFST/
│   ├── Inc/                  # Header files, by subsystem
│   │   ├── fusion/           # AHRS, Kalman filter, geodesy, timebase
│   │   ├── gps/              # UBX protocol
│   │   ├── imu/              # LSM6DSO32, sensor-hub, mag-cal
│   │   ├── canbus/           # CAN messages, broadcast state
│   │   ├── laptimer/         # Gate storage, lap timing
│   │   ├── persist/          # Flash-backed persistence
│   │   ├── sys/              # App-level task wiring, buttons, LEDs
│   │   ├── board/            # Compile-time config (rates, pins)
│   │   └── app.h             # Inter-task messaging
│   └── Src/                  # Implementation (.c files, mirrors Inc/)
│       ├── fusion/
│       ├── gps/
│       ├── imu/
│       ├── canbus/
│       ├── laptimer/
│       ├── persist/
│       ├── sys/
│       └── app.c             # Task creation, queues, main app loop
│
├── Core/                     # STM32 HAL, FreeRTOS, MSP (auto-gen)
├── Drivers/                  # Vendor drivers (STM32Cube SDK)
├── Middlewares/              # FreeRTOS, CMSIS
│
├── tests/                    # Unit tests (host-side)
│   ├── test_ubx.c           # UBX parsing tests
│   ├── test_can_pack.c      # CAN codec tests
│   ├── test_kf6.c           # Kalman filter tests
│   ├── CMakeLists.txt       # Test build config
│   └── *.deps               # Test-to-source mappings
│
├── tools/
│   ├── GPS.dbc              # CAN database (messages, signals)
│   └── dashboard/           # PyQt6 reference UI
│       ├── src/gps_dashboard/
│       │   ├── __main__.py
│       │   ├── ui/          # Main window, plots, track map
│       │   └── track_memory.py  # Gate persistence (desktop)
│       ├── tests/
│       ├── pyproject.toml
│       └── README.md        # Dashboard-specific docs
│
└── build/                   # CMake build outputs (not committed)
```

## Continuous integration

The project builds automatically on each commit (via GitHub Actions or similar). The CI pipeline:

1. Builds Release and Debug firmware
2. Checks binary size (must fit in 128 KB flash)
3. Runs all unit tests
4. Validates code formatting (clang-format)
5. Checks for common issues (missing includes, typos)

Before pushing, run locally:

```bash
# Build both variants
cmake --preset Release && cmake --build --preset Release
cmake --preset Debug && cmake --build --preset Debug

# Run tests
cmake -B build-host -DGPS_HOST_TESTS=ON -G Ninja
cmake --build build-host && ctest --test-dir build-host

# Check formatting
clang-format -i SUFST/Src/**/*.c SUFST/Inc/**/*.h
```

## Performance

The firmware is designed to run soft real-time on a 170 MHz STM32G431:

| Task | Rate | CPU load | Notes |
|------|------|----------|-------|
| `imu_task` | 104 Hz | ~18% | SPI read + AHRS + KF predict + lap check |
| `gps_task` | ~1 Hz (varies) | ~3% | UBX parsing, PVT handling, boot config |
| `can_task` | ~100 Hz (staggered) | ~8% | CAN TX (periodic broadcast + events) |
| `aux_task` | ~1 Hz | ~1% | NMEA synthesis to MoTeC (USART3 TX) |
| `sys_task` | 10 Hz | ~2% | Buttons, LEDs, temperature |
| **Idle** | — | ~68% | CPU available for other systems |
| **Total load** | — | **~32%** | Plenty of headroom |

Monitor live via the dashboard's `GPS_Status` message (CAN ID 0x6B9), field `cpu_load_pct`.

## Troubleshooting

### Build errors

| Error | Fix |
|-------|-----|
| `arm-none-eabi-gcc: command not found` | Install ARM toolchain; check PATH |
| `CMake error: preset not found` | Run `cmake --list-presets` |
| `undefined reference to 'function'` | Check CMakeLists.txt includes all `.c` sources |
| Flash size exceeded | Check binary with `arm-none-eabi-size` |

### Test failures

| Error | Fix |
|-------|-----|
| `undefined reference to '__errno'` | Update GCC to 13.x+; workaround is in `Core/Src/syscalls_errno.c` |
| Tests don't link | Check `test_<module>.deps` includes all required `.c` files |
| Floating-point assertion | Add `-lm` to linker; CMakeLists.txt already does this |

### Hardware issues

| Symptom | Diagnosis |
|---------|-----------|
| Board doesn't program | Check ST-LINK connection; verify USB cable |
| No debug output | Check USART1 baud (115200) and TX pin (PB6) |
| GPS never locks | Antenna not connected or clear sky view blocked |
| CAN traffic missing | Check termination resistor (120 Ω); verify CAN transceiver power |

---

**Need help?** Check [ARCHITECTURE.md](ARCHITECTURE.md) for deep dives, or ask the team.
