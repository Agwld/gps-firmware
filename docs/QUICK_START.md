# Quick Start — Get the firmware building in 5 minutes

## Prerequisites

Install the ARM embedded toolchain:

```bash
# Ubuntu/Debian
sudo apt-get install gcc-arm-none-eabi binutils-arm-none-eabi arm-none-eabi-gdb cmake ninja-build

# macOS (homebrew)
brew install arm-none-eabi-gcc cmake ninja

# Windows
# Download GNU Arm Embedded Toolchain from https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
# Add bin/ directory to PATH
```

Verify the install:
```bash
arm-none-eabi-gcc --version  # should show 13.x or later
cmake --version             # should show 3.27 or later
```

## Build the firmware (Release)

```bash
cd /path/to/gps-firmware
cmake --preset Release
cmake --build --preset Release
```

**Output:** `build/gps_firmware.elf` (~58 KB, fits comfortably in 128 KB flash)

## Build the firmware (Debug)

Debug build includes symbols and `-O0` optimization (useful for debugging on hardware):

```bash
cmake --preset Debug
cmake --build --preset Debug
```

**Output:** `build/gps_firmware.elf` (~94 KB)

## Run unit tests

No ARM toolchain needed—tests run on your native machine:

```bash
cmake -B build-host -DGPS_HOST_TESTS=ON -G Ninja
cmake --build build-host
ctest --test-dir build-host -V
```

**Expected:** 11/11 tests pass (UBX parsing, CAN, fusion, timing, persistence, etc.)

## Upload to the board

Using a STLINK-V2 or equivalent:

```bash
# Install STM32 tools (if not already present)
sudo apt-get install stlink-tools  # Linux
brew install stlink                 # macOS

# Program the board
st-flash write build/gps_firmware.elf 0x08000000
```

Alternatively, use STM32CubeProgrammer GUI or your IDE's built-in programmer.

## Quick verification

Once programmed:

1. **Check the build output**:
   ```bash
   arm-none-eabi-size build/gps_firmware.elf
   ```
   Should show CODE < 65536 (fits C8 variant) or CODE < 131072 (CB variant)

2. **Inspect the binary**:
   ```bash
   arm-none-eabi-objdump -h build/gps_firmware.elf
   ```
   Verify `.text` section is < 128 KB

3. **Run the test suite**:
   ```bash
   cd build-host
   ctest -V
   ```

## Next steps

- **[DEVELOPER.md](DEVELOPER.md)** — Detailed build system, debugging, contributing
- **[ARCHITECTURE.md](ARCHITECTURE.md)** — How the firmware works
- **[DATASHEET.md](DATASHEET.md)** — Hardware, pinouts, CAN messages

## Common issues

| Issue | Fix |
|-------|-----|
| `arm-none-eabi-gcc: command not found` | Install ARM toolchain and add bin/ to PATH |
| CMake preset not found | Run `cmake --list-presets` to see available presets |
| Flash size error (code > 65536) | Use CB variant (128 KB): the build defaults to it |
| Tests fail with `libm` error | Update GCC (needs 13.x+); the old prebuilt libm has a bug |

---

Ready to develop? Go to [DEVELOPER.md](DEVELOPER.md) for the full guide.
