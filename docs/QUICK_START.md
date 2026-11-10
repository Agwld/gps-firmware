# Quick Start — Get the firmware building in 5 minutes

## Prerequisites

You need the Arm GNU toolchain (`arm-none-eabi-gcc`, 13.x), CMake ≥ 3.22 and
Ninja on your `PATH`.

```bash
# Ubuntu/Debian
sudo apt-get install gcc-arm-none-eabi cmake ninja-build

# macOS
brew install --cask gcc-arm-embedded && brew install cmake ninja
```

On Windows, install the [Arm GNU toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
or reuse the copy inside STM32CubeIDE
(`…/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.<version>/tools/bin`),
and add its `bin` directory to `PATH`.

```bash
arm-none-eabi-gcc --version
cmake --version
```

## Build

```bash
cmake --preset Release
cmake --build --preset Release
```

Output: `build/Release/gps-firmware.{elf,hex,bin}` — about 66 KB of the 124 KB
code flash. The build prints the memory use at the end.

For on-target debugging use the `Debug` preset instead (`build/Debug/`, ~110 KB).

## Run the unit tests

No ARM toolchain needed — they run on your machine:

```bash
cmake --preset Test
cmake --build --preset Test
ctest --preset Test
```

Expected: 11/11 pass.

## Flash the board

With an ST-LINK on the SWD header:

```bash
STM32_Programmer_CLI -c port=SWD -w build/Release/gps-firmware.hex -v -rst
```

or `st-flash write build/Release/gps-firmware.bin 0x08000000`.

Check the version you flashed with
`strings build/Release/gps-firmware.bin | grep gps-firmware`.

Then follow the bring-up checklist in [DEVELOPER.md](DEVELOPER.md#bring-up-checklist).

## Next steps

- **[DEVELOPER.md](DEVELOPER.md)** — build options, tests, flashing, debugging
- **[ARCHITECTURE.md](ARCHITECTURE.md)** — how the firmware works
- **[DATASHEET.md](DATASHEET.md)** — hardware, pins, CAN messages

## Common issues

| Issue | Fix |
|-------|-----|
| `arm-none-eabi-gcc: command not found` | Add the toolchain's `bin/` to `PATH` |
| `No such preset` | Run CMake from the repository root; `cmake --list-presets` |
| `undefined reference to '__errno'` | Don't remove `Core/Src/syscalls_errno.c` or `-fno-math-errno` |
