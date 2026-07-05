set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# arm-none-eabi- must be part of path environment
set(TOOLCHAIN_PREFIX                arm-none-eabi-)

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX    ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags: STM32G431 = Cortex-M4F, single-precision hard float.
# -mthumb is required (not implied) for correct multilib selection - without
# it the linker picked a libm.a variant that didn't match this ISA/float-abi
# combination, producing "Unknown destination type (ARM/Thumb)" relocation
# errors against __errno inside the prebuilt library.
set(TARGET_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections")

# Keep every float literal and libm call single-precision: double promotion is
# both a flash-size and a deadline hazard on the M4F (soft double).
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsingle-precision-constant -Wdouble-promotion")

# nano.specs' reduced libc doesn't provide the reentrant errno plumbing the
# default errno-setting libm wrappers (w_sqrt, wf_asin, ...) need, which
# fails at link time with "undefined reference to `__errno`". Nothing in
# this firmware inspects errno after a math call (there's no per-task
# errno context on a bare Cortex-M anyway), so telling the compiler not to
# route through those wrappers avoids the dependency entirely.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-math-errno")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

# MCU variant fitted on the board. The gps-mainboard is confirmed CB
# (128 KB flash); C8 (64 KB) is kept selectable for interchangeability
# testing via -DGPS_MCU_VARIANT=C8. GPS_FLASH_TOTAL_KB is forwarded to the
# main CMakeLists.txt as a compile definition so flash_store.c derives its
# reserved-page address from the same number that sized the linker script,
# instead of a second hand-maintained constant that could drift from it.
set(GPS_MCU_VARIANT "CB" CACHE STRING "STM32G431 flash variant: C8 (64K) or CB (128K, fitted part)")
set_property(CACHE GPS_MCU_VARIANT PROPERTY STRINGS C8 CB)

if(GPS_MCU_VARIANT STREQUAL "C8")
    set(GPS_LINKER_SCRIPT "STM32G431X8_FLASH.ld" CACHE INTERNAL "")
    set(GPS_FLASH_TOTAL_KB 64 CACHE INTERNAL "")
elseif(GPS_MCU_VARIANT STREQUAL "CB")
    set(GPS_LINKER_SCRIPT "STM32G431XB_FLASH.ld" CACHE INTERNAL "")
    set(GPS_FLASH_TOTAL_KB 128 CACHE INTERNAL "")
else()
    message(FATAL_ERROR "GPS_MCU_VARIANT must be C8 or CB, got '${GPS_MCU_VARIANT}'")
endif()

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/${GPS_LINKER_SCRIPT}\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")

# newlib-nano's libm (e.g. the errno-setting sqrt/asinf wrappers) can need
# a libc symbol (__errno) that a single left-to-right archive scan won't
# find if libc.a was already searched before libm.a pulled in the need for
# it. --start-group/--end-group make the linker rescan the group until
# nothing new resolves, which is the standard fix for this ordering issue.
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--start-group -lc -lm -Wl,--end-group")
set(TOOLCHAIN_LINK_LIBRARIES "m")
