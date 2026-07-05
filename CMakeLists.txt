cmake_minimum_required(VERSION 3.22)

project(gps-firmware C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

option(GPS_HOST_TESTS "Build host unit tests instead of the target firmware" OFF)

if(GPS_HOST_TESTS)
    enable_testing()
    add_subdirectory(tests)
    return()
endif()

# ---------------------------------------------------------------------------
# Target firmware build
# ---------------------------------------------------------------------------

if(NOT DEFINED GPS_FLASH_TOTAL_KB)
    message(FATAL_ERROR "GPS_FLASH_TOTAL_KB not set - toolchain file "
                         "(cmake/gcc-arm-none-eabi.cmake) must run before "
                         "this point to select the MCU variant.")
endif()

add_compile_definitions(
    USE_HAL_DRIVER
    STM32G431xx
    GPS_FLASH_TOTAL_KB=${GPS_FLASH_TOTAL_KB}
)

message(STATUS "GPS MCU variant: ${GPS_MCU_VARIANT} (${GPS_FLASH_TOTAL_KB} KB flash, linker: ${GPS_LINKER_SCRIPT})")

# --- CMSIS + device startup ------------------------------------------------

set(CMSIS_DIR       ${CMAKE_SOURCE_DIR}/Drivers/CMSIS)
set(DEVICE_DIR       ${CMSIS_DIR}/Device/ST/STM32G4xx)

set(STARTUP_SOURCE ${DEVICE_DIR}/Source/Templates/gcc/startup_stm32g431xx.s)

# --- HAL + LL driver sources (only the modules enabled in stm32g4xx_hal_conf.h) ---

set(HAL_SOURCES
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_iwdg.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_dma.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_gpio.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_pwr.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_rcc.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_spi.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_tim.c
)

# --- FreeRTOS ----------------------------------------------------------------

set(FREERTOS_DIR Middlewares/Third_Party/FreeRTOS/Source)

set(FREERTOS_SOURCES
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/event_groups.c
    ${FREERTOS_DIR}/stream_buffer.c
    ${FREERTOS_DIR}/portable/GCC/ARM_CM4F/port.c
)

# --- Core (CubeMX-style application glue) -----------------------------------
# Populated incrementally as Core/Src files are added (step 8+).

file(GLOB CORE_SOURCES ${CMAKE_SOURCE_DIR}/Core/Src/*.c)

# --- SUFST application modules -----------------------------------------------
# Populated incrementally; each module directory owns its own sources.

file(GLOB_RECURSE SUFST_SOURCES ${CMAKE_SOURCE_DIR}/SUFST/Src/*.c)

add_executable(${PROJECT_NAME}
    ${STARTUP_SOURCE}
    ${DEVICE_DIR}/Source/Templates/system_stm32g4xx.c
    ${HAL_SOURCES}
    ${FREERTOS_SOURCES}
    ${CORE_SOURCES}
    ${SUFST_SOURCES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    Core/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    ${CMSIS_DIR}/Include
    ${DEVICE_DIR}/Include
    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/portable/GCC/ARM_CM4F
    SUFST/Inc
)

set(CMAKE_EXECUTABLE_SUFFIX ".elf")

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
)
