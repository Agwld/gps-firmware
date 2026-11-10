/**
 * @file    sys_task.c
 * @brief   Lap-button FSM, LEDs, board (MCP9800) and MCU temperatures,
 *          IWDG refresh, status frame.
 *
 * CPU load is 100 minus the idle task's share of run time, per
 * FreeRTOSConfig.h's configGENERATE_RUN_TIME_STATS wiring (the TIM3
 * microsecond tick already used for GPS PPS timing, reused as the
 * run-time-stats clock).
 */

#include "sys/sys_task.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "main.h"

#include "board/board_config.h"
#include "canbus/can_defs.h"
#include "canbus/canbc.h"
#include "gps/gps_task.h"
#include "imu/imu_task.h"
#include "sys/app.h"

/* MCP9800, I2C2. Fitted part per schematic (U21) is MCP9800A5T-M/OT -
 * the SOT-23-5 package with a factory-fixed address, no A0-A2 pins.
 * Datasheet Table 3-2 gives the A5 variant's 7-bit address as 1001101b
 * = 0x4D (NOT the commonly-assumed 0x48, which is the A0 variant).
 * Ambient temperature register 0x00; default power-up resolution is
 * 9-bit / 0.5 degC per LSB with only bit 15..7 of the 16-bit big-endian
 * register populated (bits 6..0 forced 0), so raw>>7 * 0.5 == raw/256,
 * matching datasheet Eq. 5-2 (TA = Code * 2^-4) for that register
 * layout. Confirmed against 21909d.pdf (MCP9800/1/2/3 datasheet). */
#define MCP9800_I2C_ADDR (0x4DU << 1)
#define MCP9800_REG_TEMP 0x00U

/* Lap button press-duration thresholds (board_config.h), sampled every
 * SYS_TASK_PERIOD_MS. */
#define SYS_TASK_PERIOD_MS 20U

typedef enum {
    BUTTON_IDLE,
    BUTTON_PRESSED,
} button_state_t;

/* Returns false (leaving *temp_c alone) if the sensor didn't answer, so
 * the caller keeps broadcasting the last good reading rather than a
 * plausible-looking 0.0 degC. */
static bool
read_mcp9800_temp_c(float *temp_c)
{
    uint8_t reg = MCP9800_REG_TEMP;
    uint8_t data[2];

    /* I2C2 is shared with the F9P's DDC port (gps_i2c.c). */
    app_i2c_lock();
    bool ok =
        (HAL_I2C_Master_Transmit(&hi2c2, MCP9800_I2C_ADDR, &reg, 1U, 20U) ==
         HAL_OK) &&
        (HAL_I2C_Master_Receive(&hi2c2, MCP9800_I2C_ADDR, data, 2U, 20U) ==
         HAL_OK);
    app_i2c_unlock();

    if (!ok) {
        return false;
    }

    int16_t raw = (int16_t) (uint16_t) ((data[0] << 8) | data[1]);
    *temp_c = (float) (raw >> 7) * 0.5f;
    return true;
}

static void
update_leds(void)
{
    uint32_t events = app_get_events();
    bool fault = (events & (SYS_EVT_GPS_FAULT | SYS_EVT_IMU_FAULT |
                             SYS_EVT_MAG_FAULT)) != 0U;

    HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN,
                       fault ? GPIO_PIN_SET : GPIO_PIN_RESET);

    bool ready = (events & (SYS_EVT_GPS_READY | SYS_EVT_IMU_READY)) ==
                 (SYS_EVT_GPS_READY | SYS_EVT_IMU_READY);
    HAL_GPIO_WritePin(USER_LED_PORT, USER_LED_PIN,
                       ready ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Lap button FSM: short press = new sector gate, long press (>1 s) = new
 * start/finish (which also clears the sectors), very long press (>5 s) =
 * clear all gates. Gate position/heading come from the current fused
 * state, so this posts through the same CAN_CMD_GATE_* queue canbc_task
 * uses for CAN-triggered gate commands - one consumer (imu_task) either
 * way. Which sector slot a short press fills is decided there too
 * (CAN_GATE_SLOT_NEXT_SECTOR), from the actual gate table. */
static void
poll_lap_button(void)
{
    static button_state_t state = BUTTON_IDLE;
    static uint32_t held_ms = 0U;

    bool pressed =
        HAL_GPIO_ReadPin(LAP_BUTTON_PORT, LAP_BUTTON_PIN) == GPIO_PIN_SET;

    if (pressed) {
        if (state == BUTTON_IDLE) {
            state = BUTTON_PRESSED;
            held_ms = 0U;
        } else {
            held_ms += SYS_TASK_PERIOD_MS;
        }
        return;
    }

    if (state != BUTTON_PRESSED) {
        return; /* released while already idle: nothing to do */
    }
    state = BUTTON_IDLE;

    app_cmd_t cmd;
    if (held_ms >= LAP_BUTTON_CLEAR_MS) {
        cmd.cmd = CAN_CMD_GATE_CLEAR;
        cmd.arg0 = CAN_GATE_SLOT_ALL;
    } else if (held_ms >= LAP_BUTTON_LONG_MS) {
        cmd.cmd = CAN_CMD_GATE_SET;
        cmd.arg0 = 0U; /* start/finish */
    } else {
        cmd.cmd = CAN_CMD_GATE_SET;
        cmd.arg0 = CAN_GATE_SLOT_NEXT_SECTOR;
    }
    cmd.arg1 = 0U;
    xQueueSend(g_gate_cmd_queue, &cmd, 0);
}

void
sys_task_main(void *argument)
{
    (void) argument;

    /* The IWDG is started in main() before the scheduler (so a boot-time
     * hang is still caught); this task just keeps refreshing it. */

    uint32_t uptime_ms = 0U;
    uint32_t temp_decim = 0U;
    float board_temp_c = 0.0f;
    float mcu_temp_c = 0.0f;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        HAL_IWDG_Refresh(&hiwdg);

        poll_lap_button();
        update_leds();

        /* Temperatures at 1 Hz. A failed read keeps the last good value
         * on the bus. (The IMU die temperature in the same CAN frame is
         * published by imu_task.) */
        if ((temp_decim++ % (1000U / SYS_TASK_PERIOD_MS)) == 0U) {
            (void) read_mcp9800_temp_c(&board_temp_c);
            (void) adc_read_mcu_temp_c(&mcu_temp_c);
            canbc_state_set_board_temp(board_temp_c, mcu_temp_c);
        }

        uptime_ms += SYS_TASK_PERIOD_MS;
        uint32_t uptime_s = uptime_ms / 1000U;
        uint32_t idle_pct = ulTaskGetIdleRunTimePercent();
        uint8_t cpu_load_pct =
            (uint8_t) ((idle_pct >= 100U) ? 0U : (100U - idle_pct));
        canbc_state_set_status(
            (uint16_t) ((uptime_s > UINT16_MAX) ? UINT16_MAX : uptime_s),
            (uint16_t) app_get_events(), gps_task_get_retry_count(),
            imu_task_get_error_count(), cpu_load_pct);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SYS_TASK_PERIOD_MS));
    }
}
