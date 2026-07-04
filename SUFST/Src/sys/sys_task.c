/**
 * @file    sys_task.c
 * @brief   Buttons FSM, LEDs, MCP9800 temperature (sole I2C2 user),
 *          config-save/flash persistence, IWDG, status frame.
 *
 * CPU load reporting is a placeholder (0): configGENERATE_RUN_TIME_STATS
 * is off in FreeRTOSConfig.h, so there's no run-time-stats timer wired up
 * to compute it from yet - a real number needs that enabled plus a
 * portCONFIGURE_TIMER_FOR_RUN_TIME_STATS definition, not fabricated here.
 */

#include "sys/sys_task.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "main.h"

#include "board/board_config.h"
#include "canbus/can_defs.h"
#include "canbus/canbc.h"
#include "persist/flash_store.h"
#include "sys/app.h"

/* MCP9800, I2C2. Ambient temperature register 0x00; default 9-bit
 * resolution, 0.5 C/LSB, left-justified in a 16-bit big-endian word
 * (value = (raw >> 7) * 0.5). Verify against the MCP9800 datasheet
 * before trusting the resolution/register assumption on the bench. */
#define MCP9800_I2C_ADDR (0x48U << 1)
#define MCP9800_REG_TEMP 0x00U

/* Lap button press-duration thresholds (board_config.h), sampled every
 * SYS_TASK_PERIOD_MS. */
#define SYS_TASK_PERIOD_MS 20U

typedef enum {
    BUTTON_IDLE,
    BUTTON_PRESSED,
} button_state_t;

static float
read_mcp9800_temp_c(void)
{
    uint8_t reg = MCP9800_REG_TEMP;
    uint8_t data[2];

    if (HAL_I2C_Master_Transmit(&hi2c2, MCP9800_I2C_ADDR, &reg, 1U, 20U) !=
        HAL_OK) {
        return 0.0f;
    }
    if (HAL_I2C_Master_Receive(&hi2c2, MCP9800_I2C_ADDR, data, 2U, 20U) !=
        HAL_OK) {
        return 0.0f;
    }

    int16_t raw = (int16_t) (uint16_t) ((data[0] << 8) | data[1]);
    return (float) (raw >> 7) * 0.5f;
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
 * start/finish (gates.c itself clears sectors on a gate-0 set), very
 * long press (>5 s) = clear all gates. Gate position/heading come from
 * the current fused state, so this posts through the same
 * CAN_CMD_GATE_* queue canbc_task uses for CAN-triggered gate commands -
 * one consumer (imu_task) either way. */
static void
poll_lap_button(void)
{
    static button_state_t state = BUTTON_IDLE;
    static uint32_t held_ms = 0U;
    static uint8_t next_sector = 1U;

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
        cmd.arg0 = 0xFFU;
        next_sector = 1U;
    } else if (held_ms >= LAP_BUTTON_LONG_MS) {
        cmd.cmd = CAN_CMD_GATE_SET;
        cmd.arg0 = 0U; /* start/finish */
        next_sector = 1U;
    } else {
        cmd.cmd = CAN_CMD_GATE_SET;
        cmd.arg0 = next_sector;
        if (next_sector < LAP_MAX_GATES - 1U) {
            next_sector++;
        }
    }
    cmd.arg1 = 0U;
    xQueueSend(g_gate_cmd_queue, &cmd, 0);
}

static void
handle_sys_commands(bool *want_compact)
{
    app_cmd_t cmd;
    while (xQueueReceive(g_sys_cmd_queue, &cmd, 0) == pdTRUE) {
        if (cmd.cmd == CAN_CMD_CONFIG_SAVE) {
            *want_compact = true;
        }
    }
}

void
sys_task_main(void *argument)
{
    (void) argument;

    IWDG_HandleTypeDef hiwdg = {0};
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    hiwdg.Init.Reload = 1250U; /* ~1 s at 32 kHz LSI / 32 */
    HAL_IWDG_Init(&hiwdg);

    uint32_t uptime_ticks = 0U;
    uint32_t temp_decim = 0U;
    bool want_compact = false;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        HAL_IWDG_Refresh(&hiwdg);

        poll_lap_button();
        update_leds();
        handle_sys_commands(&want_compact);

        /* Compact only when the fused speed (canbc's latest published
         * value) suggests the car is stationary - erasing stalls flash
         * reads for tens of ms, unacceptable while moving. */
        if (want_compact) {
            canbc_state_t snap;
            canbc_state_get_snapshot(&snap);
            if (snap.speed_mps < 0.5f) {
                flash_store_erase_and_compact();
                want_compact = false;
            }
        }

        if ((temp_decim++ % (1000U / SYS_TASK_PERIOD_MS)) == 0U) {
            float board_temp_c = read_mcp9800_temp_c();
            canbc_state_set_temp(board_temp_c, 0.0f, 0.0f);
        }

        uptime_ticks += SYS_TASK_PERIOD_MS;
        canbc_state_set_status((uint16_t) (uptime_ticks / 1000U),
                                (uint16_t) app_get_events(), 0U, 0U, 0U);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SYS_TASK_PERIOD_MS));
    }
}
