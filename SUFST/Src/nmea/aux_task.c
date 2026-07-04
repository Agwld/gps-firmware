/**
 * @file    aux_task.c
 * @brief   NMEA synthesis (USART2 TX) + RTCM forward poll. Owns USART2
 *          entirely; USART3 TX is shared with gps_task via
 *          app_gps_tx_lock()/unlock().
 */

#include "nmea/aux_task.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "main.h"

#include "board/board_config.h"
#include "canbus/can_defs.h"
#include "gps/ubx.h"
#include "nmea/nmea_out.h"
#include "nmea/rtcm_bridge.h"
#include "sys/app.h"

void
aux_task_main(void *argument)
{
    (void) argument;

    rtcm_bridge_init();

    uint32_t rate_hz = NMEA_OUT_RATE_HZ;
    bool enabled = true;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        app_cmd_t cfg;
        while (xQueueReceive(g_nmea_cfg_queue, &cfg, 0) == pdTRUE) {
            if (cfg.cmd == CAN_CMD_NMEA_CFG) {
                if (cfg.arg0 > 0U) {
                    rate_hz = cfg.arg0;
                }
                enabled = (cfg.arg1 != 0U);
            }
        }

        rtcm_bridge_poll();

        if (enabled) {
            ubx_nav_pvt_t pvt;
            if (xQueuePeek(g_gps_pvt_queue_nmea, &pvt, 0) == pdTRUE) {
                char line[96];
                size_t len;

                len = nmea_format_gga(&pvt, line, sizeof(line));
                if (len > 0U) {
                    HAL_UART_Transmit(&huart2, (uint8_t *) line, len, 50U);
                }
                len = nmea_format_rmc(&pvt, line, sizeof(line));
                if (len > 0U) {
                    HAL_UART_Transmit(&huart2, (uint8_t *) line, len, 50U);
                }
                len = nmea_format_vtg(&pvt, line, sizeof(line));
                if (len > 0U) {
                    HAL_UART_Transmit(&huart2, (uint8_t *) line, len, 50U);
                }
            }
        }

        uint32_t period_ms = 1000U / rate_hz;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
    }
}
