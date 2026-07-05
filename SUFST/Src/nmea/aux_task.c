/**
 * @file    aux_task.c
 * @brief   NMEA synthesis to the MoTeC datalogger, on USART3 TX.
 *
 * USART3's two directions carry unrelated flows: RX is the F9P's UBX
 * stream (owned by gps_task), TX drives the RS232 output to the MoTeC
 * via solder jumper JP6 (bridged 2-3) - this task is TX's only writer,
 * so no lock is needed. Both directions necessarily share one baud
 * (GPS_UART_BAUD), since a USART has a single baud-rate register.
 *
 * RTCM corrections never pass through here (or the MCU at all): the
 * RS232 input is routed straight to the F9P's UART1 RX in hardware
 * (JP7 bridged 2-3).
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
#include "sys/app.h"

void
aux_task_main(void *argument)
{
    (void) argument;

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

        if (enabled) {
            ubx_nav_pvt_t pvt;
            if (xQueuePeek(g_gps_pvt_queue_nmea, &pvt, 0) == pdTRUE) {
                char line[96];
                size_t len;

                len = nmea_format_gga(&pvt, line, sizeof(line));
                if (len > 0U) {
                    HAL_UART_Transmit(&huart3, (uint8_t *) line, len, 50U);
                }
                len = nmea_format_rmc(&pvt, line, sizeof(line));
                if (len > 0U) {
                    HAL_UART_Transmit(&huart3, (uint8_t *) line, len, 50U);
                }
                len = nmea_format_vtg(&pvt, line, sizeof(line));
                if (len > 0U) {
                    HAL_UART_Transmit(&huart3, (uint8_t *) line, len, 50U);
                }
            }
        }

        uint32_t period_ms = 1000U / rate_hz;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
    }
}
