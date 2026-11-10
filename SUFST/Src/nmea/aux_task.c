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
 * Each cycle's GGA + RMC + VTG go out as one DMA transfer, so the ~16 ms
 * they take on the wire costs no CPU time.
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

#define NMEA_SENTENCE_MAX   96U
#define NMEA_SENTENCES      3U /* GGA, RMC, VTG */

/* DMA source buffer: file-scope so it outlives the call that starts the
 * transfer, and lives in ordinary SRAM (the DMA can't reach CCM RAM). */
static uint8_t s_tx_buf[NMEA_SENTENCE_MAX * NMEA_SENTENCES];

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
                    /* Faster than the fix rate would only repeat fixes
                     * (and outrun the baud rate). */
                    rate_hz = (cfg.arg0 > NMEA_OUT_RATE_MAX_HZ)
                                  ? NMEA_OUT_RATE_MAX_HZ
                                  : cfg.arg0;
                }
                enabled = (cfg.arg1 != 0U);
            }
        }

        /* Take (not peek) the newest NAV-PVT: each one is reported at
         * most once, so if the receiver stops talking the output stops
         * with it instead of replaying the last position as current.
         * Fix-less solutions are forwarded too and go out flagged invalid
         * (GGA quality 0, RMC status V), which is what a logger expects
         * to see during a loss of fix. Drained even while disabled, so
         * re-enabling can't start with a stale one. */
        ubx_nav_pvt_t pvt;
        bool have_pvt = xQueueReceive(g_gps_pvt_queue_nmea, &pvt, 0) == pdTRUE;

        /* gState is READY again once the previous cycle's transfer has
         * fully left the UART; until then the buffer is still being read
         * by the DMA and must not be rewritten. */
        if (enabled && have_pvt &&
            huart3.gState == HAL_UART_STATE_READY) {
            char *p = (char *) s_tx_buf;
            size_t room = sizeof(s_tx_buf);
            size_t len;

            len = nmea_format_gga(&pvt, p, room);
            p += len;
            room -= len;
            len = nmea_format_rmc(&pvt, p, room);
            p += len;
            room -= len;
            len = nmea_format_vtg(&pvt, p, room);
            p += len;

            uint16_t total = (uint16_t) (p - (char *) s_tx_buf);
            if (total > 0U) {
                (void) HAL_UART_Transmit_DMA(&huart3, s_tx_buf, total);
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000U / rate_hz));
    }
}
