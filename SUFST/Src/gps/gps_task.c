/**
 * @file    gps_task.c
 * @brief   GPS UBX stream: boot config, DMA-ring parsing, PVT/TM2
 *          dispatch, PPS pairing.
 */

#include "gps/gps_task.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "main.h"

#include "canbus/can_defs.h"
#include "canbus/canbc.h"
#include "fusion/timebase.h"
#include "gps/gps_config.h"
#include "gps/ubx.h"
#include "sys/app.h"

#define GPS_RX_RING_SIZE 512U

static uint8_t s_rx_ring[GPS_RX_RING_SIZE];
static TaskHandle_t s_gps_task_handle;

/* Round a NAV-PVT iTOW to the nearest GPS second boundary, so it can be
 * paired with a PPS edge tick (PPS fires exactly on the second). */
static uint32_t
round_to_second(uint32_t itow_ms)
{
    return ((itow_ms + 500U) / 1000U) * 1000U;
}

static void
handle_nav_pvt(const ubx_nav_pvt_t *pvt)
{
    uint32_t pps_tick;
    if (timebase_take_pending_pps(&pps_tick)) {
        timebase_on_pps(pps_tick, round_to_second(pvt->itow_ms));
        app_set_events(SYS_EVT_TIME_LOCKED);
    }

    bool fix_ok = (pvt->flags & 0x01U) != 0U;
    uint8_t carr_soln = (uint8_t) ((pvt->flags >> 6) & 0x03U);

    float hacc_mm = (float) pvt->hacc_mm;
    float sacc_mm_s = (float) pvt->sacc_mms;
    float pdop = (float) pvt->pdop_1e2 * 0.01f;

    uint8_t quality_flags = 0U;
    if (fix_ok) {
        quality_flags |= CAN_GPS_QUALITY_FIX_OK;
    }
    quality_flags |= (uint8_t) (carr_soln << CAN_GPS_QUALITY_CARR_SOLN_SHIFT);

    canbc_state_set_quality(hacc_mm, sacc_mm_s, pdop, quality_flags);

    if (fix_ok) {
        app_set_events(SYS_EVT_GPS_READY);
    }

    /* Forward the raw fix to imu_task (fusion) and aux_task (NMEA),
     * each on its own overwrite queue - two independent consumers can't
     * share one without stealing each other's fixes. */
    if (fix_ok) {
        xQueueOverwrite(g_gps_pvt_queue, pvt);
        xQueueOverwrite(g_gps_pvt_queue_nmea, pvt);
    }
}

static void
handle_tim_tm2(const ubx_tim_tm2_t *tm2)
{
    static uint16_t s_last_count = 0xFFFFU;

    if (!(tm2->flags & UBX_TM2_FLAG_NEW_RISING)) {
        return;
    }
    if (tm2->count == s_last_count) {
        return; /* dedupe: already forwarded this edge */
    }
    s_last_count = tm2->count;

    can_lap_event_t evt = {CAN_LAP_EVENT_TM2, 0U, tm2->tow_ms_rising, 0U};
    xQueueSend(g_lap_event_queue, &evt, 0);
}

static void
process_frame(const ubx_frame_t *f)
{
    if (f->cls == UBX_CLASS_NAV && f->id == UBX_NAV_PVT) {
        ubx_nav_pvt_t pvt;
        if (ubx_decode_nav_pvt(f, &pvt)) {
            handle_nav_pvt(&pvt);
        }
    } else if (f->cls == UBX_CLASS_TIM && f->id == UBX_TIM_TM2) {
        ubx_tim_tm2_t tm2;
        if (ubx_decode_tim_tm2(f, &tm2)) {
            handle_tim_tm2(&tm2);
        }
    }
}

void
HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART3) {
        return;
    }
    BaseType_t hp_woken = pdFALSE;
    xTaskNotifyFromISR(s_gps_task_handle, Size, eSetValueWithOverwrite,
                        &hp_woken);
    portYIELD_FROM_ISR(hp_woken);
}

void
gps_task_main(void *argument)
{
    (void) argument;

    s_gps_task_handle = xTaskGetCurrentTaskHandle();

    status_t cfg_status = gps_config_run_boot_sequence();
    if (cfg_status != STATUS_OK) {
        app_set_events(SYS_EVT_GPS_FAULT);
        /* Best-effort: keep going and parse whatever the receiver
         * happens to output at whatever baud HAL_UART_Init() last left
         * it at, rather than giving up on the node entirely. */
    }

    ubx_parser_t parser;
    ubx_parser_init(&parser);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_rx_ring, GPS_RX_RING_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

    uint16_t read_idx = 0U;

    for (;;) {
        uint32_t event_pos;
        if (xTaskNotifyWait(0U, 0xFFFFFFFFU, &event_pos,
                             pdMS_TO_TICKS(100U)) != pdTRUE) {
            continue; /* periodic wake with nothing new; loop and re-wait */
        }

        uint16_t write_idx = (uint16_t) event_pos;
        while (read_idx != write_idx) {
            if (ubx_parser_feed(&parser, s_rx_ring[read_idx])) {
                process_frame(&parser.frame);
            }
            read_idx = (uint16_t) ((read_idx + 1U) % GPS_RX_RING_SIZE);
        }
    }
}
