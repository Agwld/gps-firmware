/**
 * @file    gps_task.c
 * @brief   GPS UBX stream: boot config, DMA-ring parsing, PVT/TM2
 *          dispatch, PPS pairing, and link supervision (re-arm / re-
 *          configure when the NAV-PVT stream stops).
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

/* NAV-PVT arrives every 50 ms whether or not there is a fix, so this long
 * without one means the link itself is down - not merely "no fix". */
#define GPS_STREAM_TIMEOUT_MS    2000U

/* While the link is down, how often to re-run the boot configuration.
 * The receiver keeps its settings in RAM only: if it alone resets or
 * browns out it comes back at 38400 baud talking NMEA, which this node
 * can't parse, and only re-sending the configuration brings it back. */
#define GPS_RECONFIG_INTERVAL_MS 5000U

static uint8_t s_rx_ring[GPS_RX_RING_SIZE];
static TaskHandle_t s_gps_task_handle;

/* Set by HAL_UART_ErrorCallback, cleared by gps_task_main once it has
 * re-armed the DMA reception - see the callback below for why this is
 * needed at all. */
static volatile bool s_uart_error;

/* Link recoveries since boot (UART re-arms + re-configurations), for
 * GPS_Status. Written by this task only; a single byte, so sys_task's
 * read needs no lock. */
static volatile uint8_t s_retry_count;

/* Link supervision state (this task only). */
static TickType_t s_last_pvt_tick;
static bool s_link_fault;
static bool s_fix_ready;

uint8_t
gps_task_get_retry_count(void)
{
    return s_retry_count;
}

static void
count_retry(void)
{
    if (s_retry_count < UINT8_MAX) {
        s_retry_count++;
    }
}

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
    /* The stream is alive (regardless of fix). */
    s_last_pvt_tick = xTaskGetTickCount();
    if (s_link_fault) {
        s_link_fault = false;
        app_clear_events(SYS_EVT_GPS_FAULT);
    }

    uint32_t pps_tick;
    if (timebase_take_pending_pps(&pps_tick)) {
        /* Only discipline the timebase once the receiver has actually
         * resolved time-of-week: pairing the edge with an unresolved
         * iTOW would lock the mapping to the wrong GPS second. The edge
         * is consumed either way (take_pending_pps clears it) so a stale
         * one doesn't linger - the next 1 Hz edge is paired once valid. */
        if (pvt->valid & UBX_PVT_VALID_TIME) {
            timebase_on_pps(pps_tick, round_to_second(pvt->itow_ms));
            app_set_events(SYS_EVT_TIME_LOCKED);
        }
    }

    bool fix_ok = (pvt->flags & UBX_PVT_FLAG_GNSS_FIX_OK) != 0U;
    uint8_t carr_soln = (uint8_t) ((pvt->flags & UBX_PVT_FLAG_CARR_SOLN_MASK) >>
                                   UBX_PVT_FLAG_CARR_SOLN_SHIFT);

    float hacc_mm = (float) pvt->hacc_mm;
    float sacc_mm_s = (float) pvt->sacc_mms;
    float pdop = (float) pvt->pdop_1e2 * 0.01f;

    uint8_t quality_flags = 0U;
    if (fix_ok) {
        quality_flags |= CAN_GPS_QUALITY_FIX_OK;
    }
    quality_flags |= (uint8_t) (carr_soln << CAN_GPS_QUALITY_CARR_SOLN_SHIFT);
    if ((pvt->flags & UBX_PVT_FLAG_DIFF_SOLN) != 0U) {
        quality_flags |= CAN_GPS_QUALITY_DIFF_SOLN;
    }

    canbc_state_set_quality(hacc_mm, sacc_mm_s, pdop, quality_flags);

    /* Publish GPS time even without a position fix - the receiver can
     * resolve time from fewer satellites than a fix needs, and the
     * validity flags tell consumers how much to trust it. */
    uint8_t time_flags = 0U;
    if (pvt->valid & UBX_PVT_VALID_TIME) {
        time_flags |= CAN_GPS_TIME_FLAG_UTC_VALID;
    }
    if (pvt->valid & UBX_PVT_VALID_FULLY_RESOLVED) {
        time_flags |= CAN_GPS_TIME_FLAG_FULLY_RESOLVED;
    }
    canbc_state_set_time(pvt->itow_ms, pvt->hour, pvt->min, pvt->sec,
                          time_flags);

    /* GPS_READY tracks the fix, both ways: it drops again when the fix
     * does, rather than latching the first one forever. */
    if (fix_ok != s_fix_ready) {
        s_fix_ready = fix_ok;
        if (fix_ok) {
            app_set_events(SYS_EVT_GPS_READY);
        } else {
            app_clear_events(SYS_EVT_GPS_READY);
        }
    }

    /* Forward every NAV-PVT - fix or not - to imu_task (fusion) and
     * aux_task (NMEA), each on its own overwrite queue (two independent
     * consumers can't share one without stealing each other's messages).
     * Both check gnssFixOK themselves; holding back the fix-less ones
     * would leave them replaying the last good fix as if it were current. */
    xQueueOverwrite(g_gps_pvt_queue, pvt);
    xQueueOverwrite(g_gps_pvt_queue_nmea, pvt);
}

static void
handle_tim_tm2(const ubx_tim_tm2_t *tm2)
{
    /* A separate "seen one yet" flag rather than a sentinel count value:
     * the rising-edge counter is a full 16-bit field, so any specific
     * sentinel (0xFFFF) collides with a real edge count and would drop a
     * genuine edge. */
    static bool s_have_last = false;
    static uint16_t s_last_count = 0U;

    if (!(tm2->flags & UBX_TM2_FLAG_NEW_RISING)) {
        return;
    }
    if (s_have_last && tm2->count == s_last_count) {
        return; /* dedupe: already forwarded this edge */
    }
    s_have_last = true;
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
    if (huart->Instance != USART3 || s_gps_task_handle == NULL) {
        return;
    }
    BaseType_t hp_woken = pdFALSE;
    xTaskNotifyFromISR(s_gps_task_handle, Size, eSetValueWithOverwrite,
                        &hp_woken);
    portYIELD_FROM_ISR(hp_woken);
}

/* Any UART error (framing/noise/overrun/DMA) is treated as blocking by
 * the HAL: it tears the reception down (UART_EndRxTransfer + DMA abort)
 * and, with no handler here, the (weak, empty) default would just drop
 * it - permanently ending the UBX stream until reset, since nothing ever
 * re-arms HAL_UARTEx_ReceiveToIdle_DMA(). A single EMI-induced framing
 * error on the car would be enough to trigger this. The re-arm itself
 * happens in gps_task_main (task context, not here) because it also has
 * to reset read_idx back in step with the ring buffer DMA restarts at. */
void
HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3 || s_gps_task_handle == NULL) {
        return;
    }
    s_uart_error = true;
    BaseType_t hp_woken = pdFALSE;
    xTaskNotifyFromISR(s_gps_task_handle, 0U, eNoAction, &hp_woken);
    portYIELD_FROM_ISR(hp_woken);
}

/* (Re)start the circular DMA reception from the top of the ring. The
 * abort first makes this safe from any state - after a HAL error
 * teardown (already idle), or with a reception still nominally running. */
static void
rearm_rx(void)
{
    (void) HAL_UART_AbortReceive(&huart3);
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_rx_ring,
                                      GPS_RX_RING_SIZE) == HAL_OK) {
        /* Only the idle-line and buffer-wrap events are wanted; the
         * half-transfer interrupt would just add wake-ups. */
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    }
    /* A failed start is retried by the stream-timeout path below. */
}

void
gps_task_main(void *argument)
{
    (void) argument;

    s_gps_task_handle = xTaskGetCurrentTaskHandle();

    if (gps_config_run_boot_sequence() != STATUS_OK) {
        s_link_fault = true;
        app_set_events(SYS_EVT_GPS_FAULT);
        /* Best-effort: keep going and parse whatever the receiver happens
         * to output; the stream supervision below keeps retrying the
         * configuration for as long as no NAV-PVT arrives. */
    }

    ubx_parser_t parser;
    ubx_parser_init(&parser);

    rearm_rx();

    uint16_t read_idx = 0U;
    TickType_t last_recover_tick = xTaskGetTickCount();
    s_last_pvt_tick = last_recover_tick;

    for (;;) {
        uint32_t event_pos = 0U;
        bool notified = xTaskNotifyWait(0U, 0xFFFFFFFFU, &event_pos,
                                         pdMS_TO_TICKS(100U)) == pdTRUE;

        if (s_uart_error) {
            /* The HAL already tore the transfer down before invoking the
             * error callback. read_idx resets with the re-arm: the DMA
             * write position restarts at 0, and any bytes the ring held
             * past whatever read_idx pointed to are gone regardless. The
             * UBX parser itself needs no reset - it already tolerates and
             * resyncs past corrupt/missing bytes (see gps_config.c). */
            s_uart_error = false;
            count_retry();
            rearm_rx();
            read_idx = 0U;
            continue;
        }

        if (notified) {
            /* HAL reports the absolute byte position in the ring
             * (0..SIZE). On a DMA transfer-complete (buffer wrap) it
             * reports the full SIZE, which must fold back to 0 -
             * otherwise write_idx can never equal the always-modulo
             * read_idx and the drain loop spins forever, hanging the task
             * until the watchdog resets the node. */
            uint16_t write_idx = (uint16_t) (event_pos % GPS_RX_RING_SIZE);
            while (read_idx != write_idx) {
                if (ubx_parser_feed(&parser, s_rx_ring[read_idx])) {
                    process_frame(&parser.frame);
                }
                read_idx = (uint16_t) ((read_idx + 1U) % GPS_RX_RING_SIZE);
            }
        }

        /* Link supervision: flag a stalled stream, and periodically try
         * to bring it back - re-send the configuration (covers a receiver
         * that reset on its own) and restart reception (covers a UART
         * that stopped without raising an error). */
        TickType_t now = xTaskGetTickCount();
        if ((now - s_last_pvt_tick) < pdMS_TO_TICKS(GPS_STREAM_TIMEOUT_MS)) {
            continue;
        }

        if (!s_link_fault) {
            s_link_fault = true;
            app_set_events(SYS_EVT_GPS_FAULT);
        }
        if (s_fix_ready) {
            s_fix_ready = false;
            app_clear_events(SYS_EVT_GPS_READY);
        }

        if ((now - last_recover_tick) >=
            pdMS_TO_TICKS(GPS_RECONFIG_INTERVAL_MS)) {
            count_retry();
            (void) gps_config_run_boot_sequence();
            rearm_rx();
            read_idx = 0U;
            ubx_parser_init(&parser);
            last_recover_tick = xTaskGetTickCount();
        }
    }
}
