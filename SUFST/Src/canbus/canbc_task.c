/**
 * @file    canbc_task.c
 * @brief   CAN broadcast task: staggered periodic TX rota, immediate
 *          Lap_Event TX, and RX (GPS_Command, Wheel_Speeds, pitot)
 *          dispatch.
 *
 * Producers (imu_task, gps_task, sys_task) never touch FDCAN directly -
 * they publish into canbc.c's mutex-guarded state, and this task alone
 * owns the peripheral.
 */

#include "canbus/canbc_task.h"

#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "main.h"

#include "board/board_config.h"
#include "canbus/can_defs.h"
#include "canbus/canbc.h"
#include "sys/app.h"

/* Wheel_Speeds (0x251): owned by the VCU, confirmed byte-for-byte
 * against sufst/can-defs (stag-12 branch, dbc/CAN-S.dbc):
 *   BO_ 593 Wheel_Speeds: 8 VCU
 *    SG_ WHEEL_FR_SPEED : 48|16@1- (0.01,0) "m/s"
 *    SG_ WHEEL_FL_SPEED : 32|16@1- (0.01,0) "m/s"
 *    SG_ WHEEL_RR_SPEED : 16|16@1- (0.01,0) "m/s"
 *    SG_ WHEEL_RL_SPEED :  0|16@1- (0.01,0) "m/s"
 * All four are little-endian (@1) signed i16, so for a DBC start bit N
 * the byte pair is [N/8 .. N/8+1] LSB-first: RL is bytes[0:1], RR
 * bytes[2:3], FL bytes[4:5], FR bytes[6:7] - ascending start bit does
 * NOT mean FR-first, it means RL-first.
 *
 * Only the front-left sensor gives a trustworthy reading on the car -
 * the other three channels are known to fail - so FL is used on its own
 * and the rest of the frame is ignored. (Averaging all four would let
 * one dead channel reading 0 drag the "speed" down by a quarter.) Only
 * acted on when KF_WHEELSPEED_AIDING is enabled (board_config.h); the
 * frame is received regardless. */
#define CAN_ID_WHEEL_SPEEDS 0x251U
#define WHEEL_SPEEDS_FL_BYTE 4U /* WHEEL_FL_SPEED: start bit 32 */

/* Staggered periodic broadcast rota: each entry fires every
 * period_slots * CANBC_SLOT_MS, offset by phase_slot so different-rate
 * messages don't all land on the same 10 ms tick. slot wraps at 100, the
 * LCM of every period_slots value below. */
#define CANBC_SLOT_WRAP 100U

/* Upper bound (in 1 ms ticks) on how long send_frame() waits for a free
 * TX mailbox before giving up. The 3-deep FIFO drains several classic
 * frames per tick, so this only ever trips on a genuinely stuck/bus-off
 * peripheral - the bound just guarantees the task can't hang there. */
#define CANBC_TX_WAIT_MAX_TICKS 3U

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint16_t period_slots;
    uint16_t phase_slot;
} canbc_rota_entry_t;

enum {
    ROTA_GPS_POSITION = 0,
    ROTA_GPS_VELOCITY,
    ROTA_GPS_ATTITUDE,
    ROTA_LAP_STATUS,
    ROTA_GPS_QUALITY,
    ROTA_GPS_IMU_ACCEL,
    ROTA_GPS_IMU_GYRO,
    ROTA_GPS_TEMP,
    ROTA_GPS_STATUS,
    ROTA_GPS_MAG,
    ROTA_GPS_FRAME_ORIGIN,
    ROTA_GPS_GATE,
    ROTA_GPS_TIME,
    ROTA_GPS_AIRSPEED,
    ROTA_COUNT,
};

static const canbc_rota_entry_t s_rota[ROTA_COUNT] = {
    [ROTA_GPS_POSITION] = {CAN_ID_GPS_POSITION, CAN_DLC_GPS_POSITION, 5, 0},
    [ROTA_GPS_VELOCITY] = {CAN_ID_GPS_VELOCITY, CAN_DLC_GPS_VELOCITY, 5, 1},
    [ROTA_GPS_ATTITUDE] = {CAN_ID_GPS_ATTITUDE, CAN_DLC_GPS_ATTITUDE, 2, 0},
    [ROTA_LAP_STATUS] = {CAN_ID_LAP_STATUS, CAN_DLC_LAP_STATUS, 10, 2},
    [ROTA_GPS_QUALITY] = {CAN_ID_GPS_QUALITY, CAN_DLC_GPS_QUALITY, 20, 4},
    [ROTA_GPS_IMU_ACCEL] =
        {CAN_ID_GPS_IMU_ACCEL, CAN_DLC_GPS_IMU_ACCEL, 1, 0},
    [ROTA_GPS_IMU_GYRO] = {CAN_ID_GPS_IMU_GYRO, CAN_DLC_GPS_IMU_GYRO, 1, 0},
    [ROTA_GPS_TEMP] = {CAN_ID_GPS_TEMP, CAN_DLC_GPS_TEMP, 100, 8},
    [ROTA_GPS_STATUS] = {CAN_ID_GPS_STATUS, CAN_DLC_GPS_STATUS, 100, 58},
    [ROTA_GPS_MAG] = {CAN_ID_GPS_MAG, CAN_DLC_GPS_MAG, 10, 6},
    /* Dash-facing map data, kept slow to stay light on the bus: origin at
     * 1 Hz; the gate entry fires at 5 Hz but round-robins one slot per
     * fire, so all 8 gates refresh in ~1.6 s. */
    [ROTA_GPS_FRAME_ORIGIN] =
        {CAN_ID_GPS_FRAME_ORIGIN, CAN_DLC_GPS_FRAME_ORIGIN, 100, 9},
    [ROTA_GPS_GATE] = {CAN_ID_GPS_GATE, CAN_DLC_GPS_GATE, 20, 3},
    /* Always broadcast (even before time lock) - the validity flags make
     * an unsynced frame self-describing, and consumers get a liveness
     * signal either way. */
    [ROTA_GPS_TIME] = {CAN_ID_GPS_TIME, CAN_DLC_GPS_TIME, 100, 71},
    /* Phase 7: a slot only the 100 Hz IMU frames otherwise use. */
    [ROTA_GPS_AIRSPEED] =
        {CAN_ID_GPS_AIRSPEED, CAN_DLC_GPS_AIRSPEED, 10, 7},
};

/* Per-message rolling send counters (can_defs.h: the counter byte lets a
 * receiver detect dropped frames). Incremented once per due send - even
 * on a drop - so a gap in the sequence flags exactly that drop. Only the
 * messages that carry a counter field are indexed here; wrap at 256 is
 * intentional. */
static uint8_t s_tx_counter[ROTA_COUNT];
static uint8_t s_lap_event_counter;

static status_t
send_frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    static const uint32_t dlc_code[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8,
    };

    /* The hardware TX FIFO is only 3 deep, and a busy slot's rota queues
     * more frames than that back-to-back - faster than the bus drains
     * them. Wait briefly for a free mailbox (a classic frame clears in
     * well under 1 ms) rather than dropping: dropping-on-full always
     * sacrificed the frames that sort last in the rota (temp, status,
     * time, mag), so they never reached the bus at all. Bounded so a
     * stuck/bus-off peripheral that never drains can't stall the task. */
    for (uint32_t waited = 0U;
         HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U; waited++) {
        if (waited >= CANBC_TX_WAIT_MAX_TICKS) {
            return STATUS_FULL;
        }
        vTaskDelay(1U);
    }

    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = dlc_code[dlc];
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    uint8_t padded[8] = {0};
    memcpy(padded, data, dlc);

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, padded) != HAL_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

static void
send_rota_entry(uint32_t which, const canbc_state_t *s)
{
    uint8_t buf[8];

    switch (which) {
    case ROTA_GPS_POSITION: {
        can_gps_position_t m = {s->lat_deg, s->lon_deg};
        can_pack_gps_position(&m, buf);
        break;
    }
    case ROTA_GPS_VELOCITY: {
        can_gps_velocity_t m = {s->speed_mps, s->course_deg, s->alt_m,
                                 s->fix_type, s->num_sv,
                                 s_tx_counter[ROTA_GPS_VELOCITY]++};
        can_pack_gps_velocity(&m, buf);
        break;
    }
    case ROTA_GPS_ATTITUDE: {
        can_gps_attitude_t m = {s->yaw_deg, s->pitch_deg, s->roll_deg,
                                 s->fusion_status,
                                 s_tx_counter[ROTA_GPS_ATTITUDE]++};
        can_pack_gps_attitude(&m, buf);
        break;
    }
    case ROTA_LAP_STATUS: {
        can_lap_status_t m = {s->lap, s->running_time_ms, s->sector,
                               s->lap_flags};
        can_pack_lap_status(&m, buf);
        break;
    }
    case ROTA_GPS_QUALITY: {
        can_gps_quality_t m = {s->hacc_mm, s->sacc_mm_s, s->pdop,
                                s->quality_flags};
        can_pack_gps_quality(&m, buf);
        break;
    }
    case ROTA_GPS_IMU_ACCEL: {
        can_gps_imu_accel_t m = {s->ax_mg, s->ay_mg, s->az_mg,
                                  s_tx_counter[ROTA_GPS_IMU_ACCEL]++};
        can_pack_gps_imu_accel(&m, buf);
        break;
    }
    case ROTA_GPS_IMU_GYRO: {
        can_gps_imu_gyro_t m = {s->gx_dps, s->gy_dps, s->gz_dps,
                                 s_tx_counter[ROTA_GPS_IMU_GYRO]++};
        can_pack_gps_imu_gyro(&m, buf);
        break;
    }
    case ROTA_GPS_TEMP: {
        can_gps_temp_t m = {s->mcp9800_temp_c, s->imu_temp_c,
                             s->mcu_temp_c};
        can_pack_gps_temp(&m, buf);
        break;
    }
    case ROTA_GPS_STATUS: {
        can_gps_status_t m = {s->uptime_s, s->fault_bits,
                               s->gps_retry_count, s->imu_retry_count,
                               s->cpu_load_pct};
        can_pack_gps_status(&m, buf);
        break;
    }
    case ROTA_GPS_MAG: {
        can_gps_mag_t m = {s->mx_ut, s->my_ut, s->mz_ut,
                            s->mag_cal_status};
        can_pack_gps_mag(&m, buf);
        break;
    }
    case ROTA_GPS_FRAME_ORIGIN: {
        /* Don't advertise an origin until the frame is anchored - a (0,0)
         * origin would drop the dash's gates in the Gulf of Guinea. */
        if (!s->origin_valid) {
            return;
        }
        can_gps_frame_origin_t m = {(double) s->origin_lat_1e7 * 1e-7,
                                     (double) s->origin_lon_1e7 * 1e-7};
        can_pack_gps_frame_origin(&m, buf);
        break;
    }
    case ROTA_GPS_GATE: {
        /* Round-robin one slot per fire so the whole gate set refreshes
         * over a few frames without a burst. */
        static uint8_t rr = 0U;
        uint8_t idx = rr;
        rr = (uint8_t) ((rr + 1U) % LAP_MAX_GATES);

        const canbc_gate_t *g = &s->gates[idx];
        /* Stored as an atan2 result in (-pi, pi]; broadcast as [0, 360).
         * A value a float-rounding hair under zero would round up to
         * exactly 360.0 after the += below, hence the second fold. */
        float heading_deg = g->heading_rad * (180.0f / 3.14159265358979f);
        if (heading_deg < 0.0f) {
            heading_deg += 360.0f;
        }
        if (heading_deg >= 360.0f) {
            heading_deg -= 360.0f;
        }
        can_gps_gate_t m = {
            idx,
            (uint8_t) (g->valid ? CAN_GPS_GATE_FLAG_VALID : 0U),
            g->east_m, g->north_m, heading_deg};
        can_pack_gps_gate(&m, buf);
        send_frame(CAN_ID_GPS_GATE, buf, CAN_DLC_GPS_GATE);
        return;
    }
    case ROTA_GPS_TIME: {
        can_gps_time_t m = {s->itow_ms, s->utc_hour, s->utc_min,
                             s->utc_sec, s->time_flags};
        can_pack_gps_time(&m, buf);
        break;
    }
    case ROTA_GPS_AIRSPEED: {
        /* 4-bit counter: it shares byte 7 with the flags. */
        can_gps_airspeed_t m = {
            s->airspeed_mps, s->wind_speed_mps, s->wind_from_deg,
            s->pitot_scale, s->air_flags,
            (uint8_t) (s_tx_counter[ROTA_GPS_AIRSPEED]++ & 0x0FU)};
        can_pack_gps_airspeed(&m, buf);
        break;
    }
    default:
        return;
    }

    send_frame(s_rota[which].id, buf, s_rota[which].dlc);
}

static void
dispatch_rx_command(const can_gps_command_t *cmd)
{
    app_cmd_t out = {cmd->cmd, cmd->arg0, cmd->arg1};

    switch (cmd->cmd) {
    case CAN_CMD_GATE_SET:
    case CAN_CMD_GATE_CLEAR:
    case CAN_CMD_CONFIG_SAVE: /* imu_task owns the flash store */
        xQueueSend(g_gate_cmd_queue, &out, 0);
        break;
    case CAN_CMD_MAG_CAL_START:
    case CAN_CMD_MAG_CAL_STOP:
        xQueueSend(g_mag_cal_cmd_queue, &out, 0);
        break;
    case CAN_CMD_NMEA_CFG:
        xQueueSend(g_nmea_cfg_queue, &out, 0);
        break;
    default:
        break;
    }
}

static void
canbc_hw_init(void)
{
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

    filter.FilterIndex = 0U;
    filter.FilterID1 = CAN_ID_GPS_COMMAND;
    filter.FilterID2 = 0x7FFU; /* exact-match mask */
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    filter.FilterIndex = 1U;
    filter.FilterID1 = CAN_ID_WHEEL_SPEEDS;
    filter.FilterID2 = 0x7FFU;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    filter.FilterIndex = 2U;
    filter.FilterID1 = CAN_ID_PITOT_PRESSURE;
    filter.FilterID2 = 0x7FFU;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    filter.FilterIndex = 3U;
    filter.FilterID1 = CAN_ID_PITOT_STATUS;
    filter.FilterID2 = 0x7FFU;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    filter.FilterIndex = 4U; /* StdFiltersNbr in fdcan.c counts these */
    filter.FilterID1 = CAN_ID_GPS_GATE_SET;
    filter.FilterID2 = 0x7FFU;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                  FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                    0U);

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
}

/* Bus-off (a shorted or badly disturbed bus drove the transmit error
 * counter past 255) is not self-clearing on this controller: it parks
 * itself in INIT mode and stays silent until software clears INIT, after
 * which it rejoins once it has seen the 129 x 11 recessive bits the CAN
 * spec requires. Without this the node would stay off the bus until the
 * next power cycle after a single wiring glitch. Checked once a second,
 * so a bus that is still faulty isn't hammered. */
static void
recover_from_bus_off(void)
{
    FDCAN_ProtocolStatusTypeDef status;
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &status) == HAL_OK &&
        status.BusOff != 0U) {
        CLEAR_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
    }
}

void
canbc_task_main(void *argument)
{
    (void) argument;

    canbc_hw_init();

    uint32_t slot = 0U;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        canbc_state_t snap;
        canbc_state_get_snapshot(&snap);

        for (uint32_t i = 0U; i < ROTA_COUNT; i++) {
            if ((slot % s_rota[i].period_slots) ==
                (s_rota[i].phase_slot % s_rota[i].period_slots)) {
                send_rota_entry(i, &snap);
            }
        }

        can_lap_event_t evt;
        while (xQueueReceive(g_lap_event_queue, &evt, 0) == pdTRUE) {
            uint8_t buf[8];
            /* Producers leave counter at 0; stamp the rolling value here,
             * where all lap events funnel through the single sender. */
            evt.counter = s_lap_event_counter++;
            can_pack_lap_event(&evt, buf);
            send_frame(CAN_ID_LAP_EVENT, buf, CAN_DLC_LAP_EVENT);
        }

        can_gps_command_t cmd;
        while (xQueueReceive(g_can_rx_cmd_queue, &cmd, 0) == pdTRUE) {
            dispatch_rx_command(&cmd);
        }

        if (slot == 0U) {
            recover_from_bus_off();
        }

        slot = (slot + 1U) % CANBC_SLOT_WRAP;

        /* If this slot overran (only possible when frames can't leave -
         * nothing on the bus to ACK them, or bus-off - so every send
         * waits out its timeout), drop the backlog instead of letting
         * vTaskDelayUntil() replay every missed slot back-to-back the
         * moment the bus returns. */
        if ((xTaskGetTickCount() - last_wake) >
            pdMS_TO_TICKS(2U * CANBC_SLOT_MS)) {
            last_wake = xTaskGetTickCount();
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CANBC_SLOT_MS));
    }
}

void
HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance != FDCAN1) {
        return;
    }
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    /* Latest P4-A temperature, carried into every pressure sample (the
     * status frame only comes at 1 Hz). ISR-only state. */
    static float s_pitot_temp_c;
    static bool s_pitot_temp_valid;

    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
    BaseType_t hp_woken = pdFALSE;

    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) ==
           HAL_OK) {
        if (header.Identifier == CAN_ID_GPS_COMMAND) {
            can_gps_command_t cmd;
            can_unpack_gps_command(data, &cmd);
            xQueueSendFromISR(g_can_rx_cmd_queue, &cmd, &hp_woken);
        } else if (header.Identifier == CAN_ID_GPS_GATE_SET) {
            /* Straight to imu_task, which owns the gate table. A short
             * frame is ignored rather than read past its payload. */
            if (header.DataLength == FDCAN_DLC_BYTES_8) {
                can_gps_gate_t gate;
                can_unpack_gps_gate(data, &gate);
                xQueueSendFromISR(g_gate_set_queue, &gate, &hp_woken);
            }
        } else if (header.Identifier == CAN_ID_WHEEL_SPEEDS) {
            /* Front-left only - see CAN_ID_WHEEL_SPEEDS above. */
            int16_t fl = (int16_t) (uint16_t) (
                data[WHEEL_SPEEDS_FL_BYTE] |
                (data[WHEEL_SPEEDS_FL_BYTE + 1U] << 8));
            float speed_mps = 0.01f * (float) fl;
            xQueueOverwriteFromISR(g_wheelspeed_queue, &speed_mps,
                                    &hp_woken);
        } else if (header.Identifier == CAN_ID_PITOT_PRESSURE) {
            can_pitot_pressure_t p;
            can_unpack_pitot_pressure(data, &p);
            app_pitot_sample_t sample;
            for (uint32_t i = 0U; i < 4U; i++) {
                sample.ch_mbar[i] = p.ch_mbar[i];
            }
            sample.temp_c = s_pitot_temp_c;
            sample.temp_valid = s_pitot_temp_valid;
            xQueueOverwriteFromISR(g_pitot_queue, &sample, &hp_woken);
        } else if (header.Identifier == CAN_ID_PITOT_STATUS) {
            can_pitot_status_t st;
            can_unpack_pitot_status(data, &st);
            s_pitot_temp_c = st.temperature_c;
            s_pitot_temp_valid = true;
        }
    }

    portYIELD_FROM_ISR(hp_woken);
}
