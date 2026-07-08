/**
 * @file    gps_config.c
 * @brief   ZED-F9P boot configuration over I2C (see gps_config.h).
 *
 * All frames go out - and all ACK/VALGET responses come back - on the
 * I2C transport (gps_i2c.c), because the board has no MCU->GPS UART
 * path. The configuration itself still targets UART1: that is the port
 * the UBX stream leaves on (hard-wired to USART3 RX) and the port RTCM
 * corrections enter on (RS232 in via JP7 bridged 2-3).
 */

#include "gps/gps_config.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"

#include "board/board_config.h"
#include "gps/gps_i2c.h"

/* CFG-* key IDs (u-blox VALSET/VALGET), with their storage width in
 * bytes. See gps_config.h - verify against the ZED-F9P interface manual. */
#define KEY_UART1_BAUDRATE      0x40520001U /* U4 */
#define KEY_UART1OUTPROT_UBX    0x10740001U /* L  */
#define KEY_UART1OUTPROT_NMEA   0x10740002U /* L  */
#define KEY_UART1INPROT_RTCM3X  0x10730004U /* L  */
#define KEY_RATE_MEAS           0x30210001U /* U2, ms */
#define KEY_NAVSPG_DYNMODEL     0x20110021U /* E1 */
#define KEY_SIGNAL_GPS_ENA      0x1031001FU /* L */
#define KEY_SIGNAL_GAL_ENA      0x10310021U /* L */
#define KEY_SIGNAL_GLO_ENA      0x10310025U /* L */
#define KEY_SIGNAL_BDS_ENA      0x10310022U /* L */
#define KEY_SIGNAL_QZSS_ENA     0x10310024U /* L */
#define KEY_MSGOUT_NAV_PVT_U1   0x20910007U /* U1, rate in meas. cycles */
#define KEY_MSGOUT_TIM_TM2_U1   0x20910179U /* U1 */

#define DYNMODEL_AUTOMOTIVE 4U

#define VALSET_LAYER_RAM (1U << 0)

#define GPS_CFG_RETRY_LIMIT    3U
#define GPS_CFG_ACK_TIMEOUT_MS 500U
/* The F9P takes up to ~1 s after power-up before its DDC port answers;
 * probe attempts are spaced to ride that out without tripping on it. */
#define GPS_PROBE_RETRY_LIMIT  8U
#define GPS_PROBE_TIMEOUT_MS   300U

typedef struct {
    uint32_t key;
    uint32_t value;
    uint8_t width; /* 1, 2 or 4 bytes of `value` are written, LE */
} kv_t;

static uint16_t
build_valset(uint8_t *buf, uint16_t buf_size, const kv_t *kv, uint16_t count)
{
    uint8_t payload[192];
    uint16_t idx = 0U;

    payload[idx++] = 0x00U; /* version */
    payload[idx++] = VALSET_LAYER_RAM;
    payload[idx++] = 0x00U; /* reserved */
    payload[idx++] = 0x00U; /* reserved */

    for (uint16_t i = 0U; i < count; i++) {
        /* 4 key bytes + up to `width` value bytes; bail rather than
         * overrun the fixed payload buffer if the table ever outgrows it. */
        if ((size_t) idx + 4U + kv[i].width > sizeof(payload)) {
            return 0U;
        }
        payload[idx++] = (uint8_t) (kv[i].key & 0xFFU);
        payload[idx++] = (uint8_t) ((kv[i].key >> 8) & 0xFFU);
        payload[idx++] = (uint8_t) ((kv[i].key >> 16) & 0xFFU);
        payload[idx++] = (uint8_t) ((kv[i].key >> 24) & 0xFFU);

        for (uint8_t b = 0U; b < kv[i].width; b++) {
            payload[idx++] = (uint8_t) ((kv[i].value >> (8U * b)) & 0xFFU);
        }
    }

    return ubx_frame_build(buf, buf_size, UBX_CLASS_CFG, UBX_CFG_VALSET,
                            payload, idx);
}

static uint16_t
build_valget(uint8_t *buf, uint16_t buf_size, uint32_t key)
{
    uint8_t payload[8];
    payload[0] = 0x00U; /* version: request */
    payload[1] = 0x00U; /* layer: RAM */
    payload[2] = 0x00U;
    payload[3] = 0x00U;
    payload[4] = (uint8_t) (key & 0xFFU);
    payload[5] = (uint8_t) ((key >> 8) & 0xFFU);
    payload[6] = (uint8_t) ((key >> 16) & 0xFFU);
    payload[7] = (uint8_t) ((key >> 24) & 0xFFU);

    return ubx_frame_build(buf, buf_size, UBX_CLASS_CFG, UBX_CFG_VALGET,
                            payload, sizeof(payload));
}

/* Send a frame over I2C and poll the DDC output stream for a matching
 * UBX response, feeding the byte-at-a-time parser. The stream may carry
 * unrelated traffic (the F9P outputs NMEA on I2C by default until this
 * boot sequence quiets UART1 - I2C output protocol is left at default);
 * the parser simply resyncs past it. Polls yield with vTaskDelay so the
 * boot sequence can't starve lower-priority tasks (sys_task feeds the
 * watchdog). */
static bool
send_and_wait(const uint8_t *frame, uint16_t frame_len, ubx_parser_t *p,
              uint8_t want_cls, uint8_t want_id, uint32_t timeout_ms)
{
    if (gps_i2c_write(frame, frame_len) != STATUS_OK) {
        return false;
    }

    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms)) {
        uint8_t chunk[32];
        uint16_t n = 0U;
        if (gps_i2c_read(chunk, sizeof(chunk), &n) != STATUS_OK) {
            return false;
        }
        for (uint16_t i = 0U; i < n; i++) {
            if (ubx_parser_feed(p, chunk[i])) {
                if (p->frame.cls == want_cls && p->frame.id == want_id) {
                    return true;
                }
            }
        }
        /* Yield every iteration, not only on an empty read: the DDC port
         * streams NMEA continuously at boot, so a data-present fast path
         * would busy-loop on blocking I2C reads at task priority and
         * starve lower-priority tasks (sys_task feeds the watchdog). */
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return false;
}

/* Is the receiver awake and talking UBX on the DDC port yet? */
static bool
probe(void)
{
    uint8_t frame[16];
    uint16_t len = ubx_frame_build(frame, sizeof(frame), UBX_CLASS_MON,
                                    UBX_MON_VER, NULL, 0U);

    ubx_parser_t p;
    ubx_parser_init(&p);
    return send_and_wait(frame, len, &p, UBX_CLASS_MON, UBX_MON_VER,
                          GPS_PROBE_TIMEOUT_MS);
}

static bool
apply_valset(const kv_t *kv, uint16_t count)
{
    uint8_t frame[256];
    uint16_t len = build_valset(frame, sizeof(frame), kv, count);
    if (len == 0U) {
        return false;
    }

    ubx_parser_t p;
    ubx_parser_init(&p);
    if (!send_and_wait(frame, len, &p, UBX_CLASS_ACK, UBX_ACK_ACK,
                        GPS_CFG_ACK_TIMEOUT_MS)) {
        /* Might have been an ACK-NAK, which send_and_wait() also parses
         * but doesn't match on cls/id here since NAK's id differs -
         * either way, no ACK-ACK means treat this attempt as failed. */
        return false;
    }

    ubx_ack_t ack;
    /* Confirm the ACK actually acknowledges *our* CFG-VALSET, not some
     * unrelated ACK-ACK that happened to be on the DDC stream. */
    return ubx_decode_ack(&p.frame, &ack) && ack.ack &&
           ack.acked_cls == UBX_CLASS_CFG && ack.acked_id == UBX_CFG_VALSET;
}

static bool
verify_u4(uint32_t key, uint32_t expected)
{
    uint8_t frame[16];
    uint16_t len = build_valget(frame, sizeof(frame), key);

    ubx_parser_t p;
    ubx_parser_init(&p);
    if (!send_and_wait(frame, len, &p, UBX_CLASS_CFG, UBX_CFG_VALGET,
                        GPS_CFG_ACK_TIMEOUT_MS)) {
        return false;
    }

    /* VALGET response payload: 4-byte header (version, layer, reserved
     * x2) then repeated key(4)+value entries, same layout as VALSET. A U4
     * value therefore needs 4 (header) + 4 (key) + 4 (value) = 12 bytes;
     * anything shorter can't hold the value read at payload[8..11]. */
    if (p.frame.len < 12U) {
        return false;
    }
    uint32_t got_key = ubx_rd_u4(&p.frame.payload[4]);
    if (got_key != key) {
        return false;
    }
    uint32_t got_val = ubx_rd_u4(&p.frame.payload[8]);
    return got_val == expected;
}

status_t
gps_config_run_boot_sequence(void)
{
    bool alive = false;
    for (uint32_t attempt = 0U;
         attempt < GPS_PROBE_RETRY_LIMIT && !alive; attempt++) {
        alive = probe();
    }
    if (!alive) {
        return STATUS_TIMEOUT;
    }

    /* One shot, RAM layer (reapplied every boot). The baud key switches
     * UART1 to GPS_UART_BAUD - safe to include in the batch because the
     * ACK comes back on I2C, unaffected by the UART changing under it
     * (the old firmware's careful old-baud/new-baud ACK dance existed
     * only because config used to ride the same UART it was retuning). */
    kv_t cfg[] = {
        {KEY_UART1_BAUDRATE, GPS_UART_BAUD, 4U},
        {KEY_UART1OUTPROT_UBX, 1U, 1U},
        {KEY_UART1OUTPROT_NMEA, 0U, 1U},
        /* RTCM corrections arrive on UART1 directly from the RS232
         * input (JP7 bridged 2-3) - the MCU never sees them. Enabled
         * explicitly even though it's the F9P default, so the intent
         * survives a future default-tightening pass. */
        {KEY_UART1INPROT_RTCM3X, 1U, 1U},
        {KEY_RATE_MEAS, GPS_MEAS_PERIOD_MS, 2U},
        {KEY_NAVSPG_DYNMODEL, DYNMODEL_AUTOMOTIVE, 1U},
        {KEY_SIGNAL_GPS_ENA, 1U, 1U},
        {KEY_SIGNAL_GAL_ENA, 1U, 1U},
        {KEY_SIGNAL_GLO_ENA, 0U, 1U},
        {KEY_SIGNAL_BDS_ENA, 0U, 1U},
        {KEY_SIGNAL_QZSS_ENA, 0U, 1U},
        {KEY_MSGOUT_NAV_PVT_U1, 1U, 1U},
        {KEY_MSGOUT_TIM_TM2_U1, 1U, 1U},
    };

    for (uint32_t attempt = 0U; attempt < GPS_CFG_RETRY_LIMIT; attempt++) {
        if (apply_valset(cfg, sizeof(cfg) / sizeof(cfg[0]))) {
            /* Readback (not the ACK) is what's trusted, per gps_config.h. */
            if (verify_u4(KEY_UART1_BAUDRATE, GPS_UART_BAUD)) {
                return STATUS_OK;
            }
            return STATUS_ERROR;
        }
    }

    return STATUS_ERROR;
}
