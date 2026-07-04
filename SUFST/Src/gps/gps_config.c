/**
 * @file    gps_config.c
 * @brief   ZED-F9P boot configuration (see gps_config.h for caveats on
 *          the CFG key table below).
 */

#include "gps/gps_config.h"

#include <string.h>

#include "main.h"

#include "board/board_config.h"

/* CFG-* key IDs (u-blox VALSET/VALGET), with their storage width in
 * bytes. See gps_config.h - verify against the ZED-F9P interface manual. */
#define KEY_UART1_BAUDRATE     0x40520001U /* U4 */
#define KEY_UART1OUTPROT_UBX   0x10740001U /* L  */
#define KEY_UART1OUTPROT_NMEA  0x10740002U /* L  */
#define KEY_RATE_MEAS          0x30210001U /* U2, ms */
#define KEY_NAVSPG_DYNMODEL    0x20110021U /* E1 */
#define KEY_SIGNAL_GPS_ENA     0x1031001FU /* L */
#define KEY_SIGNAL_GAL_ENA     0x10310021U /* L */
#define KEY_SIGNAL_GLO_ENA     0x10310025U /* L */
#define KEY_SIGNAL_BDS_ENA     0x10310022U /* L */
#define KEY_SIGNAL_QZSS_ENA    0x10310024U /* L */
#define KEY_MSGOUT_NAV_PVT_U1  0x20910007U /* U1, rate in measurement cycles */
#define KEY_MSGOUT_TIM_TM2_U1  0x20910179U /* U1 */

#define DYNMODEL_AUTOMOTIVE 4U

#define VALSET_LAYER_RAM (1U << 0)

#define GPS_CFG_RETRY_LIMIT   3U
#define GPS_CFG_ACK_TIMEOUT_MS 500U

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

/* Blocking UART TX (config happens once at boot, well before any task
 * needs USART3 concurrently) with a bounded wait for a matching UBX
 * frame, feeding the byte-at-a-time parser directly from HAL_UART_Receive
 * polling - simpler than standing up the DMA ring this early, and boot
 * config only runs once. */
static bool
send_and_wait(const uint8_t *frame, uint16_t frame_len, ubx_parser_t *p,
              uint8_t want_cls, uint8_t want_id, uint32_t timeout_ms)
{
    if (HAL_UART_Transmit(&huart3, (uint8_t *) frame, frame_len, 200U) !=
        HAL_OK) {
        return false;
    }

    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms) {
        uint8_t byte;
        if (HAL_UART_Receive(&huart3, &byte, 1U, 20U) != HAL_OK) {
            continue;
        }
        if (ubx_parser_feed(p, byte)) {
            if (p->frame.cls == want_cls && p->frame.id == want_id) {
                return true;
            }
        }
    }
    return false;
}

static bool
try_probe_baud(uint32_t baud)
{
    huart3.Init.BaudRate = baud;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        return false;
    }

    uint8_t frame[16];
    uint16_t len =
        ubx_frame_build(frame, sizeof(frame), UBX_CLASS_MON, UBX_MON_VER,
                        NULL, 0U);

    ubx_parser_t p;
    ubx_parser_init(&p);
    return send_and_wait(frame, len, &p, UBX_CLASS_MON, UBX_MON_VER, 300U);
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
    return ubx_decode_ack(&p.frame, &ack) && ack.ack;
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
     * x2) then repeated key(4)+value entries, same layout as VALSET. */
    if (p.frame.len < 8U) {
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
    bool at_target = try_probe_baud(GPS_BAUD_TARGET);

    if (!at_target) {
        if (!try_probe_baud(GPS_BAUD_DEFAULT)) {
            return STATUS_TIMEOUT;
        }

        /* The ACK to this VALSET is sent at the OLD baud but may
         * straddle the switch to the new one - per gps_config.h, it is
         * never trusted; only the VALGET readback below is. Retrying
         * on a missing/garbled ACK here would just resend into a
         * receiver that may already have applied it, so this is a
         * single best-effort attempt. */
        kv_t baud_kv = {KEY_UART1_BAUDRATE, GPS_BAUD_TARGET, 4U};
        (void) apply_valset(&baud_kv, 1U);

        huart3.Init.BaudRate = GPS_BAUD_TARGET;
        if (HAL_UART_Init(&huart3) != HAL_OK) {
            return STATUS_ERROR;
        }

        if (!verify_u4(KEY_UART1_BAUDRATE, GPS_BAUD_TARGET)) {
            return STATUS_ERROR;
        }
    }

    kv_t cfg[] = {
        {KEY_UART1OUTPROT_UBX, 1U, 1U},
        {KEY_UART1OUTPROT_NMEA, 0U, 1U},
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
            return STATUS_OK;
        }
    }

    return STATUS_ERROR;
}
