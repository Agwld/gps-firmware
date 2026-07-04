/**
 * @file    ubx.h
 * @brief   u-blox UBX protocol: byte-stream parser, frame builder and
 *          message decoders. Pure C, no HAL - host testable.
 *
 * The parser is fed one byte at a time so it is inherently safe against
 * frames straddling the DMA ring wrap; the caller just iterates the ring
 * with modulo indices.
 */

#ifndef UBX_H
#define UBX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Message classes / IDs used by this firmware */
#define UBX_CLASS_NAV 0x01U
#define UBX_CLASS_ACK 0x05U
#define UBX_CLASS_CFG 0x06U
#define UBX_CLASS_MON 0x0AU
#define UBX_CLASS_TIM 0x0DU

#define UBX_NAV_PVT   0x07U
#define UBX_ACK_NAK   0x00U
#define UBX_ACK_ACK   0x01U
#define UBX_CFG_VALSET 0x8AU
#define UBX_CFG_VALGET 0x8BU
#define UBX_MON_VER   0x04U
#define UBX_TIM_TM2   0x03U

/* Largest payload fully captured (NAV-PVT = 92). Longer frames (MON-VER)
 * are checksum-validated but delivered truncated. */
#define UBX_MAX_PAYLOAD 96U

typedef struct {
    uint8_t cls;
    uint8_t id;
    uint16_t len; /* length from header (may exceed captured payload) */
    bool truncated;
    uint8_t payload[UBX_MAX_PAYLOAD];
} ubx_frame_t;

typedef struct {
    uint8_t state;
    uint16_t idx;
    uint8_t ck_a;
    uint8_t ck_b;
    ubx_frame_t frame;
} ubx_parser_t;

void ubx_parser_init(ubx_parser_t *p);

/**
 * @brief Feed one byte; returns true when p->frame holds a complete frame
 *        with a valid checksum.
 */
bool ubx_parser_feed(ubx_parser_t *p, uint8_t byte);

/**
 * @brief Build a full UBX frame (sync + header + payload + checksum).
 * @return total frame size, or 0 if buf_size is too small.
 */
uint16_t ubx_frame_build(uint8_t *buf, uint16_t buf_size, uint8_t cls,
                         uint8_t id, const uint8_t *payload, uint16_t len);

/* ------------------------------------------------------------------ */
/* Decoded messages                                                    */
/* ------------------------------------------------------------------ */

typedef struct {
    uint32_t itow_ms;
    uint16_t year;
    uint8_t month, day, hour, min, sec;
    uint8_t valid;      /* validDate/validTime/fullyResolved bits */
    uint8_t fix_type;   /* 0 none, 2 2D, 3 3D, 4 GNSS+DR */
    uint8_t flags;      /* gnssFixOK in bit 0, carrSoln in bits 6-7 */
    uint8_t num_sv;
    int32_t lon_1e7;    /* deg * 1e-7 */
    int32_t lat_1e7;
    int32_t height_mm;  /* above ellipsoid */
    int32_t hmsl_mm;    /* above mean sea level */
    uint32_t hacc_mm;
    uint32_t vacc_mm;
    int32_t vel_n_mms;
    int32_t vel_e_mms;
    int32_t vel_d_mms;
    int32_t gspeed_mms;
    int32_t head_motion_1e5; /* deg * 1e-5 */
    uint32_t sacc_mms;
    uint32_t headacc_1e5;
    uint16_t pdop_1e2;
} ubx_nav_pvt_t;

typedef struct {
    uint8_t ch;
    uint8_t flags;       /* bit2 newFallingEdge, bit7 newRisingEdge */
    uint16_t count;      /* rising edge counter */
    uint16_t wn_rising;
    uint16_t wn_falling;
    uint32_t tow_ms_rising;
    uint32_t tow_sub_ms_rising_ns;
    uint32_t tow_ms_falling;
    uint32_t tow_sub_ms_falling_ns;
    uint32_t acc_est_ns;
} ubx_tim_tm2_t;

#define UBX_TM2_FLAG_NEW_RISING  (1U << 7)
#define UBX_TM2_FLAG_NEW_FALLING (1U << 2)

typedef struct {
    uint8_t acked_cls;
    uint8_t acked_id;
    bool ack; /* true = ACK-ACK, false = ACK-NAK */
} ubx_ack_t;

/* Each returns true when the frame matches and is long enough. */
bool ubx_decode_nav_pvt(const ubx_frame_t *f, ubx_nav_pvt_t *out);
bool ubx_decode_tim_tm2(const ubx_frame_t *f, ubx_tim_tm2_t *out);
bool ubx_decode_ack(const ubx_frame_t *f, ubx_ack_t *out);

/* Little-endian readers shared with other decoders */
uint16_t ubx_rd_u2(const uint8_t *b);
uint32_t ubx_rd_u4(const uint8_t *b);
int32_t ubx_rd_i4(const uint8_t *b);

#ifdef __cplusplus
}
#endif

#endif /* UBX_H */
