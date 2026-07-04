/**
 * @file    ubx.c
 * @brief   u-blox UBX protocol: byte-stream parser, frame builder and
 *          message decoders.
 */

#include "gps/ubx.h"

#include <string.h>

#define UBX_SYNC1 0xB5U
#define UBX_SYNC2 0x62U

enum {
    UBX_ST_SYNC1 = 0,
    UBX_ST_SYNC2,
    UBX_ST_CLASS,
    UBX_ST_ID,
    UBX_ST_LEN1,
    UBX_ST_LEN2,
    UBX_ST_PAYLOAD,
    UBX_ST_CK_A,
    UBX_ST_CK_B,
};

/* Fletcher-style running checksum: fold in bytes as they are seen so a
 * truncated capture still validates against the sender's checksum, which
 * covers the full declared length, not just what fit in payload[]. */
static void
ubx_ck_update(ubx_parser_t *p, uint8_t byte)
{
    p->ck_a = (uint8_t) (p->ck_a + byte);
    p->ck_b = (uint8_t) (p->ck_b + p->ck_a);
}

void
ubx_parser_init(ubx_parser_t *p)
{
    p->state = UBX_ST_SYNC1;
    p->idx = 0U;
    p->ck_a = 0U;
    p->ck_b = 0U;
    memset(&p->frame, 0, sizeof(p->frame));
}

bool
ubx_parser_feed(ubx_parser_t *p, uint8_t byte)
{
    switch (p->state) {
    case UBX_ST_SYNC1:
        if (byte == UBX_SYNC1) {
            p->state = UBX_ST_SYNC2;
        }
        break;

    case UBX_ST_SYNC2:
        if (byte == UBX_SYNC2) {
            p->ck_a = 0U;
            p->ck_b = 0U;
            p->state = UBX_ST_CLASS;
        } else if (byte != UBX_SYNC1) {
            /* not a sync pair; stay parked on SYNC1 unless this byte
             * itself could start a new one */
            p->state = UBX_ST_SYNC1;
        }
        break;

    case UBX_ST_CLASS:
        p->frame.cls = byte;
        ubx_ck_update(p, byte);
        p->state = UBX_ST_ID;
        break;

    case UBX_ST_ID:
        p->frame.id = byte;
        ubx_ck_update(p, byte);
        p->state = UBX_ST_LEN1;
        break;

    case UBX_ST_LEN1:
        p->frame.len = byte;
        ubx_ck_update(p, byte);
        p->state = UBX_ST_LEN2;
        break;

    case UBX_ST_LEN2:
        p->frame.len = (uint16_t) (p->frame.len | ((uint16_t) byte << 8));
        ubx_ck_update(p, byte);
        p->idx = 0U;
        p->frame.truncated = (p->frame.len > UBX_MAX_PAYLOAD);
        p->state = (p->frame.len == 0U) ? UBX_ST_CK_A : UBX_ST_PAYLOAD;
        break;

    case UBX_ST_PAYLOAD:
        ubx_ck_update(p, byte);
        if (p->idx < UBX_MAX_PAYLOAD) {
            p->frame.payload[p->idx] = byte;
        }
        p->idx++;
        if (p->idx >= p->frame.len) {
            p->state = UBX_ST_CK_A;
        }
        break;

    case UBX_ST_CK_A:
        p->state = UBX_ST_SYNC1;
        if (byte != p->ck_a) {
            return false;
        }
        p->state = UBX_ST_CK_B;
        break;

    case UBX_ST_CK_B:
        p->state = UBX_ST_SYNC1;
        return (byte == p->ck_b);

    default:
        p->state = UBX_ST_SYNC1;
        break;
    }

    return false;
}

uint16_t
ubx_frame_build(uint8_t *buf, uint16_t buf_size, uint8_t cls, uint8_t id,
                const uint8_t *payload, uint16_t len)
{
    /* sync(2) + class(1) + id(1) + len(2) + payload + ck(2) */
    uint16_t total = (uint16_t) (8U + len);
    uint8_t ck_a = 0U;
    uint8_t ck_b = 0U;
    uint16_t i;

    if (buf_size < total) {
        return 0U;
    }

    buf[0] = UBX_SYNC1;
    buf[1] = UBX_SYNC2;
    buf[2] = cls;
    buf[3] = id;
    buf[4] = (uint8_t) (len & 0xFFU);
    buf[5] = (uint8_t) (len >> 8);
    if (len > 0U) {
        memcpy(&buf[6], payload, len);
    }

    for (i = 2U; i < (uint16_t) (6U + len); i++) {
        ck_a = (uint8_t) (ck_a + buf[i]);
        ck_b = (uint8_t) (ck_b + ck_a);
    }
    buf[6U + len] = ck_a;
    buf[7U + len] = ck_b;

    return total;
}

bool
ubx_decode_nav_pvt(const ubx_frame_t *f, ubx_nav_pvt_t *out)
{
    const uint8_t *b = f->payload;

    if (f->cls != UBX_CLASS_NAV || f->id != UBX_NAV_PVT) {
        return false;
    }
    if (f->len < 92U) {
        return false;
    }

    out->itow_ms = ubx_rd_u4(&b[0]);
    out->year = ubx_rd_u2(&b[4]);
    out->month = b[6];
    out->day = b[7];
    out->hour = b[8];
    out->min = b[9];
    out->sec = b[10];
    out->valid = b[11];
    out->fix_type = b[20];
    out->flags = b[21];
    out->num_sv = b[23];
    out->lon_1e7 = ubx_rd_i4(&b[24]);
    out->lat_1e7 = ubx_rd_i4(&b[28]);
    out->height_mm = ubx_rd_i4(&b[32]);
    out->hmsl_mm = ubx_rd_i4(&b[36]);
    out->hacc_mm = ubx_rd_u4(&b[40]);
    out->vacc_mm = ubx_rd_u4(&b[44]);
    out->vel_n_mms = ubx_rd_i4(&b[48]);
    out->vel_e_mms = ubx_rd_i4(&b[52]);
    out->vel_d_mms = ubx_rd_i4(&b[56]);
    out->gspeed_mms = ubx_rd_i4(&b[60]);
    out->head_motion_1e5 = ubx_rd_i4(&b[64]);
    out->sacc_mms = ubx_rd_u4(&b[68]);
    out->headacc_1e5 = ubx_rd_u4(&b[72]);
    out->pdop_1e2 = ubx_rd_u2(&b[76]);

    return true;
}

bool
ubx_decode_tim_tm2(const ubx_frame_t *f, ubx_tim_tm2_t *out)
{
    const uint8_t *b = f->payload;

    if (f->cls != UBX_CLASS_TIM || f->id != UBX_TIM_TM2) {
        return false;
    }
    if (f->len < 28U) {
        return false;
    }

    out->ch = b[0];
    out->flags = b[1];
    out->count = ubx_rd_u2(&b[2]);
    out->wn_rising = ubx_rd_u2(&b[4]);
    out->wn_falling = ubx_rd_u2(&b[6]);
    out->tow_ms_rising = ubx_rd_u4(&b[8]);
    out->tow_sub_ms_rising_ns = ubx_rd_u4(&b[12]);
    out->tow_ms_falling = ubx_rd_u4(&b[16]);
    out->tow_sub_ms_falling_ns = ubx_rd_u4(&b[20]);
    out->acc_est_ns = ubx_rd_u4(&b[24]);

    return true;
}

bool
ubx_decode_ack(const ubx_frame_t *f, ubx_ack_t *out)
{
    if (f->cls != UBX_CLASS_ACK) {
        return false;
    }
    if (f->id != UBX_ACK_ACK && f->id != UBX_ACK_NAK) {
        return false;
    }
    if (f->len < 2U) {
        return false;
    }

    out->acked_cls = f->payload[0];
    out->acked_id = f->payload[1];
    out->ack = (f->id == UBX_ACK_ACK);

    return true;
}

uint16_t
ubx_rd_u2(const uint8_t *b)
{
    return (uint16_t) (b[0] | ((uint16_t) b[1] << 8));
}

uint32_t
ubx_rd_u4(const uint8_t *b)
{
    return (uint32_t) b[0] | ((uint32_t) b[1] << 8) |
           ((uint32_t) b[2] << 16) | ((uint32_t) b[3] << 24);
}

int32_t
ubx_rd_i4(const uint8_t *b)
{
    return (int32_t) ubx_rd_u4(b);
}
