/**
 * @file    test_ubx.c
 * @brief   Host unit tests for the UBX byte-stream parser, frame builder
 *          and message decoders.
 */

#include "gps/ubx.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void
wr_u2(uint8_t *b, uint16_t v)
{
    b[0] = (uint8_t) (v & 0xFFU);
    b[1] = (uint8_t) (v >> 8);
}

static void
wr_u4(uint8_t *b, uint32_t v)
{
    b[0] = (uint8_t) (v & 0xFFU);
    b[1] = (uint8_t) ((v >> 8) & 0xFFU);
    b[2] = (uint8_t) ((v >> 16) & 0xFFU);
    b[3] = (uint8_t) ((v >> 24) & 0xFFU);
}

static void
wr_i4(uint8_t *b, int32_t v)
{
    wr_u4(b, (uint32_t) v);
}

/* Feeds a built frame byte-by-byte, asserting the parser only reports a
 * complete frame on the final byte, and returns that final result so
 * callers can tell success from checksum failure. */
static bool
feed_frame(ubx_parser_t *p, const uint8_t *buf, uint16_t n)
{
    bool got = false;
    uint16_t i;

    for (i = 0; i < n; i++) {
        got = ubx_parser_feed(p, buf[i]);
        if (i + 1U < n) {
            assert(!got);
        }
    }
    return got;
}

static void
fill_nav_pvt_payload(uint8_t *p)
{
    memset(p, 0, 92);
    wr_u4(&p[0], 123456789U);       /* iTOW */
    wr_u2(&p[4], 2026U);             /* year */
    p[6] = 7U;                       /* month */
    p[7] = 4U;                       /* day */
    p[8] = 12U;                      /* hour */
    p[9] = 34U;                      /* min */
    p[10] = 56U;                     /* sec */
    p[11] = 0x37U;                   /* valid */
    p[20] = 3U;                      /* fixType */
    p[21] = 0x81U;                   /* flags */
    p[23] = 14U;                     /* numSV */
    wr_i4(&p[24], -1234567);         /* lon */
    wr_i4(&p[28], 543210987);        /* lat */
    wr_i4(&p[32], 111222);           /* height */
    wr_i4(&p[36], 111000);           /* hMSL */
    wr_u4(&p[40], 850U);             /* hAcc */
    wr_u4(&p[44], 1200U);            /* vAcc */
    wr_i4(&p[48], 500);              /* velN */
    wr_i4(&p[52], -300);             /* velE */
    wr_i4(&p[56], 20);               /* velD */
    wr_i4(&p[60], 583);              /* gSpeed */
    wr_i4(&p[64], 9000000);          /* headMot */
    wr_u4(&p[68], 400U);             /* sAcc */
    wr_u4(&p[72], 2500000U);         /* headAcc */
    wr_u2(&p[76], 145U);             /* pDOP */
}

static void
check_nav_pvt_fields(const ubx_nav_pvt_t *d)
{
    assert(d->itow_ms == 123456789U);
    assert(d->year == 2026U);
    assert(d->month == 7U);
    assert(d->day == 4U);
    assert(d->hour == 12U);
    assert(d->min == 34U);
    assert(d->sec == 56U);
    assert(d->valid == 0x37U);
    assert(d->fix_type == 3U);
    assert(d->flags == 0x81U);
    assert(d->num_sv == 14U);
    assert(d->lon_1e7 == -1234567);
    assert(d->lat_1e7 == 543210987);
    assert(d->height_mm == 111222);
    assert(d->hmsl_mm == 111000);
    assert(d->hacc_mm == 850U);
    assert(d->vacc_mm == 1200U);
    assert(d->vel_n_mms == 500);
    assert(d->vel_e_mms == -300);
    assert(d->vel_d_mms == 20);
    assert(d->gspeed_mms == 583);
    assert(d->head_motion_1e5 == 9000000);
    assert(d->sacc_mms == 400U);
    assert(d->headacc_1e5 == 2500000U);
    assert(d->pdop_1e2 == 145U);
}

static void
test_nav_pvt_roundtrip(void)
{
    uint8_t payload[92];
    uint8_t buf[128];
    uint16_t n;
    ubx_parser_t p;
    ubx_nav_pvt_t decoded;

    fill_nav_pvt_payload(payload);

    n = ubx_frame_build(buf, sizeof(buf), UBX_CLASS_NAV, UBX_NAV_PVT, payload,
                         sizeof(payload));
    assert(n == 8U + sizeof(payload));

    ubx_parser_init(&p);
    assert(feed_frame(&p, buf, n));

    assert(p.frame.cls == UBX_CLASS_NAV);
    assert(p.frame.id == UBX_NAV_PVT);
    assert(p.frame.len == sizeof(payload));
    assert(!p.frame.truncated);

    assert(ubx_decode_nav_pvt(&p.frame, &decoded));
    check_nav_pvt_fields(&decoded);
}

static void
fill_tim_tm2_payload(uint8_t *p)
{
    memset(p, 0, 28);
    p[0] = 0U;                                 /* ch */
    p[1] = UBX_TM2_FLAG_NEW_RISING |
           UBX_TM2_FLAG_NEW_FALLING;            /* flags */
    wr_u2(&p[2], 42U);                          /* count */
    wr_u2(&p[4], 2350U);                        /* wnR */
    wr_u2(&p[6], 2350U);                        /* wnF */
    wr_u4(&p[8], 111222333U);                   /* towMsR */
    wr_u4(&p[12], 444555U);                     /* towSubMsR */
    wr_u4(&p[16], 111222340U);                  /* towMsF */
    wr_u4(&p[20], 987654U);                     /* towSubMsF */
    wr_u4(&p[24], 15U);                         /* accEst */
}

static void
test_tim_tm2_roundtrip(void)
{
    uint8_t payload[28];
    uint8_t buf[64];
    uint16_t n;
    ubx_parser_t p;
    ubx_tim_tm2_t decoded;

    fill_tim_tm2_payload(payload);

    n = ubx_frame_build(buf, sizeof(buf), UBX_CLASS_TIM, UBX_TIM_TM2, payload,
                         sizeof(payload));
    assert(n == 8U + sizeof(payload));

    ubx_parser_init(&p);
    assert(feed_frame(&p, buf, n));

    assert(ubx_decode_tim_tm2(&p.frame, &decoded));
    assert(decoded.ch == 0U);
    assert(decoded.flags ==
           (UBX_TM2_FLAG_NEW_RISING | UBX_TM2_FLAG_NEW_FALLING));
    assert(decoded.count == 42U);
    assert(decoded.wn_rising == 2350U);
    assert(decoded.wn_falling == 2350U);
    assert(decoded.tow_ms_rising == 111222333U);
    assert(decoded.tow_sub_ms_rising_ns == 444555U);
    assert(decoded.tow_ms_falling == 111222340U);
    assert(decoded.tow_sub_ms_falling_ns == 987654U);
    assert(decoded.acc_est_ns == 15U);
}

static void
test_ack_roundtrip(void)
{
    uint8_t payload[2];
    uint8_t buf[16];
    uint16_t n;
    ubx_parser_t p;
    ubx_ack_t decoded;

    /* ACK-ACK for CFG-VALSET */
    payload[0] = UBX_CLASS_CFG;
    payload[1] = UBX_CFG_VALSET;

    n = ubx_frame_build(buf, sizeof(buf), UBX_CLASS_ACK, UBX_ACK_ACK, payload,
                         sizeof(payload));
    assert(n == 8U + sizeof(payload));

    ubx_parser_init(&p);
    assert(feed_frame(&p, buf, n));

    assert(ubx_decode_ack(&p.frame, &decoded));
    assert(decoded.ack == true);
    assert(decoded.acked_cls == UBX_CLASS_CFG);
    assert(decoded.acked_id == UBX_CFG_VALSET);

    /* ACK-NAK for the same message */
    n = ubx_frame_build(buf, sizeof(buf), UBX_CLASS_ACK, UBX_ACK_NAK, payload,
                         sizeof(payload));
    ubx_parser_init(&p);
    assert(feed_frame(&p, buf, n));

    assert(ubx_decode_ack(&p.frame, &decoded));
    assert(decoded.ack == false);
    assert(decoded.acked_cls == UBX_CLASS_CFG);
    assert(decoded.acked_id == UBX_CFG_VALSET);
}

/* A corrupted checksum must never yield a complete frame, and the parser
 * must land back in sync-hunting state ready for the very next frame -
 * exactly what happens on a real DMA ring after one glitched byte. */
static void
test_corrupted_checksum_resyncs(void)
{
    uint8_t payload[2] = {UBX_CLASS_NAV, UBX_NAV_PVT};
    uint8_t bad[16];
    uint8_t good_payload[28];
    uint8_t good[64];
    uint16_t nbad, ngood;
    ubx_parser_t p;
    ubx_tim_tm2_t decoded;
    uint16_t i;
    bool any_true;

    nbad = ubx_frame_build(bad, sizeof(bad), UBX_CLASS_ACK, UBX_ACK_ACK,
                            payload, sizeof(payload));
    bad[nbad - 1U] ^= 0xFFU; /* flip ck_b */

    fill_tim_tm2_payload(good_payload);
    ngood = ubx_frame_build(good, sizeof(good), UBX_CLASS_TIM, UBX_TIM_TM2,
                             good_payload, sizeof(good_payload));

    ubx_parser_init(&p);

    any_true = false;
    for (i = 0; i < nbad; i++) {
        if (ubx_parser_feed(&p, bad[i])) {
            any_true = true;
        }
    }
    assert(!any_true);

    assert(feed_frame(&p, good, ngood));
    assert(ubx_decode_tim_tm2(&p.frame, &decoded));
    assert(decoded.count == 42U);
}

/* Sync bytes (0xB5 0x62) appearing inside the payload must not be
 * mistaken for a new frame start while the length counter is still
 * inside the declared payload - only the checksum bytes decide that. */
static void
test_sync_bytes_inside_payload(void)
{
    uint8_t payload[92];
    uint8_t buf[128];
    uint16_t n;
    ubx_parser_t p;
    ubx_nav_pvt_t decoded;

    fill_nav_pvt_payload(payload);
    /* reserved1[6] at offset 78..83 - unused by the decoder, safe to
     * plant a fake sync sequence there. */
    payload[78] = 0xB5U;
    payload[79] = 0x62U;
    payload[80] = 0xB5U;
    payload[81] = 0x62U;

    n = ubx_frame_build(buf, sizeof(buf), UBX_CLASS_NAV, UBX_NAV_PVT, payload,
                         sizeof(payload));

    ubx_parser_init(&p);
    assert(feed_frame(&p, buf, n));

    assert(ubx_decode_nav_pvt(&p.frame, &decoded));
    check_nav_pvt_fields(&decoded);
    assert(p.frame.payload[78] == 0xB5U);
    assert(p.frame.payload[79] == 0x62U);
}

/* A frame declaring more payload than UBX_MAX_PAYLOAD must still validate
 * its checksum over the full declared length and mark itself truncated,
 * per the "captured != declared length" contract in ubx.h. */
static void
test_oversized_payload_truncates_but_validates(void)
{
    uint8_t payload[UBX_MAX_PAYLOAD + 20U];
    uint8_t buf[UBX_MAX_PAYLOAD + 40U];
    uint16_t n;
    ubx_parser_t p;
    uint16_t i;

    for (i = 0; i < sizeof(payload); i++) {
        payload[i] = (uint8_t) (i + 1U);
    }

    n = ubx_frame_build(buf, sizeof(buf), UBX_CLASS_MON, UBX_MON_VER,
                         payload, sizeof(payload));
    assert(n != 0U);

    ubx_parser_init(&p);
    assert(feed_frame(&p, buf, n));

    assert(p.frame.truncated);
    assert(p.frame.len == sizeof(payload));
    for (i = 0; i < UBX_MAX_PAYLOAD; i++) {
        assert(p.frame.payload[i] == (uint8_t) (i + 1U));
    }
}

int
main(void)
{
    test_nav_pvt_roundtrip();
    test_tim_tm2_roundtrip();
    test_ack_roundtrip();
    test_corrupted_checksum_resyncs();
    test_sync_bytes_inside_payload();
    test_oversized_payload_truncates_but_validates();

    printf("test_ubx: all tests passed\n");
    return 0;
}
