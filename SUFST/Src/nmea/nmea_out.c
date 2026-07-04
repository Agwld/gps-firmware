/**
 * @file    nmea_out.c
 * @brief   Integer-only NMEA 0183 sentence synthesis. See nmea_out.h for
 *          the per-sentence field documentation and the rationale for
 *          each simplifying choice.
 */

#include "nmea/nmea_out.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Generous upper bound on a sentence body (tag + fields, excluding the
 * leading '$', the trailing "*hh\r\n" and the NUL). The longest sentence
 * here (GGA) is ~65 bytes; this leaves comfortable headroom without
 * relying on any per-call bounds checking while the body is built. */
#define NMEA_BODY_BUF_SIZE 96U

/* ------------------------------------------------------------------ */
/* Small integer/string helpers - no floating point anywhere below.   */
/* ------------------------------------------------------------------ */

/** @brief Append one character, return the advanced cursor. */
static char *put_char(char *p, char c)
{
    *p = c;
    return p + 1;
}

/** @brief Append a NUL-terminated literal, return the advanced cursor. */
static char *put_str(char *p, const char *s)
{
    while (*s != '\0') {
        *p++ = *s++;
    }
    return p;
}

/**
 * @brief Append an unsigned integer in decimal, zero-padded to at least
 *        @p width digits (width 0 or 1 means "natural width, no pad").
 *
 * Digits are produced least-significant-first by repeated /10, %10 -
 * the only way to turn an integer into ASCII without printf - then
 * emitted most-significant-first.
 */
static char *put_udec(char *p, uint32_t val, unsigned width)
{
    char digits[10]; /* uint32_t max is 10 decimal digits */
    unsigned n = 0;

    do {
        digits[n++] = (char)('0' + (val % 10U));
        val /= 10U;
    } while (val != 0U);

    while (n < width) {
        digits[n++] = '0';
    }

    while (n > 0U) {
        p = put_char(p, digits[--n]);
    }
    return p;
}

/**
 * @brief Append "[-]<int>.<frac>" where int/frac are already the exact
 *        digit groups to print (frac zero-padded to frac_width, int
 *        zero-padded to int_width, 0/1 = natural).
 */
static char *put_fixed(char *p, bool neg, uint32_t int_part,
                        unsigned int_width, uint32_t frac_part,
                        unsigned frac_width)
{
    if (neg) {
        p = put_char(p, '-');
    }
    p = put_udec(p, int_part, int_width);
    p = put_char(p, '.');
    p = put_udec(p, frac_part, frac_width);
    return p;
}

/* ------------------------------------------------------------------ */
/* Field formatters                                                    */
/* ------------------------------------------------------------------ */

/** @brief hhmmss.ss - see nmea_out.h for the centisecond source. */
static char *put_time(char *p, const ubx_nav_pvt_t *pvt)
{
    uint32_t centisec = (pvt->itow_ms % 1000U) / 10U;

    p = put_udec(p, pvt->hour, 2U);
    p = put_udec(p, pvt->min, 2U);
    p = put_udec(p, pvt->sec, 2U);
    p = put_char(p, '.');
    p = put_udec(p, centisec, 2U);
    return p;
}

/** @brief ddmmyy. */
static char *put_date(char *p, const ubx_nav_pvt_t *pvt)
{
    p = put_udec(p, pvt->day, 2U);
    p = put_udec(p, pvt->month, 2U);
    p = put_udec(p, (uint32_t)(pvt->year % 100U), 2U);
    return p;
}

/**
 * @brief Convert a 1e-7 deg field to NMEA degrees-decimal-minutes
 *        (ddmm.mmmm / dddmm.mmmm) and append it, followed by ",<hemi>".
 *
 * All intermediates use int64_t/uint64_t so the *60 and rounding steps
 * cannot overflow, even though the practical lat/lon range would fit
 * in 32 bits - see nmea_out.h and the design notes this implements.
 */
static char *put_latlon(char *p, int32_t val_1e7, unsigned deg_width,
                         char pos_ch, char neg_ch)
{
    bool negative = (val_1e7 < 0);
    uint64_t mag = negative ? (uint64_t)(-(int64_t)val_1e7)
                             : (uint64_t)val_1e7;

    uint64_t deg = mag / 10000000ULL;
    uint64_t remainder_1e7 = mag % 10000000ULL; /* fractional degree */

    /* minutes = fractional_degree * 60; keep 4 decimal digits of
     * minutes (1e4 scale), rounded to nearest. */
    uint64_t minutes_1e7 = remainder_1e7 * 60ULL;
    uint64_t minutes_1e4 = (minutes_1e7 + 500ULL) / 1000ULL;

    /* Rounding can carry a fractional degree's minutes up to exactly
     * 60.0000 (e.g. remainder_1e7 == 9999999): fold that back into a
     * whole extra degree so the minutes field never reads "60.xxxx". */
    if (minutes_1e4 >= 600000ULL) {
        minutes_1e4 -= 600000ULL;
        deg += 1ULL;
    }

    uint32_t min_whole = (uint32_t)(minutes_1e4 / 10000ULL);
    uint32_t min_frac = (uint32_t)(minutes_1e4 % 10000ULL);

    p = put_udec(p, (uint32_t)deg, deg_width);
    p = put_udec(p, min_whole, 2U);
    p = put_char(p, '.');
    p = put_udec(p, min_frac, 4U);
    p = put_char(p, ',');
    p = put_char(p, (val_1e7 >= 0) ? pos_ch : neg_ch);
    return p;
}

/**
 * @brief Map fix_type/flags to the NMEA GGA fix-quality digit.
 *        See nmea_out.h for why quality 2 (DGPS) is never produced.
 */
static uint32_t gga_fix_quality(const ubx_nav_pvt_t *pvt)
{
    if ((pvt->flags & 0x01U) == 0U) {
        return 0U; /* gnssFixOK clear: no fix */
    }

    uint32_t carr_soln = ((uint32_t)pvt->flags >> 6) & 0x03U;
    if (carr_soln == 2U) {
        return 4U; /* RTK fixed */
    }
    if (carr_soln == 1U) {
        return 5U; /* RTK float */
    }
    return 1U; /* plain GPS/GNSS fix */
}

/** @brief Altitude (hmsl_mm) rounded to 0.1 m, as "[-]int.f". */
static char *put_altitude_m(char *p, int32_t hmsl_mm)
{
    bool negative = (hmsl_mm < 0);
    uint32_t abs_mm = negative ? (uint32_t)(-(int64_t)hmsl_mm)
                                : (uint32_t)hmsl_mm;
    uint32_t tenths = (abs_mm + 50U) / 100U; /* mm -> 0.1 m, rounded */

    return put_fixed(p, negative, tenths / 10U, 0U, tenths % 10U, 1U);
}

/** @brief PDOP (pdop_1e2) echoed into the HDOP slot, as "int.ff". */
static char *put_hdop_from_pdop(char *p, uint16_t pdop_1e2)
{
    return put_fixed(p, false, pdop_1e2 / 100U, 0U, pdop_1e2 % 100U, 2U);
}

/**
 * @brief Ground speed in knots, 3 decimals, as "int.fff".
 *        knots = mm/s * 9 / 4630 (exact reduction of 1852000/3600 mm/s
 *        per knot); scaled by 1000 for 3 decimal digits and rounded.
 */
static char *put_speed_knots(char *p, int32_t gspeed_mms)
{
    /* gspeed is defined non-negative by UBX; abs() guards against a
     * corrupt/negative value rather than trusting it blindly. */
    uint64_t mag = (gspeed_mms < 0) ? (uint64_t)(-(int64_t)gspeed_mms)
                                     : (uint64_t)gspeed_mms;
    uint64_t knots_1e3 = (mag * 900ULL + 231ULL) / 463ULL;

    return put_fixed(p, false, (uint32_t)(knots_1e3 / 1000ULL), 0U,
                      (uint32_t)(knots_1e3 % 1000ULL), 3U);
}

/**
 * @brief Ground speed in km/h, 3 decimals, as "int.fff".
 *        km/h = mm/s * 9 / 2500 (exact reduction of 1000/3.6 mm/s per
 *        km/h); scaled by 1000 for 3 decimal digits and rounded.
 */
static char *put_speed_kmh(char *p, int32_t gspeed_mms)
{
    uint64_t mag = (gspeed_mms < 0) ? (uint64_t)(-(int64_t)gspeed_mms)
                                     : (uint64_t)gspeed_mms;
    uint64_t kmh_1e3 = (mag * 18ULL + 2ULL) / 5ULL;

    return put_fixed(p, false, (uint32_t)(kmh_1e3 / 1000ULL), 0U,
                      (uint32_t)(kmh_1e3 % 1000ULL), 3U);
}

/**
 * @brief Course over ground, 1 decimal, zero-padded to 3 integer
 *        digits ("ddd.d"), from head_motion_1e5 (deg * 1e-5).
 */
static char *put_course(char *p, int32_t head_motion_1e5)
{
    /* Heading of motion is defined in [0, 360) deg by UBX; abs() is a
     * defensive guard, not an expected code path. */
    uint64_t mag = (head_motion_1e5 < 0)
                       ? (uint64_t)(-(int64_t)head_motion_1e5)
                       : (uint64_t)head_motion_1e5;
    uint64_t course_1e1 = (mag + 5000ULL) / 10000ULL; /* 1e5 -> 1e1 */

    if (course_1e1 >= 3600ULL) {
        course_1e1 -= 3600ULL; /* rounded up to 360.0: wrap to 0.0 */
    }

    return put_fixed(p, false, (uint32_t)(course_1e1 / 10ULL), 3U,
                      (uint32_t)(course_1e1 % 10ULL), 1U);
}

/* ------------------------------------------------------------------ */
/* Checksum + assembly                                                 */
/* ------------------------------------------------------------------ */

/**
 * @brief XOR-checksum @p body and assemble the full "$body*hh\r\n\0"
 *        sentence into @p buf.
 * @return Length written (excluding the NUL), or 0 if buf_size was too
 *         small - buf is left untouched in that case.
 */
static size_t finalize_sentence(const char *body, size_t body_len,
                                 char *buf, size_t buf_size)
{
    static const char hex_digits[] = "0123456789ABCDEF";
    uint8_t checksum = 0U;

    for (size_t i = 0U; i < body_len; ++i) {
        checksum ^= (uint8_t)body[i];
    }

    /* '$' + body + '*' + 2 hex digits + "\r\n" + NUL */
    size_t sentence_len = 1U + body_len + 1U + 2U + 2U;
    if (buf_size < sentence_len + 1U) {
        return 0U;
    }

    size_t idx = 0U;
    buf[idx++] = '$';
    memcpy(&buf[idx], body, body_len);
    idx += body_len;
    buf[idx++] = '*';
    buf[idx++] = hex_digits[(checksum >> 4) & 0x0FU];
    buf[idx++] = hex_digits[checksum & 0x0FU];
    buf[idx++] = '\r';
    buf[idx++] = '\n';
    buf[idx] = '\0';

    return idx; /* == sentence_len */
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

size_t nmea_format_gga(const ubx_nav_pvt_t *pvt, char *buf, size_t buf_size)
{
    char body[NMEA_BODY_BUF_SIZE];
    char *p = body;

    p = put_str(p, "GPGGA,");
    p = put_time(p, pvt);
    p = put_char(p, ',');
    p = put_latlon(p, pvt->lat_1e7, 2U, 'N', 'S');
    p = put_char(p, ',');
    p = put_latlon(p, pvt->lon_1e7, 3U, 'E', 'W');
    p = put_char(p, ',');
    p = put_udec(p, gga_fix_quality(pvt), 0U);
    p = put_char(p, ',');
    p = put_udec(p, pvt->num_sv, 2U);
    p = put_char(p, ',');
    p = put_hdop_from_pdop(p, pvt->pdop_1e2);
    p = put_char(p, ',');
    p = put_altitude_m(p, pvt->hmsl_mm);
    p = put_str(p, ",M,0.0,M,,"); /* geoid sep fixed 0.0; DGPS fields empty */

    return finalize_sentence(body, (size_t)(p - body), buf, buf_size);
}

size_t nmea_format_rmc(const ubx_nav_pvt_t *pvt, char *buf, size_t buf_size)
{
    char body[NMEA_BODY_BUF_SIZE];
    char *p = body;

    p = put_str(p, "GPRMC,");
    p = put_time(p, pvt);
    p = put_char(p, ',');
    p = put_char(p, ((pvt->flags & 0x01U) != 0U) ? 'A' : 'V');
    p = put_char(p, ',');
    p = put_latlon(p, pvt->lat_1e7, 2U, 'N', 'S');
    p = put_char(p, ',');
    p = put_latlon(p, pvt->lon_1e7, 3U, 'E', 'W');
    p = put_char(p, ',');
    p = put_speed_knots(p, pvt->gspeed_mms);
    p = put_char(p, ',');
    p = put_course(p, pvt->head_motion_1e5);
    p = put_char(p, ',');
    p = put_date(p, pvt);
    p = put_str(p, ",,"); /* magnetic variation value + E/W, not tracked */

    return finalize_sentence(body, (size_t)(p - body), buf, buf_size);
}

size_t nmea_format_vtg(const ubx_nav_pvt_t *pvt, char *buf, size_t buf_size)
{
    char body[NMEA_BODY_BUF_SIZE];
    char *p = body;

    p = put_str(p, "GPVTG,");
    p = put_course(p, pvt->head_motion_1e5);
    p = put_str(p, ",T,");
    p = put_course(p, pvt->head_motion_1e5); /* magnetic mirrors true */
    p = put_str(p, ",M,");
    p = put_speed_knots(p, pvt->gspeed_mms);
    p = put_str(p, ",N,");
    p = put_speed_kmh(p, pvt->gspeed_mms);
    p = put_char(p, ',');
    p = put_char(p, 'K');

    return finalize_sentence(body, (size_t)(p - body), buf, buf_size);
}
