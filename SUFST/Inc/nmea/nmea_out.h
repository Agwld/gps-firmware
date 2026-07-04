/**
 * @file    nmea_out.h
 * @brief   Integer-only NMEA 0183 sentence synthesis for the MoTeC
 *          datalogger link (USART2). Builds GGA/RMC/VTG from a decoded
 *          UBX-NAV-PVT, since the raw GPS UART is committed to 20 Hz UBX
 *          binary and cannot also serve MoTeC NMEA.
 *
 * No floating point and no printf-family calls anywhere in this module:
 * the target links nano.specs and float printf is both a flash-size and
 * correctness liability on this part, so every numeric field is produced
 * by hand-rolled integer division/modulo digit extraction.
 */

#ifndef NMEA_OUT_H
#define NMEA_OUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

#include "gps/ubx.h"

/**
 * @brief Format a GPGGA (fix data) sentence.
 *
 * Field notes (documented choices where the source data under-specifies
 * the NMEA field):
 * - Time is hhmmss.ss; the centisecond fraction is taken from
 *   (itow_ms % 1000) / 10 - ubx_nav_pvt_t carries no separate UTC
 *   sub-second field, and GPS/UTC differ only by whole leap seconds so
 *   the sub-second alignment still holds.
 * - Fix quality: 0 (no fix) if flags.gnssFixOK is clear; else 4/5 from
 *   flags.carrSoln (RTK fixed/float); else 1 (plain GPS fix). NMEA
 *   quality 2 (DGPS) is never emitted - this struct does not decode a
 *   diffSoln bit, so DGPS cannot be distinguished from a plain fix.
 * - HDOP slot is PDOP (pdop_1e2 * 0.01) echoed directly: no separate
 *   HDOP is decoded, and PDOP >= HDOP always, so this is a conservative
 *   stand-in rather than a true HDOP.
 * - Altitude is height above MSL (hmsl_mm), rounded to 0.1 m.
 * - Geoid separation is emitted as a fixed 0.0 M: separation is not
 *   tracked/decoded, so it is not fabricated from height_mm - hmsl_mm.
 * - The two DGPS fields (age, station id) are always empty.
 *
 * @param pvt      Decoded NAV-PVT.
 * @param buf      Destination buffer.
 * @param buf_size Size of buf, including room for the trailing NUL that
 *                 this function writes for caller convenience (the NUL
 *                 is not counted in the returned length).
 * @return Number of characters written (excluding the NUL), or 0 if
 *         buf_size was too small (buf is left untouched in that case).
 */
size_t nmea_format_gga(const ubx_nav_pvt_t *pvt, char *buf, size_t buf_size);

/**
 * @brief Format a GPRMC (recommended minimum) sentence.
 *
 * Classic (pre-NMEA-2.3) field layout - no trailing mode indicator.
 * Status is 'A' iff flags.gnssFixOK is set, else 'V'. Speed over ground
 * is in knots with 3 decimal digits, computed from gspeed_mms using the
 * exact rational knots = mm/s * 9/4630 (1 knot = 1852000/3600 mm/s,
 * reduced), avoiding the repeating decimal in "divide by 514.444...".
 * Course over ground is degrees.d from head_motion_1e5. Magnetic
 * variation fields are always empty (not tracked).
 *
 * @copydetails nmea_format_gga
 */
size_t nmea_format_rmc(const ubx_nav_pvt_t *pvt, char *buf, size_t buf_size);

/**
 * @brief Format a GPVTG (course/speed over ground) sentence.
 *
 * Classic field layout - no trailing mode indicator. Magnetic course
 * mirrors true course (no magnetic variation model is tracked). Speed
 * in km/h uses the exact rational km/h = mm/s * 9/2500 (equivalent to
 * dividing by 277.778, done without the repeating decimal).
 *
 * @copydetails nmea_format_gga
 */
size_t nmea_format_vtg(const ubx_nav_pvt_t *pvt, char *buf, size_t buf_size);

#ifdef __cplusplus
}
#endif

#endif /* NMEA_OUT_H */
