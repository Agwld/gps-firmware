/**
 * @file    board_config.h
 * @brief   Compile-time configuration: rates, scalings, mounting, defaults.
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/*
 * GPS
 */
#define GPS_NAV_RATE_HZ        20U  /* F9P: 20 Hz needs GPS+GAL only */
#define GPS_MEAS_PERIOD_MS     (1000U / GPS_NAV_RATE_HZ)
/* USART3's RX (F9P UBX stream in) and TX (NMEA out to the MoTeC via
 * JP6) share the one baud-rate register, so this single rate serves
 * both. 115200 clears the 20 Hz UBX stream (~25 kbit/s) with 4x margin
 * while staying inside the MAX3232's (~235 kbit/s) and the MoTeC's
 * RS232 limits - don't raise it without checking both. The F9P's UART1
 * is switched to this rate over I2C by gps_config at boot. */
#define GPS_UART_BAUD          115200U

/*
 * IMU: LSM6DSO32, accel +/-16 g, gyro +/-2000 dps, 104 Hz ODR
 */
#define IMU_ODR_HZ             104U
#define IMU_ACCEL_FS_G         16U
#define IMU_GYRO_FS_DPS        2000U
#define IMU_GRAVITY_MPS2       9.80665f /* standard gravity, g -> m/s^2 */

/*
 * IMU mounting: sensor axes -> vehicle axes (x fwd, y left, z up).
 * Row i of this matrix selects/signs the sensor axis that maps to vehicle
 * axis i.
 *
 * LSM6DSO32TR (U12) is mounted flat, per the datasheet's package
 * orientation diagram cross-checked against its physical mounting on
 * gps-mainboard: the vehicle's front is along the sensor's -X, the
 * vehicle's right is along the sensor's +Y (so vehicle left, this
 * project's +Y, is sensor -Y), and up is the sensor's +Z directly - a
 * flat 180 deg mounting about the vertical axis relative to the vehicle.
 */
#define IMU_MOUNT_MATRIX                                                       \
    {                                                                          \
        {-1.0f, 0.0f, 0.0f},                                                   \
        {0.0f, -1.0f, 0.0f},                                                   \
        {0.0f, 0.0f, 1.0f},                                                    \
    }

/*
 * Magnetometer mounting: mag sensor axes -> vehicle axes, same convention
 * as IMU_MOUNT_MATRIX. The IIS2MDC magnetometer is a physically separate
 * part from the LSM6DSO32 and is mounted with its pin 1 rotated 180 deg
 * relative to the IMU, so its axes do NOT match the accel/gyro - feeding
 * mag samples through the accel/gyro matrix would mis-rotate AHRS heading.
 *
 * From the IIS2MDC datasheet top-view axes (Z out of the top face) and
 * that placement: +X points to the vehicle front and +Z up, both taken
 * from the mounting; +Y is then forced to the vehicle's left by the
 * part's right-handedness (left = up x forward = Z x X = +Y). Net result
 * is the identity - a valid right-handed mount (det +1) that differs from
 * the IMU's 180-deg-about-Z orientation, which is why the two parts need
 * separate matrices.
 */
#define MAG_MOUNT_MATRIX                                                       \
    {                                                                          \
        {1.0f, 0.0f, 0.0f},                                                    \
        {0.0f, 1.0f, 0.0f},                                                    \
        {0.0f, 0.0f, 1.0f},                                                    \
    }

/*
 * Lap timing
 */
#define LAP_MAX_GATES          8U    /* start/finish + up to 7 sectors */
/* Formula Student track width is ~3 m (half-width 1.5 m); a gate wider
 * than the track itself risks false-triggering on an adjacent section
 * at a hairpin/chicane, where two parts of the circuit pass within the
 * gate's tangential reach of each other. 2.0 m gives ~0.5 m margin over
 * the track's own half-width for fused-position error, without being
 * wide enough to plausibly reach a neighbouring section. */
#define LAP_GATE_HALF_WIDTH_M  2.0f
#define LAP_MIN_LAP_TIME_MS    10000U /* reject re-crossings sooner than this */
/* Gate placement orients the gate line perpendicular to the direction of
 * travel (course over ground). Course is noise at a standstill, so at or
 * below this speed the gate heading is taken from the AHRS yaw
 * (magnetometer-referenced, valid stationary) instead - letting a gate be
 * planted with the car parked, nose pointed down the track. ~7 km/h. */
#define LAP_GATE_COURSE_MIN_SPEED_MPS 2.0f

/* Floor between background mag-cal flash writes, to cap flash wear while
 * the calibration is still converging (it stops writing once stable). */
#define MAG_CAL_SAVE_MIN_INTERVAL_MS  60000U
#define LAP_BUTTON_LONG_MS     1000U  /* long press: set start/finish */
#define LAP_BUTTON_CLEAR_MS    5000U  /* very long press: clear all gates */

/*
 * NMEA output (MoTeC), USART2
 */
#define NMEA_OUT_RATE_HZ       10U
#define NMEA_OUT_BAUD          115200U

/*
 * CAN broadcast rates (per canbc rota; periods in 10 ms slots)
 */
#define CANBC_SLOT_MS          10U

#endif /* BOARD_CONFIG_H */
