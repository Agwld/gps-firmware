/**
 * @file    gps_i2c.h
 * @brief   UBX-over-I2C transport for the ZED-F9P's DDC (I2C) port.
 *
 * The gps-mainboard has NO MCU->GPS UART path: the F9P's UART1 TX is
 * hard-wired to USART3 RX (net tie NT1), but its UART1 RX is only
 * reachable from the RS232 input via solder jumper JP7 (intended for
 * RTCM corrections straight off the loom). Configuration therefore goes
 * over I2C2, which the board wires to the F9P's I2C port (7-bit address
 * 0x42) on the same bus as the MCP9800 - hence every transaction here
 * takes the app_i2c_lock() bus mutex.
 *
 * Only the boot-time VALSET/VALGET/ACK exchange uses this transport;
 * the high-rate UBX stream (NAV-PVT, TIM-TM2) still arrives on UART.
 *
 * DDC protocol (ZED-F9P integration manual): a write transaction
 * carries raw UBX frame bytes into the receiver's message stream;
 * registers 0xFD/0xFE hold the pending output byte count (big-endian)
 * and register 0xFF streams the output bytes themselves.
 */

#ifndef GPS_I2C_H
#define GPS_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "sys/status.h"

/** @brief Send a complete UBX frame to the receiver. */
status_t gps_i2c_write(const uint8_t *frame, uint16_t len);

/**
 * @brief Read up to `max` pending stream bytes into `buf`.
 * @return STATUS_OK with *n_read = 0 when the receiver has nothing
 *         pending; STATUS_ERROR on a bus fault.
 */
status_t gps_i2c_read(uint8_t *buf, uint16_t max, uint16_t *n_read);

#ifdef __cplusplus
}
#endif

#endif /* GPS_I2C_H */
