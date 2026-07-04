/**
 * @file    status.h
 * @brief   Common return codes (vcu-style error propagation).
 */

#ifndef STATUS_H
#define STATUS_H

typedef enum {
    STATUS_OK = 0,
    STATUS_ERROR,
    STATUS_TIMEOUT,
    STATUS_INVALID_ARG,
    STATUS_NOT_READY,
    STATUS_FULL,
    STATUS_CRC,
} status_t;

#endif /* STATUS_H */
