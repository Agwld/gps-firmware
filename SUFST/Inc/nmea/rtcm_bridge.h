/**
 * @file    rtcm_bridge.h
 * @brief   Forward RTCM correction bytes from USART2 RX (RS232 in, when
 *          JP7 is in its default GPS-direct-in position) to USART3 TX
 *          (the GPS's UART1), for RTK.
 */

#ifndef RTCM_BRIDGE_H
#define RTCM_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Start the USART2 RX DMA circular reception that feeds the
 *         bridge. Call once from aux_task before its main loop. */
void rtcm_bridge_init(void);

/** @brief Forward any RTCM bytes received since the last call. Blocking
 *         only for the (short, mutexed) USART3 write itself. */
void rtcm_bridge_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* RTCM_BRIDGE_H */
