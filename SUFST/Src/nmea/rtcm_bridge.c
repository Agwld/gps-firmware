/**
 * @file    rtcm_bridge.c
 * @brief   USART2 RX -> USART3 TX byte forward (see rtcm_bridge.h).
 */

#include "nmea/rtcm_bridge.h"

#include "main.h"

#include "sys/app.h"

#define RTCM_RING_SIZE 256U

static uint8_t s_rx_ring[RTCM_RING_SIZE];
static uint16_t s_read_idx;

void
rtcm_bridge_init(void)
{
    s_read_idx = 0U;
    HAL_UART_Receive_DMA(&huart2, s_rx_ring, RTCM_RING_SIZE);
}

void
rtcm_bridge_poll(void)
{
    uint16_t remaining = (uint16_t) __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
    uint16_t write_idx = (uint16_t) (RTCM_RING_SIZE - remaining);

    if (write_idx == s_read_idx) {
        return;
    }

    /* Forward in at most two contiguous chunks (no wrap, then the
     * wrapped remainder) rather than byte-at-a-time, since RTCM has no
     * framing this bridge needs to respect - it's a dumb pipe. */
    app_gps_tx_lock();

    if (write_idx > s_read_idx) {
        HAL_UART_Transmit(&huart3, &s_rx_ring[s_read_idx],
                           (uint16_t) (write_idx - s_read_idx), 50U);
    } else {
        HAL_UART_Transmit(&huart3, &s_rx_ring[s_read_idx],
                           (uint16_t) (RTCM_RING_SIZE - s_read_idx), 50U);
        if (write_idx > 0U) {
            HAL_UART_Transmit(&huart3, &s_rx_ring[0], write_idx, 50U);
        }
    }

    app_gps_tx_unlock();

    s_read_idx = write_idx;
}
