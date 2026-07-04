/**
 * @file    fdcan.c
 * @brief   FDCAN1 init: classic CAN on the CAN-S bus.
 *
 * Kernel clock = PCLK1 = 170 MHz.
 * 1 Mbps:   prescaler 10 -> 17 MHz, 17 tq = 1 + 14 + 2, SJW 2, SP 88.2 %.
 * 500 kbps: prescaler 20, same segments (see CAN_BITRATE_500K).
 */

#include "main.h"

FDCAN_HandleTypeDef hfdcan1;

void MX_FDCAN1_Init(void)
{
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = ENABLE;
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = DISABLE;

#ifdef CAN_BITRATE_500K
    hfdcan1.Init.NominalPrescaler = 20U;
#else
    hfdcan1.Init.NominalPrescaler = 10U;
#endif
    hfdcan1.Init.NominalSyncJumpWidth = 2U;
    hfdcan1.Init.NominalTimeSeg1 = 14U;
    hfdcan1.Init.NominalTimeSeg2 = 2U;

    /* Data phase unused in classic mode; keep valid values */
    hfdcan1.Init.DataPrescaler = 10U;
    hfdcan1.Init.DataSyncJumpWidth = 2U;
    hfdcan1.Init.DataTimeSeg1 = 14U;
    hfdcan1.Init.DataTimeSeg2 = 2U;

    hfdcan1.Init.StdFiltersNbr = 2U; /* GPS_Command (0x6BF), Wheel_Speeds (0x251) */
    hfdcan1.Init.ExtFiltersNbr = 0U;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
}
