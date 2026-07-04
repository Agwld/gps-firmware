/**
 * @file    canbc_task.h
 * @brief   Entry point for can_task (see app.c for creation).
 */

#ifndef CANBC_TASK_H
#define CANBC_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void canbc_task_main(void *argument);

/** @brief FDCAN RX FIFO0 callback hook, called from HAL_FDCAN_RxFifo0Callback
 *         (stm32g4xx_it.c / HAL weak override) - ISR context. */
void canbc_task_on_rx_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* CANBC_TASK_H */
