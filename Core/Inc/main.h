/**
 * @file    main.h
 * @brief   Board pin map and peripheral handles for the SUFST GPS node.
 *
 * Pin assignments are authoritative from the gps-mainboard rev 1.0.0 KiCad
 * netlist (pcb/gps-mainboard). Do not change without a board revision.
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/*
 * Buttons (both externally conditioned: RC tau = 10 ms + Schmitt buffer)
 */
#define LAP_BUTTON_PIN        GPIO_PIN_13 /* active high */
#define LAP_BUTTON_PORT       GPIOC
#define USER_BUTTON_PIN       GPIO_PIN_14 /* active low, on-board KMR2 */
#define USER_BUTTON_PORT      GPIOC

/*
 * LEDs (active high)
 */
#define USER_LED_PIN          GPIO_PIN_15
#define USER_LED_PORT         GPIOB
#define ERROR_LED_PIN         GPIO_PIN_10
#define ERROR_LED_PORT        GPIOA

/*
 * GPS control / status
 */
#define GPS_NRESET_PIN        GPIO_PIN_1 /* open drain, low = reset */
#define GPS_NRESET_PORT       GPIOB
#define GPS_TURN_OFF_PIN      GPIO_PIN_1 /* high = GPS power off */
#define GPS_TURN_OFF_PORT     GPIOA
#define GPS_TX_READY_PIN      GPIO_PIN_0
#define GPS_TX_READY_PORT     GPIOB
#define GPS_GEO_STAT_PIN      GPIO_PIN_3
#define GPS_GEO_STAT_PORT     GPIOA
#define GPS_RTK_STAT_PIN      GPIO_PIN_4
#define GPS_RTK_STAT_PORT     GPIOA
#define GPS_EXTINT_PIN        GPIO_PIN_5 /* MCU tap of the event-button net */
#define GPS_EXTINT_PORT       GPIOA
#define GPS_PPS_PIN           GPIO_PIN_7 /* TIM3_CH2 input capture */
#define GPS_PPS_PORT          GPIOA
#define GPS_EEPROM_WP_PIN     GPIO_PIN_13 /* high = EEPROM write-protected */
#define GPS_EEPROM_WP_PORT    GPIOB

/*
 * IMU
 */
#define IMU_CS_PIN            GPIO_PIN_9 /* active low chip select */
#define IMU_CS_PORT           GPIOB
#define IMU_INT_PIN           GPIO_PIN_15 /* LSM6DSO32 INT1 (DRDY), EXTI15 */
#define IMU_INT_PORT          GPIOC

/*
 * Misc inputs
 */
#define TOO_HOT_PIN           GPIO_PIN_14 /* MCP9800 alert, open drain */
#define TOO_HOT_PORT          GPIOB

/*
 * Peripheral handles (defined in the Core/Src peripheral init files)
 */
extern UART_HandleTypeDef huart1; /* debug */
extern UART_HandleTypeDef huart2; /* RS232: NMEA out / RTCM in */
extern UART_HandleTypeDef huart3; /* ZED-F9P UART1 */
extern SPI_HandleTypeDef hspi1;   /* LSM6DSO32 */
extern I2C_HandleTypeDef hi2c2;   /* F9P I2C, MCP9800, EEPROM */
extern FDCAN_HandleTypeDef hfdcan1;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
