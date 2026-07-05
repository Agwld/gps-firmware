/**
 * @file    stm32g4xx_hal_msp.c
 * @brief   Peripheral low-level init: GPIO AF, DMA binding, NVIC.
 */

#include "main.h"

DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_DisableUCPDDeadBattery();
}

static void msp_dma_link(DMA_HandleTypeDef *hdma, DMA_Channel_TypeDef *ch,
                         uint32_t request, uint32_t dir, uint32_t mode,
                         uint32_t prio)
{
    hdma->Instance = ch;
    hdma->Init.Request = request;
    hdma->Init.Direction = dir;
    hdma->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma->Init.MemInc = DMA_MINC_ENABLE;
    hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma->Init.Mode = mode;
    hdma->Init.Priority = prio;

    if (HAL_DMA_Init(hdma) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef g = {0};

    if (huart->Instance == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* PB6 TX, PB7 RX, AF7 */
        g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        g.Mode = GPIO_MODE_AF_PP;
        g.Pull = GPIO_NOPULL;
        g.Speed = GPIO_SPEED_FREQ_LOW;
        g.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOB, &g);
    }
    else if (huart->Instance == USART3) {
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* PB10 TX, PB11 RX, AF7 */
        g.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        g.Mode = GPIO_MODE_AF_PP;
        g.Pull = GPIO_NOPULL;
        g.Speed = GPIO_SPEED_FREQ_LOW;
        g.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB, &g);

        msp_dma_link(&hdma_usart3_rx, DMA1_Channel1, DMA_REQUEST_USART3_RX,
                     DMA_PERIPH_TO_MEMORY, DMA_CIRCULAR, DMA_PRIORITY_HIGH);
        __HAL_LINKDMA(huart, hdmarx, hdma_usart3_rx);

        msp_dma_link(&hdma_usart3_tx, DMA1_Channel6, DMA_REQUEST_USART3_TX,
                     DMA_MEMORY_TO_PERIPH, DMA_NORMAL, DMA_PRIORITY_LOW);
        __HAL_LINKDMA(huart, hdmatx, hdma_usart3_tx);

        HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        __HAL_RCC_USART1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
    }
    else if (huart->Instance == USART3) {
        __HAL_RCC_USART3_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
        HAL_DMA_DeInit(huart->hdmarx);
        HAL_DMA_DeInit(huart->hdmatx);
        HAL_NVIC_DisableIRQ(USART3_IRQn);
    }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef g = {0};

    if (hspi->Instance == SPI1) {
        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* PB3 SCK, PB4 MISO, PB5 MOSI, AF5 */
        g.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
        g.Mode = GPIO_MODE_AF_PP;
        g.Pull = GPIO_NOPULL;
        g.Speed = GPIO_SPEED_FREQ_HIGH;
        g.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOB, &g);

        msp_dma_link(&hdma_spi1_rx, DMA1_Channel4, DMA_REQUEST_SPI1_RX,
                     DMA_PERIPH_TO_MEMORY, DMA_NORMAL, DMA_PRIORITY_HIGH);
        __HAL_LINKDMA(hspi, hdmarx, hdma_spi1_rx);

        msp_dma_link(&hdma_spi1_tx, DMA1_Channel5, DMA_REQUEST_SPI1_TX,
                     DMA_MEMORY_TO_PERIPH, DMA_NORMAL, DMA_PRIORITY_HIGH);
        __HAL_LINKDMA(hspi, hdmatx, hdma_spi1_tx);

        /* SPI error interrupt: priority 5 to match the SPI DMA channels
         * and stay >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, so
         * the FromISR give in HAL_SPI_ErrorCallback is legal. */
        HAL_NVIC_SetPriority(SPI1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(SPI1_IRQn);
    }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef g = {0};

    if (hi2c->Instance == I2C2) {
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA8 SDA, PA9 SCL, AF4, open-drain (I2C requires external
         * pull-ups, already present on the bus per the board design). */
        g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        g.Mode = GPIO_MODE_AF_OD;
        g.Pull = GPIO_NOPULL;
        g.Speed = GPIO_SPEED_FREQ_HIGH;
        g.Alternate = GPIO_AF4_I2C2;
        HAL_GPIO_Init(GPIOA, &g);

        __HAL_RCC_I2C2_CLK_ENABLE();
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2) {
        __HAL_RCC_I2C2_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8 | GPIO_PIN_9);
    }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        __HAL_RCC_SPI1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
        HAL_DMA_DeInit(hspi->hdmarx);
        HAL_DMA_DeInit(hspi->hdmatx);
    }
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
    GPIO_InitTypeDef g = {0};
    RCC_PeriphCLKInitTypeDef pclk = {0};

    if (hfdcan->Instance == FDCAN1) {
        pclk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        pclk.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;

        if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) {
            Error_Handler();
        }

        __HAL_RCC_FDCAN_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA11 RX, PA12 TX, AF9 */
        g.Pin = GPIO_PIN_11 | GPIO_PIN_12;
        g.Mode = GPIO_MODE_AF_PP;
        g.Pull = GPIO_NOPULL;
        g.Speed = GPIO_SPEED_FREQ_HIGH;
        g.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOA, &g);

        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
        HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
    }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance == FDCAN1) {
        __HAL_RCC_FDCAN_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
        HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
        HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
    }
}
