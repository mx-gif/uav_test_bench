/**
 * @file RS485Bus.c
 * @brief USART2共享RS485总线驱动
 */
#include "RS485Bus.h"
#include "main.h"

#define RS485_DIR_TX() \
    HAL_GPIO_WritePin(RS485_GPIO_Port, RS485_Pin, GPIO_PIN_SET)

#define RS485_DIR_RX() \
    HAL_GPIO_WritePin(RS485_GPIO_Port, RS485_Pin, GPIO_PIN_RESET)

/* 给RS485收发器预留方向切换时间 */
#define RS485_DIR_SWITCH_DELAY_MS    1U

static UART_HandleTypeDef *g_rs485_uart = NULL;
static volatile uint8_t g_rs485_busy = 0U;
static volatile RS485Bus_Status_t g_rs485_last_status =
    RS485_BUS_ERR_NOT_INIT;

/**
 * @brief 清除UART接收寄存器中的残留字节和错误标志
 * @note  STM32F1通过依次读取SR、DR清除ORE/FE/NE/PE等标志。
 */
static void RS485Bus_ClearRxState(void)
{
    volatile uint32_t temp;

    if (g_rs485_uart == NULL)
    {
        return;
    }

    temp = g_rs485_uart->Instance->SR;
    temp = g_rs485_uart->Instance->DR;
    (void)temp;
}

/**
 * @brief 在不重新配置GPIO和DMA的情况下更新UART波特率
 */
static RS485Bus_Status_t RS485Bus_SetBaudRate(uint32_t baud_rate)
{
    if (g_rs485_uart == NULL)
    {
        return RS485_BUS_ERR_NOT_INIT;
    }

    if (baud_rate == 0U)
    {
        return RS485_BUS_ERR_PARAM;
    }

    if (g_rs485_uart->Init.BaudRate == baud_rate)
    {
        return RS485_BUS_OK;
    }

    /*
     * 本工程USART2不再启动中断/DMA接收，所有RS485设备均采用
     * 阻塞式完整事务，因此此处可以安全重新写入BRR等UART参数。
     */
    g_rs485_uart->Init.BaudRate = baud_rate;

    if (HAL_UART_Init(g_rs485_uart) != HAL_OK)
    {
        return RS485_BUS_ERR_CONFIG;
    }

    RS485Bus_ClearRxState();
    return RS485_BUS_OK;
}

void RS485Bus_Init(UART_HandleTypeDef *huart)
{
    g_rs485_uart = huart;
    g_rs485_busy = 0U;

    /* 上电默认保持接收状态，避免占用总线 */
    RS485_DIR_RX();

    if (huart == NULL)
    {
        g_rs485_last_status = RS485_BUS_ERR_NOT_INIT;
    }
    else
    {
        g_rs485_last_status = RS485_BUS_OK;
        RS485Bus_ClearRxState();
    }
}

RS485Bus_Status_t RS485Bus_Transceive(
    uint32_t baud_rate,
    const uint8_t *tx_data,
    uint16_t tx_length,
    uint8_t *rx_data,
    uint16_t rx_length,
    uint32_t tx_timeout_ms,
    uint32_t rx_timeout_ms)
{
    HAL_StatusTypeDef hal_status;
    RS485Bus_Status_t bus_status;

    if (g_rs485_uart == NULL)
    {
        g_rs485_last_status = RS485_BUS_ERR_NOT_INIT;
        return g_rs485_last_status;
    }

    if ((tx_data == NULL) ||
        (tx_length == 0U) ||
        (rx_data == NULL) ||
        (rx_length == 0U) ||
        (baud_rate == 0U))
    {
        g_rs485_last_status = RS485_BUS_ERR_PARAM;
        return g_rs485_last_status;
    }

    if (g_rs485_busy != 0U)
    {
        g_rs485_last_status = RS485_BUS_ERR_BUSY;
        return g_rs485_last_status;
    }

    g_rs485_busy = 1U;

    bus_status = RS485Bus_SetBaudRate(baud_rate);
    if (bus_status != RS485_BUS_OK)
    {
        RS485_DIR_RX();
        g_rs485_busy = 0U;
        g_rs485_last_status = bus_status;
        return bus_status;
    }

    /* 丢弃上一笔事务可能残留的字节 */
    RS485Bus_ClearRxState();

    RS485_DIR_TX();
    HAL_Delay(RS485_DIR_SWITCH_DELAY_MS);

    hal_status = HAL_UART_Transmit(g_rs485_uart,
                                   (uint8_t *)tx_data,
                                   tx_length,
                                   tx_timeout_ms);

    /* HAL_UART_Transmit返回前已经等待TC置位，可立即转接收 */
    RS485_DIR_RX();

    if (hal_status != HAL_OK)
    {
        g_rs485_busy = 0U;
        g_rs485_last_status = RS485_BUS_ERR_TX;
        return g_rs485_last_status;
    }

    /* 立即启动接收，避免从站快速响应时发生首字节丢失 */
    hal_status = HAL_UART_Receive(g_rs485_uart,
                                  rx_data,
                                  rx_length,
                                  rx_timeout_ms);

    g_rs485_busy = 0U;

    if (hal_status == HAL_TIMEOUT)
    {
        g_rs485_last_status = RS485_BUS_ERR_RX_TIMEOUT;
        return g_rs485_last_status;
    }

    if (hal_status != HAL_OK)
    {
        g_rs485_last_status = RS485_BUS_ERR_RX;
        return g_rs485_last_status;
    }

    g_rs485_last_status = RS485_BUS_OK;
    return g_rs485_last_status;
}

uint32_t RS485Bus_GetBaudRate(void)
{
    if (g_rs485_uart == NULL)
    {
        return 0U;
    }

    return g_rs485_uart->Init.BaudRate;
}

RS485Bus_Status_t RS485Bus_GetLastStatus(void)
{
    return g_rs485_last_status;
}
