/**
 * @file RS485Bus.h
 * @brief USART2共享RS485总线驱动
 *
 * 编码器与力传感器共用同一个USART和同一个RS485收发器。
 * 本模块负责：
 * 1. 统一控制PB2的DE/RE方向；
 * 2. 根据目标设备切换USART波特率；
 * 3. 保证一次查询和一次响应作为完整事务顺序执行。
 */
#ifndef __RS485_BUS_H
#define __RS485_BUS_H

#include "stm32f1xx_hal.h"

typedef enum
{
    RS485_BUS_OK = 0,
    RS485_BUS_ERR_NOT_INIT,
    RS485_BUS_ERR_PARAM,
    RS485_BUS_ERR_BUSY,
    RS485_BUS_ERR_CONFIG,
    RS485_BUS_ERR_TX,
    RS485_BUS_ERR_RX_TIMEOUT,
    RS485_BUS_ERR_RX
} RS485Bus_Status_t;

/**
 * @brief 初始化共享RS485总线
 * @param huart 共用的UART句柄，本工程传入&huart2
 */
void RS485Bus_Init(UART_HandleTypeDef *huart);

/**
 * @brief 执行一次完整的RS485查询事务
 * @param baud_rate 本次目标设备的波特率
 * @param tx_data 发送缓冲区
 * @param tx_length 发送长度
 * @param rx_data 接收缓冲区
 * @param rx_length 期望接收长度
 * @param tx_timeout_ms 发送超时
 * @param rx_timeout_ms 接收超时
 */
RS485Bus_Status_t RS485Bus_Transceive(
    uint32_t baud_rate,
    const uint8_t *tx_data,
    uint16_t tx_length,
    uint8_t *rx_data,
    uint16_t rx_length,
    uint32_t tx_timeout_ms,
    uint32_t rx_timeout_ms);

/** @brief 获取当前USART配置的波特率 */
uint32_t RS485Bus_GetBaudRate(void);

/** @brief 获取最近一次总线事务状态，方便Keil Watch观察 */
RS485Bus_Status_t RS485Bus_GetLastStatus(void);

#endif /* __RS485_BUS_H */
