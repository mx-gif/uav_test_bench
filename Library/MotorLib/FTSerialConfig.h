/**
 * @file FTSerialConfig.h
 * @brief FT芯片串口底层驱动头文件
 * @author wangziwei
 */

#ifndef FT_SERIAL_CONFIG_H
#define FT_SERIAL_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含必要的头文件 */
#include "main.h"  // 包含HAL库和UART句柄定义
#include <stdint.h>

/* ==================== 配置参数 ==================== */
//#define FT_UART_RX_BUF_SIZE  128    // DMA接收缓冲区大小
//#define FT_UART_TIMEOUT      100    // 读取超时时间(ms)

/* ==================== 函数声明 ==================== */

/**
 * @brief 初始化FT芯片使用的串口
 * @param huart 串口句柄指针，如&huart1、&huart2
 * @retval None
 */
void ftUart_Init(UART_HandleTypeDef *huart);

/**
 * @brief 向舵机发送数据
 * @param nDat 要发送的数据缓冲区指针
 * @param nLen 要发送的数据长度
 * @retval None
 */
void ftUart_Send(uint8_t *nDat, int nLen);

/**
 * @brief 从舵机读取数据
 * @param nDat 接收数据缓冲区指针
 * @param nLen 期望接收的数据长度
 * @retval int 实际接收到的数据长度，0表示接收失败
 */
int ftUart_Read(uint8_t *nDat, int nLen);

/**
 * @brief FT芯片总线延时函数
 * @note 延时约10us（实际为1ms，需要根据系统时钟调整）
 */
void ftBus_Delay(void);


/**
 * @brief 清空接收缓冲区(用于异常恢复)
 */
void ftUart_Flush(void);

/* ==================== 中断回调(需要在main.c中调用) ==================== */

/**
 * @brief 空闲中断回调(在HAL_UARTEx_RxEventCallback中调用)
 * @param huart UART句柄
 * @param Size 实际接收的字节数
 */
//void ftUart_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);



#ifdef __cplusplus
}
#endif

#endif /* FT_SERIAL_CONFIG_H */

