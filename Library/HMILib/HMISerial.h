/**
 * @file HMISerial.h
 * @brief FFT Gyroboard上位机通信底层串口驱动
 * @author wangziwei
 * @date 2025-12-18
 */

#ifndef __HMISERIAL_H
#define __HMISERIAL_H

#include "stm32f1xx_hal.h"  // 根据你的MCU型号修改
#include <stdint.h>

/**
 * @brief 初始化FFT Gyroboard使用的串口
 * @param huart 串口句柄指针，如&huart1、&huart2
 * @retval None
 */
void HMI_Serial_Init(UART_HandleTypeDef *huart);


/* ==================== 串口发送 ==================== */
/**
 * @brief 向上位机发送数据包（异步发送）
 * @param pData 要发送的数据缓冲区指针
 * @param Size 要发送的数据长度
 * @retval HAL状态：HAL_OK=成功，HAL_ERROR=失败
 */
HAL_StatusTypeDef HMI_Serial_Send(uint8_t *pData, uint16_t Size);

/**
 * @brief 检查串口是否发送完成
 * @retval 1=发送完成，0=正在发送
 */
uint8_t HMI_Serial_IsTxComplete(void);



/* ==================== 串口接收 ==================== */
/****************接口函数*****************/
/**
 * @brief 获取接收完成标志
 * @retval 1=有新数据,0=无数据
 */
uint8_t HMI_Serial_GetRxFlag(void);

/**
 * @brief 获取接收缓冲区指针
 * @retval 接收缓冲区 的首地址
 */
uint8_t* HMI_Serial_GetRxBuffer(void);

/**
 * @brief 获取接收数据长度
 * @retval 接收数据长度(字节)
 */
uint16_t HMI_Serial_GetRxLength(void);


/**
 * @brief 清除接收标志并重启DMA接收
 * @retval 无
 * @note 处理完数据后必须调用此函数重启DMA
 */
void HMI_Serial_RestartReceive(void);


/* ==================== 中断回调 ==================== */
/**
 * @brief DMA接收完成回调函数
 * @param huart UART句柄
 * @param Size 实际接收的字节数
 * @note 此函数由HAL_UARTEx_RxEventCallback()调用
 */
void HMI_Serial_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif /* __HMISERIAL_H */

