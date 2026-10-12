/**
 * @file EncoderSerial.h
 * @brief 编码器串口底层驱动（中断接收方式）
 */

#ifndef __ENCODER_SERIAL_H
#define __ENCODER_SERIAL_H

#include "stm32f1xx_hal.h"

/* RS485方向控制引脚定义 */
#define RS485_DIR_PORT      GPIOB
#define RS485_DIR_PIN       GPIO_PIN_2
#define RS485_DIR_TX()      HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_PIN_SET)
#define RS485_DIR_RX()      HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_PIN_RESET)

/* 编码器通信参数 */
#define ENCODER_RX_SIZE     7       // 接收数据包大小

/* 函数声明 */
void Encoder_Serial_Init(UART_HandleTypeDef *huart);
void Encoder_Serial_SendArray(uint8_t *Array, uint16_t Length);
uint8_t Encoder_Serial_GetRxFlag(void);
uint8_t* Encoder_Serial_GetRxBuffer(void);
void Encoder_Serial_RxHandler(UART_HandleTypeDef *huart);  // 中断回调处理函数

#endif // __ENCODER_SERIAL_H
