#ifndef __FORCE_SENSOR_H
#define __FORCE_SENSOR_H

#include "stm32f1xx_hal.h"

#define FORCE_SENSOR_BAUD_RATE       19200U
#define FORCE_SENSOR_ADDRESS         0x01U
#define FORCE_SENSOR_RX_FRAME_SIZE   9U

typedef enum
{
    FORCE_SENSOR_OK = 0,
    FORCE_SENSOR_ERR_NOT_INIT,
    FORCE_SENSOR_ERR_PARAM,
    FORCE_SENSOR_ERR_TX,
    FORCE_SENSOR_ERR_TIMEOUT,
    FORCE_SENSOR_ERR_UART,
    FORCE_SENSOR_ERR_FRAME,
    FORCE_SENSOR_ERR_CRC
} ForceSensor_Status_t;

/* 保存最近一次接收到的完整帧，方便在Keil中观察 */
extern uint8_t ForceSensor_LastRx[FORCE_SENSOR_RX_FRAME_SIZE];

/** @brief 初始化力传感器，并绑定共享RS485 UART */
void ForceSensor_Init(UART_HandleTypeDef *huart);

/** @brief 读取一次32位有符号原始力值 */
ForceSensor_Status_t ForceSensor_ReadRaw(int32_t *raw_value);

uint16_t ForceSensor_ModbusCRC16(const uint8_t *data,
                                uint16_t length);

#endif /* __FORCE_SENSOR_H */
