/**
 * @file HMIProtocol.h
 * @brief FFT Gyroboard上位机通信顶层接口
 * @author wangziwei
 * @date 2025-12-18
 */

#ifndef __HMIPROTOCOL_H
#define __HMIPROTOCOL_H

#include "HMIPacket.h"
#include "stm32f1xx_hal.h"


/*
 * 0x15 = 所有电机Wheel模式 (01 01 01)
 * 0x2A = 所有电机Joint模式 (10 10 10)
 * 协议位定义：Bit[5:4]=M1, Bit[3:2]=M2, Bit[1:0]=M3
 */
#define HMI_MOTORMODE_ALL_WHEEL    0x15
#define HMI_MOTORMODE_ALL_JOINT    0x2A

// 定义3个舵机对应的上位机HMI_MotorPacket_t结构体，用于保存参数值
extern HMI_MotorPacket_t HMImotorpacket;
// 定义3个编码器对应的上位机HMI_EncoderPacket_t结构体，用于保存参数值
extern HMI_EncoderPacket_t  HMIencoderpacket;

/* 串口发送错误代码定义 */
typedef enum {
    HMI_OK = 0,           // 操作成功
    HMI_ERR_INIT,         // 未初始化
    HMI_ERR_PARAM,        // 参数错误
    HMI_ERR_BUSY,         // 串口忙
    HMI_ERR_TIMEOUT       // 超时
} HMI_Status_typedef; // 串口状态

/**
 * @brief 初始化上位机使用的串口
 * @param huart 串口句柄，如&huart1
 * @retval HMI_OK=成功
 */
HMI_Status_typedef HMI_Init(UART_HandleTypeDef *huart);

/**
 * @brief 发送电机数据包到上位机
 * @param motorData 电机数据结构指针
 * @param dataRate 数据速率倍数 (0~255)
 * @retval HMI_Status_typedef 串口状态
 */
HMI_Status_typedef HMI_SendMotorData(const HMI_MotorPacket_t *motorData,
                               uint8_t dataRate);

/**
 * @brief 发送编码器数据包到上位机
 * @param encoderData 编码器数据结构指针
 * @param dataRate 数据速率倍数
 * @retval HMI_Status_typedef 串口状态
 */
HMI_Status_typedef HMI_SendEncoderData(const HMI_EncoderPacket_t *encoderData,
                                 uint8_t dataRate);



/**
 * @brief 发送力传感器数据到上位机
 * @param forceRaw 力传感器32位有符号原始值
 * @retval HMI_Status_typedef 串口发送状态
 */
HMI_Status_typedef HMI_SendForceData(int32_t forceRaw);


/**
 * @brief 扭矩/速度方向 位（bit15）
 * @param value 绝对值 (0~1023)
 * @param direction 方向：0=CCW（逆时针），1=CW（顺时针）
 * @retval uint16_t 编码后的值 (0~2047)
 */
uint16_t HMI_EncodeTorqueVelocity(uint16_t value, uint8_t direction);

#endif /* __HMIPROTOCOL_H */

