/**
 * @file HMIPacket.h
 * @brief FFT Gyroboard中层数据包结构定义与封装
 * @author wangziwei
 * @date 2025-12-18
 */

#ifndef __HMIPACKET_H
#define __HMIPACKET_H

#include <stdint.h>

/* 协议常量定义 */
#define HMI_PACKET_SEND_SIZE         32      // 数据包大小（字节）
#define HMI_PACKET_SEND_HEADER       0x7A    // 包头标识
#define HMI_PACKET_SEND_TAIL         0x7B    // 包尾标识

#define HMI_MODE_MOTOR          0x01    // 电机模式
#define HMI_MODE_ENCODER        0x02    // 编码器模式
#define HMI_FORCE_PACKET_SEND_SIZE   7U      // 力传感器数据包长度

#define HMI_MODE_FORCE               0xF0U   // 力传感器模式码

/**
 * @brief 单个电机数据结构
 */
typedef struct {
    uint8_t  error;         // 错误状态字节
    uint16_t torque;        // 扭矩值 (0~2047, 0.1%步进)
    uint16_t velocity;      // 速度值 (0~2047, 0.1%步进)
    uint16_t position;      // 位置值 (0~1023)
    uint8_t  voltage;       // 电压值 (实际值×10)
    uint8_t  temperature;   // 温度值 (°C)
} HMI_MotorData_t;

/**
 * @brief 电机模式完整数据包结构
 */
typedef struct {
    HMI_MotorData_t motor1;   // 电机1数据
    HMI_MotorData_t motor2;   // 电机2数据
    HMI_MotorData_t motor3;   // 电机3数据
    uint8_t         modeMotors; // 电机模式字节（不是来源于舵机回传信息，而是来源于上位机设置）
} HMI_MotorPacket_t;

/**
 * @brief 编码器数据结构
 */
typedef struct {
    float encoder1_position;  // 编码器1位置 (度)
    float encoder2_position;  // 编码器2位置 (度)
    float encoder3_position;  // 编码器3位置 (度)
} HMI_EncoderPacket_t;

/**
 * @brief 构建电机模式数据包
 * @param packet 电机数据包结构指针
 * @param dataRate 数据速率倍数 (0=10ms, 1=20ms, 10=100ms, 100=1s)
 * @param buffer 输出缓冲区（至少32字节）
 * @retval None
 */
void HMI_BuildMotorPacket(const HMI_MotorPacket_t *packet, 
                          uint8_t dataRate, 
                          uint8_t *buffer);

/**
 * @brief 构建编码器模式数据包
 * @param packet 编码器数据包结构指针
 * @param dataRate 数据速率倍数
 * @param buffer 输出缓冲区（至少32字节）
 * @retval None
 */
void HMI_BuildEncoderPacket(const HMI_EncoderPacket_t *packet,
                            uint8_t dataRate,
                            uint8_t *buffer);


/**
 * @brief 构建力传感器数据包
 * @param forceRaw 力传感器32位有符号原始值
 * @param buffer 输出缓冲区，长度至少为7字节
 * @retval None
 */
void HMI_BuildForcePacket(int32_t forceRaw,
                          uint8_t *buffer);

#endif /* __HMIPACKET_H */
