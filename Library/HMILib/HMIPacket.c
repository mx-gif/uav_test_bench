/**
 * @file HMIPacket.c
 * @brief FFT Gyroboard中层数据包封装实现
 * @author wangziwei
 * @date 2025-12-18
 */

#include "HMIPacket.h"
#include <stdio.h>
#include <string.h>

/**
 * @brief 构建电机模式数据包（32字节）
 * @note 数据包格式参考PDF第31-33页
 */
void HMI_BuildMotorPacket(const HMI_MotorPacket_t *packet, 
                          uint8_t dataRate, 
                          uint8_t *buffer)
{
    uint8_t index = 0;
    
    // Byte 0: 包头
    buffer[index++] = HMI_PACKET_SEND_HEADER;
    
    // Byte 1: 系统模式
    buffer[index++] = HMI_MODE_MOTOR;
    
    // Byte 2: 数据速率
    buffer[index++] = dataRate;
    
    // Byte 3-5: 三个电机的错误状态
    buffer[index++] = packet->motor1.error;
    buffer[index++] = packet->motor2.error;
    buffer[index++] = packet->motor3.error;
    
    // Byte 6-11: 三个电机的扭矩值（每个2字节，小端格式）
    buffer[index++] = (uint8_t)(packet->motor1.torque & 0xFF);
    buffer[index++] = (uint8_t)(packet->motor1.torque >> 8);
    buffer[index++] = (uint8_t)(packet->motor2.torque & 0xFF);
    buffer[index++] = (uint8_t)(packet->motor2.torque >> 8);
    buffer[index++] = (uint8_t)(packet->motor3.torque & 0xFF);
    buffer[index++] = (uint8_t)(packet->motor3.torque >> 8);
    
    // Byte 12-17: 三个电机的速度值（每个2字节，小端格式）
    buffer[index++] = (uint8_t)(packet->motor1.velocity & 0xFF);
	//buffer[index++] = (uint8_t)0x4B;
    buffer[index++] = (uint8_t)(packet->motor1.velocity >> 8);
    buffer[index++] = (uint8_t)(packet->motor2.velocity & 0xFF);
	//buffer[index++] = (uint8_t)0x07;
    buffer[index++] = (uint8_t)(packet->motor2.velocity >> 8);
    buffer[index++] = (uint8_t)(packet->motor3.velocity & 0xFF);
	//buffer[index++] = (uint8_t)0x07;
    buffer[index++] = (uint8_t)(packet->motor3.velocity >> 8);
    
    // Byte 18-23: 三个电机的位置值（每个2字节，小端格式）
    buffer[index++] = (uint8_t)(packet->motor1.position & 0xFF);
    buffer[index++] = (uint8_t)(packet->motor1.position >> 8);
    buffer[index++] = (uint8_t)(packet->motor2.position & 0xFF);
    buffer[index++] = (uint8_t)(packet->motor2.position >> 8);
    buffer[index++] = (uint8_t)(packet->motor3.position & 0xFF);
    buffer[index++] = (uint8_t)(packet->motor3.position >> 8);
    
    // Byte 24-26: 三个电机的电压值（每个1字节）
    buffer[index++] = packet->motor1.voltage;
    buffer[index++] = packet->motor2.voltage;
    buffer[index++] = packet->motor3.voltage;
    
    // Byte 27-29: 三个电机的温度值（每个1字节）
    buffer[index++] = packet->motor1.temperature;
    buffer[index++] = packet->motor2.temperature;
    buffer[index++] = packet->motor3.temperature;
    
    // Byte 30: 电机模式字节
    buffer[index++] = packet->modeMotors;
    
    // Byte 31: 包尾
    buffer[index++] = HMI_PACKET_SEND_TAIL;
}





/**
 * @brief 构建编码器模式数据包（32字节）
 * @note 数据包格式参考PDF第29-31页
 */
void HMI_BuildEncoderPacket(const HMI_EncoderPacket_t *packet,
                            uint8_t dataRate,
                            uint8_t *buffer)
{
    uint8_t index = 0;
    char posStr[9];  // 存储格式化的位置字符串
    
    // Byte 0: 包头
    buffer[index++] = HMI_PACKET_SEND_HEADER;
    
    // Byte 1: 系统模式
    buffer[index++] = HMI_MODE_ENCODER;
    
    // Byte 2: 上位机数据速率
    buffer[index++] = dataRate;
    
    // Byte 3: 分隔符
    buffer[index++] = ',';
    
    // Byte 4-11: 编码器1位置（格式：XXX.XXXX，8字节ASCII）
    snprintf(posStr, sizeof(posStr), "%08.4f", packet->encoder1_position);
    memcpy(&buffer[index], posStr, 8);
    index += 8;
    
    // Byte 12: 分隔符
    buffer[index++] = ',';
    
    // Byte 13-20: 编码器2位置
    snprintf(posStr, sizeof(posStr), "%08.4f", packet->encoder2_position);
    memcpy(&buffer[index], posStr, 8);
    index += 8;
    
    // Byte 21: 分隔符
    buffer[index++] = ',';
    
    // Byte 22-29: 编码器3位置
    snprintf(posStr, sizeof(posStr), "%08.4f", packet->encoder3_position);
    memcpy(&buffer[index], posStr, 8);
    index += 8;
    
    // Byte 30: 分隔符
    buffer[index++] = ',';
    
    // Byte 31: 包尾
    buffer[index++] = HMI_PACKET_SEND_TAIL;
}


/**
 * @brief 构建力传感器数据包（7字节）
 * @param forceRaw 力传感器32位有符号原始值
 * @param buffer 输出缓冲区，长度至少为7字节
 * @note 力值采用小端顺序发送，低字节在前
 */
void HMI_BuildForcePacket(int32_t forceRaw,
                          uint8_t *buffer)
{
    uint32_t forceData;

    /*
     * 转换为无符号数后再进行移位。
     * 只改变数据的解释方式，不改变原来的32位二进制内容，
     * 因此正数和负数都可以正确传输。
     */
    forceData = (uint32_t)forceRaw;

    // Byte 0：统一包头
    buffer[0] = HMI_PACKET_SEND_HEADER;

    // Byte 1：力传感器模式码
    buffer[1] = HMI_MODE_FORCE;

    // Byte 2～5：32位力值，小端顺序发送
    buffer[2] = (uint8_t)(forceData & 0xFFU);
    buffer[3] = (uint8_t)((forceData >> 8)  & 0xFFU);
    buffer[4] = (uint8_t)((forceData >> 16) & 0xFFU);
    buffer[5] = (uint8_t)((forceData >> 24) & 0xFFU);

    // Byte 6：统一包尾
    buffer[6] = HMI_PACKET_SEND_TAIL;
}

