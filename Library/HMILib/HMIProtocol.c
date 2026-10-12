/**
 * @file HMIProtocol.c
 * @brief FFT Gyroboard上位机通信顶层实现
 * @author wangziwei
 * @date 2025-12-18
 */

#include "HMIProtocol.h"
#include "HMISerial.h"
#include <string.h>




// 发送数据包格式
//uint8_t HMI_MotorTxPacket[32] = {
//	0x7A, 0x01, 0x31, 0x32, 0x00, 0x00,
//	0x02, 0x02, 0x02, 0x02, 0x02, 0x02,//电机扭矩
//	0x02, 0x02, 0x02, 0x02, 0x02, 0x02,//电机速度
//	0x02, 0x02, 0x02, 0x02, 0x02, 0x02,//电机位置
//	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//电机电压  电机温度
//	0x2A, 0x7B//模式选择
//};
//uint8_t HMI_EncoderTxPacket[32] = {
//	0x7A, 0x02, 0x0A, 
//	',', '0', '1', '0', '.', '1', '2', '0', '9',//编码器1
//	',', '0', '1', '0', '.', '1', '2', '0', '9',//编码器2
//	',', '0', '1', '0', '.', '1', '2', '0', '9',//编码器3
//	0x2C, 0x7B//模式选择
//};



/* 发送缓冲区 */
static uint8_t HMI_txBuffer[HMI_PACKET_SEND_SIZE];
static uint8_t HMI_isInitialized = 0;



// 定义3个舵机对应的上位机HMI_MotorPacket_t结构体，用于保存参数值
HMI_MotorPacket_t HMImotorpacket;
// 定义3个编码器对应的上位机HMI_EncoderPacket_t结构体，用于保存参数值
HMI_EncoderPacket_t  HMIencoderpacket;


/**
 * @brief 初始化上位机使用的串口
 * @param huart 串口句柄，如&huart1
 * @retval HMI_OK=成功
 */
HMI_Status_typedef HMI_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return HMI_ERR_PARAM; // 串口参数配置错误
    }
    
    HMI_Serial_Init(huart);
    HMI_isInitialized = 1;
    
    return HMI_OK;
}



/**
 * @brief 发送电机数据包到上位机
 * @param motorData 电机数据结构指针
 * @param dataRate 数据速率倍数 (0~255)
 * @retval HMI_Status_typedef 状态码
 */
HMI_Status_typedef HMI_SendMotorData(const HMI_MotorPacket_t *motorData,
                               uint8_t dataRate)
{
	/* 1. 检查HMI串口是否初始化 */
    if (!HMI_isInitialized) {  
        return HMI_ERR_INIT;
    }
    
	/* 2. 检查输入参数有效性 */
    if (motorData == NULL) {
        return HMI_ERR_PARAM;
    }
    
    /* 3. 检查串口是否空闲 */
    if (!HMI_Serial_IsTxComplete()) {
        return HMI_ERR_BUSY;
    }
    
    /* 4. 构建数据包 */
    HMI_BuildMotorPacket(motorData, dataRate, HMI_txBuffer);
    
    /* 5. 发送数据包 */
    if (HAL_OK != HMI_Serial_Send(HMI_txBuffer, HMI_PACKET_SEND_SIZE)) {
        return HMI_ERR_TIMEOUT; // 串口异步发送失败，认为串口发送超时
    }
    
    return HMI_OK;
}


/**
 * @brief 发送编码器数据包到上位机
 * @param encoderData 编码器数据结构指针
 * @param dataRate 数据速率倍数
 * @retval HMI_Status_typedef 串口状态
 */
HMI_Status_typedef HMI_SendEncoderData(const HMI_EncoderPacket_t *encoderData,
                                 uint8_t dataRate)
{
	/* 1. 检查HMI串口是否初始化 */
    if (!HMI_isInitialized) {
        return HMI_ERR_INIT;
    }
    
	/* 2. 检查输入参数有效性 */
    if (encoderData == NULL) {
        return HMI_ERR_PARAM;
    }
    
	/* 3. 检查串口是否空闲 */
    if (!HMI_Serial_IsTxComplete()) {
        return HMI_ERR_BUSY;
    }
    
    /* 4. 构建数据包 */
    HMI_BuildEncoderPacket(encoderData, dataRate, HMI_txBuffer);
    
    /* 5. 发送数据包 */
    if (HAL_OK != HMI_Serial_Send(HMI_txBuffer, HMI_PACKET_SEND_SIZE)) {
        return HMI_ERR_TIMEOUT;
    }
    
    return HMI_OK;
}



/**
 * @brief 发送力传感器数据到上位机
 * @param forceRaw 力传感器32位有符号原始值
 * @retval HMI_Status_typedef 串口发送状态
 */
HMI_Status_typedef HMI_SendForceData(int32_t forceRaw)
{
    /* 1. 检查HMI串口是否已经初始化 */
    if (!HMI_isInitialized)
    {
        return HMI_ERR_INIT;
    }

    /* 2. 检查上一次DMA发送是否已经完成 */
    if (!HMI_Serial_IsTxComplete())
    {
        return HMI_ERR_BUSY;
    }

    /* 3. 将力传感器原始值封装成7字节数据包 */
    HMI_BuildForcePacket(forceRaw, HMI_txBuffer);

    /* 4. 使用DMA将7字节数据包发送给上位机 */
    if (HAL_OK != HMI_Serial_Send(HMI_txBuffer,
                                  HMI_FORCE_PACKET_SEND_SIZE))
    {
        return HMI_ERR_TIMEOUT;
    }

    return HMI_OK;
}



/**
 * @brief 扭矩/速度方向 位（bit15）
 * @note 范围0~1023=CCW方向，1024~2047=CW方向
 */
uint16_t HMI_EncodeTorqueVelocity(uint16_t value, uint8_t direction)
{
    if (value > 1023) {
        value = 1023;  // 限幅
    }
    
    if (direction == 1) {
        // CW方向：加上1024
        return (value == 0) ? 1024 : (value + 1024);
    } else {
        // CCW方向：直接返回
        return value;
    }
}

