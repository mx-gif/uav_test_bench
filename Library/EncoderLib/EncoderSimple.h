/**
 * @file EncoderSimple.h
 * @brief 通过共享USART2/RS485总线读取1～3号编码器
 */
#ifndef __ENCODER_SIMPLE_H
#define __ENCODER_SIMPLE_H

#include "stm32f1xx_hal.h"

#define ENCODER_COUNT             3U
#define ENCODER_RX_FRAME_SIZE     7U
#define ENCODER_TX_FRAME_SIZE     8U
#define ENCODER_ALL_MASK          0x07U
#define ENCODER_BAUD_RATE         115200
#define ENCODER_DEFAULT_TIMEOUT_MS 30U

typedef enum
{
    ENCODER_OK = 0,
    ENCODER_ERR_NOT_INIT,
    ENCODER_ERR_PARAM,
    ENCODER_ERR_TX,
    ENCODER_ERR_TIMEOUT,
    ENCODER_ERR_UART,
    ENCODER_ERR_FRAME,
    ENCODER_ERR_CRC,
    ENCODER_ERR_NO_ACTIVE_DEVICE
} Encoder_Status_t;

/* ID 1、2、3对应数组下标0、1、2 */
extern float Encoder_Position[ENCODER_COUNT];

/* 最近一次收发帧和状态，方便Keil Watch观察 */
extern uint8_t Encorder_Txpacket[ENCODER_TX_FRAME_SIZE];
extern uint8_t Encorder_Rxpacket[ENCODER_RX_FRAME_SIZE];
extern volatile Encoder_Status_t Encoder_LastStatus;
extern volatile uint8_t Encoder_LastPolledId;

/** @brief 初始化编码器驱动，并绑定共享RS485 UART */
void Encoder_Init(UART_HandleTypeDef *huart);

/**
 * @brief 设置运行阶段需要轮询的编码器
 * @param active_mask bit0、bit1、bit2分别对应ID 1、2、3
 */
void Encoder_SetActiveMask(uint8_t active_mask);

/**
 * @brief 探测并读取指定ID编码器
 * @retval 1：收到正确响应；0：未收到或响应错误
 */
uint8_t Encoder_Probe(uint8_t encoder_id, uint32_t timeout_ms);

/**
 * @brief 阻塞式查询下一台有效编码器
 * @note  函数内部完成发送、接收、CRC校验和角度更新，随后切换到下一ID。
 */
Encoder_Status_t Encoder_PollNext(void);

/**
 * @brief 直接读取指定ID编码器
 */
Encoder_Status_t Encoder_ReadById(uint8_t encoder_id,
                                  uint32_t timeout_ms);

/** @brief Modbus RTU CRC16，保留原工程函数名 */
unsigned int Crc_Count(unsigned char pbuf[], unsigned char num);

#endif /* __ENCODER_SIMPLE_H */
