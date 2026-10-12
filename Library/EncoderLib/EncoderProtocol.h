/**
 * @file EncoderProtocol.h
 * @brief 编码器协议封装层
 * @details 基于Modbus RTU协议的绝对式编码器通信库
 *          采用中断接收 + 异步Ask/Read模式
 * @author wangziwei
 * @date 2025-01-10
 */

#ifndef __ENCODER_PROTOCOL_H
#define __ENCODER_PROTOCOL_H

#include "stm32f1xx_hal.h"
#include "EncoderSerial.h"

/*============================ 配置参数 ============================*/

/* 编码器物理参数 */
#define ENCODER_COUNT           3           // 编码器数量
#define ENCODER_RESOLUTION      4096        // 分辨率（12位）
#define ENCODER_MAX_ANGLE       360.0f      // 最大角度

/* 通信参数 */
#define ENCODER_BAUDRATE        9600        // 波特率
#define ENCODER_RETRY_TIMES     3           // 重试次数
#define ENCODER_TIMEOUT_MS      100         // 超时时间(ms)

/*============================ 状态码定义 ============================*/

/**
 * @brief 编码器操作状态码
 */
typedef enum {
    ENCODER_OK = 0,             // 操作成功
    ENCODER_ERR_INIT,           // 未初始化
    ENCODER_ERR_PARAM,          // 参数错误
    ENCODER_ERR_TIMEOUT,        // 通信超时
    ENCODER_ERR_CRC,            // CRC校验失败
    ENCODER_ERR_BUSY,           // 设备忙（上次询问未完成）
    ENCODER_ERR_NO_RESPONSE     // 未收到响应
} Encoder_Status_t;

/*============================ 数据结构 ============================*/

/**
 * @brief 单个编码器的数据结构
 */
typedef struct {
    float    position;          // 当前角度位置(°) [0.0 ~ 360.0]
    uint16_t raw_value;         // 原始编码值 [0 ~ 4095]
    uint8_t  encoder_id;        // 编码器ID [1, 2, 3]
    uint8_t  is_valid;          // 数据有效标志 (1=有效, 0=无效)
    uint32_t last_update_tick;  // 最后更新时间戳(HAL_GetTick)
} Encoder_Data_t;

/**
 * @brief 编码器系统状态结构
 */
typedef struct {
    uint8_t  initialized;       // 初始化标志
    uint8_t  current_id;        // 当前正在查询的编码器ID
    uint8_t  ask_read_flag;     // Ask/Read状态标志 (0=可Ask, 1=等待Read)
    uint32_t total_requests;    // 总请求次数（调试用）
    uint32_t failed_requests;   // 失败请求次数（调试用）
} Encoder_System_t;

/*============================ 函数声明 ============================*/

/**
 * @brief 初始化编码器系统
 * @param huart UART句柄指针（已配置为9600, 8N1）
 * @retval ENCODER_OK=成功, 其他=失败
 * @note 必须先调用MX_USARTx_UART_Init()
 */
Encoder_Status_t Encoder_Init(UART_HandleTypeDef *huart);

/**
 * @brief 发送位置查询请求（异步）
 * @retval ENCODER_OK=发送成功
 *         ENCODER_ERR_BUSY=上次请求未完成
 *         ENCODER_ERR_INIT=未初始化
 * @note 调用后需等待10-50ms，然后调用Encoder_ReadPosition()读取
 */
Encoder_Status_t Encoder_AskPosition(void);

/**
 * @brief 读取编码器响应数据
 * @param data 3个编码器的数据数组（至少3个元素）
 * @retval ENCODER_OK=读取成功
 *         ENCODER_ERR_TIMEOUT=未收到数据
 *         ENCODER_ERR_PARAM=参数错误
 * @note 仅在Encoder_AskPosition()发送后调用
 */
Encoder_Status_t Encoder_ReadPosition(Encoder_Data_t data[3]);

/**
 * @brief 一次性读取单个编码器（同步方式）
 * @param encoder_id 编码器ID [1, 2, 3]
 * @param data 单个编码器数据结构指针
 * @retval Encoder_Status_t 状态码
 * @note 此函数会阻塞约10-20ms，适合低频采样
 */
Encoder_Status_t Encoder_ReadSingle(uint8_t encoder_id, Encoder_Data_t *data);

/**
 * @brief 获取编码器系统状态（调试用）
 * @param status 系统状态结构指针
 */
void Encoder_GetSystemStatus(Encoder_System_t *status);

/**
 * @brief 复位编码器系统
 * @note 清除所有状态标志，重新开始通信
 */
void Encoder_Reset(void);

/*============================ 辅助宏定义 ============================*/

/**
 * @brief 检查编码器ID是否有效
 */
#define IS_ENCODER_ID_VALID(id) ((id) >= 1 && (id) <= ENCODER_COUNT)

/**
 * @brief 检查角度值是否有效
 */
#define IS_ANGLE_VALID(angle) ((angle) >= 0.0f && (angle) < ENCODER_MAX_ANGLE)

/**
 * @brief 原始值转角度
 */
#define RAW_TO_ANGLE(raw) (((float)(raw) * ENCODER_MAX_ANGLE) / ENCODER_RESOLUTION)

/**
 * @brief 角度转原始值
 */
#define ANGLE_TO_RAW(angle) ((uint16_t)((angle) * ENCODER_RESOLUTION / ENCODER_MAX_ANGLE))

#endif // __ENCODER_PROTOCOL_H
