/**
 * @file EncoderProtocol.c
 * @brief 编码器协议实现（简化版）
 */

#include "EncoderProtocol.h"
#include <string.h>

/* 预定义请求指令 */
static const uint8_t CMD[3][8] = {
    {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A},
    {0x02, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x39},
    {0x03, 0x03, 0x00, 0x00, 0x00, 0x01, 0x85, 0xE8}
};

static uint8_t initialized = 0;
static uint8_t current_id = 0;  // 当前查询的ID (0~2)
static uint8_t ask_pending = 0; // 询问已发送标志

/**
 * @brief 初始化
 */
Encoder_Status_t Encoder_Init(UART_HandleTypeDef *huart)
{
    if (!huart) return ENCODER_ERR_PARAM;
    
    Encoder_Serial_Init(huart);
    initialized = 1;
    current_id = 0;
    ask_pending = 0;
    
    return ENCODER_OK;
}

/**
 * @brief 发送查询请求
 */
Encoder_Status_t Encoder_AskPosition(void)
{
    if (!initialized) return ENCODER_ERR_INIT;
    
    // 发送当前ID的查询指令
    Encoder_Serial_SendArray((uint8_t*)CMD[current_id], 8);
    
    // 轮询到下一个ID
    current_id = (current_id + 1) % 3;
    //ask_pending = 1;
    
    return ENCODER_OK;
}

/**
 * @brief 读取响应数据
 */
Encoder_Status_t Encoder_ReadPosition(Encoder_Data_t data[3])
{
    if (!initialized || !data) return ENCODER_ERR_PARAM;
    if (!Encoder_Serial_GetRxFlag()) return ENCODER_ERR_TIMEOUT;
    
    uint8_t *rx = Encoder_Serial_GetRxBuffer();
    uint8_t id = rx[0];
    
    // 验证ID范围
    if (id < 0x01 || id > 0x03) {
        ask_pending = 0;
        return ENCODER_ERR_NO_RESPONSE;
    }
    
    uint8_t idx = id - 1;  // 转换为数组索引 (0~2)
    
    // 解析数据
    uint16_t raw = (rx[4] << 8) | rx[3];
    data[idx].raw_value = raw;
    data[idx].position = (float)raw * 360.0f / 4096.0f;
    data[idx].encoder_id = id;
    data[idx].is_valid = 1;
    data[idx].last_update_tick = HAL_GetTick();
    
    ask_pending = 0;
    return ENCODER_OK;
}

/**
 * @brief 同步读取单个编码器
 */
Encoder_Status_t Encoder_ReadSingle(uint8_t encoder_id, Encoder_Data_t *data)
{
    if (!initialized || !data || encoder_id < 1 || encoder_id > 3) {
        return ENCODER_ERR_PARAM;
    }
    
    // 发送查询
    Encoder_Serial_SendArray((uint8_t*)CMD[encoder_id - 1], 8);
    
    // 等待响应（最多100ms）
    uint32_t start = HAL_GetTick();
    while (!Encoder_Serial_GetRxFlag()) {
        if (HAL_GetTick() - start > 100) {
            return ENCODER_ERR_TIMEOUT;
        }
    }
    
    // 解析数据
    uint8_t *rx = Encoder_Serial_GetRxBuffer();
    if (rx[0] != encoder_id) {
        return ENCODER_ERR_NO_RESPONSE;
    }
    
    uint16_t raw = (rx[4] << 8) | rx[3];
    data->raw_value = raw;
    data->position = (float)raw * 360.0f / 4096.0f;
    data->encoder_id = encoder_id;
    data->is_valid = 1;
    data->last_update_tick = HAL_GetTick();
    
    return ENCODER_OK;
}
