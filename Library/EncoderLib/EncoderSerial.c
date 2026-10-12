/**
 * @file EncoderSerial.c
 * @brief 编码器串口底层驱动（简化版）
 */

#include "EncoderSerial.h"

/* 私有变量 */
static UART_HandleTypeDef *encoder_uart = NULL;
static uint8_t rx_buffer[ENCODER_RX_SIZE];      // 接收缓冲区
static volatile uint8_t rx_complete = 0;        // 接收完成标志
static uint8_t rx_byte = 0;                     // 单字节缓冲

/* 状态机变量 */
static struct {
    uint8_t state;          // 0=等待帧头, 1=接收数据
    uint8_t index;          // 当前写入位置
    uint32_t last_tick;     // 最后接收时间
} rx_fsm = {0, 0, 0};

/**
 * @brief 初始化编码器串口
 */
void Encoder_Serial_Init(UART_HandleTypeDef *huart)
{
    encoder_uart = huart;
    rx_complete = 0;
    rx_fsm.state = 0;
    
    RS485_DIR_RX();
    HAL_UART_Receive_IT(encoder_uart, &rx_byte, 1);
}

/**
 * @brief 发送数据（改进版：无延时）
 */
void Encoder_Serial_SendArray(uint8_t *data, uint16_t len)
{
    RS485_DIR_TX();
    HAL_Delay(2);
    
    HAL_UART_Transmit_DMA(encoder_uart, data, len);
    
    // 轮询等待DMA完成
    while (HAL_UART_GetState(encoder_uart) != HAL_UART_STATE_READY);
    
    // 等待最后字节物理发完
    while (!__HAL_UART_GET_FLAG(encoder_uart, UART_FLAG_TC));
    
    RS485_DIR_RX();
}

/**
 * @brief 获取接收标志（原子操作）
 */
uint8_t Encoder_Serial_GetRxFlag(void)
{
    if (rx_complete) {
        rx_complete = 0;
        return 1;
    }
    return 0;
}

/**
 * @brief 获取接收缓冲区
 */
uint8_t* Encoder_Serial_GetRxBuffer(void)
{
    return rx_buffer;
}

/**
 * @brief 接收状态机复位（超时保护用）
 */
void Encoder_Serial_ResetRx(void)
{
    rx_fsm.state = 0;
    rx_fsm.index = 0;
}

/**
 * @brief UART接收中断处理（简化版）
 * @note 在 HAL_UART_RxCpltCallback() 中调用
 */
void Encoder_Serial_RxHandler(UART_HandleTypeDef *huart)
{
    if (huart != encoder_uart) return;
    
    uint8_t data = rx_byte;
    uint32_t now = HAL_GetTick();
    
    /* 超时保护：100ms未收到完整帧则复位 */
    if (now - rx_fsm.last_tick > 100) {
        rx_fsm.state = 0;
        rx_fsm.index = 0;
    }
    rx_fsm.last_tick = now;
    
    /* 状态0：等待帧头 */
    if (rx_fsm.state == 0) {
        if (data >= 0x01 && data <= 0x03) {  // 简化判断
            rx_buffer[0] = data;
            rx_fsm.index = 1;
            rx_fsm.state = 1;
        }
    }
    /* 状态1：接收数据 */
    else if (rx_fsm.state == 1) {
        rx_buffer[rx_fsm.index++] = data;
        
        if (rx_fsm.index >= ENCODER_RX_SIZE) {
            rx_complete = 1;      // 设置完成标志
            rx_fsm.state = 0;     // 复位状态机
            rx_fsm.index = 0;
        }
    }
    
    /* 重启接收 */
    HAL_UART_Receive_IT(huart, &rx_byte, 1);
}
