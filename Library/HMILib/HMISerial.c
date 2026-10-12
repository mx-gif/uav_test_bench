/**
 * @file HMISerial.c
 * @brief FFT Gyroboard上位机通信底层串口实现
 * @author wangziwei
 * @date 2025-12-18
 */
#include "HMISerial.h"
#include <string.h>


#define HMI_UART_RX_BUF_SIZE 32 // DMA接收缓冲区长度

/* 全局上位机串口句柄指针 */
static UART_HandleTypeDef *hmi_uart_handle = NULL;


/* 串口接收关键变量 */
static uint8_t hmi_uart_rx_buffer[HMI_UART_RX_BUF_SIZE]; // DMA接收缓冲区
static volatile uint16_t hmi_uart_rx_length = 0;         // 接收数据长度
static volatile uint8_t hmi_uart_rx_flag = 0;            // 接收完成标志



/**
 * @brief 初始化上位机串口
 * @param huart 串口句柄指针(如&huart1)
 * @retval 无
 */
void HMI_Serial_Init(UART_HandleTypeDef *huart)
{
    hmi_uart_handle = huart;
    
    // 清空缓冲区，接收数据长度、接收完成标志初始化
    memset(hmi_uart_rx_buffer, 0, sizeof(hmi_uart_rx_buffer));
    hmi_uart_rx_length = 0;
    hmi_uart_rx_flag = 0;
    
    // 启动DMA接收(空闲中断模式)
    HAL_UARTEx_ReceiveToIdle_DMA(hmi_uart_handle, 
                                  hmi_uart_rx_buffer, 
                                  HMI_UART_RX_BUF_SIZE);
    
    // 禁用DMA半传输中断
    __HAL_DMA_DISABLE_IT(hmi_uart_handle->hdmarx, DMA_IT_HT);
}

/* ==================== 串口发送 ==================== */

/**
 * @brief 向上位机发送数据包（异步发送）
 * @param pData 要发送的数据缓冲区指针
 * @param Size 要发送的数据长度
 * @retval HAL状态：HAL_OK=成功，HAL_ERROR=失败
 */
HAL_StatusTypeDef HMI_Serial_Send(uint8_t *pData, uint16_t Size)
{
    if (hmi_uart_handle == NULL) {
        return HAL_ERROR;  // 出错：串口未初始化
    }
    
    // 使用DMA异步发送，不阻塞主程序
    return HAL_UART_Transmit_DMA(hmi_uart_handle, pData, Size);
}

/**
 * @brief 检查串口是否发送完成
 * @retval 1=发送完成，0=正在发送
 */
uint8_t HMI_Serial_IsTxComplete(void)
{
    if (hmi_uart_handle == NULL) {
        return 1;
    }
    
    // 检查UART状态，如果状态是 READY，且TX不忙，表示可以开始新的传输
	// 使用HAL库提供的判断宏
    return (hmi_uart_handle->gState == HAL_UART_STATE_READY);
}

/* ==================== 串口接收 ==================== */

/****************接口函数*****************/
/**
 * @brief 获取接收完成标志
 * @retval 1=有新数据,0=无数据
 */
uint8_t HMI_Serial_GetRxFlag(void)
{
    return hmi_uart_rx_flag;
}

/**
 * @brief 获取接收缓冲区指针
 * @retval 接收缓冲区 的首地址
 */
uint8_t* HMI_Serial_GetRxBuffer(void)
{
    return hmi_uart_rx_buffer;
}

/**
 * @brief 获取接收数据长度
 * @retval 接收数据长度(字节)
 */
uint16_t HMI_Serial_GetRxLength(void)
{
    return hmi_uart_rx_length;
}

/****************重启DMA接收*****************/
/**
 * @brief 清除接收标志并重启DMA接收
 * @retval 无
 * @note 处理完数据后必须调用此函数重启DMA
 */
void HMI_Serial_RestartReceive(void)
{
    hmi_uart_rx_flag = 0U;
    hmi_uart_rx_length = 0U;

    memset(hmi_uart_rx_buffer, 0,
           sizeof(hmi_uart_rx_buffer));

    HAL_UARTEx_ReceiveToIdle_DMA(hmi_uart_handle,
                                 hmi_uart_rx_buffer,
                                 HMI_UART_RX_BUF_SIZE);

    __HAL_DMA_DISABLE_IT(hmi_uart_handle->hdmarx,
                         DMA_IT_HT);
}



/* ==================== 中断回调 ==================== */

/**
 * @brief DMA接收完成回调函数
 * @param huart UART句柄
 * @param Size 实际接收的字节数
 * @note 此函数由HAL_UARTEx_RxEventCallback()调用
 */
void HMI_Serial_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == hmi_uart_handle) {
        hmi_uart_rx_length = Size;  // 保存接收长度
        hmi_uart_rx_flag = 1;       // 设置接收完成标志
    }
}


