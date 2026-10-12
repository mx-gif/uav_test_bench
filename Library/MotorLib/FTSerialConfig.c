/**
 * @file FTSerialConfig.c
 * @brief FT芯片串口底层驱动
 * @author wangziwei
 */

#include "FTSerialConfig.h"

/* 全局串口句柄指针 */
static UART_HandleTypeDef *ft_uart_handle = NULL;

/**
 * @brief 初始化FT芯片使用的串口
 * @param huart 串口句柄指针，如&huart1、&huart2
 * @retval None
 */
void ftUart_Init(UART_HandleTypeDef *huart)
{
    ft_uart_handle = huart;
    /* 可选：这里可以添加串口初始化参数配置 */
}

/**
 * @brief 向舵机发送数据
 * @param nDat 要发送的数据缓冲区指针
 * @param nLen 要发送的数据长度
 * @retval None
 */
void ftUart_Send(uint8_t *nDat, int nLen)
{
    if (ft_uart_handle == NULL) {
        /* 错误处理：串口未初始化 */
        return;
    }
    HAL_UART_Transmit(ft_uart_handle, nDat, nLen, 10);
}

/**
 * @brief 从舵机读取数据
 * @param nDat 接收数据缓冲区指针
 * @param nLen 期望接收的数据长度
 * @retval int 实际接收到的数据长度，0表示接收失败
 */
int ftUart_Read(uint8_t *nDat, int nLen)
{
    if (ft_uart_handle == NULL) {
        return 0;
    }
    
    if (HAL_OK != HAL_UART_Receive(ft_uart_handle, nDat, nLen, 10)) {
        return 0;
    } else {
        return nLen;
    }
}

/**
 * @brief FT芯片总线延时函数
 * @note 延时约10us（实际为1ms，需要根据系统时钟调整）
 */
void ftBus_Delay(void)
{
    HAL_Delay(1);  // 延时1ms
}

/* ========================== 使用方法 ==========================
1. 在main.c中初始化：
   ftUart_Init(&huart4);  // 使用串口4

2. 发送数据：
   uint8_t data[] = {0x01, 0x02, 0x03};
   ftUart_Send(data, sizeof(data));

3. 接收数据：
   uint8_t buffer[10];
   int received = ftUart_Read(buffer, 10);
============================================================== */

