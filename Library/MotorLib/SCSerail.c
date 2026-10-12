/*
 * SCServo.c
 * 串口舵机底层通信驱动
 * 创建日期: 2024.12.2
 * 作者: txl
 */
#include <stdint.h>

/* 发送数据缓冲区 */
uint8_t wBuf[128];
uint8_t wLen = 0;

/* 外部函数声明 */
void ftUart_Send(uint8_t *nDat, int nLen);
int ftUart_Read(uint8_t *nDat, int nLen);
void ftBus_Delay(void);

/**
 * @brief 从串口读取舵机数据
 * @param nDat 数据接收缓冲区指针
 * @param nLen 期望读取的数据长度
 * @return int 实际读取到的数据长度
 */
int readSCS(unsigned char *nDat, int nLen)
{
    return ftUart_Read(nDat, nLen);
}

/**
 * @brief 写入数据到发送缓冲区（数据暂存，需要调用wFlushSCS发送）
 * @param nDat 要发送的数据缓冲区指针
 * @param nLen 要发送的数据长度
 * @return int 当前缓冲区中的数据长度
 */
int writeSCS(unsigned char *nDat, int nLen)
{
    while(nLen--){
        if(wLen < sizeof(wBuf)){
            wBuf[wLen] = *nDat;
            wLen++;
            nDat++;
        }
    }
    return wLen;
}

/**
 * @brief 写入单个字节到发送缓冲区
 * @param bDat 要发送的字节数据
 * @return int 当前缓冲区中的数据长度
 */
int writeByteSCS(unsigned char bDat)
{
    if(wLen < sizeof(wBuf)){
        wBuf[wLen] = bDat;
        wLen++;
    }
    return wLen;
}

/**
 * @brief 清空接收缓冲区（通过延时等待数据接收完成）
 * @note 该函数通过延时操作，确保所有待接收数据都已到达
 */
void rFlushSCS()
{
    ftBus_Delay();  // 延时等待数据接收完成
}

/**
 * @brief 刷新并发送缓冲区中的数据
 * @note 将缓冲区中暂存的数据通过串口发送出去，并清空缓冲区
 */
void wFlushSCS()
{
    if(wLen){
        ftUart_Send(wBuf, wLen);  // 发送缓冲区中的数据
        wLen = 0;  // 清空缓冲区
    }
}


