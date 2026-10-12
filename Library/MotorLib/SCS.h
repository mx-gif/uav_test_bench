/*
 * SCS.h
 * 飞特SCS系列串行舵机协议头文件
 * 日期: 2025.6.30
 * 作者: txl
 */

#ifndef _SCS_H
#define _SCS_H

#include <stdint.h>

/**
 * 错误码枚举定义
 */
enum SCS_ERR_LIST
{
  SCS_ERR_NO_REPLY = 1,  // 无应答错误
  SCS_ERR_CRC_CMP  = 2,  // 校验和错误
  SCS_ERR_SLAVE_ID = 3,  // 从机ID错误
  SCS_ERR_BUFF_LEN = 4,  // 缓冲区长度错误
};

/**
 * 基础通信函数声明
 */

// 指令写入相关函数
extern int genWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);      // 普通写指令（立即执行）
extern int regWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);      // 异步写指令（寄存器写入）
extern int regAction(uint8_t ID);                                                    // 异步写执行指令
extern void syncWrite(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen); // 同步写指令（批量控制）
extern int writeByte(uint8_t ID, uint8_t MemAddr, uint8_t bDat);                     // 写入单字节数据
extern int writeWord(uint8_t ID, uint8_t MemAddr, uint16_t wDat);                    // 写入双字节数据

// 数据读取相关函数
extern int Read(uint8_t ID, uint8_t MemAddr, uint8_t *nData, uint8_t nLen);         // 读取指令
extern int readByte(uint8_t ID, uint8_t MemAddr);                                   // 读取单字节数据
extern int readWord(uint8_t ID, uint8_t MemAddr);                                   // 读取双字节数据

// 系统控制相关函数
extern int Ping(uint8_t ID);                                                        // Ping指令（检测在线状态）
extern int Reset(uint8_t ID);                                                       // Reset指令（重置舵机）

// 同步读相关函数
extern int syncReadPacketTx(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t nLen); // 发送同步读请求包
extern int syncReadPacketRx(uint8_t ID, uint8_t *nDat);                               // 解析同步读接收包数据
extern int syncReadRxPacketToByte(void);                                             // 从同步读包中读取一个字节
extern int syncReadRxPacketToWrod(uint8_t negBit);                                   // 从同步读包中读取一个字（支持负数）
extern void syncReadBegin(uint8_t IDN, uint8_t rxLen);                               // 同步读开始（分配缓冲区）
extern void syncReadEnd(void);                                                       // 同步读结束（释放缓冲区）

/**
 * 内部辅助函数声明（通常不需要直接调用）
 */
extern void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun); // 底层数据包构建函数
extern void Host2SCS(uint8_t *DataL, uint8_t* DataH, int Data);                              // 主机字节序转舵机字节序
extern int SCS2Host(uint8_t DataL, uint8_t DataH);                                           // 舵机字节序转主机字节序
extern int Ack(uint8_t ID);                                                                  // 指令应答处理
extern int checkHead(void);                                                                  // 检查数据包头

/**
 * 系统配置函数声明
 */
extern void setEnd(uint8_t _End);           // 设置处理器字节序（大小端模式）
extern uint8_t getEnd(void);                // 获取当前字节序设置
extern void setLevel(uint8_t _Level);       // 设置舵机返回等级
extern int getState(void);                  // 获取最近一次通信的舵机状态
extern int getLastError(void);              // 获取最近一次通信的错误码

/**
 * 硬件抽象层函数声明（需要根据具体平台实现）
 * 这些函数提供了与硬件通信的接口
 */
extern int writeSCS(uint8_t *nDat, int nLen);        // 写入数据到舵机总线
extern int writeByteSCS(unsigned char bDat);         // 写入单个字节到舵机总线
extern int readSCS(uint8_t *nDat, int nLen);         // 从舵机总线读取数据
extern void rFlushSCS(void);                         // 清空接收缓冲区
extern void wFlushSCS(void);                         // 清空发送缓冲区

#endif

