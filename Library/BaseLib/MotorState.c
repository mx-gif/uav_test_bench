/*
 * MotorState.c
 * 串行舵机应用层程序
 */

#include <string.h>
#include "INST.h"
#include "SCS.h"
#include "MotorState.h"


/* ===== 字节序配置 ===== */
// 使用需要的字节序方式
//#define MOTOR_LITTLE_ENDIAN   // 小端序（低字节在前）
#define MOTOR_BIG_ENDIAN   // 大端序（高字节在前）


// 为每个舵机分配独立的缓存，每个缓存15字节
static uint8_t Mem[MAX_SERVOS][SMS_STS_PRESENT_CURRENT_H - SMS_STS_PRESENT_POSITION_L + 1];


// 定义3个舵机对应的motorstate结构体，用于保存参数值
MotorState_typedef motorstate[MAX_SERVOS];

// 定义舵机id表
uint8_t servo_id_map[MAX_SERVOS] = {1, 2, 3};



/* ===== 主函数 ===== */
#ifdef MOTOR_LITTLE_ENDIAN // 使用小端序

// servo_id与servo_index分开，这样可以id=2,5,7但是还是Mem[0],Mem[1],Mem[2]
// 批量读取舵机状态到缓存并解析（小端序）
int MotorFeedBack(int servo_id, int servo_index)
{
    if(servo_index < 0 || servo_index >= MAX_SERVOS) { // 检查Mem[servo_index]索引是否正常
        return -1;
    }
    
    int nLen = Read(servo_id, SMS_STS_PRESENT_POSITION_L, 
                    Mem[servo_index], sizeof(Mem[0]));
    
    if(nLen != sizeof(Mem[0])) {
        return -1;
    }
    
    // 获取当前舵机缓存指针
    uint8_t* current_mem = Mem[servo_index];
    
	
    // 1.解析位置
    int pos = current_mem[SMS_STS_PRESENT_POSITION_L - SMS_STS_PRESENT_POSITION_L];
    pos <<= 8;
    pos |= current_mem[SMS_STS_PRESENT_POSITION_H - SMS_STS_PRESENT_POSITION_L];
    pos = pos & 0x0FFF;  // 只保留 0-4095 低12位
    motorstate[servo_index].position = pos;
    
	
    // 2.解析速度
    int speed = current_mem[SMS_STS_PRESENT_SPEED_L - SMS_STS_PRESENT_POSITION_L];
    speed <<= 8;
    speed |= current_mem[SMS_STS_PRESENT_SPEED_H - SMS_STS_PRESENT_POSITION_L];
    speed = speed & 0x7FFF;  // 只保留低15位，不需要速度的正负信息
    motorstate[servo_index].speed = speed;
    
	
    // 3.解析负载
    int load = current_mem[SMS_STS_PRESENT_LOAD_L - SMS_STS_PRESENT_POSITION_L];
    load <<= 8;
    load |= current_mem[SMS_STS_PRESENT_LOAD_H - SMS_STS_PRESENT_POSITION_L];
    if(load & (1 << 10)) load = -(load & ~(1 << 10)); // 判断正负
    motorstate[servo_index].torque = load;
    
	
    // 4.解析电压
    motorstate[servo_index].voltage = 
        current_mem[SMS_STS_PRESENT_VOLTAGE - SMS_STS_PRESENT_POSITION_L];
    
	
    // 5.解析温度
    motorstate[servo_index].temperature = 
        current_mem[SMS_STS_PRESENT_TEMPERATURE - SMS_STS_PRESENT_POSITION_L];
    
    return nLen;
}




#else  // MOTOR_BIG_ENDIAN // 使用大端序

/**
 * @brief 批量读取舵机状态（大端序版本）
 * @param servo_id 舵机物理ID
 * @param servo_index 舵机数组索引
 * @return 成功返回读取字节数，失败返回-1
 */
int MotorFeedBack(int servo_id, int servo_index)
{
    if(servo_index < 0 || servo_index >= MAX_SERVOS) {
        return -1;
    }
    
    int nLen = Read(servo_id, SMS_STS_PRESENT_POSITION_L, 
                    Mem[servo_index], sizeof(Mem[0]));
    
    if(nLen != sizeof(Mem[0])) {
        return -1;
    }
    
    // 获取当前舵机缓存指针
    uint8_t* current_mem = Mem[servo_index];
    
    
    // 1.解析位置（大端序：先读高字节，再读低字节）
    int pos = current_mem[SMS_STS_PRESENT_POSITION_H - SMS_STS_PRESENT_POSITION_L] << 8;  // 高字节
    pos |= current_mem[SMS_STS_PRESENT_POSITION_L - SMS_STS_PRESENT_POSITION_L];          // 低字节
    pos = pos & 0x0FFF;  // 只保留 0-4095 低12位
    motorstate[servo_index].position = pos;
    
    
    // 2.解析速度（大端序）
    int speed = current_mem[SMS_STS_PRESENT_SPEED_H - SMS_STS_PRESENT_POSITION_L] << 8;   // 高字节
    speed |= current_mem[SMS_STS_PRESENT_SPEED_L - SMS_STS_PRESENT_POSITION_L];           // 低字节
    //if(speed & (1 << 15)) speed = -(speed & ~(1 << 15));
	speed = speed & 0x7FFF;  // 只保留低15位，不需要速度的正负信息
    motorstate[servo_index].speed = speed;
    
    
    // 3.解析负载（大端序）
    int load = current_mem[SMS_STS_PRESENT_LOAD_H - SMS_STS_PRESENT_POSITION_L] << 8;     // 高字节
    load |= current_mem[SMS_STS_PRESENT_LOAD_L - SMS_STS_PRESENT_POSITION_L];             // 低字节
    if(load & (1 << 10)) load = -(load & ~(1 << 10));
    motorstate[servo_index].torque = load;
    
    
    // 4.解析电压
    motorstate[servo_index].voltage = 
        current_mem[SMS_STS_PRESENT_VOLTAGE - SMS_STS_PRESENT_POSITION_L];
    
    
    // 5.解析温度
    motorstate[servo_index].temperature = 
        current_mem[SMS_STS_PRESENT_TEMPERATURE - SMS_STS_PRESENT_POSITION_L];
    
    return nLen;
}

#endif  // MOTOR_LITTLE_ENDIAN


