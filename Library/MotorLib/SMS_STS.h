/*
 * SMS_STS.h
 * 飞特SMS/STS系列串行舵机应用层程序
 * 日期: 2025.10.13
 * 作者: 
 */

#ifndef _SMS_STS_H
#define _SMS_STS_H

#include <stdint.h>

//内存表定义
//-------EPROM(只读)--------
#define SMS_STS_MODEL_L 3
#define SMS_STS_MODEL_H 4

//-------EPROM(读写)--------
#define SMS_STS_ID 5
#define SMS_STS_BAUD_RATE 6
#define SMS_STS_MIN_ANGLE_LIMIT_L 9
#define SMS_STS_MIN_ANGLE_LIMIT_H 10
#define SMS_STS_MAX_ANGLE_LIMIT_L 11
#define SMS_STS_MAX_ANGLE_LIMIT_H 12
#define SMS_STS_CW_DEAD 26
#define SMS_STS_CCW_DEAD 27
#define SMS_STS_OFS_L 31
#define SMS_STS_OFS_H 32
#define SMS_STS_MODE 33

//-------SRAM(读写)--------
#define SMS_STS_TORQUE_ENABLE 40
#define SMS_STS_ACC 41
#define SMS_STS_GOAL_POSITION_L 42
#define SMS_STS_GOAL_POSITION_H 43
#define SMS_STS_GOAL_TIME_L 44
#define SMS_STS_GOAL_TIME_H 45
#define SMS_STS_GOAL_SPEED_L 46
#define SMS_STS_GOAL_SPEED_H 47
#define SMS_STS_TORQUE_LIMIT_L 48
#define SMS_STS_TORQUE_LIMIT_H 49
#define SMS_STS_LOCK 55

//-------SRAM(只读)--------
#define SMS_STS_PRESENT_POSITION_L 56
#define SMS_STS_PRESENT_POSITION_H 57
#define SMS_STS_PRESENT_SPEED_L 58
#define SMS_STS_PRESENT_SPEED_H 59
#define SMS_STS_PRESENT_LOAD_L 60
#define SMS_STS_PRESENT_LOAD_H 61
#define SMS_STS_PRESENT_VOLTAGE 62
#define SMS_STS_PRESENT_TEMPERATURE 63
#define SMS_STS_MOVING 66
#define SMS_STS_PRESENT_CURRENT_L 69
#define SMS_STS_PRESENT_CURRENT_H 70



/**
 * 功能：控制单个舵机以指定加速度和速度运动到指定位置（立即执行）
 * 参数：ID-舵机ID，Position-目标位置，Speed-运动速度，ACC-加速度
 * 说明：位置值支持负数表示反向，通过最高位标识方向
 */
extern int WritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC);



/**
 * 功能：设置单个舵机为关节模式（有角度限制）
 * 参数：ID-舵机ID
 * 说明：将舵机从速度控制模式切换到位置模式
 */
extern int JointMode(uint8_t ID);


/**
 * 功能：将单个舵机的扭矩使能/失能
 * 参数：ID-舵机ID
 * 说明：启动该舵机的扭矩/让舵机卸力
 */
extern int EnableTorqueSTS(uint8_t ID, uint8_t ifEnable);

/**
 * 功能：设置单个舵机扭矩限制值（最大100.0%）
 * 参数：ID-舵机ID，torque_limit-最大扭矩的百分比(0-1000)
 * 说明：将舵机从速度控制模式切换到位置模式
 */
int SetTorqueSTS(uint8_t ID, uint16_t torque_limit);

/**
 * 功能：设置单个舵机为轮式模式（连续旋转模式）
 * 参数：ID-舵机ID
 * 说明：将舵机从位置模式切换到速度控制模式
 */
extern int WheelMode(uint8_t ID);

/**
 * 功能：在轮式模式下控制舵机以指定加速度和速度连续旋转
 * 参数：ID-舵机ID，Speed-旋转速度，ACC-加速度
 * 说明：仅当舵机处于轮式模式时有效，速度值决定方向和转速
 */
extern int WriteSpe(uint8_t ID, int16_t Speed, uint8_t ACC);//恒速模式控制指令



/**[校准函数]
 * 功能：校准舵机偏移量
 * 参数：ID-舵机ID
 * 说明：将当前位置设置为想要的角度，上电后调用，需要先获取舵机实际位置
 */
int CalibrationOfs(uint8_t ID, uint16_t offset);


/** 辅助函数
  * @brief  计算将当前实际位置显示为指定角度所需的偏移值
  * @param  current_value: 当前舵机实际位置值 (0-4095)
  * @param  target_angle: 想要显示的角度值 (0-359.99度)
  * @retval 偏移寄存器的值 (0-4095)
  * @note   根据公式：显示值 = (实际值 - 偏移) mod 4096
  *         推导：偏移 = 实际值 - 期望显示值 (mod 4096)
  */
uint16_t Calculate_Offset_Angle(uint16_t current_value, uint16_t target_angle);



/**
 * 功能：设置舵机角度值为0-4095
 * 参数：ID-舵机ID
 * 说明：上电后调用，设置舵机角度最大为4095，使用多圈编码器会出现不正常控制情况
 */
int SetMaxAngleLimit(uint8_t ID);





///////////////////////////////////////////////////////////////////////////////////////// [不使用]
extern int RegWritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC);//异步写位置指令

/**[核心函数SyncWritePosEx]
 * 功能：同步控制多个舵机以各自的加速度和速度运动到各自的位置
 * 参数：ID[]-舵机ID数组，IDN-舵机数量，Position[]-位置数组，Speed[]-速度数组，ACC[]-加速度数组
 * 说明：最多支持32个舵机同步运动，速度/加速度数组为NULL时使用默认值0
 */
extern void SyncWritePosEx(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[]);





extern int unLockEpromEx(uint8_t ID); //锁定舵机EEPROM，保护配置参数
extern int LockEpromEx(uint8_t ID);	  //解锁舵机EEPROM，允许修改配置参数
#endif
