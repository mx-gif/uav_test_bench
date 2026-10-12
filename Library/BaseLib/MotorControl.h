/**
 * @file MotorControl.h
 * @brief 电机控制实现（根据数据包类型分别处理）
 *        主管函数，负责调用和分配舵机控制的相关函数库
 */

#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "ServoController.h"
#include "MotorState.h"


// 3个舵机，物理ID分别是1、2、3
// uint8_t servo_id_map[MAX_SERVOS] = {1, 2, 3};  来自于"MotorState.h"

/**
 * @brief 电机控制主函数
 * @param packet_type: 数据包类型
 */
void Motor_Control(uint8_t packet_type);

/**
 * @brief 应用扭矩配置
 */
void Motor_ApplyTorqueConfig(const Servo_ControlParams_t servo_command[MOTOR_COUNT]);

/**
 * @brief 应用运动控制命令
 */
void Motor_ApplyMotionCommand(const Servo_ControlParams_t servo_command[MOTOR_COUNT]);

/**
 * @brief 应用模式切换
 */
void Motor_ApplyModeSwitch(const Servo_ControlParams_t servo_command[MOTOR_COUNT]);

#endif /* __MOTOR_CONTROL_H */
