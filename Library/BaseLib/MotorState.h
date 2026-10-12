#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

#include <stdint.h>

// 定义最大舵机数量
#define MAX_SERVOS 3

// SMS_STS舵机状态寄存器地址
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

// 舵机状态结构体
typedef struct {
    int16_t position;    // 位置
    int16_t speed;       // 速度
    int16_t torque;      // 负载/扭矩
    uint8_t voltage;     // 电压
    uint8_t temperature; // 温度
} MotorState_typedef;





// 声明全局变量
extern MotorState_typedef motorstate[MAX_SERVOS];
extern uint8_t servo_id_map[MAX_SERVOS];

// 函数声明
int MotorFeedBack(int servo_id, int servo_index);

#endif
