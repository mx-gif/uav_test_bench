/**
 * @file ServoController.h
 * @brief 飞特舵机与上位机协议适配层
 * @description 定义上位机指令与舵机参数之间的映射关系和转换函数
 * @author wangziwei
 */

#ifndef __SERVO_CONTROLLER_H
#define __SERVO_CONTROLLER_H

#include "stm32f1xx_hal.h"
#include "HMICommandProtocol.h"
#include <stdint.h>
#include <stdbool.h>

// 注意：如果整型是先乘后除，就无需化为浮点数，f1系列不包含浮点数硬件加速，能不使用就不使用

/* ==================== 上位机侧参数范围定义 ==================== */
#define HMI_TORQUE_MAX_MIN          0       // 最大扭矩最小值
#define HMI_TORQUE_MAX_MAX          1023    // 最大扭矩最大值（对应100%）
#define HMI_TORQUE_MAX_PERCENT      100.0f  // 满量程对应百分比

#define HMI_TORQUE_LIMIT_MIN        0       // 扭矩限制最小值
#define HMI_TORQUE_LIMIT_MAX        1023    // 扭矩限制最大值（对应100%）
#define HMI_TORQUE_LIMIT_PERCENT    100.0f  // 满量程对应百分比

#define HMI_POSITION_MIN            0       // 目标位置最小值
#define HMI_POSITION_MAX            1023    // 目标位置最大值
#define HMI_POSITION_DEGREE_MAX     300.0f  // 满量程对应角度（度）

#define HMI_SPEED_JOINT_MIN         0       // Joint模式速度最小值
#define HMI_SPEED_JOINT_MAX         1023    // Joint模式速度最大值
#define HMI_SPEED_JOINT_RPM_MAX     114.0f  // 满量程对应转速（RPM）

#define HMI_SPEED_WHEEL_MIN         0       // Wheel模式速度最小值（CCW）
#define HMI_SPEED_WHEEL_STOP        1024    // Wheel模式停止值:0,1024
#define HMI_SPEED_WHEEL_MAX         2047    // Wheel模式速度最大值（CW）

/* ==================== 飞特舵机侧参数范围定义 ==================== */

// 舵机型号选择：当前工程使用SM80BL（飞特自定义串行总线协议）
#define SERVO_MODEL_SM80BL



#define SERVO_POSITION_MIN             0       // 舵机位置最小值
#define SERVO_POSITION_MAX             4095    // 舵机位置最大值
#define SERVO_POSITION_DEGREE_MAX      360.0f  // 满量程对应角度（度） // 舵机位置缩放因子（4095对应360° / 1步=0.0879°）

#ifdef SERVO_MODEL_SM80BL
    /* SM系列出厂默认速度单位：1步 = 0.732 RPM。 */
    #define SERVO_SPEED_SCALE          0.732f
    /* SM80BL-C002 @24V空载约70RPM；C001 @12V约62RPM。 */
	#define SERVO_SPEED_SCALE_RPM_MAX  70.0f
#else
    #error "请选择舵机型号：定义 SERVO_MODEL_SM80BL"
#endif

#define SERVO_TORQUE_MIN               0       // 舵机扭矩最小值
#define SERVO_TORQUE_MAX               1000    // 舵机扭矩最大值
#define SERVO_TORQUE_SCALE             0.1f   // 舵机扭矩缩放因子（0~1000 / 1步=0.1%）

/* ==================== 电机模式定义 ==================== */
#define HMI_MOTORMODE_SINGLE_JOINT  0x32
#define HMI_MOTORMODE_SINGLE_WHEEL  0x31

#define MOTOR_COUNT                 3       // 电机数量

/* ==================== 数据结构定义 ==================== */

/**
 * @brief 上位机控制结构体（位于HMICommandProtocol.h中）
 */
//typedef struct {
//    uint8_t  torque_enable[MOTOR_COUNT];    // 扭矩使能：0=关闭, 1=使能
//    uint16_t torque_limit[MOTOR_COUNT];     // 扭矩限制：0-1023 (0-100%)
//    uint16_t torque_max[MOTOR_COUNT];       // 最大扭矩：0-1023 (0-100%)
//    uint16_t moving_speed[MOTOR_COUNT];     // 目标速度：Joint(0-1023) Wheel(0-2047)
//    uint16_t position[MOTOR_COUNT];         // 目标位置：0-1023 (0-300°)
//    uint8_t  motor_mode;                    // 电机模式：JOINT/WHEEL
//} HMI_Command_t;

/**
 * @brief 舵机控制参数结构体（转换后的舵机指令）
 */
typedef struct {
    bool     torque_enable;         // 扭矩使能
    uint16_t torque;                // 实际扭矩值：0-1000 (0.1%/step)
    uint16_t position;              // 目标位置：0-4095
    int16_t  speed;                 // 目标速度：Joint(0-2730) Wheel(-32766~32766)
    uint8_t  mode;                  // 电机模式
} Servo_ControlParams_t;

/* ==================== 转换函数声明 ==================== */

/**
 * @brief 转换扭矩配置参数（数据包1）
 */
int8_t ConvertHMI_TorqueParams(const HMI_Command_t *hmi_command,
                                Servo_ControlParams_t servo_command[MOTOR_COUNT]);

/**
 * @brief 转换运动控制参数（数据包2）
 */
int8_t ConvertHMI_MotionParams(const HMI_Command_t *hmi_command,
                                Servo_ControlParams_t servo_command[MOTOR_COUNT]);

/**
 * @brief 转换模式配置参数（数据包3）
 */
int8_t ConvertHMI_ModeParams(const HMI_Command_t *hmi_command,
                              Servo_ControlParams_t servo_command[MOTOR_COUNT]);



///**
// * @brief 批量转换所有电机的HMI参数为舵机参数（一次性完成）
// * @param hmi_command: 上位机命令指针
// * @param servo_command: 输出舵机命令数组指针（MOTOR_COUNT个元素）
// * @retval 0:成功  -1:参数错误
// */
//int8_t ConvertHMI_ToServo_Batch(const HMI_Command_t *hmi_command,
//                                Servo_ControlParams_t servo_command[MOTOR_COUNT]);

/* ==================== 辅助宏定义 ==================== */

// 限幅宏
#define CLAMP(x, min, max) \
    ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

// 线性映射宏（整数版本）
#define MAP_INT(x, in_min, in_max, out_min, out_max) \
    (((int32_t)(x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))

// 线性映射宏（浮点版本，更精确）
#define MAP_FLOAT(x, in_min, in_max, out_min, out_max) \
    ((float)((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))

#endif /* __FEETECH_SERVO_ADAPTER_H */
