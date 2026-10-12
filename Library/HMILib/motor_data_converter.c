/**
 * @file motor_data_converter.c
 * @brief 舵机数据发送到HMI数据的量程转换
 */

#include "motor_data_converter.h"
#include <string.h>
#include <stdlib.h>   // C标准库中的 abs()
#include <math.h>

/* ==================== 配置区：根据实际参数范围修改 ==================== */

// 当前工程使用SM80BL（飞特自定义串行总线协议）
#define SERVO_MODEL_SM80BL

// 舵机侧参数缩放因子
#define SERVO_POSITION_SCALE    (4095.0/360.0)  // 舵机位置缩放因子（4095对应360° / 1步=0.0879°）

/*
 * 上位机关节模式速度协议：
 *
 * 上位机显示速度RPM = 数据包原始值 × 0.1113
 *
 * 因此STM32上传值：
 *
 * 数据包原始值
 * = 舵机反馈原始值 × 舵机每步RPM ÷ 0.1113
 */

/* 上位机速度字段低10位有效，范围0～1023 */
#define HMI_SPEED_RAW_MAX           1023U

#if defined(SERVO_MODEL_SM80BL)

    /*
     * SM80BL：
     * SM系列出厂默认速度单位为0.732 RPM/步。
     * 24V C002空载约70RPM，因此有效原始速度约0～96。
     *
     * HMI原始值
     * = SM80BL原始值 × 0.732 ÷ 0.1113
     * = SM80BL原始值 × 7320 ÷ 1113
     */
    #define SERVO_SPEED_RAW_MAX       96U
    #define SERVO_TO_HMI_SPEED_NUM    7320UL
    #define SERVO_TO_HMI_SPEED_DEN    1113UL

#else

    #error "请选择舵机型号：定义 SERVO_MODEL_SM80BL"

#endif

#define SERVO_TORQUE_SCALE      0.1             // 舵机扭矩缩放因子（0~1000 / 1步=0.1%）
#define SERVO_VOLTAGE_SCALE     10              // 舵机电压缩放因子（voltage=120表示12.0V / 1步=0.1V）
#define SERVO_TEMP_OFFSET       0               // 舵机温度偏移（通常为0）

// HMI侧参数范围
#define HMI_POSITION_MIN        0                           // HMI位置最小值
#define HMI_POSITION_MAX        (1023.0/300.0*360.0)        // HMI位置最大值（1023对应300° → 1227.6°）
//速度直接传递舵机实际速度即可（单位：rpm）
#define HMI_TORQUE_MIN          0                           // HMI扭矩最小值
#define HMI_TORQUE_MAX          1023                        // HMI扭矩最大值
#define HMI_VOLTAGE_SCALE       10                          // HMI电压缩放因子（实际值×10）

/* ==================== 辅助宏定义 ==================== */

// 线性映射宏：将x从[in_min, in_max]映射到[out_min, out_max]
// 注意：为避免浮点数计算，必须先乘后除
#define MAP(x, in_min, in_max, out_min, out_max) \
    (((int32_t)(x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))

/* ==================== 核心转换函数 ==================== */

/**
 * @brief 将舵机速度寄存器值转换成上位机速度协议值
 *
 * SM80BL：
 * 实际速度RPM = 舵机寄存器值 × 0.732
 *
 * 上位机：
 * 显示速度RPM = 数据包值 × 0.1113
 *
 * 所以：
 * 数据包值 = 舵机寄存器值 × 0.732 ÷ 0.1113
 */
static uint16_t ConvertServoSpeedToHMIRaw(int32_t servo_speed_raw)
{
    uint32_t speed_abs;
    uint32_t hmi_speed_raw;

    /* 当前上位机只显示速度大小，因此先取绝对值 */
    if (servo_speed_raw < 0)
    {
        speed_abs = (uint32_t)(-servo_speed_raw);
    }
    else
    {
        speed_abs = (uint32_t)servo_speed_raw;
    }

    /* 限制在当前舵机的有效速度范围内 */
    if (speed_abs > SERVO_SPEED_RAW_MAX)
    {
        speed_abs = SERVO_SPEED_RAW_MAX;
    }

    /*
     * SM80BL：
     * hmi_raw = speed_abs × 7320 ÷ 1113
     *
     * 加上分母的一半，用于实现四舍五入。
     */
    hmi_speed_raw =
        (speed_abs * SERVO_TO_HMI_SPEED_NUM
         + SERVO_TO_HMI_SPEED_DEN / 2U)
        / SERVO_TO_HMI_SPEED_DEN;

    /* 上位机速度协议范围为0～1023 */
    if (hmi_speed_raw > HMI_SPEED_RAW_MAX)
    {
        hmi_speed_raw = HMI_SPEED_RAW_MAX;
    }

    return (uint16_t)hmi_speed_raw;
}

/**
 * @brief 将单个舵机状态转换为HMI电机数据
 * @param motorstate: 源舵机状态指针
 * @param HMImotordata: 目标HMI电机数据指针
 * @param error: 错误状态（需要根据实际情况设置，0表示无错误）
 * @retval 0:成功  -1:参数错误
 * 
 * @note 位置、扭矩映射计算均先乘后除，避免浮点数运算
 * @note 速度直接传递舵机寄存器值，由上位机乘以SERVO_SPEED_SCALE显示
 */
int8_t ConvertMotorStateToHMI(const MotorState_typedef *motorstate, 
                               HMI_MotorData_t *HMImotordata, 
                               uint8_t error)
{
    if(motorstate == NULL || HMImotordata == NULL)
        return -1;
    
    int32_t temp_value;
    
    // ========== 1. 错误状态 ==========
    HMImotordata->error = error;
    
    // ========== 2. 位置转换 ==========
    // 舵机: position值（0-4095，对应0-360°）
    // HMI:  0 ~ 1227.6（1023对应300°，4095对应1227.6°）
    // 计算: position * 12276 / 40950（先乘后除）
    temp_value = ((int32_t)motorstate->position * 12276) / 40950;
    HMImotordata->position = (uint16_t)(temp_value);
    
    // ========== 3. 速度转换 ==========
    // 舵机: SM80BL速度寄存器原始值，默认1步=0.732RPM
    // HMI:  按既有上位机速度协议换算后发送
    // 直接复制，不做映射转换
    HMImotordata->velocity =
    ConvertServoSpeedToHMIRaw(
        (int32_t)motorstate->speed
    );
    
    // ========== 4. 扭矩转换 ==========
    // 舵机: torque值（0 ~ 1000，对应0% ~ 100%）
    // HMI:  0 ~ 1023
    // 计算: |torque| * 1023 / 1000（先乘后除）
    temp_value = ((int32_t)abs(motorstate->torque) * 1023) / 1000;
    HMImotordata->torque = (uint16_t)(temp_value);
    
    // ========== 5. 电压转换 ==========
    // 舵机和HMI都是实际电压×10，直接复制
    HMImotordata->voltage = (uint8_t)(motorstate->voltage);
    
    // ========== 6. 温度转换 ==========
    // 温度单位都是摄氏度，直接复制
    HMImotordata->temperature = (uint8_t)(motorstate->temperature + SERVO_TEMP_OFFSET);
    
    return 0;
}

/**
 * @brief 转换所有舵机数据到HMI数据包
 * @param motorstate: 舵机状态结构体数组（连续存储的3个舵机数据）
 * @param packet: HMI数据包指针
 * @param modeMotors: 电机模式字节
 * @retval 0:成功  -1:参数错误
 */
int8_t ConvertAllMotorsToHMI(const MotorState_typedef *motorstate,
                              HMI_MotorPacket_t *packet, 
                              uint8_t modeMotors)
{
    if(motorstate == NULL || packet == NULL)
        return -1;
    
    // 转换3个电机数据（错误状态默认为0）
    ConvertMotorStateToHMI(&motorstate[0], &packet->motor1, 0);
    ConvertMotorStateToHMI(&motorstate[1], &packet->motor2, 0);
    ConvertMotorStateToHMI(&motorstate[2], &packet->motor3, 0);
    
    return 0;
}

/**
 * @brief 批量转换并添加错误检测
 * @param motorstate: 舵机状态数组（3个电机）
 * @param packet: HMI数据包指针
 * @param modeMotors: 电机模式字节
 * @param error_flags: 各电机的错误标志数组（可选，传NULL则所有error为0）
 * @retval 0:成功  -1:参数错误
 */
int8_t ConvertAllMotorsToHMI_WithError(const MotorState_typedef *motorstate, 
                                        HMI_MotorPacket_t *packet, 
                                        uint8_t modeMotors,
                                        const uint8_t *error_flags)
{
    if(motorstate == NULL || packet == NULL)
        return -1;
    
    uint8_t errors[3] = {0, 0, 0};
    
    if(error_flags != NULL)
    {
        memcpy(errors, error_flags, 3);
    }
    
    ConvertMotorStateToHMI(&motorstate[0], &packet->motor1, errors[0]);
    ConvertMotorStateToHMI(&motorstate[1], &packet->motor2, errors[1]);
    ConvertMotorStateToHMI(&motorstate[2], &packet->motor3, errors[2]);
    
    return 0;
}
