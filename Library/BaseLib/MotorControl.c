/**
 * @file MotorControl.c
 * @brief 电机控制实现（根据数据包类型分别处理）
 *        主管函数，负责调用和分配舵机控制的相关函数库
 */

#include "MotorControl.h"
#include "ServoController.h"
#include "HMICommandProtocol.h"
#include "SMS_STS.h"
#include "SCS.h"
#include "HMIProtocol.h"


// 当前工程使用SM80BL（飞特自定义串行总线协议）
#define SERVO_MODEL_SM80BL


#ifdef SERVO_MODEL_SM80BL
    #define SERVO_SPEED_SCALE          0.732f
	#define SERVO_SPEED_SCALE_RPM_MAX  70.0f
#else
    #error "请选择舵机型号：定义 SERVO_MODEL_SM80BL"
#endif

#define SERVO_POSITION_MODULO          4096U
#define SERVO_OFFSET_SIGN_BIT          0x0800U
#define SERVO_OFFSET_MAG_MASK          0x07FFU
#define SERVO_ZERO_VERIFY_TOLERANCE    5U

static int16_t Motor_DecodeOffset(uint16_t encoded_offset);
static uint16_t Motor_EncodeOffset(int16_t signed_offset);
static uint16_t Motor_WrapPosition(int32_t value);
static uint8_t Motor_ZeroOneServo(uint8_t servo_id);
static void Motor_ApplyZeroCalibration(void);

// 全局舵机指令缓存（保持最新状态）
static Servo_ControlParams_t g_servo_command[MOTOR_COUNT] = {0}; // 舵机侧指令结构体

/**
 * @brief 电机控制主函数（根据数据包类型处理）
 * @param packet_type: 数据包类型（来自HMI_Command_Parse返回值）
 */
void Motor_Control(uint8_t packet_type)
{
    HMI_Command_t *hmi_command = HMI_Command_GetParams(); // 上位机侧指令结构体
    
    // 根据数据包类型分别转换和执行
    switch (packet_type)
    {
        case PKT_TYPE_CONTROL_1:
            // 数据包1：只更新扭矩参数
			ConvertHMI_TorqueParams(hmi_command, g_servo_command); // 将上位机侧指令翻译为舵机侧指令
            // 扭矩配置立即生效
            Motor_ApplyTorqueConfig(g_servo_command);
            break;
            
        case PKT_TYPE_CONTROL_2:
            // 数据包2：只更新运动参数
            ConvertHMI_MotionParams(hmi_command, g_servo_command); // 将上位机侧指令翻译为舵机侧指令
            // 运动控制立即执行
            Motor_ApplyMotionCommand(g_servo_command);
            break;
            
        case PKT_TYPE_CONFIG:
            // 数据包3：切换舵机模式
            ConvertHMI_ModeParams(hmi_command, g_servo_command); // 将上位机侧指令翻译为舵机侧指令
            // 模式切换立即执行
            Motor_ApplyModeSwitch(g_servo_command);
            break;

        case PKT_TYPE_ZERO:
            // 数据包4：三台舵机当前位置置零
            Motor_ApplyZeroCalibration();
            break;
            
        default:
            // 未知数据包类型，不处理
            break;
    }
}





/**
 * @brief 应用扭矩配置（仅更新扭矩使能和扭矩值）
 */
void Motor_ApplyTorqueConfig(const Servo_ControlParams_t servo_command[MOTOR_COUNT])
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        uint8_t servo_id = servo_id_map[i]; // 对应舵机实际id
        
		// 更新最大扭矩百分比值
		SetTorqueSTS(servo_id, (uint16_t)servo_command[i].torque);
        // 更新扭矩使能
		EnableTorqueSTS(servo_id,  servo_command[i].torque_enable ? 1 : 0);
    }
}

/**
 * @brief 应用运动控制命令（位置和速度）
 */
void Motor_ApplyMotionCommand(const Servo_ControlParams_t servo_command[MOTOR_COUNT])
{
    uint8_t mode = servo_command[0].mode;
    
    if (mode == HMI_MOTORMODE_SINGLE_JOINT)
    {
        // Joint模式
        for (uint8_t i = 0; i < MOTOR_COUNT; i++)
        {
            if (servo_command[i].torque_enable == false)
            {
                continue;
            }

            uint8_t servo_id = servo_id_map[i]; // 对应舵机实际id
            
			// SM80BL位置模式控制
			WritePosEx(servo_id,
					   servo_command[i].position,
					   (uint16_t)servo_command[i].speed,
					   50);
        }
    }
    else if (mode == HMI_MOTORMODE_SINGLE_WHEEL)
    {
        // Wheel模式
        for (uint8_t i = 0; i < MOTOR_COUNT; i++)
        {
            if (servo_command[i].torque_enable == false)
            {
                continue;
            }

            uint8_t servo_id = servo_id_map[i]; // 对应舵机实际id
            
			// SM80BL恒速模式控制
			WriteSpe(servo_id, servo_command[i].speed, 50);

        }
    }
}

/**
 * @brief 应用模式切换
 */
void Motor_ApplyModeSwitch(const Servo_ControlParams_t servo_command[MOTOR_COUNT])
{
    uint8_t mode = servo_command[0].mode; // 舵机想要切换到的模式
    
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        uint8_t servo_id = servo_id_map[i]; // 对应舵机实际id
        
        if (mode == HMI_MOTORMODE_SINGLE_JOINT)
        {
            JointMode(servo_id);  // 舵机切换为Joint模式
			HMImotorpacket.modeMotors = HMI_MOTORMODE_ALL_JOINT; // 将发送数据包HMImotorpacket切换到joint模式
        }
        else if (mode == HMI_MOTORMODE_SINGLE_WHEEL)
        {
            WheelMode(servo_id);  // 舵机切换为Wheel模式
			HMImotorpacket.modeMotors = HMI_MOTORMODE_ALL_WHEEL; // 将发送数据包HMImotorpacket切换到wheel模式
        }
    }
}


/**
 * @brief 将飞特位置偏移寄存器值转换为有符号偏移量
 * @note  BIT11为方向位，BIT0~10为偏移幅值。
 */
static int16_t Motor_DecodeOffset(uint16_t encoded_offset)
{
    int16_t magnitude = (int16_t)(encoded_offset & SERVO_OFFSET_MAG_MASK);

    if ((encoded_offset & SERVO_OFFSET_SIGN_BIT) != 0U)
    {
        return (int16_t)(-magnitude);
    }

    return magnitude;
}

/**
 * @brief 将有符号偏移量编码为飞特位置偏移寄存器格式
 */
static uint16_t Motor_EncodeOffset(int16_t signed_offset)
{
    uint16_t magnitude;

    if (signed_offset < 0)
    {
        magnitude = (uint16_t)(-signed_offset);
        return (uint16_t)(SERVO_OFFSET_SIGN_BIT |
                          (magnitude & SERVO_OFFSET_MAG_MASK));
    }

    return (uint16_t)signed_offset & SERVO_OFFSET_MAG_MASK;
}

/**
 * @brief 将位置值归一化到0~4095
 */
static uint16_t Motor_WrapPosition(int32_t value)
{
    int32_t wrapped = value % (int32_t)SERVO_POSITION_MODULO;

    if (wrapped < 0)
    {
        wrapped += (int32_t)SERVO_POSITION_MODULO;
    }

    return (uint16_t)wrapped;
}

/**
 * @brief 将一台舵机的当前位置设置为新的0位置
 * @note  上位机已保证置零前舵机处于失能状态，因此这里不操作扭矩开关。
 */
static uint8_t Motor_ZeroOneServo(uint8_t servo_id)
{
    int current_position;
    int old_offset_reg;
    int verify_position;
    int16_t old_signed_offset;
    int16_t new_signed_offset;
    uint16_t raw_position;
    uint16_t new_offset_reg;

    /* 1. 读取当前校正后的位置。 */
    current_position = readWord(servo_id, SMS_STS_PRESENT_POSITION_L);
    if (current_position < 0)
    {
        return 0U;
    }

    /* 2. 读取当前EEPROM中的位置偏移。 */
    old_offset_reg = readWord(servo_id, SMS_STS_OFS_L);
    if (old_offset_reg < 0)
    {
        return 0U;
    }

    /*
     * 当前显示位置 = 原始位置 - 有符号偏移 (mod 4096)。
     * 因此原始位置 = 当前显示位置 + 旧偏移 (mod 4096)。
     * 要让当前位置显示为0，新偏移应等价于该原始位置。
     */
    old_signed_offset = Motor_DecodeOffset((uint16_t)old_offset_reg);
    raw_position = Motor_WrapPosition((int32_t)current_position +
                                      (int32_t)old_signed_offset);

    if (raw_position < 2048U)
    {
        new_signed_offset = (int16_t)raw_position;
    }
    else if (raw_position > 2048U)
    {
        new_signed_offset = (int16_t)((int32_t)raw_position -
                                      (int32_t)SERVO_POSITION_MODULO);
    }
    else
    {
        /* 地址31的幅值只有11位，±2048无法精确表示；取2047后误差仅1步。 */
        new_signed_offset = 2047;
    }

    new_offset_reg = Motor_EncodeOffset(new_signed_offset);

    /* 3. 解锁EEPROM并写入新的位置偏移。 */
    if (unLockEpromEx(servo_id) == 0)
    {
        return 0U;
    }

    if (CalibrationOfs(servo_id, new_offset_reg) == 0)
    {
        (void)LockEpromEx(servo_id);
        return 0U;
    }

    /* 4. 无论后续验证结果如何，都重新锁定EEPROM。 */
    if (LockEpromEx(servo_id) == 0)
    {
        return 0U;
    }

    /* 5. 重新读取当前位置，确认已经位于0附近。 */
    verify_position = readWord(servo_id, SMS_STS_PRESENT_POSITION_L);
    if (verify_position < 0)
    {
        return 0U;
    }

    if (((uint16_t)verify_position <= SERVO_ZERO_VERIFY_TOLERANCE) ||
        ((uint16_t)verify_position >=
         (SERVO_POSITION_MODULO - SERVO_ZERO_VERIFY_TOLERANCE)))
    {
        return 1U;
    }

    return 0U;
}

/**
 * @brief 对ID1、ID2、ID3三台舵机执行当前位置置零
 */
static void Motor_ApplyZeroCalibration(void)
{
    uint8_t i;

    for (i = 0U; i < MOTOR_COUNT; i++)
    {
        (void)Motor_ZeroOneServo(servo_id_map[i]);
    }
}
