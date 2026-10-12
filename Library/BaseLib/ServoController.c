/**
 * @file ServoController.c
 * @brief 飞特舵机与上位机协议适配层实现（支持分包转换）
 * @author wangziwei
 */

#include "ServoController.h"
#include <stdlib.h>
#include <math.h>

/* ==================== 工具函数声明 ==================== */
static inline int16_t ConvertHMISpeedToRPM(uint16_t hmi_speed, uint8_t mode);


/* ==================== 分包转换函数 ==================== */

/**
 * @brief 转换扭矩配置参数（数据包1）
 * @param hmi_command: 上位机命令指针
 * @param servo_command: 输出舵机命令数组指针
 * @retval 0:成功  -1:参数错误
 */
int8_t ConvertHMI_TorqueParams(const HMI_Command_t *hmi_command,
                                Servo_ControlParams_t servo_command[MOTOR_COUNT])
{
    if (hmi_command == NULL || servo_command == NULL)
        return -1;
    
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        // ========== 1. 扭矩使能 ==========
        servo_command[i].torque_enable = (hmi_command->torque_enable[i] != 0);
        
        // ========== 2. 扭矩值转换 ==========
		// 上位机侧：0-1023对应0-100.0%
		// 舵机侧：  0-1000对应0-100.0%
        uint16_t torque_max = CLAMP(hmi_command->torque_max[i], 
                                     HMI_TORQUE_MAX_MIN, HMI_TORQUE_MAX_MAX);
        uint16_t torque_limit = CLAMP(hmi_command->torque_limit[i], 
                                       HMI_TORQUE_LIMIT_MIN, HMI_TORQUE_LIMIT_MAX);
        
        uint32_t torque = ((uint32_t)torque_max * torque_limit * SERVO_TORQUE_MAX) 
                          / (HMI_TORQUE_MAX_MAX * HMI_TORQUE_LIMIT_MAX);
        servo_command[i].torque = (uint16_t)CLAMP(torque, SERVO_TORQUE_MIN, SERVO_TORQUE_MAX);
    }
    
    return 0;
}

/**
 * @brief 转换运动控制参数（数据包2）
 * @param hmi_command: 上位机命令指针
 * @param servo_command: 输出舵机命令数组指针
 * @retval 0:成功  -1:参数错误
 */
int8_t ConvertHMI_MotionParams(const HMI_Command_t *hmi_command,
                                Servo_ControlParams_t servo_command[MOTOR_COUNT])
{
    if (hmi_command == NULL || servo_command == NULL)
        return -1;
    
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        // ========== 1. 位置转换 ==========
		// 上位机侧：0-1023对应0-300°
		// 舵机侧：  0-4095对应0-360°
		servo_command[i].mode = hmi_command->motor_mode; // 获取上位机当前模式
		
		if (hmi_command->motor_mode == HMI_MOTORMODE_SINGLE_JOINT)
        {
			uint16_t hmi_pos = CLAMP(hmi_command->position[i], HMI_POSITION_MIN, HMI_POSITION_MAX);
			uint32_t servo_pos = MAP_INT(hmi_pos, 
										 HMI_POSITION_MIN, HMI_POSITION_MAX,
										 SERVO_POSITION_MIN, 
										 (uint16_t)(SERVO_POSITION_MAX * HMI_POSITION_DEGREE_MAX / SERVO_POSITION_DEGREE_MAX)); // 把舵机上限限制到300°对应的值
			servo_command[i].position = (uint16_t)CLAMP(servo_pos, SERVO_POSITION_MIN, SERVO_POSITION_MAX);
        }
			

        // ========== 2. 速度转换（根据模式） ==========
		// 上位机侧：0-1023对应0-114rpm
		// 舵机侧：SM80BL默认速度单位1步=0.732RPM，最高转速按70RPM限制
        servo_command[i].mode = hmi_command->motor_mode; // 获取上位机当前模式
        
        if (hmi_command->motor_mode == HMI_MOTORMODE_SINGLE_JOINT)
        {
            // Joint模式速度转换
            // 步骤1：求出上位机设置的目标速度在真实世界中的对应值（RPM）
            int16_t target_rpm = ConvertHMISpeedToRPM(hmi_command->moving_speed[i], 
                                                      HMI_MOTORMODE_SINGLE_JOINT);
            
            // 步骤2：将上位机的目标速度限制在舵机速度范围内（0-最大正转速）
            target_rpm = CLAMP(target_rpm, 0, (int16_t)SERVO_SPEED_SCALE_RPM_MAX);
            
            // 步骤3：计算舵机步数
            servo_command[i].speed = (int16_t)(target_rpm / SERVO_SPEED_SCALE);
            
            // 确保为正方向（Joint模式通常只有正方向）
            if (servo_command[i].speed < 0)
                servo_command[i].speed = 0;
        }
        else if (hmi_command->motor_mode == HMI_MOTORMODE_SINGLE_WHEEL)
        {
            // Wheel模式速度转换
            // 步骤1：求出上位机设置的目标速度在真实世界中的对应值（RPM，带符号）
            int16_t target_rpm = ConvertHMISpeedToRPM(hmi_command->moving_speed[i], 
                                                      HMI_MOTORMODE_SINGLE_WHEEL);
            
            // 步骤2：将上位机的目标速度限制在舵机速度范围内（保留正负）
            target_rpm = CLAMP(target_rpm, (int16_t)-SERVO_SPEED_SCALE_RPM_MAX, (int16_t)SERVO_SPEED_SCALE_RPM_MAX);
            
            // 步骤3：计算舵机步数
            if (target_rpm == 0)
            {
				// 速度为0
                servo_command[i].speed = 0; // 停止
            }
            else if (target_rpm < 0)
            {
                // 负方向
                float abs_rpm = (float)(-target_rpm); // 取速度幅值（绝对值）
                int16_t steps = (int16_t)(abs_rpm / SERVO_SPEED_SCALE);
                servo_command[i].speed = - steps;
            }
            else
            {
                // 正方向
                float abs_rpm = (float)target_rpm; // 取速度幅值（绝对值）
                int16_t steps = (int16_t)(abs_rpm / SERVO_SPEED_SCALE);
                servo_command[i].speed = steps;
            }
        }
        else
        {
            return -1; // 未知模式
        }
    }
    
    return 0;
}



/**
 * @brief 转换模式配置参数（数据包3）
 * @param hmi_command: 上位机命令指针
 * @param servo_command: 输出舵机命令数组指针
 * @retval 0:成功  -1:参数错误
 */
int8_t ConvertHMI_ModeParams(const HMI_Command_t *hmi_command,
                              Servo_ControlParams_t servo_command[MOTOR_COUNT])
{
    if (hmi_command == NULL || servo_command == NULL)
        return -1;
    
    // 只更新模式，不修改其他参数
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        servo_command[i].mode = hmi_command->motor_mode;
    }
    
    return 0;
}

///**
// * @brief 批量转换所有参数（兼容函数）
// */
//int8_t ConvertHMI_ToServo_Batch(const HMI_Command_t *hmi_command,
//                                Servo_ControlParams_t servo_command[MOTOR_COUNT])
//{
//    if (hmi_command == NULL || servo_command == NULL)
//        return -1;
//    
//    // 转换扭矩参数
//    if (ConvertHMI_TorqueParams(hmi_command, servo_command) != 0)
//        return -1;
//    
//    // 转换运动参数
//    if (ConvertHMI_MotionParams(hmi_command, servo_command) != 0)
//        return -1;
//    
//    // 转换模式参数
//    if (ConvertHMI_ModeParams(hmi_command, servo_command) != 0)
//        return -1;
//    
//    return 0;
//}



/* ==================== 工具函数 ==================== */
/**
 * @brief 将上位机速度值转换为RPM值
 * @param hmi_speed: 上位机速度值
 * @param mode: 电机模式
 * @return RPM值，正负表示方向
 */
static inline int16_t ConvertHMISpeedToRPM(uint16_t hmi_speed, uint8_t mode)
{
    float rpm = 0.0f;
    
    if (mode == HMI_MOTORMODE_SINGLE_JOINT)
    {
        // Joint模式：0-1023 -> 0-114 RPM
        rpm = (float)hmi_speed * HMI_SPEED_JOINT_RPM_MAX / 1023.0f;
    }
    else if (mode == HMI_MOTORMODE_SINGLE_WHEEL)
    {
        // Wheel模式：0-2047 -> -114~114 RPM
        // 其中0,1024为停止
        if ((hmi_speed == 1024) || (hmi_speed == 0))
        {
            rpm = 0.0f;
        }
        else if (hmi_speed < 1024)
        {
            // 正方向：0-1023 -> 0~114 RPM
            rpm = (float)(hmi_speed) * HMI_SPEED_JOINT_RPM_MAX / 1023.0f;
        }
        else
        {
            // 负方向：1025-2047 -> -114~0 RPM // hmi_speed - 1024对应幅值
            rpm = - (float)(hmi_speed - 1024) * HMI_SPEED_JOINT_RPM_MAX / 1023.0f;
        }
    }
    
    return (int16_t)round(rpm); // 使用round函数进行四舍五入
}







///* ==================== 上位机到舵机的转换函数 ==================== */

///**
// * @brief 批量转换所有电机的HMI参数为舵机参数（一次性完成）
// * @param hmi_command: 上位机命令指针
// * @param servo_command: 输出舵机命令数组指针（MOTOR_COUNT个元素）
// * @retval 0:成功  -1:参数错误
// */
//int8_t ConvertHMI_ToServo_Batch(const HMI_Command_t *hmi_command,
//                                Servo_ControlParams_t servo_command[MOTOR_COUNT])
//{
//    if (hmi_command == NULL || servo_command == NULL)
//        return -1;
//    
//    // 循环处理3个电机
//    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
//    {
//        // ========== 1. 扭矩使能 ==========
//        servo_command[i].torque_enable = (hmi_command->torque_enable[i] != 0);
//        
//        // ========== 2. 扭矩值转换 ==========
//		// HMI:  0 ~ 1023 对应 0-100%
//		// 舵机: 0 ~ 1000 对应 0-100.0%
//        // 限幅
//        uint16_t torque_max = CLAMP(hmi_command->torque_max[i], HMI_TORQUE_MAX_MIN, HMI_TORQUE_MAX_MAX);
//        uint16_t torque_limit = CLAMP(hmi_command->torque_limit[i], HMI_TORQUE_LIMIT_MIN, HMI_TORQUE_LIMIT_MAX);
//        
//        // 计算：(torque_max * torque_limit * 1000) / (1023 * 1023) // 扭矩限制和最大扭矩共同计算百分比
//        uint32_t torque = ((uint32_t)torque_max * torque_limit * SERVO_TORQUE_MAX) 
//                          / (HMI_TORQUE_MAX_MAX * HMI_TORQUE_LIMIT_MAX);
//        servo_command[i].torque = (uint16_t)CLAMP(torque, SERVO_TORQUE_MIN, SERVO_TORQUE_MAX);
//        
//        // ========== 3. 位置转换 ==========
//		// HMI:  0 ~ 1023 对应 0-300°
//		// 舵机: 0 ~ 4095 对应 0-360°
//        uint16_t hmi_pos = CLAMP(hmi_command->position[i], HMI_POSITION_MIN, HMI_POSITION_MAX);
//        
//        // 映射：0-1023 (0-300°) -> 0-4095.0*300/360 (0-300°)
//        uint32_t servo_pos = MAP_INT(hmi_pos, 
//                                      HMI_POSITION_MIN, HMI_POSITION_MAX,
//                                      SERVO_POSITION_MIN, 
//                                      (uint16_t)(SERVO_POSITION_MAX * HMI_POSITION_DEGREE_MAX / SERVO_POSITION_DEGREE_MAX));
//        servo_command[i].position = (uint16_t)CLAMP(servo_pos, SERVO_POSITION_MIN, SERVO_POSITION_MAX);
//        
//        // ========== 4. 速度转换（根据模式） ==========
//        servo_command[i].mode = hmi_command->motor_mode;
//        
//        if (hmi_command->motor_mode == HMI_MOTORMODE_SINGLE_JOINT)
//        {
//			// HMI:  0 ~ 1023 对应 0-114 rpm
//			// 舵机: 0 ~ 4095 对应 0-50  rpm (0.732RPM)
//            // Joint模式：0-1023 -> 0-2730
//            uint16_t hmi_speed = CLAMP(hmi_command->moving_speed[i], HMI_SPEED_JOINT_MIN, HMI_SPEED_JOINT_MAX);
//            uint32_t servo_speed = MAP_INT(hmi_speed,
//                                            HMI_SPEED_JOINT_MIN, HMI_SPEED_JOINT_MAX,
//                                            SERVO_SPEED_JOINT_MIN, SERVO_SPEED_JOINT_MAX);
//            servo_command[i].speed = (int16_t)CLAMP(servo_speed, SERVO_SPEED_JOINT_MIN, SERVO_SPEED_JOINT_MAX);
//        }
//        else if (hmi_command->motor_mode == HMI_MOTORMODE_SINGLE_WHEEL)
//        {
//            // Wheel模式：0-2047 -> -32767~32767
//            uint16_t hmi_speed = CLAMP(hmi_command->moving_speed[i], HMI_SPEED_WHEEL_MIN, HMI_SPEED_WHEEL_MAX);
//            
//            if (hmi_speed == HMI_SPEED_WHEEL_STOP)
//            {
//                servo_command[i].speed = SERVO_SPEED_WHEEL_STOP;
//            }
//            else if (hmi_speed < HMI_SPEED_WHEEL_STOP)
//            {
//                // CCW：0-1023 -> -32767 ~ -1
//                servo_command[i].speed = (int16_t)MAP_INT(hmi_speed,
//                                                         HMI_SPEED_WHEEL_MIN, HMI_SPEED_WHEEL_STOP - 1,
//                                                         SERVO_SPEED_WHEEL_CCW_MIN, -1);
//            }
//            else
//            {
//                // CW：1025-2047 -> 1 ~ 32767
//                servo_command[i].speed = (int16_t)MAP_INT(hmi_speed,
//                                                         HMI_SPEED_WHEEL_STOP + 1, HMI_SPEED_WHEEL_MAX,
//                                                         1, SERVO_SPEED_WHEEL_CW_MAX);
//            }
//        }
//        else
//        {
//            return -1; // 未知模式
//        }
//    }
//    
//    return 0;
//}
