/**
 * @file motor_data_converter.h
 */

#ifndef MOTOR_DATA_CONVERTER_H
#define MOTOR_DATA_CONVERTER_H

#include <stdint.h>


#include "MotorState.h"  // MotorState_typedef
#include "HMIPacket.h"    // HMI_MotorData_t, HMI_MotorPacket_t



/**
 * @brief 单个电机数据转换
 */
int8_t ConvertMotorStateToHMI(const MotorState_typedef *motorstate, 
                               HMI_MotorData_t *HMImotordata, 
                               uint8_t error);

/**
 * @brief 批量电机数据转换
 */
int8_t ConvertAllMotorsToHMI(const MotorState_typedef *motorstate, 
                              HMI_MotorPacket_t *packet, 
                              uint8_t modeMotors);

/**
 * @brief 批量转换（带错误标志）
 */
int8_t ConvertAllMotorsToHMI_WithError(const MotorState_typedef *motorstate, 
                                        HMI_MotorPacket_t *packet, 
                                        uint8_t modeMotors,
                                        const uint8_t *error_flags);

#endif // MOTOR_DATA_CONVERTER_H
