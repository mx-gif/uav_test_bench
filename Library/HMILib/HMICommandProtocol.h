/**
 * @file HMICommandProtocol.h
 * @brief HMI指令数据包协议解析器
 * @author wangziwei
 */

#ifndef __HMI_COMMAND_PROTOCOL_H
#define __HMI_COMMAND_PROTOCOL_H

#include <stdint.h>

/* ==================== 数据包类型定义 ==================== */
#define PKT_TYPE_CONTROL_1      0x01  // 控制包1:扭矩、数据率
#define PKT_TYPE_CONTROL_2      0x02  // 控制包2:速度、位置
#define PKT_TYPE_CONFIG         0x03  // 配置包:模式切换
#define PKT_TYPE_ZERO           0x04  // 置零包
#define PKT_TYPE_UNKNOWN        0x00  // 未知类型

#define HMI_COMMAND_PACKET_SIZE    32U
#define HMI_COMMAND_HEADER         0x7AU
#define HMI_COMMAND_TAIL           0x7BU

#define PKT_IDX_TAIL               31U


/* ==================== 数据包索引定义 ==================== */
#define PKT_IDX_HEADER        0   // 包头
#define PKT_IDX_TYPE          1   // 类型
#define PKT_IDX_DATARATE      3   // 数据率
// 控制包1
#define PKT_IDX_TORQUE_EN     4   // 扭矩使能
#define PKT_IDX_TORQUE_LIM    5   // 设置扭矩限制
#define PKT_IDX_TORQUE_MAX    12  // 设置最大扭矩标志
// 控制包2
#define PKT_IDX_SPEED_FLAG    15  // 设置速度
#define PKT_IDX_POS_FLAG      22  // 设置位置
// 控制包3
#define PKT_IDX_SHOULD_CHANGE_MODE   2   // 需要修改舵机模式字段
#define PKT_IDX_MODE          5   // 舵机目标模式字段



#define MOTOR_COUNT           3  // 电机数量


/* 电机参数结构体(替代分散的全局变量) */
typedef struct {
    uint8_t  torque_enable[MOTOR_COUNT];      // 扭矩使能
    uint16_t torque_limit[MOTOR_COUNT];       // 扭矩限制
    uint16_t torque_max[MOTOR_COUNT];         // 最大扭矩
    uint16_t moving_speed[MOTOR_COUNT];       // 目标速度
    uint16_t position[MOTOR_COUNT];           // 目标位置
    uint8_t  motor_mode ;               // 电机模式
} HMI_Command_t;




/* ==================== 函数声明 ==================== */

/**
 * @brief 解析HMI数据包
 * @param packet: 数据包缓冲区
 * @param length: 数据包长度
 * @retval 数据包类型(PKT_TYPE_xxx)，失败返回PKT_TYPE_UNKNOWN
 */
uint8_t HMI_Command_Parse(uint8_t *packet, uint16_t length);

/**
 * @brief 获取全局电机命令参数指针
 * @retval 电机命令结构体指针
 */
HMI_Command_t* HMI_Command_GetParams(void);

#endif /* __HMI_COMMAND_PROTOCOL_H */



