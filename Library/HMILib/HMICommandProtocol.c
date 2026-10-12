/**
 * @file HMICommandProtocol.c
 * @brief HMI数据包协议解析器实现
 * @author wangziwei
 */

#include "HMICommandProtocol.h"
#include <string.h>

#define HMI_MOTORMODE_SINGLE_WHEEL    0x31
#define HMI_MOTORMODE_SINGLE_JOINT    0x32

/* 全局电机命令：上位机侧 */
HMI_Command_t g_motor_command = {
    .torque_enable = {0, 0, 0},
    .torque_limit  = {0x03FF, 0x03FF, 0x03FF},
    .torque_max    = {0x03FF, 0x03FF, 0x03FF},
    .moving_speed  = {0, 0, 0},
    .position      = {0, 0, 0},
    .motor_mode    = HMI_MOTORMODE_SINGLE_JOINT,
};

/* 私有函数声明 */
static uint16_t parse_u16(uint8_t low, uint8_t high);
static void handle_packet_type1(uint8_t *pkt);
static void handle_packet_type2(uint8_t *pkt);
static void handle_packet_type3(uint8_t *pkt);
static uint8_t is_zero_packet(const uint8_t *pkt);
static uint16_t clamp_u16(uint16_t value, uint16_t min, uint16_t max);

uint8_t HMI_Command_Parse(uint8_t *packet, uint16_t length)
{
    uint8_t packet_type;

    /* 检查指针和长度 */
    if ((packet == NULL) ||
        (length != HMI_COMMAND_PACKET_SIZE))
    {
        return PKT_TYPE_UNKNOWN;
    }

    /* 检查包头和包尾 */
    if ((packet[PKT_IDX_HEADER] != HMI_COMMAND_HEADER) ||
        (packet[PKT_IDX_TAIL] != HMI_COMMAND_TAIL))
    {
        return PKT_TYPE_UNKNOWN;
    }

    packet_type = packet[PKT_IDX_TYPE];

    switch (packet_type)
    {
        case PKT_TYPE_CONTROL_1:
            handle_packet_type1(packet);
            return PKT_TYPE_CONTROL_1;

        case PKT_TYPE_CONTROL_2:
            handle_packet_type2(packet);
            return PKT_TYPE_CONTROL_2;

        case PKT_TYPE_CONFIG:
            handle_packet_type3(packet);
            return PKT_TYPE_CONFIG;

        case PKT_TYPE_ZERO:
            if (is_zero_packet(packet) != 0U)
            {
                return PKT_TYPE_ZERO;
            }
            return PKT_TYPE_UNKNOWN;

        default:
            return PKT_TYPE_UNKNOWN;
    }
}

/**
 * @brief 获取电机控制指针
 */
HMI_Command_t* HMI_Command_GetParams(void)
{
    return &g_motor_command;
}

/* ==================== 私有函数实现 ==================== */

static uint16_t parse_u16(uint8_t low, uint8_t high)
{
    return (uint16_t)((high << 8) | low);
}

static uint16_t clamp_u16(uint16_t value, uint16_t min, uint16_t max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief 处理控制包1(扭矩配置)
 */
static void handle_packet_type1(uint8_t *pkt)
{
    /* 功能2:电机扭矩使能 */
    for (uint8_t i = 0; i < 3; i++) {
        g_motor_command.torque_enable[i] = (pkt[PKT_IDX_TORQUE_EN] >> i) & 0x01;
    }
    
    /* 功能3:修改电机扭矩限制 */
    if (pkt[PKT_IDX_TORQUE_LIM] == 0x07) {
        for (uint8_t i = 0; i < 3; i++) {
            uint16_t limit_torque = parse_u16(pkt[6 + 2*i], pkt[7 + 2*i]);
            g_motor_command.torque_limit[i] = limit_torque;
        }
    }
    
    /* 功能4:修改电机最大扭矩 */
    if (pkt[PKT_IDX_TORQUE_MAX] == 0x04) {
        for (uint8_t i = 0; i < 3; i++) {
            uint16_t max_torque = parse_u16(pkt[13 + 2*i], pkt[14 + 2*i]);
            g_motor_command.torque_max[i] = max_torque;
        }
    }
}

/**
 * @brief 处理控制包2(速度、位置)
 */
static void handle_packet_type2(uint8_t *pkt)
{
    /* 功能1:设置旋转速度 */
    for (uint8_t i = 0; i < 3; i++) {
        if ((pkt[PKT_IDX_SPEED_FLAG] & (1U << i)) != 0U) {
            uint16_t speed = parse_u16(pkt[16 + 2*i], pkt[17 + 2*i]);
            
            if (g_motor_command.motor_mode == HMI_MOTORMODE_SINGLE_JOINT) {
                g_motor_command.moving_speed[i] = clamp_u16(speed, 0, 1023);
            } else {
                g_motor_command.moving_speed[i] = clamp_u16(speed, 0, 2047);
            }
        }
    }
    
    /* 功能2:设置电机位置(仅JOINT模式) */
    if (g_motor_command.motor_mode == HMI_MOTORMODE_SINGLE_JOINT) {
        for (uint8_t i = 0; i < 3; i++) {
            if ((pkt[PKT_IDX_POS_FLAG] & (1U << i)) != 0U) {
                uint16_t pos = parse_u16(pkt[23 + 2*i], pkt[24 + 2*i]);
                g_motor_command.position[i] = pos;
            }
        }
    }
}

/**
 * @brief 处理配置包(模式切换)
 */
static void handle_packet_type3(uint8_t *pkt)
{
    /* 功能1:电机模式切换 */
    if (pkt[PKT_IDX_SHOULD_CHANGE_MODE] == 0x31) { // 上位机想要切换模式
        g_motor_command.motor_mode = pkt[PKT_IDX_MODE]; // 上位机当前所处的模式
    }
}


/**
 * @brief 检查是否为约定的置零指令
 * @note  固定格式：7A 04 01 01 07 00 ... 00 7B
 */
static uint8_t is_zero_packet(const uint8_t *pkt)
{
    uint8_t i;

    if ((pkt[2] != 0x01U) ||
        (pkt[3] != 0x01U) ||
        (pkt[4] != 0x07U))
    {
        return 0U;
    }

    for (i = 5U; i < PKT_IDX_TAIL; i++)
    {
        if (pkt[i] != 0x00U)
        {
            return 0U;
        }
    }

    return 1U;
}
