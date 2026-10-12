/**
 * @file EncoderSimple.c
 * @brief 编码器阻塞式事务驱动
 *
 * 编码器使用115200-8-N-1；力传感器使用19200-8-N-1。
 * 两类设备虽然共用USART2和A/B总线，但每次事务前切换到目标波特率，
 * 因此不会同时响应同一个地址帧。
 */
#include "EncoderSimple.h"
#include "RS485Bus.h"
#include <string.h>

static uint8_t g_encoder_initialized = 0U;
static uint8_t g_encoder_active_mask = ENCODER_ALL_MASK;
static uint8_t g_encoder_next_id = 1U;

uint8_t Encorder_Txpacket[ENCODER_TX_FRAME_SIZE] =
{
    0x01U, 0x03U, 0x00U, 0x00U,
    0x00U, 0x01U, 0x84U, 0x0AU
};

uint8_t Encorder_Rxpacket[ENCODER_RX_FRAME_SIZE] = {0U};

float Encoder_Position[ENCODER_COUNT] =
{
    0.0f, 0.0f, 0.0f
};

volatile Encoder_Status_t Encoder_LastStatus =
    ENCODER_ERR_NOT_INIT;

volatile uint8_t Encoder_LastPolledId = 0U;

static uint16_t Encoder_CombineU16(uint8_t high,
                                   uint8_t low)
{
    return ((uint16_t)high << 8U) | (uint16_t)low;
}

static uint8_t Encoder_FindActiveId(uint8_t start_id)
{
    uint8_t checked;
    uint8_t id = start_id;

    if ((id < 1U) || (id > ENCODER_COUNT))
    {
        id = 1U;
    }

    for (checked = 0U; checked < ENCODER_COUNT; checked++)
    {
        if ((g_encoder_active_mask &
             (uint8_t)(1U << (id - 1U))) != 0U)
        {
            return id;
        }

        id++;
        if (id > ENCODER_COUNT)
        {
            id = 1U;
        }
    }

    return 0U;
}

static void Encoder_AdvanceNextId(uint8_t current_id)
{
    uint8_t start_id = (uint8_t)(current_id + 1U);
    uint8_t next_id;

    if (start_id > ENCODER_COUNT)
    {
        start_id = 1U;
    }

    next_id = Encoder_FindActiveId(start_id);
    if (next_id != 0U)
    {
        g_encoder_next_id = next_id;
    }
}

static Encoder_Status_t Encoder_MapBusStatus(
    RS485Bus_Status_t bus_status)
{
    switch (bus_status)
    {
        case RS485_BUS_OK:
            return ENCODER_OK;

        case RS485_BUS_ERR_RX_TIMEOUT:
            return ENCODER_ERR_TIMEOUT;

        case RS485_BUS_ERR_TX:
            return ENCODER_ERR_TX;

        case RS485_BUS_ERR_NOT_INIT:
            return ENCODER_ERR_NOT_INIT;

        case RS485_BUS_ERR_PARAM:
            return ENCODER_ERR_PARAM;

        default:
            return ENCODER_ERR_UART;
    }
}

static Encoder_Status_t Encoder_ValidateAndParse(
    uint8_t expected_id)
{
    uint16_t crc_calculated;
    uint16_t crc_received;
    uint16_t raw_position;

    if ((Encorder_Rxpacket[0] != expected_id) ||
        (Encorder_Rxpacket[1] != 0x03U) ||
        (Encorder_Rxpacket[2] != 0x02U))
    {
        return ENCODER_ERR_FRAME;
    }

    crc_calculated =
        (uint16_t)Crc_Count(Encorder_Rxpacket, 5U);

    crc_received =
        (uint16_t)Encorder_Rxpacket[5] |
        ((uint16_t)Encorder_Rxpacket[6] << 8U);

    if (crc_calculated != crc_received)
    {
        return ENCODER_ERR_CRC;
    }

    raw_position = Encoder_CombineU16(
        Encorder_Rxpacket[3],
        Encorder_Rxpacket[4]);

    Encoder_Position[expected_id - 1U] =
        (float)raw_position * 360.0f / 1024.0f;

    return ENCODER_OK;
}

unsigned int Crc_Count(unsigned char pbuf[], unsigned char num)
{
    unsigned char i;
    unsigned char bit;
    unsigned int crc = 0xFFFFU;

    for (i = 0U; i < num; i++)
    {
        crc ^= (unsigned int)pbuf[i];

        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc >>= 1U;
                crc ^= 0xA001U;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

void Encoder_Init(UART_HandleTypeDef *huart)
{
    RS485Bus_Init(huart);

    g_encoder_initialized = (huart != NULL) ? 1U : 0U;
    g_encoder_active_mask = ENCODER_ALL_MASK;
    g_encoder_next_id = 1U;
    Encoder_LastPolledId = 0U;
    Encoder_LastStatus = (huart != NULL) ?
        ENCODER_OK : ENCODER_ERR_NOT_INIT;

    memset(Encorder_Rxpacket, 0, sizeof(Encorder_Rxpacket));
    memset(Encoder_Position, 0, sizeof(Encoder_Position));
}

void Encoder_SetActiveMask(uint8_t active_mask)
{
    g_encoder_active_mask =
        (uint8_t)(active_mask & ENCODER_ALL_MASK);

    if (g_encoder_active_mask != 0U)
    {
        g_encoder_next_id = Encoder_FindActiveId(1U);
    }
    else
    {
        g_encoder_next_id = 1U;
    }
}

Encoder_Status_t Encoder_ReadById(uint8_t encoder_id,
                                  uint32_t timeout_ms)
{
    uint16_t crc;
    RS485Bus_Status_t bus_status;
    Encoder_Status_t status;

    if (g_encoder_initialized == 0U)
    {
        Encoder_LastStatus = ENCODER_ERR_NOT_INIT;
        return Encoder_LastStatus;
    }

    if ((encoder_id < 1U) ||
        (encoder_id > ENCODER_COUNT) ||
        (timeout_ms == 0U))
    {
        Encoder_LastStatus = ENCODER_ERR_PARAM;
        return Encoder_LastStatus;
    }

    Encorder_Txpacket[0] = encoder_id;

    crc = (uint16_t)Crc_Count(Encorder_Txpacket, 6U);
    Encorder_Txpacket[6] = (uint8_t)(crc & 0x00FFU);
    Encorder_Txpacket[7] = (uint8_t)(crc >> 8U);

    memset(Encorder_Rxpacket, 0, sizeof(Encorder_Rxpacket));
    Encoder_LastPolledId = encoder_id;

    bus_status = RS485Bus_Transceive(
        ENCODER_BAUD_RATE,
        Encorder_Txpacket,
        ENCODER_TX_FRAME_SIZE,
        Encorder_Rxpacket,
        ENCODER_RX_FRAME_SIZE,
        30U,
        timeout_ms);

    status = Encoder_MapBusStatus(bus_status);
    if (status != ENCODER_OK)
    {
        Encoder_LastStatus = status;
        return status;
    }

    status = Encoder_ValidateAndParse(encoder_id);
    Encoder_LastStatus = status;
    return status;
}

uint8_t Encoder_Probe(uint8_t encoder_id, uint32_t timeout_ms)
{
    return (Encoder_ReadById(encoder_id, timeout_ms) ==
            ENCODER_OK) ? 1U : 0U;
}

Encoder_Status_t Encoder_PollNext(void)
{
    uint8_t current_id;
    Encoder_Status_t status;

    if (g_encoder_initialized == 0U)
    {
        Encoder_LastStatus = ENCODER_ERR_NOT_INIT;
        return Encoder_LastStatus;
    }

    current_id = Encoder_FindActiveId(g_encoder_next_id);
    if (current_id == 0U)
    {
        Encoder_LastStatus = ENCODER_ERR_NO_ACTIVE_DEVICE;
        return Encoder_LastStatus;
    }

    status = Encoder_ReadById(current_id,
                              ENCODER_DEFAULT_TIMEOUT_MS);

    /* 无论本次成功或超时，下一轮都继续查询下一台，避免卡死 */
    Encoder_AdvanceNextId(current_id);

    return status;
}
