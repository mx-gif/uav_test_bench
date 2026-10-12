#include "ForceSensor.h"
#include "RS485Bus.h"
#include <string.h>

static uint8_t g_force_initialized = 0U;

uint8_t ForceSensor_LastRx[FORCE_SENSOR_RX_FRAME_SIZE] = {0U};

uint16_t ForceSensor_ModbusCRC16(const uint8_t *data,
                                uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;

    for (i = 0U; i < length; i++)
    {
        crc ^= data[i];

        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (crc >> 1U) ^ 0xA001U;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static ForceSensor_Status_t ForceSensor_MapBusStatus(
    RS485Bus_Status_t bus_status)
{
    switch (bus_status)
    {
        case RS485_BUS_OK:
            return FORCE_SENSOR_OK;

        case RS485_BUS_ERR_NOT_INIT:
            return FORCE_SENSOR_ERR_NOT_INIT;

        case RS485_BUS_ERR_PARAM:
            return FORCE_SENSOR_ERR_PARAM;

        case RS485_BUS_ERR_TX:
            return FORCE_SENSOR_ERR_TX;

        case RS485_BUS_ERR_RX_TIMEOUT:
            return FORCE_SENSOR_ERR_TIMEOUT;

        default:
            return FORCE_SENSOR_ERR_UART;
    }
}

void ForceSensor_Init(UART_HandleTypeDef *huart)
{
    /*
     * 编码器初始化和力传感器初始化传入的都是&huart2。
     * 重复绑定同一个句柄是安全的，最终均由RS485Bus顺序管理。
     */
    RS485Bus_Init(huart);
    g_force_initialized = (huart != NULL) ? 1U : 0U;
    memset(ForceSensor_LastRx, 0, sizeof(ForceSensor_LastRx));
}

ForceSensor_Status_t ForceSensor_ReadRaw(int32_t *raw_value)
{
    uint8_t tx_frame[8] =
    {
        FORCE_SENSOR_ADDRESS,
        0x03U,
        0x30U,
        0x00U,
        0x00U,
        0x02U,
        0x00U,
        0x00U
    };

    uint16_t crc_calculated;
    uint16_t crc_received;
    uint32_t raw_u32;
    RS485Bus_Status_t bus_status;
    ForceSensor_Status_t status;

    if (g_force_initialized == 0U)
    {
        return FORCE_SENSOR_ERR_NOT_INIT;
    }

    if (raw_value == NULL)
    {
        return FORCE_SENSOR_ERR_PARAM;
    }

    crc_calculated = ForceSensor_ModbusCRC16(tx_frame, 6U);
    tx_frame[6] = (uint8_t)(crc_calculated & 0x00FFU);
    tx_frame[7] = (uint8_t)(crc_calculated >> 8U);

    memset(ForceSensor_LastRx, 0, sizeof(ForceSensor_LastRx));

    bus_status = RS485Bus_Transceive(
        FORCE_SENSOR_BAUD_RATE,
         tx_frame,
        (uint16_t)sizeof(tx_frame),
        ForceSensor_LastRx,
        (uint16_t)sizeof(ForceSensor_LastRx),
        30U,
        40U);

    status = ForceSensor_MapBusStatus(bus_status);
    if (status != FORCE_SENSOR_OK)
    {
        return status;
    }

    if ((ForceSensor_LastRx[0] != FORCE_SENSOR_ADDRESS) ||
        (ForceSensor_LastRx[1] != 0x03U) ||
        (ForceSensor_LastRx[2] != 0x04U))
    {
        return FORCE_SENSOR_ERR_FRAME;
    }

    crc_calculated =
        ForceSensor_ModbusCRC16(ForceSensor_LastRx, 7U);

    crc_received =
        (uint16_t)ForceSensor_LastRx[7] |
        ((uint16_t)ForceSensor_LastRx[8] << 8U);

    if (crc_calculated != crc_received)
    {
        return FORCE_SENSOR_ERR_CRC;
    }

    /* 传感器4字节数据为大端顺序 */
    raw_u32 =
        ((uint32_t)ForceSensor_LastRx[3] << 24U) |
        ((uint32_t)ForceSensor_LastRx[4] << 16U) |
        ((uint32_t)ForceSensor_LastRx[5] << 8U)  |
        ((uint32_t)ForceSensor_LastRx[6]);

    *raw_value = (int32_t)raw_u32;
    return FORCE_SENSOR_OK;
}
