/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "MotorState.h"
#include "FTSerialConfig.h"
#include "HMIProtocol.h" // 上位机顶层发送库
#include "HMISerial.h"	 // 上位机底层串口库
#include <string.h>
#include "motor_data_converter.h" // 上位机与舵机数据转换函数库
#include "HMICommandProtocol.h" // 上位机接收解析库
#include "MotorControl.h"    // 上位机控制指令转换库
#include "SMS_STS.h"     // 飞特舵机顶层库
#include "SCS.h"         // 提供 Ping() 舵机在线检测函数
#include "stm32f1xx_hal_uart.h"   // deinit串口函数




#include "EncoderSimple.h"
#include "ForceSensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    APP_MODE_NONE = 0,    // 尚未检测到设备
    APP_MODE_SERVO,       // 舵机模式
    APP_MODE_ENCODER,     // 编码器模式
    APP_MODE_FORCE        // 力传感器模式
} App_Mode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 等待舵机、编码器等外部设备完成上电 */
#define APP_POWER_ON_DELAY_MS      1000U

/* 每次自动检测最多尝试4轮 */
#define APP_DETECT_RETRY_COUNT     4U

/* 一轮检测失败后，等待500 ms再检测 */
#define APP_DETECT_INTERVAL_MS     500U

/* 编码器探测响应超时时间 */
#define ENCODER_PROBE_TIMEOUT_MS   30U

/* 力传感器读取周期 */
#define FORCE_POLL_PERIOD_MS       100U
/*
 * 原始值与实际显示值的比例。
 * 暂时设为1，之后根据仪表小数点修改。
 */
#define FORCE_DECIMAL_DIVISOR      1.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*
 * 当前系统模式。
 * 可以放入Keil的Watch窗口观察。
 */
volatile App_Mode_t g_app_mode = APP_MODE_NONE;

/*
 * 在线设备掩码：
 * bit0对应ID 1
 * bit1对应ID 2
 * bit2对应ID 3
 * 上电检测得到的在线设备 *
 */
volatile uint8_t g_servo_online_mask = 0U;
volatile uint8_t g_encoder_online_mask = 0U;

/* 力传感器最近一次数据 */
volatile int32_t g_force_raw = 0;
volatile float g_force_value = 0.0f;
volatile uint8_t g_force_valid = 0;

volatile ForceSensor_Status_t g_force_status =
    FORCE_SENSOR_ERR_NOT_INIT;

/* 力传感器数据上传状态，可放入Watch窗口观察 */
volatile HMI_Status_typedef g_force_send_status =
    HMI_ERR_INIT;

volatile uint32_t g_force_task_count = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* 检测1～3号舵机，返回在线设备掩码 */
static uint8_t App_DetectServoMask(void);

/* 检测1～3号编码器，返回在线设备掩码 */
static uint8_t App_DetectEncoderMask(void);

/* 检测力传感器是否在线 */
static uint8_t App_DetectForceSensor(void);

/* 按照“舵机→编码器→力传感器”的规则选择系统模式 */
static App_Mode_t App_DetectMode(void);

/* 只初始化上电时检测到的舵机 */
static void ServoMode_Init(void);

/* 力传感器模式周期任务 */
static void ForceSensor_Task(void);

/* 编码器模式运行循环 */
static void EncoderMode_Run(void);

/* 舵机模式运行循环 */
static void ServoMode_Run(void);

/* 力传感器模式运行循环 */
static void ForceMode_Run(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief 检测ID为1～3的飞特舵机
 * @retval 在线设备掩码
 *         bit0：ID 1舵机
 *         bit1：ID 2舵机
 *         bit2：ID 3舵机
 */
static uint8_t App_DetectServoMask(void)
{
    uint8_t id;
    uint8_t online_mask = 0U;

    for (id = 1U; id <= MAX_SERVOS; id++)
    {
        /*
         * Ping成功时返回对应的舵机ID；
         * 失败时返回-1。
         */
        if (Ping(id) == (int)id)
        {
            online_mask |= (uint8_t)(1U << (id - 1U));
        }
    }

    return online_mask;
}

/**
 * @brief 检测ID为1～3的RS485编码器
 * @retval 在线设备掩码
 *         bit0：ID 1编码器
 *         bit1：ID 2编码器
 *         bit2：ID 3编码器
 */
static uint8_t App_DetectEncoderMask(void)
{
    uint8_t id;
    uint8_t online_mask = 0U;

    for (id = 1U; id <= ENCODER_COUNT; id++)
    {
        /*
         * Encoder_Probe()会发送查询帧，
         * 并在规定时间内等待合法响应。
         */
        if (Encoder_Probe(id, ENCODER_PROBE_TIMEOUT_MS) != 0U)
        {
            online_mask |= (uint8_t)(1U << (id - 1U));
        }
    }

    return online_mask;
}

/**
 * @brief 检测RS485力传感器是否在线
 * @retval 1：收到合法力传感器响应
 * @retval 0：未收到合法响应
 */
static uint8_t App_DetectForceSensor(void)
{
    int32_t raw_value = 0;

    g_force_status = ForceSensor_ReadRaw(&raw_value);

    if (g_force_status == FORCE_SENSOR_OK)
    {
        g_force_raw = raw_value;
        g_force_value =
            (float)raw_value / FORCE_DECIMAL_DIVISOR;
        g_force_valid = 1U;
        return 1U;
    }

    g_force_valid = 0U;
    return 0U;
}

/**
 * @brief 上电检测系统模式
 * @note  每轮依次检测舵机、编码器、力传感器。
 *        若同时连接多类设备，优先级为：舵机 > 编码器 > 力传感器。
 * @retval APP_MODE_SERVO   检测到至少一个舵机
 * @retval APP_MODE_ENCODER 没有舵机，但检测到至少一个编码器
 * @retval APP_MODE_FORCE   没有舵机和编码器，但检测到力传感器
 * @retval APP_MODE_NONE    多轮检测后仍未发现设备
 */
static App_Mode_t App_DetectMode(void)
{
    uint8_t attempt;

    g_servo_online_mask = 0U;
    g_encoder_online_mask = 0U;

    for (attempt = 0U;
         attempt < APP_DETECT_RETRY_COUNT;
         attempt++)
    {
        /*
         * 检测ID 1、2、3舵机。
         * 掩码不等于0，表示至少一台舵机在线。
         */
        g_servo_online_mask =
            App_DetectServoMask();

        if (g_servo_online_mask != 0U)
        {
            return APP_MODE_SERVO;
        }

        /*
         * 没检测到舵机时，
         * 再检测ID 1、2、3编码器。
         * 掩码不等于0，表示至少一台编码器在线。
         */
        g_encoder_online_mask =
            App_DetectEncoderMask();

        if (g_encoder_online_mask != 0U)
        {
            return APP_MODE_ENCODER;
        }

        /*
         * 没检测到舵机和编码器时，
         * 再检测力传感器。
         */
        if (App_DetectForceSensor() != 0U)
        {
            return APP_MODE_FORCE;
        }

        /*
         * 三类设备都没有检测到，
         * 延时后重新检测。
         */
        if ((attempt + 1U) <
            APP_DETECT_RETRY_COUNT)
        {
            HAL_Delay(APP_DETECT_INTERVAL_MS);
        }
    }

    return APP_MODE_NONE;
}

/**
 * @brief 力传感器周期读取任务
 * @note  未到100 ms周期时立即返回。
 */
static void ForceSensor_Task(void)
{
	g_force_task_count++;
	
    static uint32_t last_poll_tick = 0U;
    uint32_t now;
    int32_t raw_value;

    now = HAL_GetTick();

    if ((last_poll_tick != 0U) &&
        ((uint32_t)(now - last_poll_tick) <
         FORCE_POLL_PERIOD_MS))
    {
        return;
    }

    last_poll_tick = now;

    g_force_status = ForceSensor_ReadRaw(&raw_value);
	//raw_value = 1;

	if (g_force_status == FORCE_SENSOR_OK)
	{
		/* 保存最近一次读取成功的数据 */
		g_force_raw = raw_value;

		/* 转换为实际显示值，仅供程序内部观察 */
		g_force_value =
			(float)raw_value / FORCE_DECIMAL_DIVISOR;

		g_force_valid = 1U;

		/*
		 * 将原始32位力值上传给上位机。
		 * 实际发送格式：
		 * 7A F0 D0 D1 D2 D3 7B
		 */
		g_force_send_status =
			HMI_SendForceData(g_force_raw);
		}
	else
		{
		/*
		 * 读取失败时保留上一次正确数值，
		 * 只清除有效标志，不上传错误数据。
		 */
		g_force_valid = 0U;
		}
}

/**
 * @brief 初始化舵机模式
 * @note  进入舵机模式后，固定配置ID 1、2、3。
 */
static void ServoMode_Init(void)
{
    uint8_t index;

    /* 清空舵机状态和上位机数据包 */
    memset(motorstate, 0, sizeof(motorstate));
    memset(&HMImotorpacket, 0, sizeof(HMImotorpacket));

    /* 默认使用位置控制模式 */
    HMImotorpacket.modeMotors =
        HMI_MOTORMODE_ALL_JOINT;

    /*
     * servo_id_map[] = {1, 2, 3}
     * 固定初始化三台舵机。
     */
    for (index = 0U;
         index < MAX_SERVOS;
         index++)
    {
        JointMode(servo_id_map[index]);

        SetMaxAngleLimit(
            servo_id_map[index]);

        /* 读取一次初始状态 */
        (void)MotorFeedBack(
            servo_id_map[index],
            index);
    }
}

/**
 * @brief 编码器模式运行循环
 * @note  只轮询上电时检测到的编码器。
 */
static void EncoderMode_Run(void)
{
    while (1)
    {
        if (Encoder_PollNext() == ENCODER_OK)
        {
            HMIencoderpacket.encoder1_position =
                Encoder_Position[0];

            HMIencoderpacket.encoder2_position =
                Encoder_Position[1];

            HMIencoderpacket.encoder3_position =
                Encoder_Position[2];

            HMI_SendEncoderData(&HMIencoderpacket, 1U);
        }

        HAL_Delay(10U);
    }
}


/**
 * @brief 检查是否进入舵机外部调试模式
 *
 * PC13按下后：
 * 1. 关闭STM32使用的舵机串口UART4；
 * 2. 释放PC10、PC11；
 * 3. 停止主程序，避免STM32继续访问舵机；
 * 4. 外部调试数据线可以接管舵机总线。
 */
static void ExternalServoDebug_Check(void)
{
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
    {
        HAL_Delay(50U);

        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
        {
            /*
             * 当前工程中：
             * UART4用于SM80BL舵机，波特率115200。
             */
            HAL_UART_DeInit(&huart4);

            /*
             * 停止程序，防止STM32继续发送舵机指令。
             * 不建议使用Error_Handler()，因为这不是程序错误，
             * 而是主动进入外部调试模式。
             */
            while (1)
            {
                HAL_Delay(100U);
            }
        }
    }
}


/**
 * @brief 舵机模式运行循环
 */
static void ServoMode_Run(void)
{
    uint8_t index;
	
    while (1)
    {
		
		/*
         * 每次循环先检查外部调试按键。
         * 按下PC13后，停止STM32访问舵机。
         */
        ExternalServoDebug_Check();

		/* 固定读取ID 1、2、3三台舵机状态 */
		for (index = 0U;
			 index < MAX_SERVOS;
			 index++)
		{
			(void)MotorFeedBack(
				servo_id_map[index],
				index);
		}

        ConvertAllMotorsToHMI(motorstate,
                              &HMImotorpacket,
                              HMImotorpacket.modeMotors);

        HMI_SendMotorData(&HMImotorpacket, 1U);

        /* 检查上位机控制指令 */
		if (HMI_Serial_GetRxFlag() != 0U)
		{
			uint8_t *rx_buf = HMI_Serial_GetRxBuffer();
			uint16_t rx_len = HMI_Serial_GetRxLength();

			/*
			 * 只有完整的32字节数据包才解析。
			 * 半包、干扰数据直接丢弃，不能进入Error_Handler。
			 */
			if (rx_len == HMI_COMMAND_PACKET_SIZE)
			{
				uint8_t packet_type =
					HMI_Command_Parse(rx_buf, rx_len);

				if (packet_type != PKT_TYPE_UNKNOWN)
				{
					Motor_Control(packet_type);
				}
			}

			/* 无论数据正确与否，都重新启动接收 */
			HMI_Serial_RestartReceive();
		}
    }
}

/**
 * @brief 力传感器模式运行循环
 */
static void ForceMode_Run(void)
{
    while (1)
    {
        ForceSensor_Task();
        HAL_Delay(10U);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  
	ftUart_Init(&huart4);  // SM80BL舵机使用串口4 波特率115200
  
	/* USART1：上位机通信,上位机使用串口1 波特率57600 */
	if (HMI_Init(&huart1) != HMI_OK)
	{
		Error_Handler();
	} 
	
	/*
	 * USART2：编码器和力传感器共用同一条RS485总线。
	 * 编码器使用115200，力传感器使用19200；二者作为独立模式运行。
	 * 驱动仍会在每笔事务前自动切换到目标波特率。
	 */
	Encoder_Init(&huart2);
	ForceSensor_Init(&huart2);
 
    /* 等待外部设备完成上电 */
    HAL_Delay(APP_POWER_ON_DELAY_MS);
	
	 /*
	 * 如果没有检测到设备，就继续检测；
	 * 检测成功后，模式保持到下次复位。
	 */
    do
    {
        g_app_mode = App_DetectMode();
    }
    while (g_app_mode == APP_MODE_NONE);
	
	 /* 只有舵机模式才执行舵机初始化指令 */
    if (g_app_mode == APP_MODE_SERVO)
    {
        ServoMode_Init();
    }
	
	else if (g_app_mode == APP_MODE_ENCODER)
	{
		/*
		 * 运行阶段只查询上电时检测到的编码器，
		 * 不再等待没有连接的ID。
		 */
		Encoder_SetActiveMask(ENCODER_ALL_MASK);
	}
	else if (g_app_mode == APP_MODE_FORCE)
	{
		/* 力传感器模式无需额外初始化。 */
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if (g_app_mode == APP_MODE_SERVO)
		{
			ServoMode_Run();
		}
		else if (g_app_mode == APP_MODE_ENCODER)
		{
			EncoderMode_Run();
		}
		else if (g_app_mode == APP_MODE_FORCE)
		{
			ForceMode_Run();
		}
		else
		{
        /* 异常保护：重新检测设备 */
			g_app_mode = App_DetectMode();
		}
	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief 空闲中断 + DMA接收满 回调
 * @note HAL_UARTEx_ReceiveToIdle_DMA 会在两种情况触发此回调:
 *       1. UART空闲中断
 *       2. DMA接收完成中断(缓冲区满)
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    HMI_Serial_RxEventCallback(huart, Size);
}

/**
 * @brief DMA接收完成中断回调(备用)
 * @note 某些HAL版本可能需要同时实现此回调
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {  // 根据使用的串口修改
        // 获取接收长度(缓冲区大小)
        HMI_Serial_RxEventCallback(huart, 32);
    }

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
