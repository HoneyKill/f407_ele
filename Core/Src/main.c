/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Emm_V5_App.h"
#include "Emm_V5.h"
#include "Uart1_Dma.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAKE_SPEED_MAX_RPM          (5000U)
#define MAKE_POS_ACC                (EMM_GIMBAL_DEFAULT_ACC)
#define MAKE_POS_PULSE_PER_REV      (3200U)
#define MAKE_POS_IS_ABSOLUTE        (false)
#define MAKE_POS_SYNC_FLAG          (false)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void make1(float speed, float weizhi);
void make2(float speed, float weizhi);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t make_speed_to_rpm(float speed)
{
  if (speed <= 0.0f) {
    return 0U;
  }

  if (speed > (float)MAKE_SPEED_MAX_RPM) {
    speed = (float)MAKE_SPEED_MAX_RPM;
  }

  return (uint16_t)(speed + 0.5f);
}

static uint32_t make_angle_to_pulse(float angle_deg)
{
  float abs_angle = (angle_deg >= 0.0f) ? angle_deg : -angle_deg;

  return (uint32_t)((abs_angle * (float)MAKE_POS_PULSE_PER_REV / 360.0f) + 0.5f);
}

static uint8_t make_dir_from_angle(float angle_deg, bool invert)
{
  uint8_t dir = (angle_deg >= 0.0f) ? EMM_GIMBAL_DIR_CW : EMM_GIMBAL_DIR_CCW;

  if (invert) {
    dir = (dir == EMM_GIMBAL_DIR_CW) ? EMM_GIMBAL_DIR_CCW : EMM_GIMBAL_DIR_CW;
  }

  return dir;
}

static void make_axis(uint8_t addr, bool invert, float speed, float weizhi)
{
  uint16_t vel_rpm = make_speed_to_rpm(speed);
  uint32_t clk = make_angle_to_pulse(weizhi);

  if ((vel_rpm == 0U) || (clk == 0U)) {
    return;
  }

  Emm_V5_Pos_Control(addr,
                     make_dir_from_angle(weizhi, invert),
                     vel_rpm,
                     MAKE_POS_ACC,
                     clk,
                     MAKE_POS_IS_ABSOLUTE,
                     MAKE_POS_SYNC_FLAG);
}

void make1(float speed, float weizhi)
{
  make_axis(EMM_GIMBAL_PAN_MOTOR_ADDR, (EMM_GIMBAL_PAN_DIR_INVERT != 0U), speed, weizhi);
}

void make2(float speed, float weizhi)
{
  make_axis(EMM_GIMBAL_TILT_MOTOR_ADDR, (EMM_GIMBAL_TILT_DIR_INVERT != 0U), speed, weizhi);
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
  MX_USB_DEVICE_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
    //  Emm_V5_Vel_Control(2U, 0U, 100U, 100U, false);

		// 	Emm_V5_Vel_Control(1U, 0U, 100U, 100U, false);


  UART1_DmaCommInit();

  /* 等待 EMM 驱动器上电就绪（关键！不等的话后面指令全部丢失） */
  HAL_Delay(500);

  Emm_V5_App_Init();

  /* ---- 开机测试动作：直接用底层函数，和能跑的程序一样 ---- */
  HAL_Delay(200);
  Emm_V5_Pos_Control(0x01, 0, 200, 80, 44, false, false);  /* 1号 CW 5度 */
  HAL_Delay(5);
  Emm_V5_Pos_Control(0x02, 0, 200, 80, 44, false, false);  /* 2号 CW 5度 */
  HAL_Delay(1000);
  Emm_V5_Pos_Control(0x01, 1, 200, 80, 44, false, false);  /* 1号 CCW 5度 */
  HAL_Delay(5);
  Emm_V5_Pos_Control(0x02, 1, 200, 80, 44, false, false);  /* 2号 CCW 5度 */
  HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Emm_V5_App_Task();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
