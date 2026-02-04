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
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
#include "CAN.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint8_t DM4310_ID;
    uint8_t DM4310_ERR;
    float DM4310_POS;
    float DM4310_VEL;
    float DM4310_T;
    float DM4310_T_MOS;
    float DM4310_T_Rotor;
    float DM4310_POS_Last;
    float DM4310_VEL_Last;
    float DM4310_T_Last;
		uint16_t key;
}DM4310_Rx_Data_t;
typedef struct {
    uint8_t DM4310_ID;
    float DM4310_P_des;
    float DM4310_V_des;
    float DM4310_Kp;
    float DM4310_Kd;
    float DM4310_T_ff;
}DM4310_Tx_Data_t;
//typedef struct{
//    float kp;            //比例系数
//    float ki;            //积分系数
//    float kd;            //微分系数
//    float goal;          //目标值
//    float current;       //当前值
//    float error;         //误差
//    float last_error;    //上次误差
//    float error_sum;      //误差和
//    float ki_output;     //积分值
//    float kd_output;
//    float kp_output;   //微分值
//}PID_Data;
uint8_t rx_data[8];
uint8_t tx_data[8];
DM4310_Rx_Data_t DM4310_Rx_Data;
DM4310_Tx_Data_t DM4310_Tx_Data;
PID_Data pid_data;
uint16_t a;
//uint16_t b;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//		if (GPIO_Pin == GPIO_PIN_0) 
//    {
//		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_RESET);
//		DM4310_DisEnable(&hcan1);
//    }
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
/* 手动开启 FPU */
SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));  /* 设置 CP10 和 CP11 全访问权限 */
__DSB();
__ISB();  /* USER CODE END 1 */

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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
	// 1. 配置过滤器
	CAN_Filter_Config();

// 2. 启动 CAN 外设
		HAL_CAN_Start(&hcan1);

// 3. 开启接收中断（如果需要）
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  PIDInit(&pid_data,0.73,0.01,0.4);
  PID_Changegoal(&pid_data,20);
  DM4310_Tx_Data_Init(&DM4310_Tx_Data,0,0,0,0,0);
  DM4310_Enable(&hcan1); // 使能 ID 为 0x01 的电机

	DM4310_Control(&hcan1,&DM4310_Tx_Data);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_SET);
//			if(DM4310_Rx_Data.DM4310_ERR==0x08 || DM4310_Rx_Data.DM4310_ERR==0x09 || DM4310_Rx_Data.DM4310_ERR==0x0A || DM4310_Rx_Data.DM4310_ERR==0x0B || DM4310_Rx_Data.DM4310_ERR==0x0C || DM4310_Rx_Data.DM4310_ERR==0x0D || DM4310_Rx_Data.DM4310_ERR==0x0E){
//					DM4310_DisEnable(&hcan1);
//			}	
//			else{		DM4310_Control(&hcan1,&DM4310_Tx_Data);}
		HAL_Delay(1);
		DM4310_Tx_Data.DM4310_T_ff=PIDCompute(&pid_data,DM4310_Rx_Data.DM4310_VEL);
		DM4310_Control(&hcan1,&DM4310_Tx_Data);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
