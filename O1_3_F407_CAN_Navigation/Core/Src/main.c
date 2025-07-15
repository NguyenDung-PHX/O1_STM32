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
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "FS-iA6B.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define ERPM_MAX   4000
#define ERPM_MIN  -4000

#define VESC1 0x00000040
#define VESC2 0x00000043
#define VESC3 0x00000064
#define VESC4 0x00000065

#define SERVO 0x00000000

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

extern uint8_t ibus_rx_buf[32];  // Ibus
extern uint8_t ibus_rx_cplt_flag;

CAN_TxHeaderTypeDef Can_Tx_Header;

uint32_t TxMailBox;

uint8_t Can_Tx_Data[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };


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
  MX_CAN1_Init();
  MX_UART5_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */

  LL_USART_EnableIT_RXNE(UART5); // bat ngat ibus

  HAL_CAN_Start(&hcan1);
  Can_Tx_Header.DLC = 8;
  Can_Tx_Header.IDE = CAN_ID_EXT;
  Can_Tx_Header.RTR = CAN_RTR_DATA;
  Can_Tx_Header.TransmitGlobalTime = DISABLE;

  HAL_CAN_Start(&hcan2);
  Can_Tx_Header.ExtId = SERVO;
  Can_Tx_Header.DLC = 8;
  Can_Tx_Header.IDE = CAN_ID_EXT;
  Can_Tx_Header.RTR = CAN_RTR_DATA;
  Can_Tx_Header.TransmitGlobalTime = DISABLE;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(iBus.SwA == 2000)
	  {
		  if(iBus.RV > 1530 || iBus.RV < 1470)
		  {
			  static int16_t control_prev;
			  int16_t control = 0.6f*control_prev + 0.4f*(iBus.RV - 1500)/2.0f;
			  control_prev = control;
			  Can_Tx_Data[0] = (control >> 8) & 0xFF;
			  Can_Tx_Data[1] = (control >> 8) & 0xFF;
			  Can_Tx_Data[2] = control & 0xFF;

			  Can_Tx_Header.ExtId = VESC1;
			  HAL_CAN_AddTxMessage(&hcan1, &Can_Tx_Header, Can_Tx_Data, &TxMailBox);
			  Can_Tx_Header.ExtId = VESC2;
			  HAL_CAN_AddTxMessage(&hcan1, &Can_Tx_Header, Can_Tx_Data, &TxMailBox);
			  HAL_Delay(1);
			  Can_Tx_Header.ExtId = VESC3;
			  HAL_CAN_AddTxMessage(&hcan1, &Can_Tx_Header, Can_Tx_Data, &TxMailBox);
			  Can_Tx_Header.ExtId = VESC4;
			  HAL_CAN_AddTxMessage(&hcan1, &Can_Tx_Header, Can_Tx_Data, &TxMailBox);
		  }
		  else
		  {
			  Can_Tx_Data[0] = 0;
			  Can_Tx_Data[1] = 0;
			  Can_Tx_Data[2] = 0;

			  Can_Tx_Header.ExtId = VESC1;
			  HAL_CAN_AddTxMessage(&hcan1, &Can_Tx_Header, Can_Tx_Data, &TxMailBox);
			  Can_Tx_Header.ExtId = VESC2;
			  HAL_CAN_AddTxMessage(&hcan1, &Can_Tx_Header, Can_Tx_Data, &TxMailBox);
			  HAL_Delay(1);
			  Can_Tx_Header.ExtId = VESC3;
			  HAL_CAN_AddTxMessage(&hcan1, &Can_Tx_Header, Can_Tx_Data, &TxMailBox);
			  Can_Tx_Header.ExtId = VESC4;
			  HAL_CAN_AddTxMessage(&hcan1, &Can_Tx_Header, Can_Tx_Data, &TxMailBox);
		  }
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
