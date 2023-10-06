/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
# include "msg_handler.hpp"
# include "XNucleoIHM02A1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */


MsgHandler msg_handler(&huart2);
float to_send = 0.1;
bool oneof2led = false;
bool timer_20_ms = false;

XNucleoIHM02A1* bottom_shield;
L6470 **motors;

L6470_init_t initShield[L6470DAISYCHAINSIZE] = { // init parameters for the motors
/* First Motor.G */
{
	22.0,							   /* Motor supply voltage in V. */
	200,						   /* Min number of steps per revolution for the motor. = 360/1.8° */
	1.0,							   /* Max motor phase voltage in A. /!\ UNUSED - USELESS /!\ */
	3.5,							   /* Max motor phase voltage in V. /!\ UNUSED - USELESS /!\ */
	0,							   /* Motor initial speed [step/s]. Seems logic at 0*/
	1500.0,						   /* Motor acceleration [step/s^2] (comment for infinite acceleration mode) */
	1500.0,						   /* Motor deceleration [step/s^2] (comment for infinite deceleration mode)*/
	3000,						   /* Motor maximum speed [step/s] */
	0.0,						   /* Motor minimum speed [step/s]*/
	1500,						   /* Motor full-step speed threshold [step/s]. Limit microstep -> fullstep */
	5.3,							   /* Holding kval [V]. */
	5.3,							   /* Constant speed kval [V]. */
	5.3,							   /* Acceleration starting kval [V]. */
	5.3,							   /* Deceleration starting kval [V]. */
	269.9268,							   /* Intersect speed for bemf compensation curve slope changing [step/s]. */
	0.00072448,						   /* Start slope [s/step]. */
	0.0016,						   /* Acceleration final slope [s/step]. */
	0.0016,						   /* Deceleration final slope [s/step]. */
	0,							   /* Thermal compensation factor (range [0, 15]). */
	3.5 * 1000 * 1.10,			   /* Ocd threshold [ma] (range [375 ma, 6000 ma]). Calculated with Kval*/
	3.5 * 1000 * 1.00,			   /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). Calculated Kval */
	StepperMotor::STEP_MODE_HALF, /* Step mode selection. */
	0xFF,						   /* Alarm conditions enable. */
	0x2E88						   /* Ic configuration. */
},

/* Second Motor. */
{
	22.0,							   /* Motor supply voltage in V. */
	200,						   /* Min number of steps per revolution for the motor. = 360/1.8° */
	1.0,							   /* Max motor phase voltage in A. /!\ UNUSED - USELESS /!\ */
	3.5,							   /* Max motor phase voltage in V. /!\ UNUSED - USELESS /!\ */
	0,							   /* Motor initial speed [step/s]. */
	1500.0,						   /* Motor acceleration [step/s^2] (comment for infinite acceleration mode). */
	1500.0,						   /* Motor deceleration [step/s^2] (comment for infinite deceleration mode). */
	3000,						   /* Motor maximum speed [step/s]. */
	0.0,						   /* Motor minimum speed [step/s]. */
	1500,						   /* Motor full-step speed threshold [step/s]. Limit microstep -> fullstep */
	5.3,							   /* Holding kval [V]. */
	5.3,							   /* Constant speed kval [V]. */
	5.3,							   /* Acceleration starting kval [V]. */
	5.3,							   /* Deceleration starting kval [V]. */
	269.9268,							   /* Intersect speed for bemf compensation curve slope changing [step/s]. */
	0.00072448,						   /* Start slope [s/step]. */
	0.0016,						   /* Acceleration final slope [s/step]. */
	0.0016,						   /* Deceleration final slope [s/step]. */
	0,							   /* Thermal compensation factor (range [0, 15]). */
	3.5 * 1000 * 1.10,			   /* Ocd threshold [ma] (range [375 ma, 6000 ma]). */
	3.5 * 1000 * 1.00,			   /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). */
	StepperMotor::STEP_MODE_HALF, /* Step mode selection. */
	0xFF,						   /* Alarm conditions enable. */
	0x2E88						   /* Ic configuration. */
}};

L6470_init_t initShieldMot2[L6470DAISYCHAINSIZE] = { // init parameters for the motors
/* First Motor.G */
{
	22.0,						    /* Motor supply voltage in V. */
	200,						   /* Min number of steps per revolution for the motor. = 360/1.8° */
	1.0,							   /* Max motor phase voltage in A. /!\ UNUSED - USELESS /!\ */
	3.5,							   /* Max motor phase voltage in V. /!\ UNUSED - USELESS /!\ */
	0,							   /* Motor initial speed [step/s]. Seems logic at 0*/
	1500.0,						   /* Motor acceleration [step/s^2] (comment for infinite acceleration mode).*/
	1500.0,						   /* Motor deceleration [step/s^2] (comment for infinite deceleration mode).*/
	3000,						   /* Motor maximum speed [step/s]. */
	0.0,						   /* Motor minimum speed [step/s].*/
	1500,						   /* Motor full-step speed threshold [step/s]. Limit microstep -> fullstep */
	5.3,							   /* Holding kval [V]. */
	5.3,							   /* Constant speed kval [V]. */
	5.3,							   /* Acceleration starting kval [V]. */
	5.3,							   /* Deceleration starting kval [V]. */
	269.9268,							   /* Intersect speed for bemf compensation curve slope changing [step/s]. */
	0.00072448,						   /* Start slope [s/step]. */
	0.0016,						   /* Acceleration final slope [s/step]. */
	0.0016,						   /* Deceleration final slope [s/step]. */
	0,							   /* Thermal compensation factor (range [0, 15]). */
	3.5 * 1000 * 1.10,			   /* Ocd threshold [ma] (range [375 ma, 6000 ma]). Calculated with Kval*/
	3.5 * 1000 * 1.00,			   /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). Calculated Kval */
	StepperMotor::STEP_MODE_HALF, /* Step mode selection. */
	0xFF,						   /* Alarm conditions enable. */
	0x2E88						   /* Ic configuration. */
},

/* Second Motor. */
{
	22.0,							   /* Motor supply voltage in V. */
	200,						   /* Min number of steps per revolution for the motor. = 360/1.8° */
	1.0,							   /* Max motor phase voltage in A. /!\ UNUSED - USELESS /!\ */
	3.5,							   /* Max motor phase voltage in V. /!\ UNUSED - USELESS /!\ */
	0,							   /* Motor initial speed [step/s]. */
	1500.0,						   /* Motor acceleration [step/s^2] (comment for infinite acceleration mode). */
	1500.0,						   /* Motor deceleration [step/s^2] (comment for infinite deceleration mode). */
	3000,						   /* Motor maximum speed [step/s]. */
	0.0,						   /* Motor minimum speed [step/s]. */
	1500,						   /* Motor full-step speed threshold [step/s]. Limit microstep -> fullstep */
	5.3,							   /* Holding kval [V]. */
	5.3,							   /* Constant speed kval [V]. */
	5.3,							   /* Acceleration starting kval [V]. */
	5.3,							   /* Deceleration starting kval [V]. */
	269.9268,							   /* Intersect speed for bemf compensation curve slope changing [step/s]. */
	0.00072448,						   /* Start slope [s/step]. */
	0.0016,						   /* Acceleration final slope [s/step]. */
	0.0016,						   /* Deceleration final slope [s/step]. */
	0,							   /* Thermal compensation factor (range [0, 15]). */
	3.5 * 1000 * 1.10,			   /* Ocd threshold [ma] (range [375 ma, 6000 ma]). */
	3.5 * 1000 * 1.00,			   /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). */
	StepperMotor::STEP_MODE_HALF, /* Step mode selection. */
	0xFF,						   /* Alarm conditions enable. */
	0x2E88						   /* Ic configuration. */
}};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void send_print(const char* msg);
void send_float(float float_to_send);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  char print_msg[50] = "Hello World\n";
  float tosend1 = 0.1;
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
  bottom_shield = new XNucleoIHM02A1(&initShield[0],&initShield[1],&hspi1,reset_shields_GPIO_Port,reset_shields_Pin,ssel1_GPIO_Port,ssel1_Pin);
  motors = bottom_shield->get_components();
  HAL_Delay(1000);
  motors[0]->prepare_run(StepperMotor::direction_t::FWD, 100);
  motors[1]->prepare_run(StepperMotor::direction_t::BWD, 100);
  bottom_shield->perform_prepared_actions();
  HAL_Delay(1000);
  bottom_shield->perform_prepared_actions();
  HAL_Delay(1000);
  bottom_shield->perform_prepared_actions();
  HAL_Delay(1000);
  bottom_shield->perform_prepared_actions();
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 HAL_Delay(1000);
	//msg_handler.send_print(print_msg);
	 //msg_handler.send_float(tosend1);
	 HAL_Delay(1000);


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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 450000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ssel1_Pin|LD2_Pin|ssel2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(reset_shields_GPIO_Port, reset_shields_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ssel1_Pin LD2_Pin ssel2_Pin */
  GPIO_InitStruct.Pin = ssel1_Pin|LD2_Pin|ssel2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : reset_shields_Pin */
  GPIO_InitStruct.Pin = reset_shields_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(reset_shields_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
	{
		msg_handler.process_txclpt_callback();
	}


}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if( huart == &huart2)
	{
		msg_handler.process_rxclpt_callback();
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim2 )
  {
	  timer_20_ms = true;
	  msg_handler.process_timeout();
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
