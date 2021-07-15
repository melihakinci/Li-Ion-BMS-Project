/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include "oled.h"
#include "adc.h"
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
DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef enum {
	CHG_ENABLE = 1,
	CHG_DISABLE = 0
} CHG_EN;

typedef enum {
	IDLE = 0,
	CHG = 1,
	DCHG = 2
} STATE;

STATE currentState = IDLE;
uint8_t currentDchgPct = 0;
float lastReadBattV = 0;
float lastReadTemp=0;
float lastCurrent=0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
void Charging_Enable(CHG_EN chg_en);
void Change_State(STATE new_state);
void Discharging_Set(uint8_t pct);
void Safety_Loop();
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  //HAL_TIM_Base_Start_IT(&htim6);

  //İnitialize OLED DISPLAY
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(30,0);
  ssd1306_WriteString("Li-ion BMS",Font_7x10,White);
  //ssd1306_UpdateScreen();

  //İnitialize ADC
  LTC2990_ConfigureControlReg(&hi2c1);

  //initialize timer7
  HAL_TIM_Base_Start_IT(&htim7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  //Trigger a new conversion
	  LTC2990_Trigger(&hi2c1);
	  LTC2990_WaitForConversion(&hi2c1,50);

	  //Quick Test -Read vcc

	  float voltageADCVcc=0;
	  LTC2990_ReadVoltage(&hi2c1, VCC, &voltageADCVcc);

	  //current reading
	  float current=0;
	  float battV=0;
	  float battV_2=0;
	  LTC2990_ReadVoltage(&hi2c1, BATTV, &battV);
	  LTC2990_ReadVoltage(&hi2c1, BATTV_2, &battV_2);

	  LTC2990_ReadCurrent(&hi2c1, battV, battV_2, &current);

	  //temp reading
	  	  float temp=0;
	  	  LTC2990_ReadTemperature(&hi2c1, &temp);

	  	  char tempString[10];
	  	  sprintf(tempString, "%.3f C" ,temp);
	  	  ssd1306_SetCursor(2,40);
	  	  ssd1306_WriteString("Temp   ",Font_7x10,White);
	  	  ssd1306_SetCursor(43,40);
	  	  ssd1306_WriteString(tempString,Font_7x10,White);

	  //display adc on oled
	  char voltageADCVccString[10];
	  sprintf(voltageADCVccString, "%.3f  V" ,voltageADCVcc);
	  ssd1306_SetCursor(2,10);
	  ssd1306_WriteString("Vcc   ",Font_7x10,White);
	  ssd1306_SetCursor(43,10);
	  ssd1306_WriteString(voltageADCVccString,Font_7x10,White);

	//show current on oled
	   char currentString[10];
	   sprintf(currentString, "%.3f mA" ,current);
	   ssd1306_SetCursor(2,20);
	   ssd1306_WriteString("Cur   ",Font_7x10,White);
	   ssd1306_SetCursor(43,20);
	   ssd1306_WriteString(currentString,Font_7x10,White);

	//battery voltage
	   char battVString[10];
	   sprintf(battVString, "%.3f  V" ,battV);
	   ssd1306_SetCursor(2,30);
	   ssd1306_WriteString("BattV   ",Font_7x10,White);
	   ssd1306_SetCursor(43,30);
	   ssd1306_WriteString(battVString,Font_7x10,White);

	   //SoC estimation

	   char socString[10];
	   sprintf(socString, "%.1f %" ,(battV/3.7)*100);
	  	   ssd1306_SetCursor(2,50);
	  	   ssd1306_WriteString("SoC   ",Font_7x10,White);
	  	   ssd1306_SetCursor(43,50);
	  	   ssd1306_WriteString(socString,Font_7x10,White);

	   //update last battV to safety
	   lastReadBattV =battV;
	   //update last temp to safety
	   lastReadTemp=temp;
	   //update last current to to safety
	   lastCurrent=current;


	   ssd1306_UpdateScreen();

	   HAL_Delay(1000); //update rate
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 36000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 36000;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 2000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED_USR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CHG_EN_Pin|LED_USR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LED_USR2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LED_USR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CHG_EN_Pin LED_USR1_Pin */
  GPIO_InitStruct.Pin = CHG_EN_Pin|LED_USR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : S2_INTERRUPT_Pin S1_INTERRUPT_Pin */
  GPIO_InitStruct.Pin = S2_INTERRUPT_Pin|S1_INTERRUPT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim6)
	{
		HAL_GPIO_TogglePin(LED_USR1_GPIO_Port, LED_USR1_Pin); //toggle the led1 every 0.5 second



		//if both button pressed change the mode
		if(HAL_GPIO_ReadPin(GPIOA, S1_INTERRUPT_Pin) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(GPIOA, S2_INTERRUPT_Pin) == GPIO_PIN_RESET)
		{
			//change mode
			if(currentState == IDLE)
			{
				Change_State(CHG);

			}
			else if(currentState == CHG)
			{
				Change_State(DCHG);
			}
			else if(currentState==DCHG)
			{
				Change_State(IDLE);
			}

			else
			{
				Change_State(IDLE);
			}



		}

		//if one of the switch is pressed change discharge current
		else if(currentState == DCHG && HAL_GPIO_ReadPin(GPIOA, S1_INTERRUPT_Pin) == GPIO_PIN_RESET)
		{
			//increase discharge current
			uint8_t newDchgPct = currentDchgPct +10;
			Discharging_Set(newDchgPct);
		}
		else if(currentState == DCHG && HAL_GPIO_ReadPin(GPIOA, S2_INTERRUPT_Pin) == GPIO_PIN_RESET)
		{
			//decrease discharge current
			uint8_t newDchgPct = currentDchgPct -10;
			Discharging_Set(newDchgPct);
		}



		HAL_TIM_Base_Stop_IT(&htim6);
	}
	else if(htim==&htim7)
	{
		Safety_Loop();
	}



}
void  HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	 if(GPIO_Pin == S2_INTERRUPT_Pin || GPIO_Pin == S1_INTERRUPT_Pin)
	 {
		 //Enable charging

		 //
		 HAL_TIM_Base_Start_IT(&htim6);
	 }
}

//Where PCT should be 0 - 100
void Discharging_Set(uint8_t pct) {
	if (pct < 0) {
		pct = 0;
	} else if (pct > 100) {
		pct = 100;
	}
	//DAC is 12 bit resolution - 0 - 4095 data codes which translates to 0 - 3.3V analog

	uint32_t dacCode = (uint32_t) ((pct / 100.0) * 4095.0);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacCode);

	currentDchgPct = pct;

	//Start DAC
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
}

void Charging_Enable(CHG_EN chg_en)
{
	if (chg_en == CHG_ENABLE)
	{
		HAL_GPIO_WritePin(GPIOB, CHG_EN_Pin, GPIO_PIN_SET);
	}
	else //Disabling charging
	{
		HAL_GPIO_WritePin(GPIOB, CHG_EN_Pin, GPIO_PIN_RESET);
	}
}

void Change_State(STATE new_state) {

	currentState = new_state;

	//IDLE
	if (currentState == IDLE) {
		Charging_Enable(CHG_DISABLE);
		Discharging_Set(0); //Set discharge current to 0A
	}
	//CHARGING
	else if (currentState == CHG) {
		Charging_Enable(CHG_ENABLE);
		Discharging_Set(0); //Set discharge current to 0A
	}
	//DISCHARGING
	else if (currentState == DCHG) {
		Charging_Enable(CHG_DISABLE);
		Discharging_Set(10); //Set discharge current to 10%
	} else {
		//HANDLE DEFAULT CASE - MISRA C
	}
}
void Safety_Loop()
{
	//undervoltage
	if(lastReadBattV < 2.9)
	{
		//ssd1306_SetCursor(30,10);
		//ssd1306_WriteString("Under Voltage",Font_7x10,White);
		Change_State(IDLE);
	}
	//overvoltage
	else if(lastReadBattV > 5.5)
		{
			//ssd1306_SetCursor(30,10);
			//ssd1306_WriteString("Over Voltage",Font_7x10,White);
			Change_State(IDLE);

		}
	//overtemprature

	else if(lastReadTemp > 70.0)
	{
		//ssd1306_SetCursor(30,10);
		//ssd1306_WriteString("Over Temperature",Font_7x10,White);
		Change_State(IDLE);
	}

	//overcurrent
	else if(lastCurrent > 1625)
		{
			//ssd1306_SetCursor(30,10);
			//ssd1306_WriteString("Over Current",Font_7x10,White);
			Change_State(IDLE);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
