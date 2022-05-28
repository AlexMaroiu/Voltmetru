/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "OS.h"
#include "lcd_st7565_pinconf.h"
#include "lcd_st7565.h"
#include "font.h"
#include <stdio.h>
#include <math.h>

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
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

uint8_t state = 0;
uint8_t lastBtnState = 0;
uint8_t btnCounter = 0;
uint8_t lcdState = 0;
uint8_t indexRef = 0;
uint8_t timer2Counter = 0;
uint8_t timer7Counter = 0;
float minRef = 1.f;
float maxRef = 2.5f;

const voltage_show voltage_map [] = {
		{1, "V"},
		{1000, "mV"}
};
int8_t vMapVal = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

void voltmeter(void);
void settings(void);

void drawAnalogVoltmetru(uint16_t adcVal);
void backlightHandler(double voltage);
void drawVoltage(double voltage);
void PA5toSPI(void);
void PA5toOutput(void);
uint8_t lcdConnected(void);
uint8_t pause(uint8_t btn);
void ADC_Joystick(uint32_t rank);
void ADC_Extern(uint32_t rank);
void modify_minRef(float val);
void modify_maxRef(float val);
void modify_Voltage(float val);
uint8_t reset_func(void);
uint16_t readADC(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

const void (*modify_ref[])(float) = {modify_minRef, modify_maxRef, modify_Voltage};
void (*main_func)(void) = settings;

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
  MX_TIM16_Init();
  MX_ADC_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

	//Start timer used for displaying a char on the bottom left corner each second
	HAL_TIM_Base_Start(&htim16);

	// initialize LCD
	st7565_init();
	st7565_backlight_enable();
	HAL_Delay(500);

	ADC_Joystick(ADC_RANK_CHANNEL_NUMBER);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(!lcdConnected())
		{
			continue;
		}

		main_func();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_10B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4800;
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
  htim7.Init.Prescaler = 1000;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 4800;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 4800-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  HAL_GPIO_WritePin(SPICD_GPIO_Port, SPICD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BL_Pin|SPIRST_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPICS_GPIO_Port, SPICS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPICD_Pin */
  GPIO_InitStruct.Pin = SPICD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPICD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BL_Pin SPIRST_Pin PA10 */
  GPIO_InitStruct.Pin = BL_Pin|SPIRST_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPICS_Pin */
  GPIO_InitStruct.Pin = SPICS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPICS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void drawAnalogVoltmetru(uint16_t adcVal)
{
	const int x = 64; //centru
	const int y = 63; //centru
	int x1, x2, x3, y1, y2, y3;
	int angle = 270; //270 stanga, 90 dreapta, 180 mijloc
	int i;

	angle = 270 - 180.0f/1024 * adcVal;
	x1 = x + (40 * sin(angle * M_PI / 180)); //40 e raza
	y1 = y + (40 * cos(angle * M_PI / 180));
	st7565_drawline(buffer, x, y, x1, y1, 1);


	// liniile de arc de cerc
	for(i =0;i <= 180; i+=10)
	{
		x2 = x + (45 * sin((270-i) * M_PI / 180));
		y2 = y + (45 * cos((270-i) * M_PI / 180));
		x3 = x2 + (-5 * sin((270-i) * M_PI / 180));
		y3 = y2 + (-5 * cos((270-i) * M_PI / 180));

		st7565_drawline(buffer, x2, y2, x3, y3, 1);
	}
}

void backlightHandler(double voltage)
{
	if(state != 1 && voltage < minRef)
	{
		HAL_TIM_Base_Stop(&htim2);
		st7565_backlight_disable();
		state = 1;
	}
	else if(state != 2 && voltage >= minRef && voltage < maxRef)
	{
		HAL_TIM_Base_Stop(&htim2);
		st7565_backlight_enable();
		state = 2;
	}
	else if(state != 3 && voltage >= maxRef)
	{
		HAL_TIM_Base_Start_IT(&htim2);
		state = 3;
	}
}

void drawVoltage(double voltage)
{
	char string [10] = {0};

	sprintf(string, "%.1f ", voltage * voltage_map[vMapVal].val);
	strcat(string, voltage_map[vMapVal].string);

	st7565_drawstring(buffer, 50, 0,(uint8_t*)string);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim == &htim2)
	{
		if(timer2Counter++ % 2)
		{
			st7565_backlight_enable();
		}
		else
		{
			st7565_backlight_disable();
		}
	}
	if(htim == &htim7)
	{
		timer7Counter++;
	}
}

void PA5toSPI(void)
{
	GPIOA->MODER &= ~(3<<10);
	GPIOA->MODER |= 2 << 10; // pa5 alternate function

	GPIOA->AFR[0] &= ~(0b1111<<20); // alternate function af0 (spi)

	st7565_init();
	st7565_backlight_enable();
}
void PA5toOutput(void)
{
	st7565_backlight_disable();

	// turn lcd off
	HAL_GPIO_WritePin( SPICD_GPIO_Port, ST7565_A0_PIN, 0 );
	st7565_sendbyte( ST7565_CMD_DISPLAY_OFF );

	GPIOA->MODER &= ~(3<<10);
	GPIOA->MODER |= 1 << 10; // pa5 alternate function

	GPIOA->BSRR |= 1<<5;
}

uint8_t lcdConnected(void)
{
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0)
	{
		if(lcdState != 0)
		{
			PA5toSPI();
			lcdState = 0;
		}
		return 1;
	}
	else
	{
		if(lcdState != 1)
		{
			PA5toOutput();
			HAL_TIM_Base_Stop_IT(&htim2);
			state = 0;
			lcdState = 1;
		}
		return 0;
	}
}

uint8_t pause(uint8_t btn)
{
	if(0 == btn && lastBtnState)
	{
		btnCounter++;
		if(reset_func())
			return 1;
	}

	lastBtnState = btn;

	return btnCounter % 2;
}

uint16_t readADC(void)
{
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
	return HAL_ADC_GetValue(&hadc);
}

void voltmeter(void)
{
	if(pause(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)))
	{
		return;
	}

	uint16_t adcValue = readADC();
	double voltageV = adcValue * 0.003515625;	 // 3.3v / 2^10 = 0.00322265625

	backlightHandler(voltageV);

	st7565_clear_buffer(buffer);
	drawVoltage(voltageV);
	drawAnalogVoltmetru(adcValue);
	st7565_write_buffer(buffer);

	HAL_Delay(500);
}

void ADC_Joystick(uint32_t rank)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = rank;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}
void ADC_Extern(uint32_t rank)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = rank;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void modify_minRef(float val)
{
	val /= 5.f;
	if(maxRef - 0.3 <= minRef + val || minRef + val < 0.1)
	{
		return;
	}
	minRef += val;
}

void modify_maxRef(float val)
{
	val /= 5.f;
	if(minRef + 0.3 >= maxRef + val || maxRef + val >= 3.5)
	{
		return;
	}
	maxRef += val;
}

void modify_Voltage(float val)
{
	if(vMapVal + val < 0)
		vMapVal = 1;
	else
		vMapVal = (vMapVal + (int)val) % 2;
}

void settings(void)
{
	char string [10] = {0};
	st7565_clear_buffer(buffer);

	st7565_drawstring(buffer, 0, 0,(uint8_t*)"Stand-by voltage:");
	sprintf(string, "%.1f", minRef);
	st7565_drawstring(buffer, 10, 1, (uint8_t*)string);

	st7565_drawstring(buffer, 0, 2, (uint8_t*)"Blink voltage:");
	sprintf(string, "%.1f", maxRef);
	st7565_drawstring(buffer, 10, 3, (uint8_t*)string);

	st7565_drawstring(buffer, 0, 4, (uint8_t*)"Voltage display:");
	st7565_drawstring(buffer, 10, 5, (uint8_t*)voltage_map[vMapVal].string);

	st7565_drawstring(buffer, 0, (indexRef%3)*2+1, (uint8_t*)">");

	uint16_t joystick =  readADC();

	//sprintf(string, "%d", joystick);
	//st7565_drawstring(buffer, 0, 6, (uint8_t*)string);

	if(joystick < 50)// stanga 0
	{
		modify_ref[indexRef%3](-1.f);
	}
	else if(joystick < 350)// click joystick 300
	{
		ADC_Joystick(ADC_RANK_NONE);
		ADC_Extern(ADC_RANK_CHANNEL_NUMBER);
		main_func = voltmeter;
	}
	else if(joystick < 650)// jos 600
	{
		indexRef++;
	}
	else if(joystick < 950)// dreapta 930
	{
		modify_ref[indexRef%3](1.f);
	}

	st7565_write_buffer(buffer);

	HAL_Delay(150);
}

uint8_t reset_func()
{
	if(!timer7Counter || timer7Counter > 10)
	{
		timer7Counter = 0;
		HAL_TIM_Base_Start_IT(&htim7);
		return 0;
	}
	else// if(timer7Counter <= 10)
	{
		btnCounter = 0;
		timer7Counter = 0;
		state = 0;
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Stop_IT(&htim7);
		st7565_backlight_enable();
		main_func = settings;
		ADC_Extern(ADC_RANK_NONE);
		ADC_Joystick(ADC_RANK_CHANNEL_NUMBER);
		return 1;
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

