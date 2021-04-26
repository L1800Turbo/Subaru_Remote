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

/*
 * TODO:
 * - Sendefunktion:
 	47kHz-Timer
	Interrupt für Pin-Recv deaktivieren
	Tastereingang zum Senden
	ir als 2. Sendestruct nehmen, state machine genau so durchlaufen lassen
 *
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define IR_Test_LOW  HAL_GPIO_WritePin(IR_Testpin_GPIO_Port, IR_Testpin_Pin, GPIO_PIN_RESET)
#define IR_Test_HIGH HAL_GPIO_WritePin(IR_Testpin_GPIO_Port, IR_Testpin_Pin, GPIO_PIN_SET)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

typedef struct
{
	uint8_t id_0:4;
	uint8_t counter_0:4;
	uint8_t id_1:4;
	uint8_t counter_1:4;
	uint8_t id_2:4;
	uint8_t counter_2:4;
	uint8_t id_3:4;
	uint8_t counter_3:4;
	uint8_t id_4:4;
	uint8_t none:4; // nur zum auffuellen auf ganzes Byte
}ir_msg_dt;

typedef struct
{
	uint32_t startBit_Pulse;
	uint32_t startBit_Pause;
	uint32_t pulse;
	uint32_t bit1_Pause;
	uint32_t bit0_Pause;

	uint8_t bitLength; // Wie viele Bit breit ist die Botschaft

	char name[10];
}ir_config_dt;

typedef enum
{
	IR_LEGACY_Ia = 0,
	IR_LEGACY_Ib,
	IR_SVX,

	IR_CONFIGS_SIZE
}ir_configs_en;

ir_config_dt irConfigs[IR_CONFIGS_SIZE] =
{
		{.startBit_Pulse = 2000, .startBit_Pause = 12784, .pulse = 650, .bit1_Pause = 1800, .bit0_Pause = 845, .bitLength = 36, .name = "LEGACY a"}, /* IR_LEGACY_Ia */
		{.startBit_Pulse = 4137, .startBit_Pause =  7705, .pulse = 590, .bit1_Pause = 1530, .bit0_Pause = 590, .bitLength = 36, .name = "LEGACY b"}, /* IR_LEGACY_Ib */
		{.startBit_Pulse = 4200, .startBit_Pause =  8800, .pulse = 650, .bit1_Pause = 1650, .bit0_Pause = 555, .bitLength = 36, .name = "SVX     "}, /* IR_SVX       */
}; // TODO: zu gleiche Werte!

struct ir
{
	volatile uint8_t currentBit; // Auf welches Bit schreibt er gerade?

	enum states
	{
		IR_NONE = 0,
		IR_START_PULSE,
		IR_START_PAUSE,
		IR_DATA_STROBE,
		IR_DATA_PAUSE
	}state;

	ir_config_dt * config;
	ir_configs_en curCfg;
	uint8_t threshold;

	uint8_t buf[5];
	uint8_t byteCounter;
	uint8_t newMsg;

}ir;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *data, int len)
{
   /*if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
   {
      errno = EBADF;
      return -1;
   }*/

   // arbitrary timeout 1000
   HAL_StatusTypeDef status =
      HAL_UART_Transmit(&huart2, (uint8_t*)data, len, 1000);

   // return # of bytes written - as best we can tell
   return (status == HAL_OK ? len : 0);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // Für Timeouts
{
	if(htim == &htim10)
	{
		HAL_GPIO_TogglePin(Testpin_GPIO_Port, Testpin_Pin);
		if(ir.state != IR_NONE)
		{
			printf("Timeout\r\n");
			ir.state = IR_NONE;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef * htim)
{
	IR_Test_LOW;
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		__HAL_TIM_SET_COUNTER(htim, 0); // Counter wieder auf 0 setzen

		uint32_t currentDelta = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);// - ir.lastTime;
		uint32_t currentThreshold = (uint32_t) (currentDelta * (float) ir.threshold/100);

		//ir.lastTime = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);


		switch(ir.state)
		{
			case IR_NONE: // erste fallende Flanke
//				IR_Test_LOW;

				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

				//printf("First Flag...\r\n");
				ir.state++;
				break;

			case IR_START_PULSE: // Steigende Flanke, Ende Start-Bit
//				IR_Test_HIGH;

				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);

				// Nach Konfiguration suchen
				ir.curCfg = IR_CONFIGS_SIZE;
				for(ir_configs_en i=0; i<IR_CONFIGS_SIZE; i++)
				{
					if(currentDelta > (ir.config[i].startBit_Pulse - currentThreshold) && currentDelta < (ir.config[i].startBit_Pulse + currentThreshold))
					{
						ir.curCfg = i;
						break;
					}
				}

				if(ir.curCfg < IR_CONFIGS_SIZE) // Wenn er die Config gefunden hat
				{
					ir.state = IR_START_PAUSE;
				}
				else
				{
					ir.state = IR_NONE;
					//printf("Start Pulse wrong (got %lu), resetting...\r\n", currentDelta);
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
				}
				break;

			case IR_START_PAUSE: // Fallende Flanke, Ende Start-Pause
				// TODO: Eigentlich müsste er hier auch nochmal auf die Config prüfen...
				//IR_Test_LOW;

				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

				if(currentDelta > (ir.config[ir.curCfg].startBit_Pause - currentThreshold) && currentDelta < (ir.config[ir.curCfg].startBit_Pause + currentThreshold))
				{
					ir.currentBit = 0;
					ir.state++;
				}
				else
				{
					ir.state = IR_NONE;
					//printf("Start Pause wrong (got %lu), resetting...\r\n", currentDelta);
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
				}
				break;

			case IR_DATA_STROBE: // Steigende Flanke, Ende eines Strobes
				//IR_Test_HIGH;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);

				if(currentDelta > (ir.config[ir.curCfg].pulse - currentThreshold) && currentDelta < (ir.config[ir.curCfg].pulse + currentThreshold))
				{
					if(ir.currentBit >= ir.config[ir.curCfg].bitLength) // Wenn alle Bits empfangen wurden zurücksetzen
					{
						ir.state = IR_NONE;
						ir.currentBit = 0;
						ir.byteCounter = 0;

						ir.newMsg = 1;
						//__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
					}
					else
					{
						ir.state++;
					}
				}
				else
				{
					ir.state = IR_NONE;
					//printf("Strobe wrong, resetting...\r\n");
					//__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
				}

				break;

			case IR_DATA_PAUSE: // Fallende Flanke nach 1- oder 0-Pause
				//IR_Test_LOW;

				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

				if(currentDelta > (ir.config[ir.curCfg].bit0_Pause - currentThreshold) && currentDelta < (ir.config[ir.curCfg].bit0_Pause + currentThreshold))
				{
					ir.buf[ir.byteCounter] |= 0 << (/*7-*/(ir.currentBit % 8));
					//printf("0");
					ir.currentBit++;

					ir.state = IR_DATA_STROBE;
				}
				else if(currentDelta > (ir.config[ir.curCfg].bit1_Pause - currentThreshold) && currentDelta < (ir.config[ir.curCfg].bit1_Pause + currentThreshold))
				{
					ir.buf[ir.byteCounter] |= 1 << (/*7-*/(ir.currentBit % 8));
					//printf("1");
					ir.currentBit++;

					ir.state = IR_DATA_STROBE;
				}
				else
				{
					ir.state = IR_NONE;
					//printf("Wrong Pause at tbd., resetting...\r\n"); // TODO: Welches Bit?
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
				}

				if((ir.currentBit % 8) == 0)
				{
					ir.byteCounter++;
				}
				break;
		}
	}
	IR_Test_HIGH;
}

void IR_Init(void)
{
	ir.state = IR_NONE;

	ir.currentBit = 0;
	ir.byteCounter = 0;
	memset(ir.buf, 0, 5);

	ir.newMsg = 0;

	ir.config = irConfigs;
	ir.curCfg = IR_CONFIGS_SIZE; // Als ungültig

	ir.threshold 		=    30; // %

	IR_Test_HIGH;
	__HAL_TIM_SET_CAPTUREPOLARITY(&htim10, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);

	// Timer starten
	HAL_TIM_IC_Start_IT(&htim10, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim10);
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
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

  IR_Init();
	
  // Definieren einer Sendenachricht:
  ir_msg_dt sendMsg;

  sendMsg.id_0 = 0x1;
  sendMsg.id_1 = 0x3;
  sendMsg.id_2 = 0xB;
  sendMsg.id_3 = 0x0;
  sendMsg.id_4 = 0x0;

  sendMsg.counter_0 = 0x0;
  sendMsg.counter_1 = 0x0;
  sendMsg.counter_2 = 0x0;
  sendMsg.counter_3 = 0x1;
	
  ir_configs_en currentSendCfg = IR_LEGACY_I;

  printf("IR-Reader v0.1\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(ir.newMsg == 1)
	  {
		  ir_msg_dt msg;
		  
		  ir.newMsg = 0;
		  
		  memcpy(msg, ir.buf, sizeof(ir_msg_dt));

		  IR_Test_LOW;
		  //printf("%02X%02X%02X%02X%X\r\n", ir.buf[0], ir.buf[1], ir.buf[2], ir.buf[3], ir.buf[4]/*>>4*/&0x04);
		  printf(" %X%X%X%X%X %X%X%X%X %s\r\n", ir.buf[0]&0xF, ir.buf[1]&0xF, ir.buf[2]&0xF, ir.buf[3]&0xF, ir.buf[4]&0xF,
				  	  	  	  	  	  	  	  (ir.buf[0]>>4)&0xF, (ir.buf[1]>>4)&0xF, (ir.buf[2]>>4)&0xF, (ir.buf[3]>>4)&0xF,
											  ir.config[ir.curCfg].name);
		  IR_Test_HIGH;

		  memset(ir.buf, 0 , 5);

		  ir.curCfg = IR_CONFIGS_SIZE;
	  }
	  /*
	  if(Schalter.... && ir_send.state == IR_NONE) Schalter
	  {
		ir_send.state = IR_START_PAUSE;
		  // TODO Strobe 47kHz an...
		__HAL_TIM_SET_COUNTER(htim, 0); // Counter wieder auf 0 setzen
	  }
		  
	  switch(ir_send.state)
	  {
		case IR_NONE: 
			  // nichts tun...
			break;
			  
		  case IR_START_PULSE: // am Ende eines Startpulses...
			  if(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) > ir_send.config[currentSendCfg].startBit_Pulse)
			  {
				  // TODO Strobe 47kHz aus...
				  
				  ir_send.state = IR_START_PAUSE;
				  __HAL_TIM_SET_COUNTER(htim, 0); // Counter wieder auf 0 setzen
			  }
			  break;
		  case IR_START_PAUSE:
			if(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) > ir_send.config[currentSendCfg].startBit_Pause)
			{
				// TODO Strobe 47kHz an...
				
				ir_send.state = IR_DATA_STROBE;
				__HAL_TIM_SET_COUNTER(htim, 0); // Counter wieder auf 0 setzen
			}
			  break;
			  
		case IR_DATA_STROBE:
			  if(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) > ir_send.config[currentSendCfg].pulse)
			  {
				  // TODO Strobe 47kHz aus...
				  
				  
			  }
			  break;
			  
		case IR_DATA_PAUSE:
			  // Bits durchgehen
			  
			  break;
			  
			  //....

			  // Wenn alles durch ist: 				  sendMsg.counter_3 ++; // TODO nur bis F!!
	  }*/
	  
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 84-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65536-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim10, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Testpin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IR_Testpin_GPIO_Port, IR_Testpin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Testpin_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Testpin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_Testpin_Pin */
  GPIO_InitStruct.Pin = IR_Testpin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IR_Testpin_GPIO_Port, &GPIO_InitStruct);

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
