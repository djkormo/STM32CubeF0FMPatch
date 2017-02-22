/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "resources.h"
#include "algorithm.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint8_t countererror = 0;
volatile uint8_t counter = 0;
volatile uint16_t counteradc = 0;
volatile uint16_t counterdac = 0;

volatile uint16_t lutindex = 0;
volatile uint16_t value_dac = 0;
volatile uint16_t ADC_rawold[3];
volatile uint16_t ADC_raw[3];
volatile uint16_t ADCConv1, ADCConv2, ADCConv3;
volatile uint16_t ADCConv1old, ADCConv2old, ADCConv3old;

volatile uint8_t index = 0;

volatile uint16_t counttimer1 = 0;
volatile uint16_t counttimer2 = 0;

volatile uint32_t R = 5000000;
// 1st sine
volatile uint32_t accumulator1 = 0;
volatile uint16_t accumulator1angle = 0;
volatile uint16_t accumulator1step = 0;

volatile uint32_t accumulator1r = 5000000;
volatile double VoltValue1 = 0.0;

//2nd sine
volatile uint32_t accumulator2 = 0;
volatile uint16_t accumulator2angle = 0;
volatile uint16_t accumulator2step = 0;
volatile uint32_t accumulator2r=5000000;
volatile double VoltValue2 = 0.0;

//3rd sine
volatile uint32_t accumulator3 = 0;
volatile uint16_t accumulator3angle = 0;
volatile uint16_t accumulator3step = 0;
volatile uint32_t accumulator3r=5000000;
volatile double VoltValue3 = 0.0;

int useLeds = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_ADC_Init();

  MX_DAC_Init();

  MX_TIM6_Init();
  // something wrong above in Tim6

 // MX_TIM7_Init(); -- to check



  /* USER CODE BEGIN 2 */

	if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}




	if (HAL_DAC_Start(&hdac, DAC_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	if (useLeds)
	{
		HAL_GPIO_TogglePin(GPIOC,LD4_Pin); //Orange
		HAL_GPIO_TogglePin(GPIOC, LD5_Pin);//Green
	}
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  counter++;
	  //HAL_GPIO_Pin(GPIOC,LD4_Pin); //Orange
	 // HAL_Delay(1000);
	  counter++;
	 // HAL_GPIO_TogglePin(GPIOC, LD5_Pin); //Green
	  //HAL_Delay(1000);

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/


void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */

  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler =ADC_CLOCK_SYNC_PCLK_DIV4;//	was	ADC_CLOCK_ASYNC_DIV1;;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_14;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

	/* Set ADC callback */

	HAL_ADC_ConvCpltCallback(&hadc);

	/* ADC Calibration */

	if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_ADC_Start_IT(&hadc) != HAL_OK) {
		Error_Handler();
	}

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer =  DAC_OUTPUTBUFFER_DISABLE; // was DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

    /**DAC channel OUT2 config 
    */

  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

}

void ADC_IRQHandler() {
	HAL_ADC_IRQHandler(&hadc);

	//HAL_ADC_IRQHandler(&hadc2); <--- In case of a second ADC
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

	if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC)) {
		ADC_rawold[index]=ADC_raw[index];
		ADC_raw[index] = HAL_ADC_GetValue(hadc);
		index++;
		//counteradc++;
	}

	if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOS)) {
		index = 0;
		counteradc++;
		if (counteradc%10000==0)
		{
			if (useLeds)
			{
				HAL_GPIO_TogglePin(GPIOC, LD3_Pin); // RED
			}
		}
		ADCConv1 = (int) 0.01*ADC_raw[0]+0.99*ADC_rawold[0];
		ADCConv2 = (int) 0.01*ADC_raw[1]+0.99*ADC_rawold[1];
		ADCConv3 = (int) 0.01*ADC_raw[2]+0.99*ADC_rawold[2];
		//HAL_GPIO_TogglePin(GPIOC, LD5_Pin);
		/*filtr <- 0.1*nowa_próbka_z_adc + 0.9*filtr; wyjœcie <- filtr;*/

		accumulator1r = (uint32_t) rangeScaleLinear(ADCConv1, 0, 4095, 10,
						5000);

				accumulator2r = (uint32_t) rangeScaleLinear(ADCConv2, 0, 4095, 10,
						5000);

				accumulator3r = (uint32_t) rangeScaleLinear(ADCConv3, 0, 4095, 10,
						5000);

	}

}

void TIM6_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 50;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10;
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

  // Turn On TIM6 with Interupts


  	HAL_NVIC_SetPriority(TIM6_IRQn, 0, 0);

  	HAL_NVIC_EnableIRQ(TIM6_IRQn);


 	if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
  		Error_Handler();
  	}

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 10;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10;
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

  // Turn On TIM6 with Interupts


    	HAL_NVIC_SetPriority(TIM7_IRQn, 3, 2);

    	HAL_NVIC_EnableIRQ(TIM7_IRQn);


   	if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK) {
    		Error_Handler();
    	}

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> TSC_G1_IO3
     PA3   ------> TSC_G1_IO4
     PA6   ------> TSC_G2_IO3
     PA7   ------> TSC_G2_IO4
     PB0   ------> TSC_G3_IO2
     PB1   ------> TSC_G3_IO3
     PB10   ------> I2C2_SCL
     PB11   ------> I2C2_SDA
     PB13   ------> SPI2_SCK
     PB14   ------> SPI2_MISO
     PB15   ------> SPI2_MOSI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin 
                          |LD4_Pin|LD5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin EXT_RESET_Pin LD3_Pin LD6_Pin 
                           LD4_Pin LD5_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin 
                          |LD4_Pin|LD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C2_SCL_Pin I2C2_SDA_Pin */
  GPIO_InitStruct.Pin = I2C2_SCL_Pin|I2C2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_SCK_Pin SPI2_MISO_Pin SPI2_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI2_SCK_Pin|SPI2_MISO_Pin|SPI2_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) //check if the interrupt comes from TIM6

	{

		counterdac++;
		if (counterdac%1000==0)
				{
					HAL_GPIO_TogglePin(GPIOC, LD6_Pin); // Blue
				}

		/*
		lutindex+=12;
		if (lutindex>=1024)
		{
			lutindex-=1024;
		}
		*/

		/*
		accumulator1+=accumulator1r;
		accumulator1angle=(uint16_t)(accumulator1>>22);
		accumulator1step = Sine1024_12bit[accumulator1angle];


		accumulator2+=accumulator2r;
		accumulator2angle=(uint16_t)(accumulator2>>22);
		accumulator2step = Sine1024_12bit[accumulator2angle];

		accumulator3+=accumulator3r;
		accumulator3angle=(uint16_t)(accumulator3>>22);
		accumulator3step = Sine1024_12bit[accumulator3angle];
		value_dac=(uint16_t) ((accumulator1step)+(accumulator1step));
		//value_dac=(uint16_t)(value_dac)*0.5;
		//value_dac=(uint16_t)(0.33)*(accumulator1step+accumulator1step+accumulator1step);
		*/


		lutindex+=10;
				if (lutindex>=1024)
				{
					lutindex-=1024;
				}
		value_dac	=Sine1024_12bit[lutindex];
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value_dac);
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, value_dac);
	}

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
    HAL_GPIO_WritePin(GPIOC, LD5_Pin,GPIO_PIN_SET); // Green

  while(1) 
  {
	  countererror++;
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
