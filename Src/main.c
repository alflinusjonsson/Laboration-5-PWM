
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

typedef enum {PRESSED, nonPRESSED} state_button;
typedef enum {RED, REDWAIT, GREEN, GREENWAIT, BLUE, BLUEWAIT, WHITE, WHITEWAIT,
YELLOW, YELLOWWAIT, LIGHTBLUE, LIGHTBLUEWAIT, PURPLE, PURPLEWAIT, OFF, OFFWAIT,
MAGNETA, MAGNETAWAIT, ORANGE, ORANGEWAIT, GREY, GREYWAIT, MAROON, MAROONWAIT,
PINK, PINKWAIT} state_led;

static state_led ledState = OFF;
static state_button s_button = nonPRESSED;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void changePulse1(uint32_t pulse);
void changePulse2(uint32_t pulse);
void changePulse3(uint32_t pulse);
void walkingLed();
                                
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void walkingLed(){

	switch(ledState)
	{
		    case OFF:
		    	changePulse1(0);
		    	changePulse2(0);
		    	changePulse3(0);
		    	ledState = OFFWAIT;
		    	break;

		    case OFFWAIT:
		    	if(s_button == PRESSED)
		    	{
		    		ledState = RED;
		    		s_button = nonPRESSED;
		    	}
		    	break;

		    case RED:
		    	changePulse1(255);
		    	changePulse2(0);
		    	changePulse3(0);
		    	ledState = REDWAIT;
		   		break;

		    case REDWAIT:
		    	if(s_button == PRESSED)
		    	{
		    		ledState = GREEN;
		    		s_button = nonPRESSED;
		    	}
		   		break;

		   	case GREEN:
		   		changePulse1(0);
		   		changePulse2(255);
		   		changePulse3(0);
		   		ledState = GREENWAIT;
		  		break;

		    case GREENWAIT:
		    	if(s_button == PRESSED)
		    	{
		    		ledState = BLUE;
		    		s_button = nonPRESSED;
		    	}
		    	break;

		    case BLUE:
		    	changePulse1(0);
		    	changePulse2(0);
		    	changePulse3(255);
		    	ledState = BLUEWAIT;
		 		break;

		    case BLUEWAIT:
		    	if(s_button == PRESSED)
		    	{
		    		ledState = WHITE;
		    		s_button = nonPRESSED;
		    	}
		    	break;

		    case WHITE:
		    	changePulse1(0);
		    	changePulse2(0);
		    	changePulse3(0);
		    	ledState = WHITEWAIT;
		    	break;

		    case WHITEWAIT:
		    	if(s_button == PRESSED)
		    	{
		    		ledState = YELLOW;
		    		s_button = nonPRESSED;
		    	}
		   		break;

		   	case YELLOW:
		   		changePulse1(255);
		   		changePulse2(255);
		   		changePulse3(0);
		   		ledState = YELLOWWAIT;
		    	break;

		    case YELLOWWAIT:
		    	if(s_button == PRESSED)
		    	{
		    		ledState = LIGHTBLUE;
		    		s_button = nonPRESSED;
		    	}
		    	break;

		    case LIGHTBLUE:
		    	changePulse1(0);
		    	changePulse2(255);
		    	changePulse3(255);


		    	ledState = LIGHTBLUEWAIT;
		    	break;

		    case LIGHTBLUEWAIT:
		    	if(s_button == PRESSED)
		    	{
		    		ledState = PURPLE;
		    		s_button = nonPRESSED;
		    	}
		    	break;

		    case PURPLE:
		    	changePulse1(128);
		    	changePulse2(0);
		    	changePulse3(128);
		    	ledState = PURPLEWAIT;
		    	break;

		    case PURPLEWAIT:
		    	if(s_button == PRESSED)
		    	{
		    		ledState = OFF;
		    		s_button = nonPRESSED;
		    	}
		    	break;

		    case MAGNETAWAIT:
		   	    	if(s_button == PRESSED)
		   	    	{
		   	    		ledState = MAGNETA;
		   	    		s_button = nonPRESSED;
		   	    	}
		   	    	break;

		    case MAGNETA:
		   	    	changePulse1(255);
		   	    	changePulse2(0);
		   	    	changePulse3(255);
		   	    	ledState = ORANGEWAIT;
		   	    	break;

		    case ORANGEWAIT:
		    	 if(s_button == PRESSED)
		    	 {
		  	    	ledState = ORANGE;
		    	   	 s_button = nonPRESSED;
		    	 }
		    	   	break;

		    case ORANGE:
		    	 changePulse1(255);
		    	 changePulse2(165);
		    	 changePulse3(0);
		    	 ledState = GREYWAIT;
		    	 break;

		    case GREYWAIT:
		    	 if(s_button == PRESSED)
		    	  {
		    	  	  ledState = GREY;
		    	      s_button = nonPRESSED;
		    	  }
		    	  break;

		    case GREY:
		    	  changePulse1(128);
		    	  changePulse2(128);
		    	  changePulse3(128);
		    	  ledState = MAROONWAIT;
		    	  	  break;

		    case MAROONWAIT:
		    	 if(s_button == PRESSED)
		    	 {
		    	  	  ledState = MAROON;
		    	      s_button = nonPRESSED;
		    	 }
		       	  break;

		    case MAROON:
		   	    changePulse1(128);
		   	    changePulse2(0);
		       	changePulse3(0);
		       	ledState = PINKWAIT;
		       		break;

		    case PINKWAIT:
		   	    if(s_button == PRESSED)
		   	    {
		   	    	ledState = PINK;
		   	    	s_button = nonPRESSED;
		   	    }
		   	       	break;

		    case PINK:
		    	changePulse1(255);
		    	changePulse2(192);
		    	changePulse3(203);
		    	ledState = OFF;
		    	  break;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  while (1)
  {
	  walkingLed();

  }


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = PRESCALER;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = PERIOD;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B2_Pin */
  GPIO_InitStruct.Pin = B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(!HAL_GPIO_ReadPin(B2_GPIO_Port, B2_Pin)){
		s_button = PRESSED;
	}
}

void changePulse1(uint32_t pulse){

	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 255 - pulse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

void changePulse2(uint32_t pulse){

	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 255 - pulse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}

void changePulse3(uint32_t pulse){

	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 255 - pulse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
