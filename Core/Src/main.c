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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dshot.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_ENABLED 0
#define DSHOT_ENABLED 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t timerTicks = 0;
volatile uint8_t timerFlag250 = 0;
volatile uint8_t timerFlag500 = 0;
volatile uint8_t timerFlag1000 = 0;
volatile uint8_t timerFlag2000 = 0;

volatile uint8_t uartRxFlag = 0;
volatile uint8_t uartRxFreqFlag = 0;
volatile uint8_t uartRxDshotPacketFlag = 0;
uint8_t uartRxBuffer[1] = {0};

volatile uint16_t pwmDutyCycle = 50;
volatile uint16_t pwmFrequency = 1000;

volatile float frequency = 0;
volatile float dutyCycle = 0;
volatile uint32_t period = 0;
volatile uint32_t highTime = 0;
//static uint32_t period = 0;
//static uint32_t highTime = 0;
//volatile uint8_t dutyCycle = 0;

volatile uint32_t dshotLastPulse = 0;
volatile uint32_t dshotPulseWidth = 0;

volatile uint8_t dshotReceiveFlag = 0;

DShotPacket dshotPacket = {0};

uint8_t dshotBuffer[16] = {0};
//uint32_t dshotBufferPulse[16];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void PWM_UpdateDutyCycle(uint8_t targetDutyCycle);
void PWM_UpdateFrequency(uint16_t targetFrequency);
void WriteDshotBuffer(uint8_t data);
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);

  if (PWM_ENABLED){
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  }
  if (DSHOT_ENABLED){
//	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  DShot_Init();
//	  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
	  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  }
  UART_Print("Init done!\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwmDutyCycle);
  PWM_UpdateDutyCycle(pwmDutyCycle);
  HAL_UART_Receive_IT(&huart1, uartRxBuffer, 1);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (timerFlag1000){
		  timerFlag1000 = 0;
		  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	  }
	  if (timerFlag2000){
		  timerFlag2000 = 0;
//		  UART_Printf("Frequency: %d, Duty: %d\n", (uint32_t)frequency, (uint32_t)dutyCycle);
//		  UART_Printf("DShot pulse width: %u\n", dshotPulseWidth);
	  }
	  if (uartRxFlag){
		  uartRxFlag = 0;
		  PWM_UpdateDutyCycle(pwmDutyCycle);
	  }
	  if (uartRxFreqFlag){
		  uartRxFreqFlag = 0;
		  PWM_UpdateFrequency(pwmFrequency);
	  }
	  if (uartRxDshotPacketFlag){
		  uartRxDshotPacketFlag = 0;
		  DShot_SendPacket(2047, 0);
	  }
	  if (dshotReceiveFlag){
		  dshotReceiveFlag = 0;
		  for (uint8_t i=0; i<16;i++){
			  UART_Printf("%d, ", dshotBuffer[i]);
		  }
		  UART_Print("\n");
		  DShot_Deserialize(&dshotPacket, dshotBuffer, 16);
		  UART_Printf("%d\n", dshotPacket.throttle);
		  UART_Printf("%d\n", dshotPacket.telemetry);
		  UART_Printf("%d\n", dshotPacket.crc);
		  uint8_t correct = DShot_ValidateCrc(&dshotPacket);
		  UART_Printf("%d\n", correct);
	  }
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
    	timerTicks++;
		if (timerTicks % 250 == 0){
			timerFlag250 = 1;
		}
		if (timerTicks % 500 == 0){
			timerFlag500 = 1;
		}
    	if (timerTicks % 1000 == 0){
    		timerFlag1000 = 1;
		}
    	if (timerTicks % 2000 == 0){
    		timerFlag2000 = 1;
		}
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
    	if (uartRxBuffer[0] == 'w'){
  		  if (pwmDutyCycle < 100){
      		pwmDutyCycle += 10;
      		uartRxFlag = 1;
  		  }
    	}
    	else if (uartRxBuffer[0] == 's'){
    		if (pwmDutyCycle >= 10){
    			pwmDutyCycle -= 10;
        		uartRxFlag = 1;
    		}
    	}
    	else if (uartRxBuffer[0] == 'r'){
    		if (pwmFrequency < 65000){
    			pwmFrequency += 1000;
    			uartRxFreqFlag = 1;
    		}
    	}
    	else if (uartRxBuffer[0] == 'f'){
    		if (pwmFrequency >= 1000){
    			pwmFrequency -= 1000;
    			uartRxFreqFlag = 1;
    		}
    	}
    	else if (uartRxBuffer[0] == 'q'){
    		if (DSHOT_ENABLED){
    			uartRxDshotPacketFlag = 1;
    		}

    	}
    	UART_Printf("%d, %d\n", pwmDutyCycle, pwmFrequency);
        HAL_UART_Receive_IT(&huart1, uartRxBuffer, 1);
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t edgeCounter = 0;
    if (htim->Instance == TIM3)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            // read period (time between rising edges)
            period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            ++edgeCounter;
        }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        {
            // read high time (rising to falling edge)
            highTime = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            ++edgeCounter;
        }
        // when both values are updated, calculate:
//        if (period != 0 && highTime != 0)
//        {
//            dutyCycle = ((float)highTime / (float)(period + 1)) * 100.0f;
////            if (edgeCounter % 2 == 0){
////                WriteDshotBuffer((uint32_t)dutyCycle);
////            }
////            dutyCycle = (uint8_t)((((float)highTime / (float)period) * 100.0f) + 0.5f);
//            frequency = (float)(SystemCoreClock / (htim->Instance->PSC + 1) / (float)(period + 1));
//        }
        dutyCycle = ((float)highTime / (float)(period + 1)) * 100.0f;
		if (edgeCounter % 2 == 0){
			WriteDshotBuffer((uint8_t)dutyCycle);
		}
//            dutyCycle = (uint8_t)((((float)highTime / (float)period) * 100.0f) + 0.5f);
        frequency = (float)(SystemCoreClock / (htim->Instance->PSC + 1) / (float)(period + 1));
    }
    else if (htim->Instance == TIM4){
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
        	uint32_t dshotPulse = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//        	dshotPulseWidth = dshotPulse - dshotLastPulse;
            if (dshotPulse >= dshotLastPulse){
            	dshotPulseWidth = dshotPulse - dshotLastPulse;
            }
            else{
            	dshotPulseWidth = (0xFFFF - dshotLastPulse) + dshotPulse;  // handle timer rollover
            }
        	dshotLastPulse = dshotPulse;
//        	WriteDshotBufferPulse(dshotPulseWidth);
        }
    }
}

void PWM_UpdateFrequency(uint16_t targetFrequency){
//	uint16_t targetARR = 1000000 / targetFrequency - 1; // clock is 1MHz
	uint16_t targetARR = SystemCoreClock / (htim2.Instance->PSC + 1) / targetFrequency - 1; // clock is 1MHz
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

//	__HAL_TIM_SET_PRESCALER(&htimX, newPrescaler);
	__HAL_TIM_SET_AUTORELOAD(&htim2, targetARR);

	__HAL_TIM_SET_COUNTER(&htim2, 0); // optional: reset counter
	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
	HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);

//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (targetARR + 1) / 2);
	PWM_UpdateDutyCycle(pwmDutyCycle);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void PWM_UpdateDutyCycle(uint8_t targetDutyCycle){
	uint16_t targetCompareValue = (targetDutyCycle * htim2.Instance->ARR) / 100;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, targetCompareValue);
}

//void WriteDshotBufferPulse(uint32_t data){
//	static uint8_t head = 0;
//	dshotBufferPulse[head] = data;
//	++head;
//	if (head > 15){
////		HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
//		highTime = 0;
//		period = 0;
//		head = 0;
//		dshotReceiveFlag = 1;
//	}
//}

void WriteDshotBuffer(uint8_t data){
	static uint8_t head = 0;
	dshotBuffer[head] = data;
	++head;
	if (head > 15){
//		HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
		highTime = 0;
		period = 0;
		head = 0;
		dshotReceiveFlag = 1;
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
