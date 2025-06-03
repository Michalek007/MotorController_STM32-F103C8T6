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
#include "pwm.h"
#include "dshot.h"
#include "communication.h"
#include "motor_pwm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_INPUT_CAPUTRE 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile uint8_t uartRxFlag = 0;
volatile uint8_t uartRxFreqFlag = 0;
volatile uint8_t uartRxDshotPacketFlag = 0;
uint8_t uartRxBuffer[1] = { 0 };

volatile uint32_t dshotLastPulse = 0;
volatile uint32_t dshotPulseWidth = 0;

volatile uint8_t dshotReceiveFlag = 0;

DShotPacket dshotPacket = { 0 };

uint8_t dshotBuffer[16] = { 0 };

uint32_t captureEdgeBuffer[32];
volatile uint8_t dshotRxFlag = 0;

uint16_t dshotCaptureBuffer[16];
uint16_t dshotCaptureBuffer2[16];
//uint32_t dshotBufferPulse[16];

CommunicationType commType = PWM;
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
int main(void) {
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

	if (commType == PWM) {
		MotorPwm_Init(&htim2, TIM_CHANNEL_1);
//		MotorPwm_SetThrottle(500);
//		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//		HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
//		HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
//		PWM_UpdateDutyCycle(pwmDutyCycle);
	}
	if (commType == DSHOT) {
		DShot_TimerInit(DSHOT150, &htim2, TIM_CHANNEL_1);
//		HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
//		HAL_TIM_IC_Start_DMA(&htim4, TIM_CHANNEL_1, (uint32_t*) dshotCaptureBuffer, 16);
//		HAL_TIM_IC_Start_DMA(&htim4, TIM_CHANNEL_2, (uint32_t*) dshotCaptureBuffer2, 16);
//
//		HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t*) dshotCaptureBuffer, 16); // Capture 32 samples from Channel 1
//		HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
//		HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	}
	Communication_TimerInit(commType, &htim3, TIM_CHANNEL_1, TIM_CHANNEL_2);
	UART_Print("Init done!\n");
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_UART_Receive_IT(&huart1, uartRxBuffer, 1);
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (timerFlag1000) {
			timerFlag1000 = 0;
			HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		}
		if (timerFlag2000) {
			timerFlag2000 = 0;
			if (PWM_INPUT_CAPUTRE) {
				UART_Printf("Frequency: %d, Duty: %d\n", (uint32_t) PWM_GetInputFreq(),
						(uint32_t) PWM_GetInputDutyCycle());
			}
		}
		if (uartRxFlag) {
			uartRxFlag = 0;
			PWM_UpdateDutyCycle(pwmDutyCycle);
		}
		if (uartRxFreqFlag) {
			uartRxFreqFlag = 0;
			PWM_UpdateFrequency(pwmFrequency);
		}
		if (uartRxDshotPacketFlag) {
			uartRxDshotPacketFlag = 0;
//			DShot_SendPacket(2047, 0);
			DShot_SendPacket(1024, 1);
		}
		Communication_SetThrottle();
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		if (uartRxBuffer[0] == 'w') {
			if (pwmDutyCycle < 100) {
				pwmDutyCycle += 10;
				uartRxFlag = 1;
			}
		} else if (uartRxBuffer[0] == 's') {
			if (pwmDutyCycle >= 10) {
				pwmDutyCycle -= 10;
				uartRxFlag = 1;
			}
		} else if (uartRxBuffer[0] == 'e') {
			if (pwmDutyCycle < 100) {
				pwmDutyCycle++;
				uartRxFlag = 1;
			}
		} else if (uartRxBuffer[0] == 'd') {
			if (pwmDutyCycle >= 1) {
				pwmDutyCycle--;
				uartRxFlag = 1;
			}
		} else if (uartRxBuffer[0] == 'r') {
			if (pwmFrequency < 65000) {
				pwmFrequency += 1000;
				uartRxFreqFlag = 1;
			}
		} else if (uartRxBuffer[0] == 'f') {
			if (pwmFrequency >= 1000) {
				pwmFrequency -= 1000;
				uartRxFreqFlag = 1;
			}
		} else if (uartRxBuffer[0] == 'q') {
			if (commType == DSHOT) {
				uartRxDshotPacketFlag = 1;
			}
		}
		UART_Printf("%d, %d\n", pwmDutyCycle, pwmFrequency);
		HAL_UART_Receive_IT(&huart1, uartRxBuffer, 1);
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
//	PWM_InputCaptureHandler(htim);
	Communication_InputCaptureHandler(htim);
}

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
//	static uint8_t edgeCounter = 0;
//	if (htim->Instance == TIM3) {
//		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
//			// read period (time between rising edges)
//			period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
////			UART_Printf("%d, ", period);
////			captureEdgeBuffer[edgeCounter] = period;
////			++edgeCounter;
//		} else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
//			// read high time (rising to falling edge)
//			highTime = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
////			UART_Printf("%d, ", highTime);
////			captureEdgeBuffer[edgeCounter] = highTime;
////			++edgeCounter;
//		}
////		if (edgeCounter >= 32) {
////			edgeCounter = 0;
////			dshotRxFlag = 1;
////		}
//
////		dutyCycle = ((float) highTime / (float) (period + 1)) * 100.0f;
////		if (edgeCounter % 2 == 0) {
////			WriteDshotBuffer((uint8_t) dutyCycle);
////		}
//		dutyCycle = (uint8_t) ((((float) highTime / (float) period) * 100.0f) + 0.5f);
//		frequency = (float) (SystemCoreClock / (htim->Instance->PSC + 1) / (float) (period + 1));
//	} else if (htim->Instance == TIM4) {
//		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
//			UART_Print("Done DMA1!");
//			for (uint8_t i = 0; i < 16; i++) {
//				UART_Printf("%d, ", dshotCaptureBuffer[i]);
//			}
////        	uint32_t dshotPulse = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//////        	dshotPulseWidth = dshotPulse - dshotLastPulse;
////            if (dshotPulse >= dshotLastPulse){
////            	dshotPulseWidth = dshotPulse - dshotLastPulse;
////            }
////            else{
////            	dshotPulseWidth = (0xFFFF - dshotLastPulse) + dshotPulse;  // handle timer rollover
////            }
////        	dshotLastPulse = dshotPulse;
//////        	WriteDshotBufferPulse(dshotPulseWidth);
//		} else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
//			UART_Print("Done DMA2!");
//			for (uint8_t i = 0; i < 16; i++) {
//				UART_Printf("%d, ", dshotCaptureBuffer2[i]);
//			}
//		}
//	}
//}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
