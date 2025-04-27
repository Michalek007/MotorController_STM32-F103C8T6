/*
 * dshot.c
 *
 *  Created on: Apr 10, 2025
 *      Author: Michał
 */

#include "dshot.h"
#include "tim.h"
#include "usart.h"


#define DSHOT_BIT_COUNT 16
uint16_t dshotData[DSHOT_BIT_COUNT] = {300, 700, 300, 700, 300, 700, 300, 700, 300, 700, 300, 700, 300, 700, 300, 700};
//uint16_t dshotData[DSHOT_BIT_COUNT] = {300, 700, 300, 700, 300, 700, 300, 700, 300, 700, 300, 700, 300, 700, 300, 700};


void DShot_Init(void){

}

void DShot_SendPacket(){
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)dshotData, DSHOT_BIT_COUNT);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
//        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
//		  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);
        // Optionally reset pin low or schedule next frame
    	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
    	UART_Print("Done");
    }
}


//
//// Sample value: throttle = 100, telemetry = 0 → compute checksum too
//uint16_t build_dshot_packet(uint16_t throttle)
//{
//    throttle <<= 1; // shift left to make room for telemetry bit
//    uint16_t csum = 0;
//    uint16_t csum_data = throttle;
//    for (int i = 0; i < 3; i++)
//        csum ^= (csum_data >> (i * 4)) & 0xF;
//
//    return (throttle << 4) | (csum & 0xF);
//}
//
//void prepare_dshot_dma_data(uint16_t dshot_packet)
//{
//    for (int i = 0; i < DSHOT_BIT_COUNT; i++)
//    {
//        if (dshot_packet & (1 << (15 - i)))
//            dshotData[i] = 47;  // logic 1: ~70% of 67
//        else
//            dshotData[i] = 20;  // logic 0: ~30% of 67
//    }
//}
//
//void send_packet(){
//	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)dshotData, DSHOT_BIT_COUNT);
//}

//void DShot_SendPacket(uint16_t throttle){
//	uint16_t dshotPacket = build_dshot_packet(throttle);
//	prepare_dshot_dma_data(dshotPacket);
//	send_packet();
//}
